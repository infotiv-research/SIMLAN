import rclpy
from rclpy.node import Node
import numpy as np
import tf2_ros
from geometry_msgs.msg import TransformStamped, Transform
from scipy.spatial.transform import Rotation as R
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time, Duration
from typing import Dict
from collections import defaultdict


# Helper function that extracts the rotation matrix from a Transform
def transform_to_matrix(transform: Transform):
    trans = transform.translation
    rot = transform.rotation

    # Convert quaternion to 3x3 rotation matrix
    r = R.from_quat([rot.x, rot.y, rot.z, rot.w])
    rot_matrix = r.as_matrix()

    # Build 4x4 transformation matrix
    t_matrix = np.eye(4)
    t_matrix[:3, :3] = rot_matrix
    t_matrix[:3, 3] = [trans.x, trans.y, trans.z]
    return t_matrix


# Helper function that combines two transforms.
def compose_transforms(tf1: Transform, tf2: Transform) -> Transform:
    mat1 = transform_to_matrix(tf1)
    mat2 = transform_to_matrix(tf2)
    composed = mat1 @ mat2

    # Extract translation
    translation = composed[:3, 3]

    # Extract rotation
    rot_matrix = composed[:3, :3]
    quaternion = R.from_matrix(rot_matrix).as_quat()  # [x, y, z, w]

    # Create new TransformStamped
    result = Transform()
    result.translation.x = translation[0]
    result.translation.y = translation[1]
    result.translation.z = translation[2]
    result.rotation.x = quaternion[0]
    result.rotation.y = quaternion[1]
    result.rotation.z = quaternion[2]
    result.rotation.w = quaternion[3]

    return result


class ArucoPosePubNode(Node):

    def __init__(self):
        super().__init__("aruco_pose_publisher")

        self.declare_parameter(
            "camera_enabled_ids",
            [
                163,
                164,
                165,
            ],
        )
        self.declare_parameter("update_rate", 100)  # Updates N times per second

        self.camera_enabled_ids = self.get_parameter("camera_enabled_ids").value
        self.update_rate = self.get_parameter("update_rate").value
        self.last_stamp = self.get_clock().now()
        self.max_frame_age = Duration(seconds=1)
        self.latest_transform_received: Dict[int, Transform] = defaultdict(list)
        # node attributes
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Publishers
        self.timer = self.create_timer(
            1 / self.update_rate, self.camera_aruco_pose_callback
        )  # Every 0.1 is 10Hz
        self.create_timer(
            1 / self.update_rate, self.publish_poses
        )  # every 0.05s is 20Hz. 20 runs per sec

    def publish_poses(self):

        if self.latest_transform_received == {}:
            return

        for (
            marker_id,
            markers_latest_transform,
        ) in self.latest_transform_received.items():

            now = self.get_clock().now()
            if now <= self.last_stamp:
                now = self.last_stamp + Duration(nanoseconds=10)

            # Creating frame with parent: robot_agent_X_odom and child: robot_agent_X_base_link.
            t_base_link_to_pallet_truck = TransformStamped()
            t_base_link_to_pallet_truck.header.stamp = self.get_clock().now().to_msg()
            t_base_link_to_pallet_truck.header.frame_id = (
                f"robot_agent_{marker_id}_odom"
            )
            t_base_link_to_pallet_truck.child_frame_id = (
                f"robot_agent_{marker_id}_base_link"
            )
            t_base_link_to_pallet_truck.transform = markers_latest_transform

            self.tf_broadcaster.sendTransform(t_base_link_to_pallet_truck)
            # we set new timestamp for next iteration
            self.last_stamp = now

    # Callbacks
    def camera_aruco_pose_callback(self):
        now = self.get_clock().now()
        markers_of_interest = [1, 2, 3, 4]

        visible_transforms = defaultdict(
            list
        )  # visible transform is a dict with key: marker id of interest and value an array of the transforms.
        now = self.get_clock().now()
        # We check every camera
        for camera_id in self.camera_enabled_ids:
            # We go over every marker of interest and see if they are noticed in camera
            for marker_id in markers_of_interest:

                target_frame = f"camera_{camera_id}_marker_{marker_id}"
                if self.tf_buffer.can_transform(
                    "base_link", target_frame, rclpy.time.Time()
                ):
                    t = self.tf_buffer.lookup_transform(
                        "base_link", target_frame, rclpy.time.Time()
                    )
                    # We only take recent frames into account based on max_frame_age
                    if (now - Time.from_msg(t.header.stamp)) < self.max_frame_age:
                        visible_transforms[marker_id].append(t)

        # For every marker ID we go over the transforms we got from the cameras and average them.
        for marker_id, current_marker_transforms in visible_transforms.items():
            # If marker had no transform then go for the next one. NOTE: Since this is a dict now this will never happen as the item won't exist
            if not current_marker_transforms:
                print(
                    f"marker {marker_id} had no transforms, continuing to the next marker."
                )
                continue
            #### AVERAGING THE TRANSFORMS. ####

            # 1. Translation averaging. We create a list of the poses from the translations and do an averaging on them
            translations = np.array(
                [
                    [
                        t.transform.translation.x,
                        t.transform.translation.y,
                        t.transform.translation.z,
                    ]
                    for t in current_marker_transforms
                ]
            )
            # average translation result
            avg_translation = np.mean(translations, axis=0)

            # 2. Rotation averaging. We create a list of the poses from the translations and do an averaging on them
            quaternions = np.array(
                [
                    [
                        t.transform.rotation.x,
                        t.transform.rotation.y,
                        t.transform.rotation.z,
                        t.transform.rotation.w,
                    ]
                    for t in current_marker_transforms
                ]
            )

            # average rotation result
            avg_quaternion = np.mean(quaternions, axis=0)
            avg_quaternion /= np.linalg.norm(avg_quaternion)

            #### PUBLISHING THE TRANSFORM ####

            # Since we want to set the base_link as the root we take the distance from the aruco marker and pallet_truck_base_link into account
            avg_transform = Transform()
            avg_transform.translation.x = float(avg_translation[0])
            avg_transform.translation.y = float(avg_translation[1])
            avg_transform.translation.z = float(avg_translation[2])
            avg_transform.rotation.x = float(avg_quaternion[0])
            avg_transform.rotation.y = float(avg_quaternion[1])
            avg_transform.rotation.z = float(avg_quaternion[2])
            avg_transform.rotation.w = float(avg_quaternion[3])

            # The transform between aruco_12_link and pallet_truck_base_link
            aruco_offset_transform = Transform()
            aruco_offset_transform.translation.x = float(0)
            aruco_offset_transform.translation.y = float(-0.8)  # -0.8 0 1.30
            aruco_offset_transform.translation.z = float(-1.3)
            aruco_offset_transform.rotation.x = float(0)
            aruco_offset_transform.rotation.y = float(0)
            aruco_offset_transform.rotation.z = float(-0.7071)  # 1.57 radians
            aruco_offset_transform.rotation.w = float(0.7071)

            # Combine the two transforms
            self.latest_transform_received[marker_id] = compose_transforms(
                avg_transform, aruco_offset_transform
            )


def main():

    rclpy.init()
    node = ArucoPosePubNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
