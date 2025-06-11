import rclpy
from rclpy.node import Node
import numpy as np
import tf2_ros
from geometry_msgs.msg import TransformStamped, Transform
from scipy.spatial.transform import Rotation as R
from tf2_ros import Buffer, TransformListener
from nav_msgs.msg import Odometry
from rclpy.time import Time, Duration


"""_summary_
    This node listens to the TF and tries to find any link from origin (base_link for now) to an aruco marker. As of now we only care about the aruco marker on the pallet truck that has id 12.
    We iterate through every camera to see if any have noticed a marker and if we for a small timeframe find multiple, we average their pose and rotation from origin to the marker, together using a simple averaging algorithm.
    lastly we publish this single transform into either odom or TF. You should not do both since a child link can only have one parent.

"""


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

        self.declare_parameter("camera_ids", [163, 164, 165, 166, 167])
        self.declare_parameter("publish_to_odom", True)

        self.publish_to_odom = self.get_parameter("publish_to_odom").value
        self.camera_ids = self.get_parameter("camera_ids").value
        self.last_stamp = self.get_clock().now()
        self.max_frame_age = Duration(seconds=1)
        self.latest_transform_received: Transform = None

        # node attributes
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Publishers
        self.timer = self.create_timer(
            0.01, self.camera_aruco_pose_callback
        )  # Every 0.1 is 10Hz
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.create_timer(
            0.01, self.publish_poses
        )  # every 0.05s is 20Hz. 20 runs per sec

    def publish_poses(self):
        if self.latest_transform_received == None:
            return
        now = self.get_clock().now()
        if now <= self.last_stamp:
            now = self.last_stamp + Duration(nanoseconds=10)

        # Publish new TF based on our computed average translation and rotation to ODOM and TF
        odom_pallet_truck = Odometry()
        odom_pallet_truck.header.stamp = now.to_msg()
        odom_pallet_truck.header.frame_id = "odom"  # Is actually base_link
        odom_pallet_truck.child_frame_id = "pallet_truck_base_link"
        odom_pallet_truck.pose.pose.position.x = (
            self.latest_transform_received.translation.x
        )
        odom_pallet_truck.pose.pose.position.y = (
            self.latest_transform_received.translation.y
        )
        odom_pallet_truck.pose.pose.position.z = (
            self.latest_transform_received.translation.z
        )
        odom_pallet_truck.pose.pose.orientation = (
            self.latest_transform_received.rotation
        )

        # odom to pallet truck base link.
        t_base_link_to_pallet_truck = TransformStamped()
        t_base_link_to_pallet_truck.header.stamp = self.get_clock().now().to_msg()
        t_base_link_to_pallet_truck.header.frame_id = "base_link"
        t_base_link_to_pallet_truck.child_frame_id = "pallet_truck_base_link"
        t_base_link_to_pallet_truck.transform = self.latest_transform_received

        # We send the transforms to odom or base_link depending on the publish_to_odom flag.
        if self.publish_to_odom:
            self.odom_pub.publish(odom_pallet_truck)
        else:
            self.tf_broadcaster.sendTransform(t_base_link_to_pallet_truck)
        # we set new timestamp for next iteration
        self.last_stamp = now

    # Callbacks
    def camera_aruco_pose_callback(self):
        now = self.get_clock().now()

        visible_transforms = []
        now = self.get_clock().now()
        for camera_id in self.camera_ids:
            target_frame = f"camera_{camera_id}_marker_12"

            if self.tf_buffer.can_transform(
                "base_link", target_frame, rclpy.time.Time()
            ):
                t = self.tf_buffer.lookup_transform(
                    "base_link", target_frame, rclpy.time.Time()
                )
                # We only take recent frames into account based on max_frame_age
                if (now - Time.from_msg(t.header.stamp)) < self.max_frame_age:
                    visible_transforms.append(t)

        if not visible_transforms:
            return

        #### AVERAGING THE TRANSFORMS. ####

        # 1. Translation averaging. We create a list of the poses from the translations and do an averaging on them
        translations = np.array(
            [
                [
                    t.transform.translation.x,
                    t.transform.translation.y,
                    t.transform.translation.z,
                ]
                for t in visible_transforms
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
                for t in visible_transforms
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
        self.latest_transform_received = compose_transforms(
            avg_transform, aruco_offset_transform
        )


def main():

    rclpy.init()
    node = ArucoPosePubNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
