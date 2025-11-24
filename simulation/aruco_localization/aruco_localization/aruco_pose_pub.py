import rclpy
from rclpy.node import Node
import numpy as np
import tf2_ros
import json
from geometry_msgs.msg import TransformStamped, Transform
from scipy.spatial.transform import Rotation as R
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time, Duration
from typing import Dict
from collections import defaultdict
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

MAX_FAILSAFE_TIME = Duration(seconds=5.0)


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

        self.declare_parameter("camera_enabled_ids", [163, 164, 165])
        self.declare_parameter("update_rate", 100)
        self.declare_parameter("all_namespaces", ["robot_agent_1"])

        self.camera_enabled_ids = self.get_parameter("camera_enabled_ids").value
        self.update_rate = self.get_parameter("update_rate").value
        self.all_namespaces = self.get_parameter("all_namespaces").value
        self.all_marker_ids = [
            int(ns.replace("robot_agent_", ""))
            for ns in self.all_namespaces
            if ns.startswith("robot_agent_")
            and ns.replace("robot_agent_", "").isdigit()
        ]

        self.last_stamp = self.get_clock().now()
        self.max_frame_age = Duration(seconds=1.0)
        self.latest_transform_received: Dict[int, Transform] = defaultdict(list)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Run detection and averaging at update_rate
        self.create_timer(1 / self.update_rate, self.camera_aruco_pose_callback)
        self.create_timer(1 / self.update_rate, self.publish_poses)

        qos_profile = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.marker_lost_pub = self.create_publisher(
            String, "/aruco_marker_seen", qos_profile
        )

        self.last_seen: Dict[int, rclpy.time.Time] = {}
        self.seen_markers = set()

        # Timer for publishing lost markers only once per second
        self.create_timer(1 / self.update_rate / 10, self.publish_seen_markers)

    def publish_poses(self):
        if not self.latest_transform_received:
            return

        for (
            marker_id,
            markers_latest_transform,
        ) in self.latest_transform_received.items():
            now = self.get_clock().now()
            if now <= self.last_stamp:
                now = self.last_stamp + Duration(nanoseconds=10)

            t_base_link_to_pallet_truck = TransformStamped()
            t_base_link_to_pallet_truck.header.stamp = now.to_msg()
            t_base_link_to_pallet_truck.header.frame_id = (
                f"robot_agent_{marker_id}/odom"
            )
            t_base_link_to_pallet_truck.child_frame_id = (
                f"robot_agent_{marker_id}/base_link"
            )
            t_base_link_to_pallet_truck.transform = markers_latest_transform

            self.tf_broadcaster.sendTransform(t_base_link_to_pallet_truck)
            self.last_stamp = now

    def camera_aruco_pose_callback(self):
        now = self.get_clock().now()
        markers_of_interest = self.all_marker_ids
        visible_transforms = defaultdict(list)

        for camera_id in self.camera_enabled_ids:
            for marker_id in markers_of_interest:
                target_frame = f"static_agents/camera_{camera_id}_marker_{marker_id}"
                if self.tf_buffer.can_transform(
                    f"robot_agent_{marker_id}/odom", target_frame, rclpy.time.Time()
                ):
                    t = self.tf_buffer.lookup_transform(
                        f"robot_agent_{marker_id}/odom", target_frame, rclpy.time.Time()
                    )
                    if (now - Time.from_msg(t.header.stamp)) < self.max_frame_age:
                        visible_transforms[marker_id].append(t)

        # Update seen markers
        for marker_id in markers_of_interest:
            if visible_transforms.get(marker_id):  # Marker is currently visible
                self.last_seen[marker_id] = now
                self.seen_markers.add(f"robot_agent_{marker_id}")
            else:
                # Marker is not visible now — check how long ago we last saw it
                if marker_id in self.last_seen:
                    if (now - self.last_seen[marker_id]) > MAX_FAILSAFE_TIME:
                        self.seen_markers.discard(f"robot_agent_{marker_id}")

        # Averaging transforms
        for marker_id, current_marker_transforms in visible_transforms.items():
            if not current_marker_transforms:
                continue
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
            avg_translation = np.mean(translations, axis=0)

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
            avg_quaternion = np.mean(quaternions, axis=0)
            avg_quaternion /= np.linalg.norm(avg_quaternion)

            avg_transform = Transform()
            avg_transform.translation.x = float(avg_translation[0])
            avg_transform.translation.y = float(avg_translation[1])
            avg_transform.translation.z = float(avg_translation[2])
            avg_transform.rotation.x = float(avg_quaternion[0])
            avg_transform.rotation.y = float(avg_quaternion[1])
            avg_transform.rotation.z = float(avg_quaternion[2])
            avg_transform.rotation.w = float(avg_quaternion[3])

            aruco_offset_transform = Transform()
            aruco_offset_transform.translation.x = 0.0
            aruco_offset_transform.translation.y = -0.8
            aruco_offset_transform.translation.z = -1.3
            aruco_offset_transform.rotation.x = 0.0
            aruco_offset_transform.rotation.y = 0.0
            aruco_offset_transform.rotation.z = -0.7071
            aruco_offset_transform.rotation.w = 0.7071

            self.latest_transform_received[marker_id] = compose_transforms(
                avg_transform, aruco_offset_transform
            )

    def publish_seen_markers(self):
        msg = String()
        msg.data = ",".join(sorted(self.seen_markers))
        self.marker_lost_pub.publish(msg)


def main():
    rclpy.init()
    node = ArucoPosePubNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
