import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Pose
from scipy.spatial.transform import Rotation as R
import subprocess
import threading
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

MAP_PUBLISHER_FREQUENCY = 10.0  # hz


class OdomUpdater(Node):
    def __init__(self):
        super().__init__("odom_updater")

        # input parameters
        self.declare_parameter("namespace", "humanoid_1")
        self.namespace = self.get_parameter("namespace").value
        self.declare_parameter("initial_pose_x", 0.0)
        self.initial_pose_x = float(self.get_parameter("initial_pose_x").value)
        self.declare_parameter("initial_pose_y", 0.0)
        self.initial_pose_y = float(self.get_parameter("initial_pose_y").value)

        # publisher with QoS
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.tf_broadcaster = TransformBroadcaster(self)
        self.latest_pose = None

        pose_topic = f"/{self.namespace}/pose"
        self.pose_sub = self.create_subscription(
            Pose, pose_topic, self.pose_callback, qos
        )

        # timer to call publish_tf
        self.create_timer(1.0 / MAP_PUBLISHER_FREQUENCY, self.publish_tf)

        self.get_logger().info("Odom updater started...")

    def pose_callback(self, msg: Pose):
        """Callback for /namespace/pose topic"""
        # Subtract initial offset from x and y
        self.latest_pose = {
            "x": msg.position.x - self.initial_pose_x,
            "y": msg.position.y - self.initial_pose_y,
            "z": msg.position.z,
            "qx": msg.orientation.x,
            "qy": msg.orientation.y,
            "qz": msg.orientation.z,
            "qw": msg.orientation.w,
        }

    def publish_tf(self):
        """Publish the latest {self.namespace} pose as Odometry and TF frame"""
        if not self.latest_pose:
            return

        p = self.latest_pose
        q = (p["qx"], p["qy"], p["qz"], p["qw"])
        r = R.from_quat(q)
        roll, pitch, yaw = r.as_euler("xyz", degrees=False)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = f"{self.namespace}/odom"
        t.child_frame_id = f"{self.namespace}/base_link"

        # position
        t.transform.translation.x = p["x"]
        t.transform.translation.y = p["y"]
        t.transform.translation.z = p["z"]

        # orientation
        t.transform.rotation = Quaternion(x=p["qx"], y=p["qy"], z=p["qz"], w=p["qw"])

        self.tf_broadcaster.sendTransform(t)
        # if logging is needed
        # self.get_logger().info(f"Published {self.namespace} pose: "
        #                        f"x={p['x']:.3f}, y={p['y']:.3f}, z={p['z']:.3f}, "
        #                        f"rpy=({roll:.2f},{pitch:.2f},{yaw:.2f})")


def main(args=None):
    rclpy.init(args=args)
    node = OdomUpdater()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
