#!/usr/bin/env python3

from nav_msgs.msg import Odometry
from simlan_custom_msg.msg import TTC

import numpy as np
import math

import rclpy
from rclpy.node import Node


class GazeboTFBroadcaster(Node):
    def __init__(self):
        super().__init__("ttc_node")
        self.robot1_name = "jackal"
        self.robot2_name = "robot_agent_1"

        # If True: twist.linear already in world frame (skip rotation)
        self.twist_in_world_frame = False

        ttc_update_rate = 10.0
        self.update_period = 1.0 / max(0.1, ttc_update_rate)

        # Subscriptions (just store latest messages)
        self.robot1_sub = self.create_subscription(
            Odometry, f"/{self.robot1_name}/pose_data", self.robot1_callback, 10
        )
        self.robot2_sub = self.create_subscription(
            Odometry, f"/{self.robot2_name}/pose_data", self.robot2_callback, 10
        )

        # Publisher
        self.ttc_pub = self.create_publisher(TTC, "/scenario_manager/ttc", 10)

        # Stored latest odometry
        self.robot1_odom: Odometry | None = None
        self.robot2_odom: Odometry | None = None

        # Timer for controlled update frequency
        self.timer = self.create_timer(self.update_period, self.timer_compute_ttc)

        self.get_logger().info(
            f"TTC node started. Update rate: {ttc_update_rate:.2f} Hz (period {self.update_period:.3f}s)"
        )

    def yaw_from_quaternion(self, q):
        """
        Extract planar yaw from geometry_msgs/Pose orientation quaternion.
        """
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def planar_world_velocity(self, pose, twist):
        vx = twist.linear.x
        vy = twist.linear.y
        if self.twist_in_world_frame:
            return np.array([vx, vy], dtype=float)

        q = pose.orientation
        yaw = self.yaw_from_quaternion(q)
        c = math.cos(yaw)
        s = math.sin(yaw)
        wx = c * vx - s * vy
        wy = s * vx + c * vy
        return np.array([wx, wy], dtype=float)

    # ------------------ Callbacks just storing data ------------------ #
    def robot1_callback(self, msg: Odometry):
        self.robot1_odom = msg

    def robot2_callback(self, msg: Odometry):
        self.robot2_odom = msg

    # ------------------ Timer (controlled frequency) ----------------- #
    def timer_compute_ttc(self):
        if self.robot1_odom is None or self.robot2_odom is None:
            return

        pose1 = self.robot1_odom.pose.pose
        twist1 = self.robot1_odom.twist.twist
        pose2 = self.robot2_odom.pose.pose
        twist2 = self.robot2_odom.twist.twist

        p1 = np.array([pose1.position.x, pose1.position.y], dtype=float)
        p2 = np.array([pose2.position.x, pose2.position.y], dtype=float)

        v1 = self.planar_world_velocity(pose1, twist1)
        v2 = self.planar_world_velocity(pose2, twist2)

        r = p2 - p1          # relative position
        v = v2 - v1          # relative velocity

        v2_norm = np.dot(v, v)
        if v2_norm < 1e-9:
            ttc = -1.0
            cpa = float(np.linalg.norm(r))
        else:
            t_star = -np.dot(r, v) / v2_norm
            if t_star <= 0.0:
                ttc = -1.0
                cpa = float(np.linalg.norm(r))
            else:
                ttc = float(t_star)
                cpa_vec = r + t_star * v
                cpa = float(np.linalg.norm(cpa_vec))

        msg = TTC()
        msg.ttc = ttc
        msg.cpa = cpa
        self.ttc_pub.publish(msg)

    # (Optional) add a dynamic rate update method if needed later


def main(args=None):
    rclpy.init(args=args)
    node = GazeboTFBroadcaster()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
