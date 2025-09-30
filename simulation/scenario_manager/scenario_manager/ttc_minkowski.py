from nav_msgs.msg import Odometry
from simlan_custom_msg.msg import TTC

import numpy as np
import math

import rclpy
from rclpy.node import Node


class TTCMinkowskiBroadcaster(Node):
    def __init__(self):
        super().__init__("ttc_node")
        self.robot1_name = "jackal"
        self.robot2_name = "robot_agent_1"

        # Velocities are in each robot's local frame (vx forward). We rotate to world.
        self.twist_in_world_frame = False

        # Rect sizes (length, width) in meters (center-based OBB)
        self.declare_parameter("jackal_size", [0.5, 0.43])
        self.declare_parameter("pallet_truck_size", [1.1, 0.87])
        s1 = self.get_parameter("jackal_size").value
        s2 = self.get_parameter("pallet_truck_size").value
        self.a_half = (s1[0] * 0.5, s1[1] * 0.5)
        self.b_half = (s2[0] * 0.5, s2[1] * 0.5)

        # Update rate (Hz)
        ttc_update_rate = 10.0
        self.update_period = 1.0 / max(0.1, ttc_update_rate)

        self.robot1_sub = self.create_subscription(
            Odometry, f"/{self.robot1_name}/pose_data", self.robot1_callback, 10
        )
        self.robot2_sub = self.create_subscription(
            Odometry, f"/{self.robot2_name}/pose_data", self.robot2_callback, 10
        )

        self.ttc_pub = self.create_publisher(TTC, "/scenario_manager/ttc", 10)

        self.robot1_odom: Odometry | None = None
        self.robot2_odom: Odometry | None = None

        self.timer = self.create_timer(self.update_period, self.timer_compute_ttc)

        self.get_logger().info(
            f"TTC node started. Update rate: {ttc_update_rate:.2f} Hz (period {self.update_period:.3f}s)"
        )

    # ---------- Helpers ----------
    def yaw_from_quaternion(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def planar_world_velocity(self, pose, twist):
        vx = twist.linear.x
        vy = twist.linear.y
        if self.twist_in_world_frame:
            return np.array([vx, vy], dtype=float)
        yaw = self.yaw_from_quaternion(pose.orientation)
        c = math.cos(yaw)
        s = math.sin(yaw)
        return np.array([c * vx - s * vy, s * vx + c * vy], dtype=float)

    def box_axes(self, yaw):
        # Unit axes of OBB (a0 forward, a1 left)
        c = math.cos(yaw)
        s = math.sin(yaw)
        return np.array([c, s]), np.array([-s, c])

    # ---------- OBB vs OBB continuous TTC (piecewise constant vel) ----------
    def ttc_oriented_boxes(self, cA, cB, vA, vB, yawA, yawB):
        """
        Returns (ttc, will_collide, currently_overlapping)
        ttc = time >= 0 when first overlap occurs (seconds) or -1.
        Assumes linear constant velocities until next update.
        """
        # Relative motion: treat A static, B moving with v = vB - vA
        r0 = cB - cA
        v = vB - vA

        # Axes to test (Separating Axis Theorem)
        a0, a1 = self.box_axes(yawA)
        b0, b1 = self.box_axes(yawB)
        axes = [a0, a1, b0, b1]

        t_entry = 0.0
        t_exit = float("inf")
        overlapping_now = True

        for n in axes:
            # Sum of projected half-extents on axis n
            proj = (
                abs(np.dot(n, a0)) * self.a_half[0]
                + abs(np.dot(n, a1)) * self.a_half[1]
                + abs(np.dot(n, b0)) * self.b_half[0]
                + abs(np.dot(n, b1)) * self.b_half[1]
            )
            p0 = np.dot(n, r0)
            pv = np.dot(n, v)

            dist_now = abs(p0)
            if dist_now > proj:
                overlapping_now = False

            if abs(pv) < 1e-9:
                # Static separation along this axis
                if dist_now > proj:
                    return -1.0, False, False  # Never collide (separated, no motion to close gap)
                # Else always overlapping on this axis -> no time constraint
                continue

            # Solve -proj <= p0 + pv t <= proj
            t1 = (-proj - p0) / pv
            t2 = (proj - p0) / pv
            t_axis_enter = min(t1, t2)
            t_axis_exit = max(t1, t2)

            # Update global interval
            t_entry = max(t_entry, t_axis_enter)
            t_exit = min(t_exit, t_axis_exit)

            if t_entry > t_exit:
                return -1.0, False, overlapping_now  # Disjoint intervals -> no collision

        if overlapping_now:
            # Already overlapping (could set ttc=0)
            return 0.0, True, True

        if t_exit < 0:
            return -1.0, False, False  # Overlap only in the past

        if t_entry < 0:
            # Collision will (or has) started before now; treat as immediate
            return 0.0, True, False

        return t_entry, True, False

    # ---------- Sub callbacks ----------
    def robot1_callback(self, msg: Odometry):
        self.robot1_odom = msg

    def robot2_callback(self, msg: Odometry):
        self.robot2_odom = msg

    # ---------- Timer ----------
    def timer_compute_ttc(self):
        if self.robot1_odom is None or self.robot2_odom is None:
            return

        pose1 = self.robot1_odom.pose.pose
        pose2 = self.robot2_odom.pose.pose
        twist1 = self.robot1_odom.twist.twist
        twist2 = self.robot2_odom.twist.twist

        cA = np.array([pose1.position.x, pose1.position.y], dtype=float)
        cB = np.array([pose2.position.x, pose2.position.y], dtype=float)
        vA = self.planar_world_velocity(pose1, twist1)
        vB = self.planar_world_velocity(pose2, twist2)
        yawA = self.yaw_from_quaternion(pose1.orientation)
        yawB = self.yaw_from_quaternion(pose2.orientation)

        ttc_box, will_collide, overlapping_now = self.ttc_oriented_boxes(
            cA, cB, vA, vB, yawA, yawB
        )

        # Fallback CPA (center-distance) if no future collision
        if will_collide:
            if ttc_box < 0:
                ttc_val = -1.0
            else:
                ttc_val = ttc_box
            # For TTC message cpa = distance at TTC (approx). Use center distance or 0 if overlapping.
            if overlapping_now or ttc_box == 0.0:
                cpa_dist = 0.0
            else:
                rel = (cB + vB * ttc_box) - (cA + vA * ttc_box)
                cpa_dist = float(np.linalg.norm(rel))
        else:
            ttc_val = -1.0
            rel_now = cB - cA
            cpa_dist = float(np.linalg.norm(rel_now))

        msg = TTC()
        msg.ttc = ttc_val
        msg.cpa = cpa_dist
        self.ttc_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TTCMinkowskiBroadcaster()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()