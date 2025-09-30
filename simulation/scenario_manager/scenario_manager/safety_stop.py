#!/usr/bin/env python3

from geometry_msgs.msg import Twist

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node

from simlan_custom_msg.msg import TTC


class SafetyStop(Node):
    def __init__(self):
        super().__init__("safety_stop")

        # SUBSCRIBERS
        self.ttc_sub = self.create_subscription(
            TTC, "/scenario_manager/ttc", self.ttc_callback, 10
        )

        # PUBLISHERS
        self.cmd_pub = self.create_publisher(Twist, "/jackal/safety_vel", 10)

        self.safety_margin = 2.0          # meters (CPA threshold)
        self.hard_stop_time = 2.0         # seconds (TTC threshold)

        self.status = "move forward"
        self.last_logged_status = None    # For flip‑flop logging only

        self.required_confidence = 15
        self.confidence = 0

        self.jackal_speed = 1.0
        self.jackal_crawl = 0.2
        self.jackal_acceleration = 0.05

        self.last_command_time = self.get_clock().now()
        self.command_duration = Duration(seconds=2.5)

    def ttc_callback(self, msg: TTC):

        # ttc: Time To Collision
        # cpa: Closest Point of Arrival
        ttc, cpa = msg.ttc, msg.cpa

        self.get_logger().debug(f"TTC: {ttc:.2f}s | CPA: {cpa:.2f}m")

        # Decide next state
        new_status = self.status
        hazard = (
            (ttc >= 0 and cpa <= self.safety_margin)
        )

        if hazard:
            self.confidence = 0
            hard = (
                (ttc >= 0 and ttc <= self.hard_stop_time)
            )
            new_status = "hard stop" if hard else "soft stop"
        else:
            self.confidence += 1
            if self.confidence > self.required_confidence:
                new_status = "move forward"
                self.last_command_time = self.get_clock().now()
                # Accelerate only when confidently moving
                self.jackal_speed = min(1.0, self.jackal_speed + self.jackal_acceleration)

        # Update & log only on change (flip‑flop logging)
        if new_status != self.status:
            self.status = new_status
            self._log_state_change(ttc, cpa)

        # Always publish command (continuous control)
        cmd = Twist()
        if self.status == "hard stop":
            self.jackal_speed = 0.0 
            cmd.linear.x = 0.0
        elif self.status == "soft stop":
            # Gradually reduce speed
            self.jackal_speed = max(self.jackal_crawl, self.jackal_speed - self.jackal_acceleration)
            cmd.linear.x = self.jackal_speed
        else:  # move forward
            cmd.linear.x = self.jackal_speed

        self.cmd_pub.publish(cmd)

    def _log_state_change(self, ttc, cpa):
        if self.status != self.last_logged_status:
            self.get_logger().info(
                f"State -> {self.status} | "
                f"TTC: {ttc:.2f}s | "
                f"CPA: {cpa:.2f}m"
            )
            self.last_logged_status = self.status

def main(args=None):
    rclpy.init(args=args)
    node = SafetyStop()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
