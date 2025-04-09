#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

from rclpy.duration import Duration

# or explicitly in
from simlan_custom_msg.msg import TTC


class SaftyStop(Node):
    def __init__(self):
        super().__init__("safty_stop")
        self.subscription = self.create_subscription(
            TTC, "/scenario_manager/ttc", self.model_states_callback, 10
        )
        # create publisher for jackal/safty_vel topic
        self.publisher = self.create_publisher(Twist, "/jackal/safty_vel", 10)
        # closes point of approach allowed messured in meters
        self.safty_margin = 2

        # time before collision to make a hard stop messured in seconds
        self.hard_stop_time = 2

        self.status = "move forward"
        # save the time the last command started:

        self.last_command_time = self.get_clock().now()
        self.command_duration = Duration(seconds=2.5)

    def model_states_callback(self, msg):
        ttc = msg.ttc
        cpa = msg.cpa
        if ttc >= 0 and cpa <= self.safty_margin:
            if ttc <= self.hard_stop_time:
                self.get_logger().info("Hard stop")
                self.satus = "hard stop"
                self.last_command_time = self.get_clock().now()

            if ttc > self.hard_stop_time:
                self.get_logger().info("Soft stop")
                self.status = "soft stop"
                self.last_command_time = self.get_clock().now()

        if self.last_command_time + self.command_duration > self.get_clock().now():
            if self.status == "hard stop":
                self.get_logger().info("Hard stop")
                hard_stop_msg = Twist()
                hard_stop_msg.linear.x = 0.0
                self.publisher.publish(hard_stop_msg)

            elif self.status == "soft stop":
                soft_stop_msg = Twist()
                soft_stop_msg.linear.x = 0.2
                self.publisher.publish(soft_stop_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SaftyStop()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
