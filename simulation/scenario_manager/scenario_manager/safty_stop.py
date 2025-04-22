#!/usr/bin/env python3
import numpy as np

from gazebo_msgs.msg import ModelStates

from geometry_msgs.msg import Twist

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node

from simlan_custom_msg.msg import TTC


class SaftyStop(Node):
    def __init__(self):
        super().__init__("safty_stop")
        self.subscription = self.create_subscription(
            ModelStates, "/gazebo/model_states", self.model_states_callback, 10
        )

        self.robot1_name = "jackal"
        self.robot2_name = "pallet_truck"

        # create publisher for jackal/safty_vel topic
        self.publisher = self.create_publisher(Twist, "/jackal/safty_vel", 10)
        # closes point of approach allowed messured in meters
        self.safty_margin = 2

        # time before collision to make a hard stop messured in seconds
        self.hard_stop_time = 2

        self.status = "move forward"
        # save the time the last command started:

        # nuber of times in a row the robot state must be move forward for it to count as such
        self.required_confidence = 15
        self.confidence = 0

        self.jackal_speed = 1.0
        self.jackal_acceleration = 0.05

        self.last_command_time = self.get_clock().now()
        self.command_duration = Duration(seconds=2.5)

    def calclutae_ttc(self, pose1, pose2, vel1, vel2):
        delta_pose = np.array(
            [pose2.position.x - pose1.position.x, pose2.position.y - pose1.position.y]
        )
        delta_vel = np.array(
            [vel2.linear.x - vel1.linear.x, vel2.linear.y - vel1.linear.y]
        )

        # Calculate the time to collision (TTC) and closest point of approach (CPA)
        ttc = -np.dot(delta_pose, delta_vel) / np.dot(delta_vel, delta_vel)
        response = TTC()
        response.ttc = ttc
        response.cpa = np.linalg.norm(delta_pose + ttc * delta_vel)

        return response

    def model_states_callback(self, msg):
        if self.robot1_name in msg.name and self.robot2_name in msg.name:
            idx1 = msg.name.index(self.robot1_name)
            pose1 = msg.pose[idx1]
            twist1 = msg.twist[idx1]  # Get the velocity data

            pose2 = msg.pose[msg.name.index(self.robot2_name)]
            twist2 = msg.twist[msg.name.index(self.robot2_name)]

            result = self.calclutae_ttc(pose1, pose2, twist1, twist2)

            vel1 = np.array([twist1.linear.x, twist1.linear.y, twist1.linear.z])
            # normalize the velocity vector
            vel1 = vel1 / np.linalg.norm(vel1) if np.linalg.norm(vel1) != 0 else vel1
            twist1_ = Twist()
            twist1_.linear.x, twist1_.linear.y, twist1_.linear.z = vel1

            result_ = self.calclutae_ttc(pose1, pose2, twist1_, twist2)

            ttc = result.ttc
            cpa = result.cpa
            ttc_ = result_.ttc
            cpa_ = result_.cpa
        else:
            return

        if ttc >= 0 and cpa <= self.safty_margin:
            self.confidence = 0
            if ttc <= self.hard_stop_time or ttc_ <= self.hard_stop_time:
                self.get_logger().info("Hard stop")
                self.satus = "hard stop"

            elif ttc > self.hard_stop_time or ttc_ > self.hard_stop_time:
                self.get_logger().info("Soft stop")
                self.status = "soft stop"

        elif ttc_ >= 0 and cpa_ <= self.safty_margin:
            self.confidence = 0
            if ttc <= self.hard_stop_time or ttc_ <= self.hard_stop_time:
                self.get_logger().info("Hard stop*")
                self.satus = "hard stop"

            elif ttc > self.hard_stop_time or ttc_ > self.hard_stop_time:
                self.get_logger().info("Soft stop*")
                self.status = "soft stop"
        else:
            self.get_logger().info("Move forward")
            self.confidence += 1
            if self.confidence > self.required_confidence:
                self.status = "move forward"
                self.last_command_time = self.get_clock().now()
                self.jackal_speed = min(
                    1.0, self.jackal_speed + self.jackal_acceleration
                )

        if self.status == "hard stop":
            self.get_logger().info("Hard stop")
            hard_stop_msg = Twist()
            hard_stop_msg.linear.x = 0.0
            self.publisher.publish(hard_stop_msg)

        elif self.status == "soft stop":
            soft_stop_msg = Twist()
            self.jackal_speed = max(self.jackal_speed - self.jackal_acceleration, 0.0)
            soft_stop_msg.linear.x = self.jackal_speed
            self.publisher.publish(soft_stop_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SaftyStop()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
