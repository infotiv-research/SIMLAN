#!/usr/bin/env python3

from nav_msgs.msg import Odometry
from simlan_custom_msg.msg import TTC

import numpy as np

import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster


class GazeboTFBroadcaster(Node):
    def __init__(self):
        super().__init__("ttc_node")
        self.robot1_name = "jackal"
        self.robot2_name = "robot_agent_1"

        self.robot1_pose = self.create_subscription(
            Odometry, f"/{self.robot1_name}/pose_data", self.robot1_state_callback, 10
        )
        self.robot2_pose_twist = self.create_subscription(
            Odometry, f"/{self.robot2_name}/pose_data", self.robot2_state_callback, 10
        )

        self.publisher = self.create_publisher(TTC, "/scenario_manager/ttc", 10)

        # Since the poses have to be listened on separate topics due to new gazebo, save the pose
        self.robot2_pose_twist: Odometry = None

        self.get_logger().info("TTC node has been started.")

    def robot1_state_callback(self, msg):
        if self.robot2_pose_twist is not None:
            pose1 = msg.pose.pose
            twist1 = msg.twist.twist  # Get the velocity data

            pose2 = self.robot2_pose_twist.pose.pose
            twist2 = self.robot2_pose_twist.twist.twist
            # calculate np arrays for the position and velocity of size [3]
            position1 = np.array([pose1.position.x, pose1.position.y, pose1.position.z])
            velocity1 = np.array([twist1.linear.x, twist1.linear.y, twist1.linear.z])
            position2 = np.array([pose2.position.x, pose2.position.y, pose2.position.z])
            velocity2 = np.array([twist2.linear.x, twist2.linear.y, twist2.linear.z])

            # calculate the relative position and velocity
            relative_position = position2 - position1
            relative_velocity = velocity2 - velocity1

            # calculate the time to collision
            ttc = -np.dot(relative_position, relative_velocity) / np.dot(
                relative_velocity, relative_velocity
            )

            # calculate closest point of approach
            cpa = relative_position + ttc * relative_velocity
            cpa = np.linalg.norm(cpa)

            # log the ttc and cpa
            self.get_logger().info("Time to collision: {}".format(ttc))
            self.get_logger().info("Closest point of approach: {}".format(cpa))

            # publish the ttc and cpa
            ttc_msg = TTC()
            ttc_msg.ttc = ttc
            ttc_msg.cpa = cpa
            self.publisher.publish(ttc_msg)

    def robot2_state_callback(self, msg):
        self.robot2_pose_twist = msg

def main(args=None):
    rclpy.init(args=args)
    node = GazeboTFBroadcaster()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
