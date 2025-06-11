#!/usr/bin/env python3

from gazebo_msgs.msg import ModelStates

from geometry_msgs.msg import TransformStamped

import numpy as np

import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster


class GazeboTFBroadcaster(Node):
    def __init__(self):
        super().__init__("gazebo_tf_broadcaster")
        self.subscription = self.create_subscription(
            ModelStates, "/gazebo/model_states", self.model_states_callback, 10
        )
        self.robot1_name = "jackal"
        self.robot2_name = "pallet_truck"

    def model_states_callback(self, msg):
        if self.robot1_name in msg.name:
            idx1 = msg.name.index(self.robot1_name)
            pose1 = msg.pose[idx1]
            twist1 = msg.twist[idx1]  # Get the velocity data

            pose2 = msg.pose[msg.name.index(self.robot2_name)]
            twist2 = msg.twist[msg.name.index(self.robot2_name)]
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


def main(args=None):
    rclpy.init(args=args)
    node = GazeboTFBroadcaster()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
