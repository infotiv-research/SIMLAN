#!/usr/bin/env python3

import numpy as np

import rclpy
from rclpy.node import Node

from simlan_custom_msg.srv import CalculateTTC  # Import your custom service

### NOTE: CURRENTLY UNUSED - TTC CALCULATION IS IN ttc.py ###
class calculateTTCService(Node):
    def __init__(self):
        super().__init__("calculate_ttc_server")
        # Create the service with type calculateTTC on the 'add_two_ints' topic
        self.srv = self.create_service(
            CalculateTTC, "calculate_ttc", self.calculate_ttc
        )
        self.get_logger().info("calculateTTC service is ready.")

    def calculate_ttc(self, request, response):
        self.get_logger().info("Received request to calculate TTC")
        robot1_pose = request.robot1_pose
        robot2_pose = request.robot2_pose
        robot1_vel = request.robot1_vel
        robot2_vel = request.robot2_vel

        # We can't subtract two poses, so we have to do it another way:

        delta_pose = np.array(
            [
                robot2_pose.position.x - robot1_pose.position.x,
                robot2_pose.position.y - robot1_pose.position.y,
            ]
        )
        delta_vel = np.array(
            [
                robot2_vel.linear.x - robot1_vel.linear.x,
                robot2_vel.linear.y - robot1_vel.linear.y,
            ]
        )

        # Calculate the time to collision (TTC) and closest point of approach (CPA)
        ttc = -np.dot(delta_pose, delta_vel) / np.dot(delta_vel, delta_vel)
        response.ttc.ttc = ttc
        response.ttc.cpa = np.linalg.norm(delta_pose + ttc * delta_vel)

        self.get_logger().info(f"TTC: {response.ttc.ttc}, CPA: {response.ttc.cpa}")
        return response


def main(args=None):
    rclpy.init(args=args)
    node = calculateTTCService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == "__main__":
    main()
