import math
import time

import rclpy
from rclpy.action import ActionClient, ActionServer
from rclpy.node import Node

from simlan_custom_msg.action import Collision, SetSpeed, TeleportRobot
from simlan_custom_msg.msg import CollisionType


class CollisionActionServer(Node):
    def __init__(self):
        super().__init__('collision_action_server')
        self._action_server = ActionServer(
            self,
            Collision,  # Define your custom action type
            'collision_action',
            self.execute_callback)
        self.collision_point = (25.0, 34.0)
        self.jackal_speed = 1.0
        self.infobot_speed = 1.0
        self.experiment_time = 5.0

        # Distance from the base-link to the front of the robot
        # Should be half of the robot's length if the base-link is centered
        self.infobot_front_distance = 1.9
        self.jackal_front_distance = 0.255

        # Width of the robot, it is assumed the robot is symmetrical
        self.jackal_width = 0.1
        self.infobot_width = 0.1

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Received action goal')
        angle = goal_handle.request.angle  # Extract the angle parameter
        infobot_speed = goal_handle.request.infobot_speed  # Extract the infobot_speed parameter
        collision_type = goal_handle.request.collision_type  # Extract the collision_type parameter
        angle = math.radians(angle)  # Convert the angle to radians
        angle += math.pi / 2  # Rotate the angle by 90 degrees

        # calculate travel distance
        infobot_travel_distance = infobot_speed * self.experiment_time
        jackal_travel_distance = self.jackal_speed * self.experiment_time

        if collision_type.collision_type == CollisionType.HEAD_ON:
            infobot_travel_distance += self.infobot_front_distance
            jackal_travel_distance += self.jackal_front_distance
        elif collision_type.collision_type == CollisionType.INFOBOT_SIDE:
            jackal_travel_distance += self.jackal_front_distance
        elif collision_type.collision_type == CollisionType.JACKAL_SIDE:
            infobot_travel_distance += self.infobot_front_distance

        # Compute the starting position of the robots
        infobot_x = self.collision_point[0]
        infobot_y = self.collision_point[1] - infobot_travel_distance

        jackal_x = self.collision_point[0] - jackal_travel_distance * math.cos(angle)
        jackal_y = self.collision_point[1] - jackal_travel_distance * math.sin(angle)

        # Compute orientation quaternion
        qx, qy, qz, qw = 0.0, 0.0, math.sin(angle / 2), math.cos(angle / 2)

        # Teleport both robots
        await self.teleport_robot('jackal', jackal_x, jackal_y, 0.1, qx, qy, qz, qw)
        goal_handle.publish_feedback(Collision.Feedback(feedback='Successfully teleported jackal'))

        await self.teleport_robot('infobot', infobot_x, infobot_y, 0.1, 0.0, 0.0, math.sqrt(2) / 2, math.sqrt(2) / 2)
        goal_handle.publish_feedback(Collision.Feedback(feedback='Successfully teleported infobot'))

        # Set speed in parallel
        self.moved_jackal = False
        self.moved_infobot = False

        await self.set_robot_speed('jackal', self.jackal_speed, 0.0, 0.0, 4 * self.experiment_time)
        await self.set_robot_speed('infobot', infobot_speed, 0.0, 0.0, 4 * self.experiment_time)

        # Temporary solution to wait for both robots to move

        time.sleep(self.experiment_time * 5)

        goal_handle.succeed()
        result = Collision.Result()
        result.success = True
        result.message = 'Successfully executed collision scenario'

        return result

    async def teleport_robot(self, robot_name: str, x: float, y: float, z: float, qx: float, qy: float, qz: float, qw: float):
        action_client = ActionClient(self, TeleportRobot, '/scenario_manager/teleport_robot')
        action_client.wait_for_server()
        goal_msg = TeleportRobot.Goal()
        goal_msg.robot_name = robot_name
        goal_msg.target_pose.position.x = x
        goal_msg.target_pose.position.y = y
        goal_msg.target_pose.position.z = z
        goal_msg.target_pose.orientation.x = qx
        goal_msg.target_pose.orientation.y = qy
        goal_msg.target_pose.orientation.z = qz
        goal_msg.target_pose.orientation.w = qw
        await action_client.send_goal_async(goal_msg)

    async def set_robot_speed(self, robot_name, vx, vy, vz, duration):
        action_client = ActionClient(self, SetSpeed, '/scenario_manager/set_robot_speed')
        action_client.wait_for_server()
        goal_msg = SetSpeed.Goal()
        goal_msg.robot_name = robot_name
        goal_msg.duration = duration
        goal_msg.twist.linear.x = vx
        goal_msg.twist.linear.y = vy
        goal_msg.twist.linear.z = vz
        goal_msg.twist.angular.x = 0.0
        goal_msg.twist.angular.y = 0.0
        goal_msg.twist.angular.z = 0.0
        # Send the goal and print the result
        action_client.send_goal_async(goal_msg).add_done_callback(self.goal_response_callback)

    async def goal_response_callback(self, future):
        goal_handle = future.result()
        if goal_handle.accepted:
            goal_handle.get_result_async().add_done_callback(self.get_result_callback)
        else:
            self.get_logger().info('Goal rejected')

    async def get_result_callback(self, future):
        result = future.result().result
        success = result.success
        message = result.message
        robot_name = result.robot_name
        if success:
            if robot_name == 'jackal':
                self.moved_jackal = True
                # print message:
                self.get_logger().info(message)
            if robot_name == 'infobot':
                self.moved_infobot = True
        else:
            self.get_logger().info(f'Failed to move {robot_name}')


def main(args=None):
    rclpy.init(args=args)
    node = CollisionActionServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
