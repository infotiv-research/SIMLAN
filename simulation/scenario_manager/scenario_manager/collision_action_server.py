import math
import time

import rclpy
from rclpy.action import ActionClient, ActionServer
from rclpy.node import Node

from simlan_custom_msg.action import Collision, SetSpeed, TeleportRobot


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

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Received action goal')
        angle = goal_handle.request.angle  # Extract the angle parameter
        angle = math.radians(angle)  # Convert the angle to radians
        angle += math.pi / 2  # Rotate the angle by 90 degrees

        # Compute the starting position of the robots
        infobot_x = self.collision_point[0]
        infobot_y = self.collision_point[1] - self.infobot_speed * self.experiment_time

        jackal_x = self.collision_point[0] - self.jackal_speed * self.experiment_time * math.cos(angle)
        jackal_y = self.collision_point[1] - self.jackal_speed * self.experiment_time * math.sin(angle)

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

        await self.set_robot_speed('jackal', self.jackal_speed, 0.0, 0.0, 3 * self.experiment_time)
        await self.set_robot_speed('infobot', self.infobot_speed, 0.0, 0.0, 3 * self.experiment_time)

        # Temporary solution to wait for both robots to move

        time.sleep(self.experiment_time * 5)

        goal_handle.succeed()
        result = Collision.Result()
        result.success = True
        result.message = 'Successfully executed collision scenario'

        return result

    async def teleport_robot(self, robot_name: str, x: float, y: float, z: float, qx: float, qy: float, qz: float, qw: float):
        self.get_logger().info(f'Teleporting {robot_name}')
        # log inputs:

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
        self.get_logger().info(f'Setting speed for {robot_name}')
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
        self.get_logger().info('Goal accepted' if goal_handle.accepted else 'Goal rejected')
        if goal_handle.accepted:
            goal_handle.get_result_async().add_done_callback(self.get_result_callback)
        else:
            self.get_logger().info('Goal rejected')

    async def get_result_callback(self, future):
        result = future.result().result
        success = result.success
        message = result.message
        robot_name = result.robot_name
        self.get_logger().info(f'Goal for {robot_name} completed with success: {success}')

        if success:
            if robot_name == 'jackal':
                self.moved_jackal = True
                self.get_logger().info('Jackal moved, shabang, bangity bang')
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
