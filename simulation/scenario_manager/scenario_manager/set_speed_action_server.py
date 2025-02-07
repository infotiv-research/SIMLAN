import time

from geometry_msgs.msg import Twist

import rclpy
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from simlan_custom_msg.action import SetSpeed


class SetSpeedActionServer(Node):
    def __init__(self):
        super().__init__('move_robot_action_server')
        self._action_server = ActionServer(
            self,
            SetSpeed,
            'set_robot_speed',
            self.execute_callback)
        topic_name = '/jackal/velocity_controller/cmd_vel_unstamped'
        self._publisher = self.create_publisher(Twist, topic_name, 10)
        self._robot_pos_publishers = {}

    def execute_callback(self, goal_handle):
        robot_name = goal_handle.request.robot_name
        self.get_logger().info(f'Executing goal for robot: {robot_name}')
        feedback_msg = SetSpeed.Feedback()
        duration = goal_handle.request.duration
        twist = goal_handle.request.twist

        # add to publishers
        if robot_name not in self._robot_pos_publishers:
            if robot_name == 'jackal':
                topic_name = '/jackal/velocity_controller/cmd_vel_unstamped'
            elif robot_name == 'infobot':
                topic_name = '/cmd_vel'
            else:
                self.get_logger().error(f'Robot {robot_name} is not known')
            self._robot_pos_publishers[robot_name] = self.create_publisher(Twist, topic_name, 10)
            _publisher = self._robot_pos_publishers[robot_name]
        else:
            _publisher = self._robot_pos_publishers[robot_name]

        start_time = self.get_clock().now().seconds_nanoseconds()[0]
        while (self.get_clock().now().seconds_nanoseconds()[0] - start_time) < duration:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return SetSpeed.Result(success=False, message='Goal canceled')

            _publisher.publish(twist)
            feedback_msg.status = f'Moving robot: {goal_handle.request.robot_name}'
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(0.1)

        goal_handle.succeed()
        result = SetSpeed.Result()
        result.success = True
        result.message = f'Movement complete for {goal_handle.request.robot_name}'
        return result


def main(args=None):
    rclpy.init(args=args)
    action_server = SetSpeedActionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(action_server)
    executor.spin()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
