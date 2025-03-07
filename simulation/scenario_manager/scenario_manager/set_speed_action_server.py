import time

from geometry_msgs.msg import Twist

import rclpy
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from simlan_custom_msg.action import SetSpeed


class SetSpeedActionServer(Node):
    def __init__(self):
        super().__init__("set_speed_action_server")
        self._action_server = ActionServer(
            self, SetSpeed, "set_robot_speed", self.execute_callback
        )
        topic_name = "/jackal/scenario_vel"
        self._publisher = self.create_publisher(Twist, topic_name, 10)
        self._robot_pos_publishers = {}

        self._active_goals = {}

    def execute_callback(self, goal_handle):
        robot_name = goal_handle.request.robot_name
        self.get_logger().info(f"Executing goal for robot: {robot_name}")

        # Ensure no duplicate goal is running for the same robot
        if robot_name in self._active_goals:
            self.get_logger().error(
                f"Robot {robot_name} already has an active goal! Rejecting duplicate request."
            )
            goal_handle.abort()
            return SetSpeed.Result(
                success=False, message=f"Robot {robot_name} already has an active goal!"
            )

        self._active_goals[robot_name] = goal_handle  # Store active goal

        feedback_msg = SetSpeed.Feedback()
        duration = goal_handle.request.duration
        twist = goal_handle.request.twist

        self.get_logger().info(
            f"Moving robot: {robot_name} for {duration} seconds with speed: {twist.linear.x}"
        )

        if robot_name not in self._robot_pos_publishers:
            if robot_name == "jackal":
                topic_name = "/jackal/scenario_vel"
            elif robot_name == "pallet_truck":
                topic_name = "/pallet_truck/velocity_controller/cmd_vel_unstamped"
            elif robot_name == "infobot":
                topic_name = "/cmd_vel"
            else:
                self.get_logger().error(f"Robot {robot_name} is not known")
                goal_handle.abort()
                return SetSpeed.Result(
                    success=False, message=f"Unknown robot: {robot_name}"
                )

            self._robot_pos_publishers[robot_name] = self.create_publisher(
                Twist, topic_name, 10
            )

        _publisher = self._robot_pos_publishers[robot_name]

        start_time = self.get_clock().now().seconds_nanoseconds()[0]
        while (self.get_clock().now().seconds_nanoseconds()[0] - start_time) < duration:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info(f"Goal canceled for {robot_name}")
                del self._active_goals[robot_name]  # Remove from active goals
                return SetSpeed.Result(
                    success=False, message=f"Goal canceled for {robot_name}"
                )

            _publisher.publish(twist)
            feedback_msg.status = f"Moving robot: {robot_name}"
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(0.1)

        # Stop the robot
        _publisher.publish(Twist())

        goal_handle.succeed()
        result = SetSpeed.Result()
        result.robot_name = robot_name
        result.success = True
        result.message = f"Movement complete for {robot_name}"

        del self._active_goals[robot_name]  # Remove from active goals
        return result


def main(args=None):
    rclpy.init(args=args)
    action_server = SetSpeedActionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(action_server)
    executor.spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
