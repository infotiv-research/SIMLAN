from scenario_execution.interfaces import Action
import rclpy
from custom_msg.action import TeleportRobot
from rclpy.action import ActionClient
from geometry_msgs.msg import Pose

class TeleportRobotAction(Action):
    def __init__(self, context):
        super().__init__(context)
        self.node = context.node
        self._action_client = ActionClient(self.node, TeleportRobot, 'teleport_robot')

    def run(self, args):
        robot_name = args.get('robot_name', 'robot_1')
        target_pose = Pose()
        target_pose.position.x = args.get('x', 0.0)
        target_pose.position.y = args.get('y', 0.0)
        target_pose.position.z = args.get('z', 0.0)

        self.node.get_logger().info(f'Teleporting {robot_name} to ({target_pose.position.x}, {target_pose.position.y})')

        goal_msg = TeleportRobot.Goal()
        goal_msg.robot_name = robot_name
        goal_msg.target_pose = target_pose

        self._action_client.wait_for_server()
        future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self.node, future)

        if future.result():
            result = future.result().result
            if result.success:
                self.node.get_logger().info(f'Success: {result.message}')
            else:
                self.node.get_logger().error(f'Failure: {result.message}')
        else:
            self.node.get_logger().error('Failed to send teleport action request')

    def shutdown(self):
        self.node.get_logger().info('Shutting down TeleportRobotAction')
