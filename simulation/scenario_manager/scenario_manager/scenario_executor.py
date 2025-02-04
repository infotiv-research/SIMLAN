import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from simlan_custom_msg.action import TeleportRobot
from geometry_msgs.msg import Pose

class ScenarioExecutor(Node):

    def __init__(self):
        super().__init__('scenario_executor')
        self._action_client = ActionClient(self, TeleportRobot, 'teleport_robot')

    def send_teleport_request(self, robot_name, pose):
        goal_msg = TeleportRobot.Goal()
        goal_msg.robot_name = robot_name
        goal_msg.target_pose = pose

        self._action_client.wait_for_server()
        self.get_logger().info(f'Sending teleport request for {robot_name}')
        future = self._action_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info(f'Success: {result.message}')
        else:
            self.get_logger().info(f'Failure: {result.message}')

def main(args=None):
    rclpy.init(args=args)
    executor = ScenarioExecutor()

    # Example: Teleport two robots
    pose1 = Pose()
    pose1.position.x = 1.0
    pose1.position.y = 2.0
    pose1.position.z = 0.0

    pose2 = Pose()
    pose2.position.x = 3.0
    pose2.position.y = 4.0
    pose2.position.z = 0.0

    executor.send_teleport_request('robot_1', pose1)
    executor.send_teleport_request('robot_2', pose2)

    rclpy.spin(executor)
    executor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
