import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from simlan_custom_msg.action import TeleportRobot
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import Pose, PoseStamped
from gazebo_msgs.msg import EntityState


class TeleportActionServer(Node):

    def __init__(self):
        super().__init__('teleport_action_server')

        # Action server
        self._action_server = ActionServer(
            self,
            TeleportRobot,
            'teleport_robot',
            self.execute_callback
        )

        # Gazebo service client
        self._client = self.create_client(SetEntityState, '/gazebo/set_entity_state')
        while not self._client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for gazebo/set_entity_state service...')

    async def execute_callback(self, goal_handle):
        self.get_logger().info(f'Received teleport request for {goal_handle.request.robot_name}')
        # Prepare Gazebo service request

        request = SetEntityState.Request()
        state = EntityState()
        state.name = goal_handle.request.robot_name
        state.pose = goal_handle.request.target_pose
        state.reference_frame = 'world'
        request.state = state

        future = self._client.call_async(request)
        await future

        if future.result() is not None:
            self.get_logger().info(f'Successfully teleported {state.name}')
            goal_handle.succeed()
            return TeleportRobot.Result(success=True, message='Teleportation successful')
        else:
            self.get_logger().error(f'Failed to teleport {state.name}')
            goal_handle.abort()
            return TeleportRobot.Result(success=False, message='Teleportation failed')

def main(args=None):
    rclpy.init(args=args)
    teleport_action_server = TeleportActionServer()
    rclpy.spin(teleport_action_server)
    teleport_action_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
