#!/usr/bin/env python3

from gazebo_msgs.msg import ModelStates

from geometry_msgs.msg import TransformStamped

import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster


class GazeboTFBroadcaster(Node):
    def __init__(self):
        super().__init__('gazebo_tf_broadcaster')
        self.subscription = self.create_subscription(
            ModelStates,
            '/gazebo/model_states',
            self.model_states_callback,
            10
        )
        self.robot_name = 'jackal'

    def model_states_callback(self, msg):
        if self.robot_name in msg.name:
            idx = msg.name.index(self.robot_name)
            pose = msg.pose[idx]

            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'world'
            t.child_frame_id = 'base_link'

            t.transform.translation.x = pose.position.x
            t.transform.translation.y = pose.position.y
            t.transform.translation.z = pose.position.z
            t.transform.rotation = pose.orientation

            # log all the information

            self.get_logger().info(f'Publishing transform from {t.header.frame_id} to {t.child_frame_id}')
            self.get_logger().info(f'Translation: {t.transform.translation}')
            self.get_logger().info(f'Rotation: {t.transform.rotation}')


def main(args=None):
    rclpy.init(args=args)
    node = GazeboTFBroadcaster()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
