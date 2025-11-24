from threading import Event, Thread

import rclpy
from rclpy.node import Node


class MakeTestNode(Node):
    def __init__(self, name="test_node", subscription_topic=None, msg_type=None):
        super().__init__(name)
        self.subscription_topic = subscription_topic
        self.msg_type = msg_type
        self.is_event_done = Event()
        self.return_msg = None
        self.ros_spin_thread = None

    def start_subscriber(self):
        if not self.subscription_topic or not self.msg_type:
            raise ValueError(
                "Must provide subscription_topic and msg_type before starting subscriber"
            )

        self.subscription = self.create_subscription(
            self.msg_type,
            self.subscription_topic,
            self.return_msg_as_string_callback,
            10,
        )
        self.ros_spin_thread = Thread(target=self._spin, daemon=True)
        self.ros_spin_thread.start()

    def _spin(self):
        rclpy.spin(self)

    def stop(self):
        """Stop spinning and clean up."""
        self.destroy_node()
        if self.ros_spin_thread and self.ros_spin_thread.is_alive():
            self.ros_spin_thread.join(timeout=1)

    def return_msg_as_string_callback(self, msg):
        self.return_msg = msg
        self.is_event_done.set()

    def get_return_msg(self):
        return self.return_msg
