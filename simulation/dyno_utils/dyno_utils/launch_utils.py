import asyncio
import collections.abc
from functools import partial
from typing import Iterable
from typing import Optional
from typing import Text, Tuple, Union

from launch.actions import TimerAction
from launch.events import TimerEvent
from launch.frontend import Entity
from launch.frontend import expose_action
from launch.frontend import Parser
from launch.launch_context import LaunchContext
from launch.launch_description_entity import LaunchDescriptionEntity
from launch.some_substitutions_type import SomeSubstitutionsType
from launch.some_substitutions_type import SomeSubstitutionsType_types_tuple
from launch.utilities import create_future
from launch.utilities import ensure_argument_type
from launch.utilities import type_utils

from launch_ros.ros_adapters import get_ros_node
import std_msgs.msg
import rclpy.node
from rclpy.qos import QoSProfile


@expose_action("dyno_timer")
class DynoWaitFor(TimerAction):
    """
    Action that defers other entities until a period of time has passed, unless canceled.

    This timer uses ROS time instead of wall clock time.
    To enable the use of sim time, you must also use the SetUseSimTime action.
    All timers are "one-shot", in that they only fire one time and never again.
    """

    def __init__(
        self,
        *,
        name: str,
        actions: Iterable[LaunchDescriptionEntity],
        message_on_topics: Iterable[
            Tuple[str, type, Union[QoSProfile, int]]
        ],  # topic, msg type, qos
        **kwargs,
    ) -> None:
        """
        Create a DynoWaitFor.

        :param period: is the time (in seconds) to set the timer for.
        :param actions: is an iterable containing actions to be executed upon on timeout.
        """
        super().__init__(period=0.5, actions=actions, **kwargs)
        self.name = name
        self.message_on_topics = message_on_topics
        self.subscriptions = {}
        self.__timer_future: Optional[asyncio.Future] = None

    async def _wait_to_fire_event(self, context):
        node: rclpy.node.Node = get_ros_node(context)

        def remove_subscription(topic, msg):
            if topic in self.subscriptions:
                node.destroy_subscription(self.subscriptions[topic])
                del self.subscriptions[topic]

        for topic, msg_type, qos in self.message_on_topics:
            self.subscriptions[topic] = node.create_subscription(
                msg_type,
                topic,
                partial(
                    context.asyncio_loop.call_soon_threadsafe,
                    partial(remove_subscription, topic),
                ),
                qos,
            )

        def timer_callback():
            if len(self.subscriptions) == 0:
                node.destroy_timer(timer)
                if not self.__timer_future.done():
                    node.get_logger().info(f"{self.name} is done waiting!")
                    self.__timer_future.set_result(True)
            else:
                node.get_logger().info(
                    f"{self.name} is waiting for: {' '.join(topic for topic in self.subscriptions.keys())}",
                    throttle_duration_sec=5.0,
                )

        timer = node.create_timer(
            self.period,
            partial(context.asyncio_loop.call_soon_threadsafe, timer_callback),
        )

        done, pending = await asyncio.wait(
            [self._canceled_future, self.__timer_future],
            return_when=asyncio.FIRST_COMPLETED,
        )

        if not self._canceled_future.done():
            await context.emit_event(TimerEvent(timer_action=self))
        self._completed_future.set_result(None)

    @classmethod
    def parse(
        cls,
        entity: Entity,
        parser: Parser,
    ):
        """Return the `DynoWaitFor` action and kwargs for constructing it."""
        _, kwargs = super().parse(entity, parser)
        kwargs["period"] = parser.parse_if_substitutions(
            entity.get_attr("period", data_type=float, can_be_str=True)
        )
        kwargs["actions"] = [parser.parse_action(child) for child in entity.children]
        return cls, kwargs

    def describe(self) -> Text:
        """Return a description of this DynoWaitFor."""
        return "DynoWaitFor(period={}, actions=<actions>)".format(self.__period)

    def execute(self, context: LaunchContext):
        self.__timer_future = create_future(context.asyncio_loop)
        return super().execute(context)
