import os
import launch
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions


def generate_launch_description():
    start_safety_stop = launch.substitutions.LaunchConfiguration("start_safety_stop", default="true")
    params = os.path.join(
        get_package_share_directory("scenario_manager"), "config", "params.yaml"
    )

    namespace = "scenario_manager"
    return launch.LaunchDescription(
        [
            launch_ros.actions.Node(
                package="scenario_manager",
                executable="teleport_action_server",
                namespace=namespace,
                name="teleport_action_server",
                output="screen",
            ),
            launch_ros.actions.Node(
                package="scenario_manager",
                executable="set_speed_action_server",
                namespace=namespace,
                name="set_speed_action_server",
                output="screen",
            ),
            launch_ros.actions.Node(
                package="scenario_manager",
                executable="collision_action_server",
                namespace=namespace,
                name="collision_action_server",
                output="screen",
                parameters=[params],
            ),
            launch_ros.actions.Node(
                package="scenario_manager",
                executable="ttc",
                namespace=namespace,
                name="ttc",
                output="screen",
                parameters=[params],
            ),
            launch_ros.actions.Node(
                package="scenario_manager",
                executable="safety_stop",
                namespace=namespace,
                name="safety_stop",
                output="screen",
                condition=launch.conditions.IfCondition(start_safety_stop),
            ),
        ]
    )
