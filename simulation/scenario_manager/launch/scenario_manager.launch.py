import launch

import launch_ros.actions


def generate_launch_description():
    namespace = 'scenario_manager'
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='scenario_manager',
            executable='teleport_action_server',
            namespace=namespace,
            name='teleport_action_server',
            output='screen'
        ),
        launch_ros.actions.Node(
            package='scenario_manager',
            executable='set_speed_action_server',
            namespace=namespace,
            name='set_speed_action_server',
            output='screen'
        ),
        launch_ros.actions.Node(
            package='scenario_manager',
            executable='collision_action_server',
            namespace=namespace,
            name='collision_action_server',
            output='screen'
        ),
    ])
