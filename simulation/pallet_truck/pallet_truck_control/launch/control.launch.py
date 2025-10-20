from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import UnlessCondition
from launch.substitutions import (
    PathJoinSubstitution,
    LaunchConfiguration,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # Configs
    config_velocity_controller = PathJoinSubstitution(
        [FindPackageShare("pallet_truck_control"), "config", "control.yaml"],
    )

    is_sim = LaunchConfiguration("is_sim", default=True)
    log_level = LaunchConfiguration("log_level", default="INFO")

    is_sim_arg = DeclareLaunchArgument("is_sim", default_value=is_sim)
    namespace = LaunchConfiguration("namespace")
    # ROS2 Controllers
    control_group_action = GroupAction(
        [
            # ROS2 Control
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                parameters=[config_velocity_controller],
                output={
                    "stdout": "screen",
                    "stderr": "screen",
                },
                remappings=[("/tf", "tf")],
                namespace=namespace,
                condition=UnlessCondition(is_sim),
            ),
            # Joint State Broadcaster
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "joint_state_broadcaster",
                    "-c",
                    "controller_manager",
                    "--ros-args", "--log-level", log_level,

                ],
                output="screen",
                namespace=namespace,

            ),
            # Velocity Controller
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "velocity_controller",
                    "-c",
                    "controller_manager",
                    "--ros-args",
                    "--log-level",
                    log_level
                ],
                output="screen",
               
                namespace=namespace,

            ),
        ]
    )

    ld = LaunchDescription()
    # ld.add_action(robot_description_command_arg)
    ld.add_action(is_sim_arg)
    # ld.add_action(localization_group_action)
    ld.add_action(control_group_action)
    return ld
