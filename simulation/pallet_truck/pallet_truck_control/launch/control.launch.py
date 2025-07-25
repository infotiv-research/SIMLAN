from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import UnlessCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
    LaunchConfiguration,
)
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    # Configs
    config_pallet_truck_ekf = PathJoinSubstitution(
        [FindPackageShare("pallet_truck_control"), "config", "localization.yaml"],
    )

    config_velocity_controller = PathJoinSubstitution(
        [FindPackageShare("pallet_truck_control"), "config", "control.yaml"],
    )

    # Launch Arguments

    robot_description_command_arg = DeclareLaunchArgument(
        "robot_description_command",
        default_value=[
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("pallet_truck_description"),
                    "urdf",
                    "pallet_truck.urdf.xacro",
                ]
            ),
        ],
    )

    is_sim = LaunchConfiguration("is_sim", default=True)

    is_sim_arg = DeclareLaunchArgument("is_sim", default_value=is_sim)
    namespace = LaunchConfiguration("namespace")

    # Localization
    localization_group_action = GroupAction(
        [
            # Extended Kalman Filter
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_node",
                output="screen",
                parameters=[config_pallet_truck_ekf],
            ),
        ]
    )

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
                ],
                output="screen",
            ),
            # Velocity Controller
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "velocity_controller",
                    "-c",
                    "controller_manager",
                ],
                output="screen",
            ),
        ]
    )

    ld = LaunchDescription()
    # ld.add_action(robot_description_command_arg)
    ld.add_action(is_sim_arg)
    # ld.add_action(localization_group_action)
    ld.add_action(control_group_action)
    return ld
