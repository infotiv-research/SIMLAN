#!/usr/bin/env python3
# Launch: plan & execute once to a random RobotState (example-style)

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    log_level = LaunchConfiguration("log_level", default="INFO")
    namespace = LaunchConfiguration("namespace", default="humanoid_1")

    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="humanoid",
            package_name="humanoid_support_moveit_config",
        )
        .robot_description(file_path="config/human_support.urdf.xacro")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .moveit_cpp(
            file_path=os.path.join(
                get_package_share_directory("motion_planner"),
                "config",
                "motion_planner.yaml",
            )
        )
        .to_moveit_configs()
    )

    moveit_py_node = Node(
        name="moveit_py",
        package="motion_planner",
        executable="motion_planner",
        output="screen",
        emulate_tty=True,
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True},
            {"group_name": "support_whole_body"},
            {"run_random_generate": LaunchConfiguration("run_random_generate")},
            {"output_dir": LaunchConfiguration("output_dir")},
            {"namespace": namespace},
        ],
        arguments=["--ros-args", "--log-level", log_level],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "run_random_generate",
                default_value="False",
                description="If true we run this node to randomly run poses.",
            ),
            DeclareLaunchArgument(
                "output_dir",
                default_value="humanoid_utility/DATASET/motion_data",
                description="Directory to save output motions",
            ),
            moveit_py_node,
        ]
    )
