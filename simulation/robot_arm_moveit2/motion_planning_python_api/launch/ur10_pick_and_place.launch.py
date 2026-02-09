"""
Launch file for the UR10 pick-and-place MoveItPy script.
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    log_level_arg = DeclareLaunchArgument(
        "log_level", default_value="info", description="Logging level"
    )
    log_level = LaunchConfiguration("log_level")

    moveitpy_yaml = os.path.join(
        get_package_share_directory("motion_planning_python_api"),
        "config",
        "ur10_moveitpy.yaml",
    )

    moveit_config = (
        MoveItConfigsBuilder(robot_name="ur", package_name="ur_moveit_config")
        .robot_description(
            file_path="config/ur10.urdf.xacro",
            mappings={"ros2_control_hardware_type": "gz"},
        )
        .robot_description_semantic(Path("srdf") / "ur.srdf.xacro", {"name": "ur"})
        .trajectory_execution(file_path="config/gazebo_moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl", "chomp"])
        .moveit_cpp(file_path=moveitpy_yaml)
        .to_moveit_configs()
    )

    moveit_config_dict = moveit_config.to_dict()
    moveit_config_dict["use_sim_time"] = True

    package_path = get_package_share_directory("motion_planning_python_api")
    small_cube_location = os.path.join(package_path, "models/small_cube.sdf")

    spawn_cube = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            "small_cube",
            "-file",
            small_cube_location,
            "-x",
            "40.31",
            "-y",
            "0.8",
            "-z",
            "1.23",
            "--ros-args",
            "--log-level",
            log_level,
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            log_level_arg,
            spawn_cube,
            Node(
                name="moveit_py",
                package="motion_planning_python_api",
                executable="ur10_pick_and_place",
                output="both",
                parameters=[moveit_config_dict],
            ),
        ]
    )
