import os
import ast
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_robots(context):
    pkg_humanoid_robot = get_package_share_directory("humanoid_robot")
    log_level = LaunchConfiguration("log_level")
    humanoid_str = context.perform_substitution(LaunchConfiguration("humanoids"))
    humanoids = ast.literal_eval(humanoid_str)

    actions = []
    # robot_namespaces=[]
    for humanoid in humanoids:
        # robot_namespaces.append(robot["namespace"])
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        pkg_humanoid_robot, "launch", "launch_humanoid.launch.py"
                    )
                ),
                launch_arguments={
                    "namespace": humanoid["namespace"],
                    "initial_pose_x": str(humanoid["initial_pose_x"]),
                    "initial_pose_y": str(humanoid["initial_pose_y"]),
                    "log_level": log_level,
                }.items(),
            )
        )
    return actions


def generate_launch_description():
    ld = LaunchDescription()
    ld.add_action(OpaqueFunction(function=launch_robots))
    return ld
