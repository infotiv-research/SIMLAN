import os
import ast
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

REQUIRED_KEYS = ["namespace", "initial_pose_x", "initial_pose_y", "robot_type", "aruco_id"]

def launch_robots(context):
    pkg_pallet_truck_bringup = get_package_share_directory("pallet_truck_bringup")
    robots_str = context.perform_substitution(LaunchConfiguration('robots'))
    log_level = LaunchConfiguration('log_level')
    robots = ast.literal_eval(robots_str)

    actions=[]
    robot_namespaces=[]
    for robot in robots:
        robot_namespaces.append(robot["namespace"])
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_pallet_truck_bringup, "launch", "single_robot_spawn.launch.py")
                ),
                launch_arguments={
                    "namespace": robot["namespace"],
                    "initial_pose_x": robot["initial_pose_x"],
                    "initial_pose_y": robot["initial_pose_y"],
                    "robot_type":robot["robot_type"],
                    "aruco_id": robot["aruco_id"],
                    "log_level": log_level
                }.items()
            )
        )
    return actions

def generate_launch_description():
    ld=LaunchDescription()
    ld.add_action(OpaqueFunction(function=launch_robots))
    return ld