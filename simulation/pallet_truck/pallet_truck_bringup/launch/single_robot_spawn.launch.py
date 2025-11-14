import os.path
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
)
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction


def launch_setup(context, *args, **kwargs):
    namespace = LaunchConfiguration("namespace").perform(context)

    initial_pose_x = LaunchConfiguration("initial_pose_x")
    initial_pose_y = LaunchConfiguration("initial_pose_y")
    aruco_id = LaunchConfiguration("aruco_id")
    robot_type = LaunchConfiguration("robot_type")
    log_level = LaunchConfiguration("log_level")

    pkg_pallet_truck_bringup = get_package_share_directory("pallet_truck_bringup")
    pkg_pallet_truck_control = get_package_share_directory("pallet_truck_control")

    gazebo_spawn = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_pallet_truck_bringup, "launch", "gazebo.launch.py")
            ),
            launch_arguments= {
                "namespace": namespace,
                "initial_pose_x": initial_pose_x,
                "initial_pose_y": initial_pose_y,
                "aruco_id": aruco_id,
                "robot_type": robot_type,
                "log_level": log_level
            }.items()
        )
    
    control = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_pallet_truck_control, "launch", "control.launch.py")
            ),
            launch_arguments={
                "namespace": namespace,
                "log_level":log_level
            }.items(),
        )

    actions=[
        gazebo_spawn,
        control,
    ]
    
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="robot_agent_1"),
        OpaqueFunction(function=launch_setup),
    ])