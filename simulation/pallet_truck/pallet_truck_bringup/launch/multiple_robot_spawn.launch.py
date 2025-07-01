import os.path
import launch
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node, PushRosNamespace, SetRemap
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory
import rclpy.qos
import std_msgs.msg
import sensor_msgs.msg
import rosgraph_msgs.msg


def generate_launch_description():
    
    # Launch data
    robots = [
        {
            "namespace": "robot_agent_1", 
            "initial_pose_x":"10.0", 
            "initial_pose_y":"1.0", 
            "robot_type":"pallet_truck", 
            "aruco_id":"1"
        },
        {
            "namespace": "robot_agent_2", 
            "initial_pose_x":"15.0", 
            "initial_pose_y":"1.0", 
            "robot_type":"pallet_truck", 
            "aruco_id":"2" 
        },
        {
            "namespace": "robot_agent_3", 
            "initial_pose_x":"12.0", 
            "initial_pose_y":"3.0", 
            "robot_type":"forklift", 
            "aruco_id":"3" 
        },
        {
            "namespace": "robot_agent_4", 
            "initial_pose_x":"5.0", 
            "initial_pose_y":"3.0", 
            "robot_type":"forklift", 
            "aruco_id":"4" 
        },
    ]

    # Launch args
    use_sim_time = LaunchConfiguration("use_sim_time", default=True)
    pkg_pallet_truck_bringup = get_package_share_directory("pallet_truck_bringup")
    
    spawn_robot_cmds = []
    for robot in robots:
        print(f"Robot: {robot['namespace']}")
        spawn_robot_cmds.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_pallet_truck_bringup, "launch", "sim.launch.py")
                ),
                launch_arguments={
                    "namespace": robot["namespace"],
                    "initial_pose_x": robot["initial_pose_x"],
                    "initial_pose_y": robot["initial_pose_y"],
                    "robot_type":robot["robot_type"],
                    "aruco_id": robot["aruco_id"]
                }.items()
            )
        )

    ld = LaunchDescription()
    
    for spawn_robot_cmd in spawn_robot_cmds:
        ld.add_action(spawn_robot_cmd)
    return ld

