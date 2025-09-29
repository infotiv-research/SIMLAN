#!/usr/bin/env python3

import os
import time
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from dyno_utils.launch_utils import DynoWaitFor
import std_msgs.msg
import nav_msgs.msg
import rosgraph_msgs.msg
import rclpy.qos


def kill_gazebo():
    # Kill lingering Gazebo processes from previous runs
    for name in ["gzserver", "gzclient"]:
        while pid := os.popen(f"pidof {name}").read().strip():
            print(f"shutting down {name}...")
            os.system(f"kill -9 {pid}")
            time.sleep(1)
        print(f"{name} is shutdown!")


def kill_daemon():
    pid = (
        os.popen("ps aux | grep -i ros2-daemon | grep -v grep | awk '{print $2}'")
        .read()
        .strip()
    )
    if pid:
        print(f"killing daemon with pid: {pid}")
        os.system(f"kill -9 {pid}")
        time.sleep(1)


def kill_ros_processes():
    pids = (
        os.popen("ps aux | grep -i ros-args | grep -v grep | awk '{print $2}'")
        .read()
        .strip()
    )
    pids = [pid.strip() for pid in pids.split("\n") if pid.strip() != ""]
    for pid in pids:
        print(f"shutting down ros process with pid: {pid}")
        os.system(f"kill -9 {pid}")
        time.sleep(1)
    if pids:
        time.sleep(3)

def generate_launch_description():
    kill_gazebo()
    kill_daemon()
    kill_ros_processes()

    # Declare launch arguments
    camera_enabled_ids_arg = DeclareLaunchArgument(
        'camera_enabled_ids',
        default_value='163 164 165 166',
        description='Camera IDs to enable in simulation'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    # Get launch configurations
    camera_enabled_ids = LaunchConfiguration('camera_enabled_ids')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Include simulation launch file
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('simlan_bringup'),
                'launch',
                'sim.launch.py'
            ])
        ]),
        launch_arguments={
            'camera_enabled_ids': camera_enabled_ids,
        }.items()
    )
    
    # Include ArUco detection launch file
    aruco_detection_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('aruco_localization'),
                'launch',
                'multi_detection.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'camera_enabled_ids': camera_enabled_ids,
        }.items()
    )
    
    # Include multiple robot spawn launch file
    robot_spawn_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('pallet_truck_bringup'),
                'launch',
                'multiple_robot_spawn.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items()
    )

    return LaunchDescription([
        camera_enabled_ids_arg,
        use_sim_time_arg,
        sim_launch,
        TimerAction(
            period=2.0,
            actions=[aruco_detection_launch]
        ),
        TimerAction(
                    period=5.0,
                    actions=[robot_spawn_launch]
        ),
        # TimerAction(
        #             period=10.0,
        #             actions=[multi_nav_launch]
        # ),
        # DynoWaitFor(
        #     name="robot_spawn",
        #     message_on_topics=[
        #         ("/clock", rosgraph_msgs.msg.Clock, rclpy.qos.qos_profile_sensor_data), # Wait for Gazebo to launch
        #         # ("/jackal/joint_states", std_msgs.msg.String, rclpy.qos.qos_profile_system_default), 
        #         # ("/static_agents/robot_description", std_msgs.msg.String, rclpy.qos.qos_profile_system_default), # Wait for static agents to launch
        #         # ("/pallet_truck/velocity_controller/odom", nav_msgs.msg.Odometry, rclpy.qos.qos_profile_system_default), # Wait for pallet truck to launch
        #         # ("/scan", sensor_msgs.msg.LaserScan, rclpy.qos.qos_profile_sensor_data), # Wait for infobot to launch
        #     ],
        #     actions=[
                
        #     ],
        # )
    ])