   
import os
import random

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch.actions import ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # pkg_humanoid_robot = get_package_share_directory('humanoid_robot')
    # world_file_name = 'empty.sdf'
    # world = os.path.join(pkg_humanoid_robot, 'worlds', world_file_name)
    log_level = LaunchConfiguration("log_level", default="INFO")

    ld = LaunchDescription()

    

    # Spawn robot from /robot_description topic

    gz_spawn = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[ 
        #    "-name", "humanoid",
            "-topic",
            "/humanoid/robot_description",
            "-x", "5.5",
            "-y", "-10.0",
            '-z', '0.0',
            "--ros-args", "--log-level", log_level,
        ],
        parameters=[{"use_sim_time": True}],
    )
    
    # Start ros_gz_bridge (clock -> ROS 2)
    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="log",
        arguments=[
            # "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "--ros-args",
            "--log-level",
            log_level,
        ],
        parameters=[{"use_sim_time": True}],
    )

    ld.add_action(gz_spawn)
    ld.add_action(gz_bridge)

    return ld