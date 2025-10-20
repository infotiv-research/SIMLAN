import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


from launch_ros.actions import Node
import xacro

# Author: Hamid Ebadi


def generate_launch_description():
    pkg_name = "static_agent_launcher"
    file_subpath = "description/agents.urdf.xacro"

    xacro_file = os.path.join(get_package_share_directory(pkg_name), file_subpath)
    robot_description_raw = xacro.process_file(xacro_file).toxml()
    # Configure the node
    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="static_agents",
        output="screen",
        parameters=[{"robot_description": robot_description_raw, "use_sim_time": True}],
    )

    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        namespace="static_agents",
        arguments=[
            "-topic",
            "robot_description",
            "-robot_namespace",
            "static_agents",
            "-name",
            "cameras"
        ],
        output="screen",
    )
    # /static_agents/camera_164/image_raw

    return LaunchDescription([node_robot_state_publisher, spawn_entity])
