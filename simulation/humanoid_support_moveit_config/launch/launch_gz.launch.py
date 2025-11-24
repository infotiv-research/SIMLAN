import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, TextSubstitution, PathJoinSubstitution

def generate_launch_description():
    log_level = LaunchConfiguration("log_level", default="INFO")
    namespace = LaunchConfiguration("namespace")
    initial_pose_x = LaunchConfiguration("initial_pose_x")
    initial_pose_y = LaunchConfiguration("initial_pose_y")

    ld = LaunchDescription()

    # Spawn robot from /robot_description topic
    gz_spawn = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-name", namespace,
            "-topic",
            PathJoinSubstitution([TextSubstitution(text="/"), namespace, TextSubstitution(text="robot_description")]),
            "-x", initial_pose_x,
            "-y", initial_pose_y,
            "-z", "0.0",
            "--ros-args", "--log-level", log_level,
        ],
        parameters=[{"use_sim_time": True}],
    )

    ld.add_action(gz_spawn)

    return ld
