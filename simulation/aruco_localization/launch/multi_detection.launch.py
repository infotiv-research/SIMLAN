from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, OpaqueFunction
import ast
import yaml
import os


def launch_setup(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration("use_sim_time")
    camera_enabled_ids = LaunchConfiguration("camera_enabled_ids").perform(context)
    camera_enabled_ids = [
        int(id) for id in camera_enabled_ids.split()
    ]  # this turns the input "163 164 165" into a list of string ['163', '164', '165'] and finally into list of int [163, 164, 165]
    use_sim_time_value = LaunchConfiguration("use_sim_time")
    nodes = []

    for camera_id in camera_enabled_ids:
        nodes.append(
            Node(
                package="aruco_localization",
                executable="aruco_detection_node",
                name="aruco_detection_node_{}".format(camera_id),
                output="screen",
                parameters=[
                    {"camera_id": camera_id},
                    {"use_sim_time": use_sim_time_value},
                ],
            )
        )
    aruco_pose_node = Node(
        package="aruco_localization",
        executable="aruco_pose_pub",
        name="aruco_pose_pub_node",
        output="screen",
        parameters=[
            {"camera_enabled_ids": camera_enabled_ids},
            {"use_sim_time": use_sim_time},
        ],
    )
    nodes.append(aruco_pose_node)
    return nodes


def generate_launch_description():

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "camera_enabled_ids",
                default_value="[163]",
                description="An array of camera IDs that will be used for detection (e.g. [101, 202])",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation (Gazebo) clock if true",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
