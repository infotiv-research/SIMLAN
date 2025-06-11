from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, OpaqueFunction
import ast
import yaml
import os

# Get camera IDs as a list of integers
camera_ids = [int(id) for id in os.environ["CAMERA_ENABLED_IDS"].split()]


def launch_setup(context, *args, **kwargs):
    use_sim_time_value = LaunchConfiguration("use_sim_time")
    nodes = []
    for camera_id in camera_ids:
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
    return nodes


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    publish_to_odom = LaunchConfiguration("publish_to_odom")
    aruco_pose_node = Node(
        package="aruco_localization",
        executable="aruco_pose_pub",
        name="aruco_pose_pub_node",
        output="screen",
        parameters=[
            {"camera_ids": camera_ids},
            {"use_sim_time": use_sim_time},
            {"publish_to_odom": publish_to_odom},
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "camera_ids",
                default_value="[163]",
                description="An array of camera IDs that will be used for detection (e.g. [101, 202])",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation (Gazebo) clock if true",
            ),
            DeclareLaunchArgument(
                "publish_to_odom",
                default_value="true",
                description="Whether to publish pose to ODOM or TF. Default is odom",
            ),
            aruco_pose_node,
            OpaqueFunction(function=launch_setup),
        ]
    )
