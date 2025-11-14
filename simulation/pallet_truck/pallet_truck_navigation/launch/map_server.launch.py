import os
import ast
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction


def launch_setup(context, *args, **kwargs):
    robots_str = LaunchConfiguration("robots").perform(context)
    log_level = LaunchConfiguration("log_level").perform(context)
    robots = ast.literal_eval(robots_str)

    # Get the launch directory
    map_file = os.path.join(
        get_package_share_directory("pallet_truck_navigation"),
        "maps",
        "warehouse.yaml",
    )

    actions=[]
    for robot in robots:
        actions.append(
            Node(  # Hosting the map
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                output="screen",
                namespace=robot["namespace"],
                parameters=[{"use_sim_time": True}, {"yaml_filename": map_file}, {'frame_id': f'{robot["namespace"]}/map'}],
                arguments=[
                    "--ros-args",
                    "--log-level",
                    log_level
                ],
            ),
        )
        actions.append(
            Node(  # Manually setting the joint between map and odom to 0 0 0, i.e. identical to each other. map -> odom
                package="tf2_ros",
                executable="static_transform_publisher",
                namespace=robot["namespace"],
                name="static_world_to_map",
                arguments=["0", "0", "0", "0", "0", "0", "world", f"{robot["namespace"]}/map"],
            ),
        )
        actions.append(
            Node(  # Manually setting the joint between map and odom to 0 0 0, i.e. identical to each other. map -> odom
                package="tf2_ros",
                executable="static_transform_publisher",
                namespace=robot["namespace"],
                name="static_map_to_odom",
                arguments=[f"{robot["initial_pose_x"]}", f"{robot["initial_pose_y"]}", "0", "0", "0", "0", f"{robot["namespace"]}/map", f"{robot["namespace"]}/odom"],
            ),
        )
        actions.append(
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_map_server",
                namespace=robot["namespace"],
                output="screen",
                arguments=[
                    "--ros-args",
                    "--log-level",
                    log_level
                ],
                parameters=[
                    {"autostart": True},
                    {
                        "node_names": [
                            "map_server",
                        ]
                    },
                ],
            ),
        )
        
    return actions

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robots", default_value=[]),
        DeclareLaunchArgument("log_level", default_value="INFO"),

        OpaqueFunction(function=launch_setup),
    ])
