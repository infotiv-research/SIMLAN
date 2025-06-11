from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # Get the launch directory
    localization_yaml = os.path.join(
        get_package_share_directory("pallet_truck_navigation"),
        "config",
        "localization_params.yaml",
    )
    map_file = os.path.join(
        get_package_share_directory("pallet_truck_navigation"),
        "maps",
        "warehouse.yaml",
    )

    return LaunchDescription(
        [
            Node(  # Localization TF link for odom -> pallet_truck_base_link
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node",
                output="screen",
                parameters=[localization_yaml],
            ),
            Node(  # Hosting the map
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                output="screen",
                parameters=[{"use_sim_time": True}, {"yaml_filename": map_file}],
            ),
            Node(  # Manually setting the joint between map and odom to 0 0 0, i.e. identical to each other. map -> odom
                package="tf2_ros",
                executable="static_transform_publisher",
                name="static_world_to_map",
                arguments=["0", "0", "0", "0", "0", "0", "world", "map"],
            ),
            Node(  # Manually setting the joint between map and odom to 0 0 0, i.e. identical to each other. map -> odom
                package="tf2_ros",
                executable="static_transform_publisher",
                name="static_map_to_odom",
                arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_localization",
                output="screen",
                parameters=[
                    {"autostart": True},
                    {
                        "node_names": [
                            "map_server",
                        ]
                    },
                ],
            ),
        ]
    )
