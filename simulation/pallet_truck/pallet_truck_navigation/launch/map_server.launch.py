from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.substitutions import LaunchConfiguration, PythonExpression

def generate_launch_description():

    map_file = os.path.join(
        get_package_share_directory("pallet_truck_navigation"),
        "maps",
        "warehouse.yaml",
    )

    return LaunchDescription(
        [
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
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_map_server",
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
