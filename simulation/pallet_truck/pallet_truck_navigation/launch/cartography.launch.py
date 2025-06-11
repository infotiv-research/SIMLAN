import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():

    cartographer_config_dir = os.path.join(
        get_package_share_directory("pallet_truck_navigation"), "config"
    )
    configuration_basename = "cartographer_params.lua"

    return LaunchDescription(
        [
            Node(
                package="cartographer_ros",
                executable="cartographer_node",
                name="cartographer_node",
                output="screen",
                remappings=[
                    ("imu", "pallet_truck/imu/data"),
                    ("scan", "pallet_truck/scan"),
                ],
                parameters=[{"use_sim_time": True}],
                arguments=[
                    "-configuration_directory",
                    cartographer_config_dir,
                    "-configuration_basename",
                    configuration_basename,
                ],
            ),
            Node(
                package="cartographer_ros",
                executable="cartographer_occupancy_grid_node",
                output="screen",
                remappings=[("imu", "/imu/data")],
                name="occupancy_grid_node",
                parameters=[{"use_sim_time": True}],
                arguments=["-resolution", "0.05", "-publish_period_sec", "1.0"],
            ),
        ]
    )
