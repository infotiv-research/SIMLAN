from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.substitutions import LaunchConfiguration, PythonExpression, TextSubstitution

def generate_launch_description():

    namespace = LaunchConfiguration("namespace") # robot_agent_X

    # Get the launch directory
    localization_yaml = os.path.join(
        get_package_share_directory("pallet_truck_navigation"),
        "config",
        "localization_params.yaml",
    )
 

    return LaunchDescription(
        [
            Node(  # Manually setting the joint between map and odom to 0 0 0, i.e. identical to each other. map -> odom
                package="tf2_ros",
                executable="static_transform_publisher",
                namespace=namespace,
                name ="static_map_to_odom",
                arguments=["0", "0", "0", "0", "0", "0", "map", [LaunchConfiguration("namespace"), "_odom"]],
            )
            
        ]
    )
