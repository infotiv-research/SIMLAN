import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Get the launch directory
    nav2_yaml = os.path.join(
        get_package_share_directory("pallet_truck_navigation"),
        "config",
        "nav2_params.yaml",
    )
    namespace = LaunchConfiguration("namespace")
    return LaunchDescription(
        [
            Node(  # controller
                package="nav2_controller",
                executable="controller_server",
                name="controller_server",
                namespace=namespace,                
                output="screen",
                parameters=[nav2_yaml], 
                remappings=[
                    ("cmd_vel", "nav_vel"),
                    ("map", "/map")],
            ),
            Node(  # Planner
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                namespace=namespace,
                remappings=[("map", "/map")],  # Makes it subscribe to global /map
                output="screen",
                parameters=[nav2_yaml],
            ),
            Node(  # Behavior
                package="nav2_behaviors",
                executable="behavior_server",
                name="behavior_server",
                namespace=namespace,
                remappings=[("map", "/map")],  # Makes it subscribe to global /map
                output="screen",
                parameters=[nav2_yaml],
            ),
            Node(  # Navigator
                package="nav2_bt_navigator",
                executable="bt_navigator",
                name="bt_navigator",
                remappings=[("map", "/map")],  # Makes it subscribe to global /map
                namespace=namespace,
                output="screen",
                parameters=[nav2_yaml],
            ),
            Node(  # Lifecycles, handle the nodes inactive/active states
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                namespace=namespace,
                output="screen",
                remappings=[("map", "/map")],  # Makes it subscribe to global /map
                parameters=[
                    {"autostart": True},
                    {
                        "node_names": [
                            "controller_server",  # Use fully-qualified name here
                            "planner_server",
                            "behavior_server",
                            "bt_navigator",
                        ]
                    },
                ],
            ),
        ]
    )
