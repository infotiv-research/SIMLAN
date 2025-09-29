import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def launch_setup(context, *args, **kwargs):
    namespace = LaunchConfiguration("namespace").perform(context)

    # Get the launch directory
    nav2_yaml = os.path.join(
        get_package_share_directory("pallet_truck_navigation"),
        "config",
        f"{namespace}_nav2_params.yaml",
    )
    return [
            Node(  # Manually setting the joint between map and odom to 0 0 0, i.e. identical to each other. map -> odom
                package="tf2_ros",
                executable="static_transform_publisher",
                name="static_map_to_odom",
                namespace=namespace,
                arguments=["0", "0", "0", "0", "0", "0", "map", f"{namespace}/odom"],
            ),
            Node(  # controller
                package="nav2_controller",
                executable="controller_server",
                name="controller_server",
                namespace=namespace,
                output="screen",
                parameters=[nav2_yaml], 
                remappings=[
                    ("cmd_vel", "nav_vel"),
                ]
            ),
            Node(  # Planner
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                output="screen",
                namespace=namespace,

                parameters=[nav2_yaml],
            ),
            Node(  # Behavior
                package="nav2_behaviors",
                executable="behavior_server",
                name="behavior_server",
                namespace=namespace,

                output="screen",
                parameters=[nav2_yaml],
            ),
            Node(  # Navigator
                package="nav2_bt_navigator",
                executable="bt_navigator",
                name="bt_navigator",
                namespace=namespace,

                output="screen",
                parameters=[nav2_yaml],
            ),
            Node(  # Lifecycles, handle the nodes inactive/active states
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                namespace=namespace,
                name="lifecycle_manager_navigation",
                output="screen",
                parameters=[
                    {"autostart": True},
                    {
                        "node_names": [
                            "controller_server",  
                            "planner_server",
                            "behavior_server",
                            "bt_navigator",
                        ]
                    },
                ],
            ),
        ]
    

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="robot_agent_1"),
        OpaqueFunction(function=launch_setup),
    ])
