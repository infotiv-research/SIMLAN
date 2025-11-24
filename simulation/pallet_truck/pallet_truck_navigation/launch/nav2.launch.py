import os
import ast
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def launch_setup(context, *args, **kwargs):
    robots_str = LaunchConfiguration("robots").perform(context)
    log_level = LaunchConfiguration("log_level").perform(context)

    robots = ast.literal_eval(robots_str)
    all_namespaces=[]
    for robot in robots:
        all_namespaces.append(robot["namespace"])
    # Get the launch directory

    actions =[]
    for robot in robots:
        if "humanoid" in robot["namespace"]:
            nav2_yaml = os.path.join(
            get_package_share_directory("pallet_truck_navigation"),
            "config",
            "nav2_params_humanoid.yaml",
        )
        else:
            nav2_yaml = os.path.join(
                get_package_share_directory("pallet_truck_navigation"),
                "config",
                "nav2_params.yaml",
            )
        actions.append(
            Node(
                package='nav2_controller', # yes
                executable='controller_server',
                output='screen',
                parameters=[nav2_yaml],
                remappings= [('cmd_vel', 'nav_vel')],
                namespace=robot["namespace"],
                arguments=['--ros-args', '--log-level', log_level],

            ),
        )
        actions.append(
            Node(
                package='nav2_planner', # yes
                executable='planner_server',
                name='planner_server',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[nav2_yaml],
                namespace=robot["namespace"]
            ),
        )
        actions.append(
            Node(
                package='nav2_behaviors', # yes
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                parameters=[nav2_yaml],
                namespace=robot["namespace"],
                arguments=['--ros-args', '--log-level', log_level],
            ),
        )
        actions.append(
            Node(
                package='nav2_bt_navigator',  # yes
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[nav2_yaml],
                namespace=robot["namespace"],
                
            ),
        )
        actions.append(   
            Node(# Lifecycles, handle the nodes inactive/active states
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                output="screen",
                namespace=robot["namespace"],
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[
                    {"autostart": True},
                    {"node_names": 
                        [
                            "planner_server",
                            "behavior_server",
                            "controller_server",  
                            "bt_navigator",
                        ]
                    },
                ],
            ),
        )
        actions.append(
            Node(
                package="pallet_truck_navigation",
                executable="update_map_node",
                name="update_map",
                output="screen",
                namespace=robot["namespace"],
                parameters=[{"namespace": robot["namespace"]},
                            {"all_namespaces":all_namespaces}],
                arguments=['--ros-args', '--log-level', log_level],

            )
        )
    return actions
    

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robots", default_value=[]),
        DeclareLaunchArgument("log_level", default_value="log_level"),
        OpaqueFunction(function=launch_setup),
    ])
