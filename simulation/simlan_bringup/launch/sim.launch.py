import os.path
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import launch
from ament_index_python.packages import get_package_share_directory

from launch.actions import LogInfo
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import OpaqueFunction


def launch_setup(context, *args, **kwargs):

    camera_enabled_ids = LaunchConfiguration("camera_enabled_ids").perform(context)
    pkg_simlan_gazebo_environment = get_package_share_directory(
        "simlan_gazebo_environment"
    )
    pkg_static_agent_launcher = get_package_share_directory("static_agent_launcher")
    pkg_pallet_truck_bringup = get_package_share_directory("pallet_truck_bringup")
    pkg_scenario_manager = get_package_share_directory("scenario_manager")

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("simlan_bringup"), "rviz", "rviz_config.rviz"]
    )

    # Launch args
    use_sim_time = LaunchConfiguration("use_sim_time", default=True)
    launch_rviz = LaunchConfiguration("rviz")
    pallet_truck_manual_control = LaunchConfiguration("pallet_truck_manual_control")

    launch_rviz_launch_argument = DeclareLaunchArgument(
        "rviz", default_value="True", description="To launch rviz"
    )

    simlan_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pkg_simlan_gazebo_environment, "launch", "simlan_factory.launch.py"
            )
        ),
        launch_arguments={"camera_enabled_ids": camera_enabled_ids}.items(),
    )

    static_agents = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_static_agent_launcher, "launch", "static-agent.launch.py")
        )
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz",
        arguments=["-d", rviz_config_file],
        output="screen",
        condition=IfCondition(launch_rviz),
    )

    return [launch_rviz_launch_argument, simlan_gazebo, static_agents, rviz2]


def generate_launch_description():

    return LaunchDescription([OpaqueFunction(function=launch_setup)])
