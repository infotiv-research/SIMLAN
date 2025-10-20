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
    # Launch args
    camera_enabled_ids = LaunchConfiguration("camera_enabled_ids").perform(context)
    log_level = LaunchConfiguration("log_level").perform(context)
    world_setup = LaunchConfiguration("world_setup").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time", default=True)
    launch_rviz = LaunchConfiguration("rviz")

    pkg_simlan_gazebo_environment = get_package_share_directory("simlan_gazebo_environment")
    pkg_static_agent_launcher = get_package_share_directory("static_agent_launcher")

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("simlan_bringup"), "rviz", "rviz_config.rviz"]
    )

    launch_rviz_launch_argument = DeclareLaunchArgument(
        "rviz", default_value="True", description="To launch rviz"
    )

    simlan_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pkg_simlan_gazebo_environment, "launch", "simlan_factory.launch.py"
            )
        ),
        launch_arguments={"camera_enabled_ids": camera_enabled_ids,
                          "use_sim_time":use_sim_time,
                          "world_setup":world_setup,
                          "log_level":log_level
                          }.items(),
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
        arguments=[
            "-d", 
            rviz_config_file,
            "--ros-args", "--log-level", log_level
            ],
        output="screen",
        condition=IfCondition(launch_rviz),
    )

    return [launch_rviz_launch_argument, simlan_gazebo, static_agents, rviz2]


def generate_launch_description():

    return LaunchDescription([OpaqueFunction(function=launch_setup)])
