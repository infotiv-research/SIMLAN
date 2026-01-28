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
    log_level = LaunchConfiguration("log_level").perform(context)
    world_setup = LaunchConfiguration("world_setup").perform(context)
    headless_gazebo = LaunchConfiguration("headless_gazebo").perform(context)
    spawn_jackal = LaunchConfiguration("spawn_jackal", default=False)
    use_sim_time = LaunchConfiguration("use_sim_time", default=True)
    rviz_config = LaunchConfiguration("rviz_config", default="rviz_config.rviz")
    launch_rviz = LaunchConfiguration("rviz")

    pkg_simlan_gazebo_environment = get_package_share_directory(
        "simlan_gazebo_environment"
    )
    pkg_static_agent_launcher = get_package_share_directory("static_agent_launcher")
    pkg_dyno_jackal_bringup = get_package_share_directory("dyno_jackal_bringup")

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("simlan_bringup"), "rviz", rviz_config]
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
        launch_arguments={
            "use_sim_time": use_sim_time,
            "world_setup": world_setup,
            "headless_gazebo": headless_gazebo,
            "log_level": log_level,
        }.items(),
    )

    jackal = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dyno_jackal_bringup, "launch", "sim.launch.py")
        ),
        condition=IfCondition(spawn_jackal),
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz",
        arguments=["-d", rviz_config_file, "--ros-args", "--log-level", log_level],
        output="screen",
        condition=IfCondition(launch_rviz),
    )

    return [launch_rviz_launch_argument, simlan_gazebo, jackal, rviz2]


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch_setup)])
