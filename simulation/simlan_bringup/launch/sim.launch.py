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


def generate_launch_description():

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
        )
    )

    static_agents = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_static_agent_launcher, "launch", "static-agent.launch.py")
        )
    )

    # spawn_multiple_robots = IncludeLaunchDescription(
    #    PythonLaunchDescriptionSource(
    #        os.path.join(
    #            pkg_pallet_truck_bringup, "launch", "multiple_robot_spawn.launch.py"
    #        )
    #    )
    # )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz",
        arguments=["-d", rviz_config_file],
        output="screen",
        condition=IfCondition(launch_rviz),
    )

    ld = LaunchDescription()

    ld.add_action(launch_rviz_launch_argument)
    ld.add_action(simlan_gazebo)
    ld.add_action(static_agents)
    # ld.add_action(spawn_multiple_robots)
    ld.add_action(rviz2)

    return ld


def main(argv=None):
    launch_service = launch.LaunchService(debug=False)
    launch_service.include_launch_description(generate_launch_description())
    return launch_service.run()


if __name__ == "__main__":
    main()
