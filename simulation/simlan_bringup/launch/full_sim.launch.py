import os.path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node, PushRosNamespace, SetRemap
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_simlan_gazebo_environment = get_package_share_directory(
        "simlan_gazebo_environment"
    )
    pkg_static_agent_launcher = get_package_share_directory("static_agent_launcher")
    pkg_infobot_agent = get_package_share_directory("infobot_agent")
    pkg_dyno_jackal_bringup = get_package_share_directory("dyno_jackal_bringup")
    pkg_scenario_manager = get_package_share_directory("scenario_manager")

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("simlan_bringup"), "rviz", "wip.rviz"]
    )

    # Launch args
    use_sim_time = LaunchConfiguration("use_sim_time", default=True)
    launch_rviz = LaunchConfiguration("rviz")
    jackal_manual_control = LaunchConfiguration("jackal_manual_control")

    launch_rviz_launch_argument = DeclareLaunchArgument(
        "rviz", default_value="False", description="To launch rviz"
    )

    jackal_manual_control_launch_argument = DeclareLaunchArgument(
        "jackal_manual_control",
        default_value="False",
        description="To launch jackal keyboard steering dashboard",
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

    infobot_agent = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_infobot_agent, "launch", "infobot.launch.py")
        )
    )

    jackal = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dyno_jackal_bringup, "launch", "sim.launch.py")
        ),
        launch_arguments={
            "jackal_manual_control": jackal_manual_control,
        }.items(),
    )

    scenario_manager = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_scenario_manager, "launch", "scenario_manager.launch.py")
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

    ld = LaunchDescription()

    ld.add_action(launch_rviz_launch_argument)
    ld.add_action(jackal_manual_control_launch_argument)

    ld.add_action(simlan_gazebo)
    ld.add_action(static_agents)
    ld.add_action(infobot_agent)
    ld.add_action(jackal)

    ld.add_action(scenario_manager)
    ld.add_action(rviz2)

    return ld


def main(argv=None):
    launch_service = launch.LaunchService(debug=False)
    launch_service.include_launch_description(generate_launch_description())
    return launch_service.run()


if __name__ == "__main__":
    main()
