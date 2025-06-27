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

    # Get package locations
    pkg_simlan_gazebo_environment = get_package_share_directory("simlan_gazebo_environment")
    pkg_static_agent_launcher = get_package_share_directory("static_agent_launcher")
    pkg_pallet_truck_bringup = get_package_share_directory("pallet_truck_bringup")
    pkg_dyno_jackal_bringup = get_package_share_directory("dyno_jackal_bringup")
    pkg_aruco_localization = get_package_share_directory("aruco_localization")
    pkg_pallet_truck_navigation = get_package_share_directory("pallet_truck_navigation") # contains both localization and nav2
    pkg_scenario_manager = get_package_share_directory("scenario_manager")

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("simlan_bringup"), "rviz", "rviz_config.rviz"]
    )

    # Launch args
    # use_sim_time = LaunchConfiguration("use_sim_time", default=True)

    launch_rviz = LaunchConfiguration("rviz")
    pallet_truck_manual_control = LaunchConfiguration("pallet_truck_manual_control")

    launch_rviz_launch_argument = DeclareLaunchArgument(
        "rviz", default_value="True", description="To launch rviz"
    )

    pallet_truck_manual_control_launch_argument = DeclareLaunchArgument(
        "pallet_truck_manual_control",
        default_value="False",
        description="To launch pallet_truck keyboard steering dashboard",
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

    pallet_truck = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_pallet_truck_bringup, "launch", "sim.launch.py")
        ),
        launch_arguments={
            "pallet_truck_manual_control": pallet_truck_manual_control,
        }.items(),
    )

    jackal = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dyno_jackal_bringup, "launch", "sim.launch.py")
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

    aruco_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_aruco_localization, "launch", "multi_detection.launch.py")
        ),
        launch_arguments={
            "use_sim_time": "true",
            "publish_to_odom": "true"
        }.items()
    )

    pallet_truck_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_pallet_truck_navigation, "launch", "localization.launch.py")
        )
    )

    pallet_truck_nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_pallet_truck_navigation, "launch", "nav2.launch.py")
        )
    )

    ld = LaunchDescription()

    ld.add_action(launch_rviz_launch_argument)
    ld.add_action(pallet_truck_manual_control_launch_argument)

    ld.add_action(simlan_gazebo)

    ld.add_action(static_agents)
    ld.add_action(pallet_truck)
    ld.add_action(jackal)

    ld.add_action(aruco_localization)
    ld.add_action(pallet_truck_localization)
    ld.add_action(pallet_truck_nav2)

    ld.add_action(rviz2)

    return ld


def main(argv=None):
    launch_service = launch.LaunchService(debug=False)
    launch_service.include_launch_description(generate_launch_description())
    return launch_service.run()


if __name__ == "__main__":
    main()
