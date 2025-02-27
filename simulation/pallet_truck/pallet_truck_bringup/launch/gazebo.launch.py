import os.path
import os

from launch import LaunchDescription
from launch.actions import RegisterEventHandler, GroupAction
from launch.event_handlers import OnProcessExit
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
import launch


def generate_launch_description():

    pkg_pallet_truck_description = get_package_share_directory(
        "pallet_truck_description"
    )

    # Launch args
    use_sim_time = LaunchConfiguration("use_sim_time", default=True)

    # Set the path to the SDF model files.
    gazebo_models_path = os.path.join(pkg_pallet_truck_description, "meshes")
    os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("pallet_truck_description"),
                    "urdf",
                    "pallet_truck.urdf.xacro",
                ]
            ),
            " ",
            "name:=pallet_truck",
            " ",
            "prefix:=pallet_truck",
            " ",
            "is_sim:=true",
        ]
    )

    robot_description = {"robot_description": ParameterValue(robot_description_content)}

    # spawn_velocity_controller = Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     arguments=['velocity_controller', '-c', 'controller_manager'],
    #     output='screen',
    # )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": True}, robot_description],
    )

    # spawn_joint_state_broadcaster = Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     arguments=['joint_state_broadcaster', '-c', 'controller_manager'],
    #     output='screen',
    # )

    # Make sure spawn_velocity_controller starts after spawn_joint_state_broadcaster
    # diffdrive_controller_spawn_callback = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=spawn_joint_state_broadcaster,
    #         on_exit=[spawn_velocity_controller],
    #     )
    # )

    # Spawn robot
    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_pallet_truck",
        arguments=[
            "-entity",
            "pallet_truck",
            "-x",
            "10.0",
            "-y",
            "1.0",
            "-z",
            "0.2",
            "-topic",
            "robot_description",
            "-robot_namespace",
            "pallet_truck",
        ],
        output="screen",
    )

    ld = LaunchDescription()
    # ld.add_action(world_launch_configuration)
    # ld.add_action(spawn_joint_state_broadcaster)
    # ld.add_action(diffdrive_controller_spawn_callback)
    ld.add_action(robot_state_publisher)
    ld.add_action(spawn_robot)

    return ld


def main(argv=None):
    launch_service = launch.LaunchService(debug=False)
    launch_service.include_launch_description(generate_launch_description())
    return launch_service.run()


if __name__ == "__main__":
    main()
