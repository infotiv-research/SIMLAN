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

from dyno_utils.launch_utils import DynoWaitFor
import rclpy.qos

import std_msgs.msg
import sensor_msgs.msg
import rosgraph_msgs.msg


def generate_launch_description():

    pkg_pallet_truck_bringup = get_package_share_directory("pallet_truck_bringup")
    pkg_pallet_truck_control = get_package_share_directory("pallet_truck_control")

    twist_mux_params = os.path.join(
        get_package_share_directory("pallet_truck_bringup"), "params", "twist_mux.yaml"
    )

    # Launch args
    use_sim_time = LaunchConfiguration("use_sim_time", default=True)
    pallet_truck_manual_control = LaunchConfiguration(
        "pallet_truck_manual_control", default=False
    )

    pallet_truck_manual_control_launch_argument = DeclareLaunchArgument(
        "pallet_truck_manual_control",
        default_value="False",
        description="To launch pallet_truck keyboard steering or not",
    )

    pallet_truck_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_pallet_truck_bringup, "launch", "gazebo.launch.py")
        )
    )

    control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_pallet_truck_control, "launch", "control.launch.py")
        )
    )

    keyboard_steering = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pkg_pallet_truck_bringup, "launch", "keyboard_steering.launch.py"
            )
        ),
        condition=IfCondition(pallet_truck_manual_control),
    )

    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        output="screen",
        remappings={("cmd_vel_out", "velocity_controller/cmd_vel_unstamped")},
        parameters=[twist_mux_params],
    )

    pallet_truck_with_namespace_and_remapping = GroupAction(
        actions=[
            PushRosNamespace("pallet_truck"),
            # SetRemap("tf", "/tf"),
            # SetRemap("tf_static", "/tf_static"),
            pallet_truck_gazebo,
            control,
            twist_mux,
            keyboard_steering,
        ]
    )

    # TODO: pallet_truck namespace is for some reason given to other robots spawned
    # *after* the pallet_truck. Until this is sorted out, we will launch the pallet_truck last
    ld = LaunchDescription()
    ld.add_action(
        DynoWaitFor(
            name="pallet_truck_launched_last",
            message_on_topics=[
                (
                    "/clock",
                    rosgraph_msgs.msg.Clock,
                    rclpy.qos.qos_profile_sensor_data,
                ),  # Wait for Gazebo to launch
            ],
            actions=[
                pallet_truck_with_namespace_and_remapping,
            ],
        )
    )

    return ld


def main(argv=None):
    launch_service = launch.LaunchService(debug=False)
    launch_service.include_launch_description(generate_launch_description())
    return launch_service.run()


if __name__ == "__main__":
    main()
