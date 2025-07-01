import os.path
import launch
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    RegisterEventHandler,
    OpaqueFunction
)
from launch.actions import TimerAction
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

    namespace = LaunchConfiguration("namespace")
    initial_pose_x = LaunchConfiguration("initial_pose_x")
    initial_pose_y = LaunchConfiguration("initial_pose_y")
    aruco_id = LaunchConfiguration("aruco_id")
    robot_type = LaunchConfiguration("robot_type")

    pkg_pallet_truck_bringup = get_package_share_directory("pallet_truck_bringup")
    pkg_pallet_truck_control = get_package_share_directory("pallet_truck_control")
    pkg_pallet_truck_navigation = get_package_share_directory("pallet_truck_navigation")

    twist_mux_params = os.path.join(get_package_share_directory("pallet_truck_bringup"), "params", "twist_mux.yaml"
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
    


    gazebo_spawn = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_pallet_truck_bringup, "launch", "gazebo.launch.py")
            ),
            launch_arguments= {
                "namespace": namespace,
                "initial_pose_x": initial_pose_x,
                "initial_pose_y": initial_pose_y,
                "aruco_id": aruco_id,
                "robot_type": robot_type
            }.items()
        )
 
    control = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_pallet_truck_control, "launch", "control.launch.py")
            ),
            launch_arguments={"namespace": namespace}.items(),
        )
    keyboard_steering =  IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pkg_pallet_truck_bringup, "launch", "keyboard_steering.launch.py"
            )
        ),
        condition=IfCondition(pallet_truck_manual_control),
        launch_arguments={"namespace": namespace}.items()
    )

    twist_mux = Node(   
        package="twist_mux",
        executable="twist_mux",
        output="screen",
        remappings={("cmd_vel_out", "velocity_controller/cmd_vel_unstamped")},
        parameters=[twist_mux_params],
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_pallet_truck_navigation, "launch", "localization.launch.py")),
        launch_arguments={
            "namespace": namespace
        }.items()
    )
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_pallet_truck_navigation, "launch", "nav2.launch.py")),
        launch_arguments={
            "namespace": namespace
        }.items()
    )


    ld = LaunchDescription()
    ld.add_action(gazebo_spawn)
        
    ld.add_action(GroupAction(
        actions=[
            PushRosNamespace(namespace),
            control,
            keyboard_steering,
            twist_mux,
            localization
        ]
    ))
    return ld


def main(argv=None):

    launch_service = launch.LaunchService(debug=False)
    launch_service.include_launch_description(generate_launch_description())
    return launch_service.run()


if __name__ == "__main__":
    main()