import os
from launch import LaunchDescription
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import launch

def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    initial_pose_x = LaunchConfiguration("initial_pose_x")
    initial_pose_y = LaunchConfiguration("initial_pose_y")
    aruco_id = LaunchConfiguration("aruco_id")
    robot_type = LaunchConfiguration("robot_type")
    log_level = LaunchConfiguration("log_level")

    pkg_pallet_truck_description = get_package_share_directory(
        "pallet_truck_description"
    )
    # Set the path to the SDF model files.
    gazebo_models_path = os.path.join(pkg_pallet_truck_description, "meshes")
    os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path
    os.environ["GZ_SIM_RESOURCE_PATH"] = gazebo_models_path
    os.environ["IGN_GAZEBO_RESOURCE_PATH"] = gazebo_models_path

    
    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("pallet_truck_description"),
                    "urdf",
                    "pallet_truck_simple_collisions.urdf.xacro",
                ]
            ),
            " ",
            "name:=", namespace,
            " ",
            "prefix:=", namespace,
            " ",
            "is_sim:=true",
            " ",
            "namespace:=", namespace,
            " ",
            "aruco_id:=", aruco_id,
            " ",
            "robot_type:=", robot_type
        ]
    )

    robot_description = {"robot_description": ParameterValue(robot_description_content)}

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        namespace=namespace,
        parameters=[
            {"use_sim_time": True},
            robot_description
        ]
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_pallet_truck",
        arguments=[
            "-name", namespace,
            "-x", initial_pose_x,
            "-y", initial_pose_y,
            "-z", "0.2",
            "-topic",  PathJoinSubstitution([namespace, "robot_description"]),
            "-robot_namespace", namespace,
            "--ros-args", "--log-level", log_level,

        ],
        output="screen",
        )
    
    ld = LaunchDescription()
    ld.add_action(robot_state_publisher)
    ld.add_action(spawn_robot)
   
    return ld


def main(argv=None):
    launch_service = launch.LaunchService(debug=False)
    launch_service.include_launch_description(generate_launch_description())
    return launch_service.run()


if __name__ == "__main__":
    main()
