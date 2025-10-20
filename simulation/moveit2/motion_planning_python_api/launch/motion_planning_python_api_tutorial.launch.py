"""
A launch file for running the motion planning python api tutorial
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder
from launch.actions import TimerAction

def generate_launch_description():
    
    # Logging level 
    log_level = LaunchConfiguration('log_level')


    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="panda", package_name="moveit_resources_panda_moveit_config"
        )
        .robot_description(file_path="config/panda.urdf.xacro")
        .trajectory_execution(file_path="config/gripper_moveit_controllers.yaml")
        .moveit_cpp(
            file_path=get_package_share_directory("motion_planning_python_api")
            + "/config/motion_planning_python_api_tutorial.yaml"
        )
        .to_moveit_configs()
    )

    example_file = DeclareLaunchArgument(
        "example_file",
        default_value="motion_planning_python_api_tutorial",
        description="Python API tutorial file name",
    )
   
    moveit_py_node = Node(
        name="moveit_py",
        package="motion_planning_python_api",
        executable=LaunchConfiguration("example_file"),
        output="both",
        parameters=[moveit_config.to_dict(),{"use_sim_time": True}],
    )
    package_path = get_package_share_directory("motion_planning_python_api")
    small_cube_location = os.path.join(package_path, "models/small_cube.sdf")

    spawn_cube = Node(
    package="ros_gz_sim",
    executable="create",
    arguments=[
        "-name", "small_cube",
        "-file", small_cube_location,
        "-x", "0.5", "-y", "0", "-z", "0.1",
        "--ros-args", "--log-level", log_level,

    ],
    output="screen",
)

    return LaunchDescription(
        [   
            example_file,
            moveit_py_node,
            spawn_cube
        ]
    )
