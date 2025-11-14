import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    log_level = LaunchConfiguration("log_level")
    initial_pose_x = LaunchConfiguration("initial_pose_x")
    initial_pose_y = LaunchConfiguration("initial_pose_y")


    launch_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('humanoid_support_moveit_config'), 'launch'),
            '/launch_moveit.launch.py']),
        launch_arguments={"log_level": log_level,
                          "namespace":namespace}.items(),
    )

    launch_gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('humanoid_support_moveit_config'), 'launch'),
            '/launch_gz.launch.py']),
        launch_arguments={"log_level": log_level,
                          "namespace":namespace,
                           "initial_pose_x":initial_pose_x,
                           "initial_pose_y":initial_pose_y}.items(),

    )
    
    launch_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('humanoid_support_moveit_config'), 'launch'),
            '/launch_controllers.launch.py']),
        launch_arguments={"log_level": log_level,
                          "namespace":namespace,
                          "initial_pose_x":initial_pose_x,
                          "initial_pose_y":initial_pose_y
                          }.items(),

    )

    groups = []
    groups.append(
        GroupAction([
            PushRosNamespace(namespace=namespace),
            launch_rviz,
            launch_gz,
            launch_control,
        ])
    )
    return LaunchDescription(groups)
 