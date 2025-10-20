import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace, Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    robots =  [
        {"namespace": "humanoid", "cam_ns": "camera0"}, 
    ]
    log_level = LaunchConfiguration("log_level", default="INFO")


    launch_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('humanoid_support_moveit_config'), 'launch'),
            '/launch_moveit.launch.py'])
        )

    launch_gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('humanoid_support_moveit_config'), 'launch'),
            '/launch_gz.launch.py']),
        launch_arguments={"log_level": log_level}.items(),

        )
    
    launch_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('humanoid_support_moveit_config'), 'launch'),
            '/launch_controllers.launch.py']),
        launch_arguments={"log_level": log_level}.items(),

        )
    # launch_camera = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([os.path.join(
    #         get_package_share_directory('camera_system'), 'launch'),
    #         '/spawn_camera.launch.py'])
    #     )
    groups = []
    for r in robots:
        ns = r["namespace"]
        cam_ns = r["cam_ns"]
        groups.append(
            GroupAction([
                PushRosNamespace(namespace=ns),
                launch_rviz,
                launch_gz,
                launch_control,
                # IncludeLaunchDescription(
                #     PythonLaunchDescriptionSource([os.path.join(
                #         get_package_share_directory('camera_system'), 'launch'),
                #         '/spawn_camera.launch.py']),
                #     launch_arguments={'cam_ns': 'camera0'}.items(),
                # ),
            ])
        )
    return LaunchDescription(groups)
    # return LaunchDescription([
    #     launch_rviz,
    #     launch_gz,
    #     launch_control,
    #     launch_camera
    # ])