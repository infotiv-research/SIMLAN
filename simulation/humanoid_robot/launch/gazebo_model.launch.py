import os

from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_share_path, get_package_prefix
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import SetEnvironmentVariable

def generate_launch_description():

    # Define share paths for files, directories etc.
    launch_file_dir = os.path.join(get_package_share_directory('humanoid_robot'), 'launch')
    pkg_humanoid_robot = get_package_share_directory('humanoid_robot')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Set whether to use sim time or not
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Logging level 
    log_level = LaunchConfiguration('log_level')

    # Get the world file
    world_file_name = 'empty.sdf'
    world = os.path.join(pkg_humanoid_robot, 'worlds', world_file_name)


    # Get the URDF file
    # urdf_file_name = 'humanSubjectWithMesh_simplified.urdf'
    urdf_file_name = 'human_support.xacro'
    urdf = os.path.join(pkg_humanoid_robot, 'model', 'support', urdf_file_name)

    # with open(urdf, 'r') as infp:
    #     robot_desc = infp.read()
    robot_desc = ParameterValue(Command(['xacro ', urdf]), value_type=str)


    # Robot state publisher node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            }],
        arguments=[urdf],
        )

    # Launch arguments for Gazebo sim
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    ros_gz_sim,
                    "launch",
                    "gz_sim.launch.py",
                )
            ]
        ),
        launch_arguments={"gz_args": [" -r -v1 ", world]}.items(),
    )

    gazebo_config_path = os.path.join(
        pkg_humanoid_robot,
        'config',
        'gazebo_bridge.yaml'
    )

    start_gazebo_ros_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={gazebo_config_path}',
        ],
        output='screen',
    )


    # Spawn robot in Gazebo
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            "-name", "human1",
            "-topic", "/robot_description",
            # "-x", "0",
            # "-y", "0",
            # "-z", "1.4",
            "--ros-args", "--log-level", log_level
        ],
        output="screen",
    )


    # Launch RViz
    rviz_config_file = os.path.join(pkg_humanoid_robot, 'rviz', 'default.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
    )

    # Launch
    print(">>>>>>>>>>>>>>>>>>>>", pkg_humanoid_robot )
    return LaunchDescription([
        # This line may not be needed
        SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=pkg_humanoid_robot),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='Use sim time if true'),
        # Start the items defined above
        node_robot_state_publisher, gz_sim, rviz_node, start_gazebo_ros_bridge_cmd, spawn_entity
    ])