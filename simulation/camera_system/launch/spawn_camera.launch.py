from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
import xacro


def generate_launch_description():

    cam_ns_arg = DeclareLaunchArgument(
        'cam_namespace', default_value='camera0', description='Camera namespace'
    )
   
    camera_description_path = os.path.join(
        get_package_share_directory('camera_system'),
        'urdf',
        'indoor_camera.xacro'
    )
    
    doc = xacro.process_file(camera_description_path)
    robot_description = doc.toxml()

    camera_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='camera_state_publisher',
        namespace=LaunchConfiguration('cam_namespace'),
        output='screen',
        parameters=[{'robot_description': robot_description}],
        remappings=[
            ('robot_description', 'camera_description'),
        ]
    )

    spawn_camera = Node(
        package='ros_gz_sim',  
        executable='create',
        namespace=LaunchConfiguration('cam_namespace'),
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '-topic', 'camera_description',
            # '-string', '/camera_description'
            '-name', 'camera_humanoid',
            '-x', '43.0',
            '-y', '0.0',
            '-z', '2'
        ]
    )


    # Bridge between ROS 2 and Ignition
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='camera_bridge',
        namespace=LaunchConfiguration('cam_namespace'),
        output='screen',
        parameters=[{'use_sim_time': True}],
        # arguments=[
        #     '/world/empty/model/camera/link/hhcamera/sensor/camera_sensor/image@sensor_msgs/msg/Image@gz.msgs.Image',
        # ],
        arguments=[
            '/world/default/model/camera_humanoid/link/hhcamera/sensor/camera_sensor/image@sensor_msgs/msg/Image@gz.msgs.Image',
            # '/world/default/model/camera_humanoid/link/hhcamera/sensor/camera_sensor/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',


            # '--ros-args',
            # '-r', '/world/default/model/camera_humanoid/link/hhcamera/sensor/camera_sensor/image:=image_raw',
            # '-r', '/world/default/model/camera_humanoid/link/hhcamera/sensor/camera_sensor/camera_info:=camera_info',
        ],
        # remappings=[
        #     ('/world/empty/model/camera/link/hhcamera/sensor/camera_sensor/image', '/camera/image_raw')
        # ]
        remappings=[
            ('/world/default/model/camera_humanoid/link/hhcamera/sensor/camera_sensor/image', '/camera0/image_raw')
        ]
    )



    return LaunchDescription([
        cam_ns_arg,
        camera_state_publisher,
        spawn_camera,
        bridge  
    ])
