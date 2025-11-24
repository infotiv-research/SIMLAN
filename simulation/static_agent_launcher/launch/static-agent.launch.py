import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import OpaqueFunction


from launch_ros.actions import Node
import xacro

# Author: Hamid Ebadi


def launch_setup(context, *args, **kwargs):

    camera_enabled_ids = LaunchConfiguration("camera_enabled_ids").perform(context)
    log_level = LaunchConfiguration("log_level").perform(context)
    camera_streams = LaunchConfiguration("camera_streams").perform(context)
    camera_enabled_ids = camera_enabled_ids.split(" ")
    camera_streams = camera_streams.split(" ")
    
    pkg_name = "static_agent_launcher"
    file_subpath = "description/agents.urdf.xacro"

    xacro_file = os.path.join(get_package_share_directory(pkg_name), file_subpath)
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    actions=[]
    # Configure the node
    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="static_agents",
        output="screen",
        parameters=[{"robot_description": robot_description_raw, "use_sim_time": True},
                    ],
    )
    actions.append(node_robot_state_publisher)

    static_transform_node = Node(
    package="tf2_ros",
    executable="static_transform_publisher",
    name="static_world_to_static_agents_tf",
    arguments=[
        "--x", "0.0",
        "--y", "0.0",
        "--z", "0.0",
        "--roll", "0.0",
        "--pitch", "0.0",
        "--yaw", "0.0",
        "world",
        "static_agents/base_link"
    ],
    output="screen",
    )
    actions.append(static_transform_node)

    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        namespace="static_agents",
        arguments=[
            "-topic",
            "robot_description",
            "-robot_namespace",
            "static_agents",
            "-name",
            "cameras"
        ],
        output="screen",
    )
    actions.append(spawn_entity)

    camera_parameters=[]
    camera_remappings=[]

    for camera_id in camera_enabled_ids:
        for streams in camera_streams:
            if streams=="image":
                camera_parameters.append(f"/camera_{camera_id}/image_raw@sensor_msgs/msg/Image@gz.msgs.Image")
                camera_parameters.append(f"/camera_{camera_id}/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo")

                camera_remappings.append((f"/camera_{camera_id}/image_raw", f"/static_agents/camera_{camera_id}/image_raw"))
                camera_remappings.append((f"/camera_{camera_id}/camera_info", f"/static_agents/camera_{camera_id}/camera_info"))

            if streams=="depth":
                camera_parameters.append(f"/depth_camera_{camera_id}/image_raw@sensor_msgs/msg/Image@gz.msgs.Image")
                camera_parameters.append(f"/depth_camera_{camera_id}/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo")

                camera_remappings.append((f"/depth_camera_{camera_id}/image_raw", f"/static_agents/depth_camera_{camera_id}/image_raw"))
                camera_remappings.append((f"/depth_camera_{camera_id}/camera_info", f"/static_agents/depth_camera_{camera_id}/camera_info"))

            if streams=="semantic":
                camera_parameters.append(f"/semantic_camera_{camera_id}/image_raw@sensor_msgs/msg/Image@gz.msgs.Image")
                camera_parameters.append(f"/semantic_camera_{camera_id}/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo")
                camera_parameters.append(f"/semantic_camera_{camera_id}/labels_map@sensor_msgs/msg/Image@gz.msgs.Image")
                camera_parameters.append(f"/semantic_camera_{camera_id}/colored_map@sensor_msgs/msg/Image@gz.msgs.Image")

                camera_remappings.append((f"/semantic_camera_{camera_id}/image_raw", f"/static_agents/semantic_camera_{camera_id}/image_raw"))
                camera_remappings.append((f"/semantic_camera_{camera_id}/camera_info", f"/static_agents/semantic_camera_{camera_id}/camera_info"))
                camera_remappings.append((f"/semantic_camera_{camera_id}/labels_map", f"/static_agents/semantic_camera_{camera_id}/labels_map"))
                camera_remappings.append((f"/semantic_camera_{camera_id}/colored_map", f"/static_agents/semantic_camera_{camera_id}/colored_map"))
    ros_gz_images=Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name=f"camera_{camera_id}_bridge",
        output="screen",
        arguments=[
                    *camera_parameters,
                    "--ros-args",
                    "--log-level",
                    log_level ],
        remappings=[*camera_remappings]
    )
    actions.append(ros_gz_images)

    return actions

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
