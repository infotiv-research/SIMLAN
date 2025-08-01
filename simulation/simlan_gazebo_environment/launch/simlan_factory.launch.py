#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Joep Tool

import os
from launch.actions import OpaqueFunction, DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import AppendEnvironmentVariable
from launch_ros.actions import Node

# from static_agent_launcher.scripts.camera_config_manager import load_cameras
import yaml


def launch_setup(context, *args, **kwargs):

    actions = []

    camera_enabled_ids = LaunchConfiguration("camera_enabled_ids").perform(context)
    camera_enabled_ids = camera_enabled_ids.split(" ")
    print(f"------------------------------> camera_enabled_ids: {camera_enabled_ids}")
    # camera_ids_list = [163, 164, 165, 166]
    launch_file_dir = os.path.join(
        get_package_share_directory("simlan_gazebo_environment"), "launch"
    )
    # os.environ["GAZEBO_MODEL_PATH"] = (
    #     os.environ["GAZEBO_MODEL_PATH"]
    #     + ":" + os.path.join(
    #         get_package_share_directory("simlan_gazebo_environment"), "models"
    #     )
    # )
    # If you want to run the simulator in headless mode, uncomment the line bew
    # os.environ["GZ_SIM_HEADLESS"]="true"
    os.environ["GZ_SIM_RESOURCE_PATH"] = (
        # os.environ["GZ_SIM_RESOURCE_PATH"]
        # + ":" +
        os.path.join(
            get_package_share_directory("simlan_gazebo_environment"), "models"
        )
    )

    os.environ["IGN_GAZEBO_RESOURCE_PATH"] = (
        # os.environ["IGN_GAZEBO_RESOURCE_PATH"]
        # + ":" +
        os.path.join(
            get_package_share_directory("simlan_gazebo_environment"), "models"
        )
    )
    print(
        "-----------------------> GZ_SIM_RESOURCE_PATH: ",
        os.environ["GZ_SIM_RESOURCE_PATH"],
    )
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")

    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    world = os.path.join(
        get_package_share_directory("simlan_gazebo_environment"),
        "worlds",
        "ign_simlan_factory.world",
    )
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": ["-r -v4 ", world],
            "on_exit_shutdown": "true",
            "use_sim_time": use_sim_time,
            "verbose": "true",
            "pause": "false",
        }.items(),
    )

    bridge_params = os.path.join(
        get_package_share_directory("simlan_gazebo_environment"),
        "config",
        "gz_bridge.yaml",
    )

    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "--ros-args",
            "-p",
            f"config_file:={bridge_params}",
        ],
    )

    ld = LaunchDescription()
    ros_gz_image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/camera/camera_info"],
    )

    for camera_id in camera_enabled_ids:
        ros_gz_image = Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name=f"camera_{camera_id}_bridge",
            output="screen",
            arguments=[
                f"/camera_{camera_id}/image_raw@sensor_msgs/msg/Image@gz.msgs.Image",
                f"/camera_{camera_id}/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
            ],
            remappings=[
                (
                    f"/camera_{camera_id}/image_raw",
                    f"/static_agents/camera_{camera_id}/image_raw",
                ),
                (
                    f"/camera_{camera_id}/camera_info",
                    f"/static_agents/camera_{camera_id}/camera_info",
                ),
            ],
        )
        actions.append(ros_gz_image)

        ros_gz_depth_image = Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name=f"depth_camera_{camera_id}_bridge",
            output="screen",
            arguments=[
                f"/depth_camera_{camera_id}/image_raw@sensor_msgs/msg/Image@gz.msgs.Image",
                f"/depth_camera_{camera_id}/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
                # f"/depth_camera_{camera_id}/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked",
            ],
            remappings=[
                (
                    f"/depth_camera_{camera_id}/image_raw",
                    f"/static_agents/depth_camera_{camera_id}/image_raw",
                ),
                (
                    f"/depth_camera_{camera_id}/camera_info",
                    f"/static_agents/depth_camera_{camera_id}/camera_info",
                ),
                # (
                #     f"/depth_camera_{camera_id}/points",
                #     f"/static_agents/depth_camera_{camera_id}/points",
                # ),
            ],
        )
        actions.append(ros_gz_depth_image)

        ros_gz_semantic_image = Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name=f"semantic_camera_{camera_id}_bridge",
            output="screen",
            arguments=[
                f"/semantic_camera_{camera_id}/image_raw@sensor_msgs/msg/Image@gz.msgs.Image",
                f"/semantic_camera_{camera_id}/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
                f"/semantic_camera_{camera_id}/labels_map@sensor_msgs/msg/Image@gz.msgs.Image",
                f"/semantic_camera_{camera_id}/colored_map@sensor_msgs/msg/Image@gz.msgs.Image",
            ],
            remappings=[
                (
                    f"/semantic_camera_{camera_id}/image_raw",
                    f"/static_agents/semantic_camera_{camera_id}/image_raw",
                ),
                (
                    f"/semantic_camera_{camera_id}/camera_info",
                    f"/static_agents/semantic_camera_{camera_id}/camera_info",
                ),
                (
                    f"/semantic_camera_{camera_id}/labels_map",
                    f"/static_agents/semantic_camera_{camera_id}/labels_map",
                ),
                (
                    f"/semantic_camera_{camera_id}/colored_map",
                    f"/static_agents/semantic_camera_{camera_id}/colored_map",
                ),
            ],
        )
        actions.append(ros_gz_semantic_image)
    # segmentation_bridge = Node(
    #     package="ros_gz_bridge",
    #     executable="parameter_bridge",
    #     name="segmentation_bridge",
    #     output="screen",
    #     arguments=[
    #         "/semantic/labels_map@sensor_msgs/msg/Image@gz.msgs.Image",
    #         "/semantic/colored_map@sensor_msgs/msg/Image@gz.msgs.Image"
    #     ],
    #     remappings=[
    #         ("/semantic/labels_map", "/segmentation/labels_map"),
    #         ("/semantic/colored_map", "/segmentation/colored_map")
    #     ]
    # )
    # ld.add_action(segmentation_bridge)

    actions.append(gzserver_cmd)
    # ld.add_action(set_env_vars_resources)
    actions.append(ros_gz_bridge)

    # ld.add_action(world)

    try:
        value = os.environ["SIM_ENV"]  # "DEV" , "CICD" or empty
        print("SIM_ENV=" + value)
    except KeyError:
        print("SIM_ENV not found, set to DEV")
        os.environ["SIM_ENV"] = "DEV"

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            # Declare the launch argument (optional if passed from parent)
            DeclareLaunchArgument("camera_enabled_ids", default_value="163 164 165"),
            OpaqueFunction(function=launch_setup),
        ]
    )
