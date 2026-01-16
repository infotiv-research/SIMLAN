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

os.environ["RMW_FASTRTPS_USE_SHARED_MEMORY"] = "OFF"
from launch.actions import OpaqueFunction, DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import AppendEnvironmentVariable
from launch_ros.actions import Node
import xacro


def launch_setup(context, *args, **kwargs):

    actions = []

    log_level = LaunchConfiguration("log_level").perform(context)
    world_setup = LaunchConfiguration("world_setup").perform(context)
    headless_gazebo = (
        LaunchConfiguration("headless_gazebo").perform(context).lower() == "true"
    )
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    print(f"[DEBUG] world_setup={world_setup}")

    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    original_world = os.path.join(
        get_package_share_directory("simlan_gazebo_environment"),
        "worlds",
        "ign_simlan_factory.world.xacro",
    )
    world_sdf_path = os.path.join(
        get_package_share_directory("simlan_gazebo_environment"),
        "worlds",
        f"{world_setup}_world.world",
    )
    doc = xacro.process_file(original_world, mappings={"world_setup": world_setup})
    with open(world_sdf_path, "w") as f:
        f.write(doc.toprettyxml(indent="  "))

    # Setting the headless flag. If true then gazebo window won't turn on.
    headless_flag = ""
    if headless_gazebo:
        headless_flag = "-s"

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "log_level": log_level,
            "gz_args": f"{headless_flag} -r -v 1 {world_sdf_path}",
            "on_exit_shutdown": "true",
            "use_sim_time": use_sim_time,
            "verbose": "false",
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
            # For some reason bridges cannot be started from the config file, so it has to be done here
            "/world/default/set_pose@ros_gz_interfaces/srv/SetEntityPose",
            "/world/default/create@ros_gz_interfaces/srv/SpawnEntity",
            "/world/default/remove@ros_gz_interfaces/srv/DeleteEntity",
            "--ros-args",
            "-p",
            f"config_file:={bridge_params}",
            "--log-level",
            log_level,
        ],
        output="screen",
    )

    actions.append(ros_gz_bridge)
    actions.append(gzserver_cmd)

    try:
        value = os.environ["SIM_ENV"]  # "DEV" , "CICD" or empty
        print("SIM_ENV=" + value)
    except KeyError:
        print("SIM_ENV not found, set to DEV")
        os.environ["SIM_ENV"] = "DEV"

    return actions


def generate_launch_description():
    set_gazebo_model_path_env = AppendEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH",
        os.path.join(
            get_package_share_directory("simlan_gazebo_environment"), "models"
        ),
    )

    return LaunchDescription(
        [
            set_gazebo_model_path_env,
            # Declare the launch argument (optional if passed from parent)
            DeclareLaunchArgument(
                "world_setup",
                default_value="empty",
                description="Select world setup: empty, light, medium, regular",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
