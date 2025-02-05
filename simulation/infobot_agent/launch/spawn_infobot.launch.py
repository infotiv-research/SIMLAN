# Copyright 2019 Open Source Robotics Foundation, Inc.
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

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import subprocess


def generate_launch_description():

    xacro_urdf_path = os.path.join(
        get_package_share_directory("infobot_agent"), "urdf", "infobot.urdf.xacro"
    )
    urdf_path = os.path.join(
        get_package_share_directory("infobot_agent"), "urdf", "infobot.urdf"
    )
    p = subprocess.Popen(["xacro", xacro_urdf_path], stdout=open(urdf_path, "w"))
    (output, err) = p.communicate()
    p.wait()
    print("urdf_path2:", urdf_path)
    x_pose = LaunchConfiguration("x_pose", default="0.0")
    y_pose = LaunchConfiguration("y_pose", default="0.0")
    z_pose = LaunchConfiguration("z_pose", default="0.0")
    yaw = LaunchConfiguration("yaw", default="0.0")

    declare_x_position_cmd = DeclareLaunchArgument(
        "x_pose", default_value="0.0", description="x_pose"
    )

    declare_y_position_cmd = DeclareLaunchArgument(
        "y_pose", default_value="0.0", description="y__pose"
    )

    declare_z_position_cmd = DeclareLaunchArgument(
        "z_pose", default_value="0.0", description="z_pose"
    )

    declare_yaw_cmd = DeclareLaunchArgument(
        "yaw", default_value="0.0", description="yaw"
    )

    start_gazebo_ros_spawner_cmd = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity",
            "infobot",
            "-file",
            urdf_path,
            "-x",
            x_pose,
            "-y",
            y_pose,
            "-z",
            z_pose,
            "-Y",
            yaw,
        ],
        output="screen",
    )

    ld = LaunchDescription()

    ld.add_action(declare_x_position_cmd)
    ld.add_action(declare_y_position_cmd)
    ld.add_action(declare_z_position_cmd)
    ld.add_action(declare_yaw_cmd)

    ld.add_action(start_gazebo_ros_spawner_cmd)

    return ld
