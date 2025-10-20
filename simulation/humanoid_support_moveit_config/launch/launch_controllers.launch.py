import os
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    # Logging level 
    log_level = LaunchConfiguration('log_level')

    moveit_config = MoveItConfigsBuilder("human_support", package_name="humanoid_support_moveit_config").to_moveit_configs()
    return generate_spawn_controllers_launch(moveit_config, log_level)


def generate_spawn_controllers_launch(moveit_config, log_level):

    twist_mux_params = os.path.join(get_package_share_directory("humanoid_support_moveit_config"), "config", "twist_mux.yaml"
    )

    controller_names = moveit_config.trajectory_execution.get(
        "moveit_simple_controller_manager", {}
    ).get("controller_names", [])
    ld = LaunchDescription()
    for controller in controller_names + ["joint_state_broadcaster", "velocity_controller"]:
        ld.add_action(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    controller, "--controller-manager", "/humanoid/controller_manager",
                    "--ros-args", "--log-level", log_level
                ],
                output="screen",
                parameters=[{"use_sim_time": True}],
                remappings=[
                ("/controller_manager/robot_description", "/humanoid/robot_description"),
            ],
            )
        )
    twist_mux = Node(   
        package="twist_mux",
        executable="twist_mux",
        output="screen",
        arguments=[
            "--ros-args",
            "--log-level",
            log_level
        ],
        remappings={("cmd_vel_out", "velocity_controller/cmd_vel_unstamped")},
        parameters=[twist_mux_params,{'use_sim_time': True}],
    )

    twist_stamper = Node(
        package = "twist_stamper",
        executable = "twist_stamper",
        name = "twist_stamper_node",
        output = "screen",
        arguments=[
            "--ros-args",
            "--log-level",
            log_level
        ],
        remappings={("cmd_vel_in", "velocity_controller/cmd_vel_unstamped"), 
            ("cmd_vel_out", "velocity_controller/cmd_vel")},
        parameters=[{"frame_id": "arm_base_link"},{'use_sim_time': True}],
    )
    ld.add_action(twist_mux)
    ld.add_action(twist_stamper)
    return ld