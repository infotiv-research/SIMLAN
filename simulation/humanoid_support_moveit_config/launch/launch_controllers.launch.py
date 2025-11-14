import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, TextSubstitution, PathJoinSubstitution

def generate_launch_description():
    ld = LaunchDescription()

    # Declare launch arguments
    ld.add_action(DeclareLaunchArgument('namespace', default_value='humanoid_1'))
    ld.add_action(DeclareLaunchArgument('log_level', default_value='info'))
    ld.add_action(DeclareLaunchArgument('initial_pose_x', default_value='0.0'))
    ld.add_action(DeclareLaunchArgument('initial_pose_y', default_value='0.0'))

    # Create moveit_config
    namespace = LaunchConfiguration('namespace')
    moveit_config = (
        MoveItConfigsBuilder("human_support", package_name="humanoid_support_moveit_config")
        .robot_description(
            file_path="config/human_support.urdf.xacro",
            mappings={"namespace": namespace}
        )
        .to_moveit_configs()
    )

    # Pass ld and moveit_config to the opaque function
    ld.add_action(OpaqueFunction(
        function=generate_spawn_controllers_launch,
        kwargs={'ld': ld, 'moveit_config': moveit_config}
    ))

    return ld


def generate_spawn_controllers_launch(context, moveit_config, *args, **kwargs):
    namespace = LaunchConfiguration('namespace').perform(context)
    initial_pose_x = float(LaunchConfiguration('initial_pose_x').perform(context))
    initial_pose_y = float(LaunchConfiguration('initial_pose_y').perform(context))
    log_level = LaunchConfiguration('log_level').perform(context)
    actions=[]
    twist_mux_params = os.path.join(
        get_package_share_directory("humanoid_support_moveit_config"), 
        "config", "twist_mux.yaml"
    )

    controller_names = moveit_config.trajectory_execution.get(
        "moveit_simple_controller_manager", {}
    ).get("controller_names", [])

    # Spawn controllers
    for controller in controller_names + ["joint_state_broadcaster", "velocity_controller"]:
        actions.append(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    controller,
                    "--controller-manager",
                    PathJoinSubstitution([TextSubstitution(text="/"), namespace, TextSubstitution(text="controller_manager")]),
                    "--ros-args",
                    "--log-level",
                    log_level
                ],
                output="screen",
                parameters=[{"use_sim_time": True}],
                remappings=[
                    (
                        "/controller_manager/robot_description",
                        PathJoinSubstitution([TextSubstitution(text="/"), namespace, TextSubstitution(text="robot_description")])
                    ),
                ],
            )
        )

    # twist_mux node
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        output="screen",
        arguments=[
            "--ros-args",
            "--log-level",
            log_level
        ],
        remappings=[
            ("cmd_vel_out", "velocity_controller/cmd_vel_unstamped")
        ],
        parameters=[twist_mux_params, {'use_sim_time': True}],
    )

    # twist_stamper node
    twist_stamper = Node(
        package="twist_stamper",
        executable="twist_stamper",
        name="twist_stamper_node",
        output="screen",
        arguments=[
            "--ros-args",
            "--log-level",
            log_level
        ],
        remappings=[
            ("cmd_vel_in", "velocity_controller/cmd_vel_unstamped"), 
            ("cmd_vel_out", "velocity_controller/cmd_vel")
        ],
        parameters=[{"frame_id": "base_link"}, {'use_sim_time': True}],
    )

    odom_publisher = Node(
        package="humanoid_odom_pub",
        executable="humanoid_odom_pub",
        name="humanoid_odom_pub",
        output="screen",
        namespace=namespace,
        parameters=[
                    {"namespace": namespace},
                    {"initial_pose_x": initial_pose_x},
                    {"initial_pose_y": initial_pose_y},
                    {'use_sim_time': True}
                    ],
    )

    actions.append(twist_mux)
    actions.append(twist_stamper)
    actions.append(odom_publisher)
    return actions
