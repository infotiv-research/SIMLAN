import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration, TextSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from srdfdom.srdf import SRDF
from moveit_configs_utils.launch_utils import add_debuggable_node, DeclareBooleanLaunchArg
from moveit_configs_utils import MoveItConfigsBuilder
from launch.conditions import IfCondition



def generate_launch_description():
    log_level = LaunchConfiguration("log_level", default="INFO")
    namespace = LaunchConfiguration("namespace")
    
    moveit_config = (
        MoveItConfigsBuilder("human_support", package_name="humanoid_support_moveit_config")
        .robot_description(
            file_path="config/human_support.urdf.xacro",
            mappings={"namespace": namespace}
        )
        .to_moveit_configs()
    )

    return generate_demo_launch(moveit_config, log_level, namespace)


def generate_rsp_launch(moveit_config, log_level, namespace):
    """Launch file for robot state publisher (rsp)"""

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument("publish_frequency", default_value="15.0"))

    # static_tf_humanoid = Node(
    #     package="tf2_ros",
    #     executable="static_transform_publisher",
    #     name="static_tf_base_to_humanoid_base",
    #     arguments=[
    #         "0", "0", "0", "0", "0", "0", 
    #         "base_link", 
    #         [namespace, TextSubstitution(text="/odom")]
    #     ],
    #     output="screen",
    # )
    # ld.add_action(static_tf_humanoid)

    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        respawn=True,
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[
            moveit_config.robot_description,
            {
                "publish_frequency": LaunchConfiguration("publish_frequency"),
            },
            {"use_sim_time": True},
            {"frame_prefix": f"{namespace}/"}
        ],
    )
    ld.add_action(TimerAction(period=4.0, actions=[rsp_node]))

    return ld


def generate_moveit_rviz_launch(moveit_config, namespace):
    """Launch file for rviz"""
    ld = LaunchDescription()

    ld.add_action(DeclareBooleanLaunchArg("debug", default_value=False))
    ld.add_action(
        DeclareLaunchArgument(
            "rviz_config",
            default_value=str(moveit_config.package_path / "config/moveit.rviz"),
        )
    )

    rviz_parameters = [
        moveit_config.planning_pipelines,
        moveit_config.robot_description_kinematics,
        moveit_config.joint_limits,
        moveit_config.robot_description,
        moveit_config.robot_description_semantic,
        {"use_sim_time": LaunchConfiguration("use_sim_time", default="true")},
        {"move_group_namespace": namespace}
    ]
    
    add_debuggable_node(
        ld,
        package="rviz2",
        executable="rviz2",
        output="log",
        respawn=False,
        arguments=["-d", LaunchConfiguration("rviz_config")],
        parameters=rviz_parameters,
        remappings=[
            ("/planning_scene", f"/{namespace}/planning_scene"),
            ("/monitored_planning_scene", f"/{namespace}/monitored_planning_scene"),
            ("/display_planned_path", f"/{namespace}/display_planned_path"),
            ("/joint_states", f"/{namespace}/joint_states"),
            ("/trajectory_execution_event", f"/{namespace}/trajectory_execution_event"),
        ],

    )

    return ld

def generate_setup_assistant_launch(moveit_config):
    """Launch file for MoveIt Setup Assistant"""
    ld = LaunchDescription()
    ld.add_action(DeclareBooleanLaunchArg("debug", default_value=False))
    add_debuggable_node(
        ld,
        package="moveit_setup_assistant",
        executable="moveit_setup_assistant",
        arguments=[["--config_pkg=", str(moveit_config.package_path)]],
    )
    return ld


def generate_static_virtual_joint_tfs_launch(moveit_config):
    ld = LaunchDescription()
    name_counter = 0

    for key, xml_contents in moveit_config.robot_description_semantic.items():
        srdf = SRDF.from_xml_string(xml_contents)
        for vj in srdf.virtual_joints:
            ld.add_action(
                Node(
                    package="tf2_ros",
                    executable="static_transform_publisher",
                    name=f"static_transform_publisher{name_counter}",
                    output="log",
                    arguments=[
                        "--frame-id", vj.parent_frame,
                        "--child-frame-id", vj.child_link
                    ],
                    parameters=[{"use_sim_time": True}]
                )
            )
            name_counter += 1
    return ld


def generate_move_group_launch(moveit_config, log_level, namespace):
    ld = LaunchDescription()

    ld.add_action(DeclareBooleanLaunchArg("debug", default_value=False))
    ld.add_action(DeclareBooleanLaunchArg("allow_trajectory_execution", default_value=True))
    ld.add_action(DeclareBooleanLaunchArg("publish_monitored_planning_scene", default_value=True))

    ld.add_action(
        DeclareLaunchArgument(
            "capabilities",
            default_value=moveit_config.move_group_capabilities["capabilities"],
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "disable_capabilities",
            default_value=moveit_config.move_group_capabilities["disable_capabilities"],
        )
    )
    ld.add_action(DeclareBooleanLaunchArg("monitor_dynamics", default_value=False))

    should_publish = LaunchConfiguration("publish_monitored_planning_scene")

    move_group_configuration = {
        "publish_robot_description_semantic": True,
        "allow_trajectory_execution": LaunchConfiguration("allow_trajectory_execution"),
        "capabilities": ParameterValue(
            LaunchConfiguration("capabilities"), value_type=str
        ),
        "disable_capabilities": ParameterValue(
            LaunchConfiguration("disable_capabilities"), value_type=str
        ),
        "publish_planning_scene": should_publish,
        "publish_geometry_updates": should_publish,
        "publish_state_updates": should_publish,
        "publish_transforms_updates": should_publish,
        "monitor_dynamics": False,
        "use_sim_time": True,
    }

    move_group_params = [
        moveit_config.to_dict(),
        move_group_configuration,
        {"moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager"},
        {"controller_manager_name": f"/{namespace}/controller_manager"}
    ]

    add_debuggable_node(
        ld,
        package="moveit_ros_move_group",
        executable="move_group",
        commands_file=str(moveit_config.package_path / "launch" / "gdb_settings.gdb"),
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        parameters=move_group_params,
        additional_env={"DISPLAY": os.environ.get("DISPLAY", "")},
        remappings=[("/joint_states", f"/{namespace}/joint_states")],
    )
    return ld

def generate_demo_launch(moveit_config, log_level, namespace, launch_package_path=None):
    """bring namespace to context to not launch multiple rviz2 nodes"""
    def namespace_to_context(context, *args, **kwargs):
        ns_val = namespace.perform(context)  # Get actual string at launch
        if ns_val == "humanoid_1":
            ld.add_action(generate_moveit_rviz_launch(moveit_config, ns_val))

        # Robot state publisher
        ld.add_action(generate_rsp_launch(moveit_config, log_level, ns_val))

        # Move group
        ld.add_action(generate_move_group_launch(moveit_config, log_level, ns_val))

    """Launches a self-contained demo"""
    if launch_package_path is None:
        launch_package_path = moveit_config.package_path

    ld = LaunchDescription()

    ld.add_action(
        DeclareBooleanLaunchArg(
            "debug",
            default_value=False,
            description="By default, we are not in debug mode",
        )
    )
    ld.add_action(DeclareBooleanLaunchArg("use_rviz", default_value=True))

    # Virtual joints
    ld.add_action(generate_static_virtual_joint_tfs_launch(moveit_config))

    # RViz
    ld.add_action(OpaqueFunction(function=namespace_to_context))
    

    return ld
