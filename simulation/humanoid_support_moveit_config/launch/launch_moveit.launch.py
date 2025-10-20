import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from srdfdom.srdf import SRDF

from moveit_configs_utils.launch_utils import (
    add_debuggable_node,
    DeclareBooleanLaunchArg,
)
from launch_ros.substitutions import FindPackageShare

from moveit_configs_utils import MoveItConfigsBuilder


from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    log_level = LaunchConfiguration("log_level", default="INFO")

    moveit_config = MoveItConfigsBuilder("human_support", package_name="humanoid_support_moveit_config").to_moveit_configs()
    return generate_demo_launch(moveit_config, log_level)


def generate_rsp_launch(moveit_config, log_level):
    """Launch file for robot state publisher (rsp)"""

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument("publish_frequency", default_value="15.0"))

    # Given the published joint states, publish tf for the robot links and the robot description
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
            {"use_sim_time": True}
        ],
    )
    ld.add_action(rsp_node)

    return ld


def generate_moveit_rviz_launch(moveit_config):
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
        {"move_group_namespace": "humanoid"}

    ]

    add_debuggable_node(
        ld,
        package="rviz2",
        executable="rviz2",
        output="log",
        respawn=False,
        arguments=["-d", LaunchConfiguration("rviz_config")],
        parameters=rviz_parameters,
        remappings=[("/planning_scene", "/humanoid/planning_scene"),
                    ("/monitored_planning_scene", "/humanoid/monitored_planning_scene"),
                    ("/display_planned_path", "/humanoid/display_planned_path"),
                    ("/joint_states", "/humanoid/joint_states"),
                    ("/trajectory_execution_event", "/humanoid/trajectory_execution_event"), ],

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
                        "--frame-id",
                        vj.parent_frame,
                        "--child-frame-id",
                        vj.child_link
                    ],
                    parameters=[{"use_sim_time": True}]
                )
            )
            name_counter += 1
    return ld


def generate_move_group_launch(moveit_config, log_level):
    ld = LaunchDescription()

    ld.add_action(DeclareBooleanLaunchArg("debug", default_value=False))
    ld.add_action(
        DeclareBooleanLaunchArg("allow_trajectory_execution", default_value=True)
    )
    ld.add_action(
        DeclareBooleanLaunchArg("publish_monitored_planning_scene", default_value=True)
    )
    # load non-default MoveGroup capabilities (space separated)
    ld.add_action(
        DeclareLaunchArgument(
            "capabilities",
            default_value=moveit_config.move_group_capabilities["capabilities"],
        )
    )
    # inhibit these default MoveGroup capabilities (space separated)
    ld.add_action(
        DeclareLaunchArgument(
            "disable_capabilities",
            default_value=moveit_config.move_group_capabilities["disable_capabilities"],
        )
    )

    # do not copy dynamics information from /joint_states to internal robot monitoring
    # default to false, because almost nothing in move_group relies on this information
    ld.add_action(DeclareBooleanLaunchArg("monitor_dynamics", default_value=False))

    should_publish = LaunchConfiguration("publish_monitored_planning_scene")

    move_group_configuration = {
        "publish_robot_description_semantic": True,
        "allow_trajectory_execution": LaunchConfiguration("allow_trajectory_execution"),
        # Note: Wrapping the following values is necessary so that the parameter value can be the empty string
        "capabilities": ParameterValue(
            LaunchConfiguration("capabilities"), value_type=str
        ),
        "disable_capabilities": ParameterValue(
            LaunchConfiguration("disable_capabilities"), value_type=str
        ),
        # Publish the planning scene of the physical robot so that rviz plugin can know actual robot
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
        {"controller_manager_name": "/humanoid/controller_manager"},


    ]

    add_debuggable_node(
        ld,
        package="moveit_ros_move_group",
        executable="move_group",
        # namespace="humanoid",
        commands_file=str(moveit_config.package_path / "launch" / "gdb_settings.gdb"),
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        parameters=move_group_params,
        # Set the display variable, in case OpenGL code is used internally
        additional_env={"DISPLAY": os.environ.get("DISPLAY", "")},
        remappings=[("/joint_states", "/humanoid/joint_states")],
        
    )
    return ld


def generate_demo_launch(moveit_config, log_level, launch_package_path=None):
    """
    Launches a self contained demo

    launch_package_path is optional to use different launch and config packages

    Includes
     * static_virtual_joint_tfs
     * robot_state_publisher
     * move_group
     * moveit_rviz
     * ros2_control_node + controller spawners
    """
    if launch_package_path == None:
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
    
    # If there are virtual joints, broadcast static tf by including virtual_joints launch
    ld.add_action(generate_static_virtual_joint_tfs_launch(moveit_config))


    ld.add_action(generate_rsp_launch(moveit_config, log_level))


    ld.add_action(generate_move_group_launch(moveit_config, log_level))

    # Run Rviz and load the default config to see the state of the move_group node
    ld.add_action(generate_moveit_rviz_launch(moveit_config))

    return ld