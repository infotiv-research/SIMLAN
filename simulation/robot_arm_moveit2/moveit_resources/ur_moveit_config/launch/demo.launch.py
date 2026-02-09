import os
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value="moveit.rviz",
        description="RViz configuration file",
    )

    log_level = LaunchConfiguration("log_level")

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True",
        description="Whether to use simulated clock.",
    )

    launch_gz_arg = DeclareLaunchArgument(
        "launch_gz", default_value="False", description="Launch an empty gazebo world."
    )

    ros2_control_hardware_type = DeclareLaunchArgument(
        "ros2_control_hardware_type",
        default_value="gz",
        description="ROS 2 control hardware interface type to use -- possible values: [mock_components, gz]",
    )

    gz_sim = ExecuteProcess(
        cmd=["gz", "sim", "-v", "4", "-r", "empty.sdf"],
        output="screen",
        condition=IfCondition(LaunchConfiguration("launch_gz")),
    )

    moveit_config = (
        MoveItConfigsBuilder(robot_name="ur", package_name="ur_moveit_config")
        .robot_description(
            file_path="config/ur10.urdf.xacro",
            mappings={
                "ros2_control_hardware_type": LaunchConfiguration(
                    "ros2_control_hardware_type"
                )
            },
        )
        .robot_description_semantic(Path("srdf") / "ur.srdf.xacro", {"name": "ur"})
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .trajectory_execution(file_path="config/gazebo_moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl", "chomp"])
        .to_moveit_configs()
    )

    # Start the actual move_group node/action server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        namespace="ur10",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            {
                "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager"
            },
            {"controller_manager_name": "/ur10/controller_manager"},
        ],
        remappings=[("/joint_states", "/ur10/joint_states")],
        arguments=["--ros-args", "--log-level", log_level],
    )

    # RViz
    rviz_base = LaunchConfiguration("rviz_config")
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("ur_moveit_config"), "config", rviz_base]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            {"move_group_namespace": "/ur10"},
        ],
        remappings=[
            ("/planning_scene", "/ur10/planning_scene"),
            ("/monitored_planning_scene", "/ur10/monitored_planning_scene"),
            ("/display_planned_path", "/ur10/display_planned_path"),
            ("/joint_states", "/ur10/joint_states"),
            ("/trajectory_execution_event", "/ur10/trajectory_execution_event"),
        ],
    )

    # Publish TF from joint states
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="ur10",
        name="robot_state_publisher",
        output="both",
        parameters=[
            moveit_config.robot_description,
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )

    # Static TF: gazebo_world -> world (URDF root)
    gazebo_world_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher_gazebo_world",
        namespace="ur10",
        output="log",
        arguments=[
            "20.0",
            "-20.0",
            "0.0",
            "0.0",
            "0.0",
            "0.0",
            "gazebo_world",
            "world",
        ],
    )

    # Spawn UR10 in Gazebo
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            "ur10",
            "-topic",
            "/ur10/robot_description",
            "-x",
            "20.0",
            "-y",
            "-20.0",
            "-z",
            "0.0",
            "-R",
            "0.0",
            "-P",
            "0.0",
            "-Y",
            "0.0",
            "--ros-args",
            "--log-level",
            log_level,
        ],
        output="screen",
        condition=IfCondition(
            PythonExpression(
                ["'", LaunchConfiguration("ros2_control_hardware_type"), "' == 'gz'"]
            )
        ),
    )

    ros2_controllers_path = os.path.join(
        get_package_share_directory("ur_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_controllers_path],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="screen",
        condition=IfCondition(
            PythonExpression(
                ["'", LaunchConfiguration("ros2_control_hardware_type"), "' != 'gz'"]
            )
        ),
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace="ur10",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/ur10/controller_manager",
        ],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )

    ur10_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace="ur10",
        arguments=["ur10_arm_controller", "-c", "/ur10/controller_manager"],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )

    ur10_gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace="ur10",
        arguments=["ur10_gripper_controller", "-c", "/ur10/controller_manager"],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )

    return LaunchDescription(
        [
            launch_gz_arg,
            use_sim_time_arg,
            rviz_config_arg,
            ros2_control_hardware_type,
            gz_sim,
            spawn_robot,
            rviz_node,
            robot_state_publisher,
            gazebo_world_tf,
            move_group_node,
            ros2_control_node,
            joint_state_broadcaster_spawner,
            ur10_arm_controller_spawner,
            ur10_gripper_controller_spawner,
        ]
    )
