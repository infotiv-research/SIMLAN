import os
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
from launch.actions import TimerAction


def generate_launch_description():
    # Command-line arguments
    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value="moveit.rviz",
        description="RViz configuration file",
    )
    db_arg = DeclareLaunchArgument(
        "db", default_value="False", description="Database flag."
    )

    # Logging level
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
        description="ROS 2 control hardware interface type to use for the launch file -- possible values: [mock_components, isaac, gz]",
    )

    gz_sim = ExecuteProcess(
        cmd=["gz", "sim", "-v", "4", "-r", "empty.sdf"],
        output="screen",
        condition=IfCondition(LaunchConfiguration("launch_gz")),
    )

    moveit_config = (
        MoveItConfigsBuilder("moveit_resources_panda")
        .robot_description(
            file_path="config/panda.urdf.xacro",
            mappings={
                "ros2_control_hardware_type": LaunchConfiguration(
                    "ros2_control_hardware_type"
                )
            },
        )
        .robot_description_semantic(file_path="config/panda.srdf")
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .trajectory_execution(file_path="config/gripper_moveit_controllers.yaml")
        .planning_pipelines(
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner", "stomp"]
        )
        .to_moveit_configs()
    )

    # Start the actual move_group node/action server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        namespace="panda",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            {
                "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager"
            },
            {"controller_manager_name": "/panda/controller_manager"},
        ],
        remappings=[("/joint_states", "/panda/joint_states")],
        arguments=["--ros-args", "--log-level", log_level],
    )

    # RViz
    rviz_base = LaunchConfiguration("rviz_config")
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("moveit_resources_panda_moveit_config"), "launch", rviz_base]
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
            {"move_group_namespace": "/panda"},  # <- this is key
        ],
        remappings=[
            ("/planning_scene", "/panda/planning_scene"),
            ("/monitored_planning_scene", "/panda/monitored_planning_scene"),
            ("/display_planned_path", "/panda/display_planned_path"),
            ("/joint_states", "/panda/joint_states"),
            ("/trajectory_execution_event", "/panda/trajectory_execution_event"),
        ],
    )

    # Static TF
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        namespace="panda",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "panda_link0"],
    )
    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="panda",
        name="robot_state_publisher",
        output="both",
        parameters=[
            moveit_config.robot_description,
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            "panda",
            "-topic",
            "/panda/robot_description",
            "-x",
            "39.8",
            "-y",
            "0.5",
            "-z",
            "1.2",
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
    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = os.path.join(
        get_package_share_directory("moveit_resources_panda_moveit_config"),
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
        namespace="panda",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/panda/controller_manager",
        ],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )

    panda_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace="panda",
        arguments=["panda_arm_controller", "-c", "/panda/controller_manager"],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )

    panda_hand_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace="panda",
        arguments=["panda_hand_controller", "-c", "/panda/controller_manager"],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )

    # Warehouse mongodb server
    db_config = LaunchConfiguration("db")
    mongodb_server_node = Node(
        package="warehouse_ros_mongo",
        executable="mongo_wrapper_ros.py",
        parameters=[
            {"warehouse_port": 33829},
            {"warehouse_host": "localhost"},
            {"warehouse_plugin": "warehouse_ros_mongo::MongoDatabaseConnection"},
        ],
        output="screen",
        condition=IfCondition(db_config),
    )

    return LaunchDescription(
        [
            launch_gz_arg,
            use_sim_time_arg,
            db_arg,
            rviz_config_arg,
            ros2_control_hardware_type,
            gz_sim,
            spawn_robot,
            rviz_node,
            static_tf_node,
            robot_state_publisher,
            move_group_node,
            ros2_control_node,
            joint_state_broadcaster_spawner,
            panda_arm_controller_spawner,
            panda_hand_controller_spawner,
            mongodb_server_node,
        ]
    )
