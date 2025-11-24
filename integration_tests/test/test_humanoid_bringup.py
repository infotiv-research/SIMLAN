import os
import sys
import time

import launch
import launch_pytest
import launch_testing
import pytest
import rclpy
from launch.actions import ExecuteProcess
from test_node import MakeTestNode

sys.path.append(".")


@launch_pytest.fixture(autouse=True)
def launch_description():

    ######## Arguments ########
    world_setup = "default"
    log_level = "error"
    humanoid_str = '[{"namespace": "humanoid_1","initial_pose_x":10,"initial_pose_y":0.0,"cam_ns": "camera0"}]'

    ######## Launch-files / Processes  ########
    os.environ["ROS_DOMAIN_ID"] = str(os.getpid() % 232)  #

    sim_proc = ExecuteProcess(
        cmd=[
            "ros2",
            "launch",
            "simlan_bringup",
            "sim.launch.py",
            f"log_level:={log_level}",
            f"world_setup:={world_setup}",
        ],
        output="screen",
    )
    humanoid_proc = ExecuteProcess(
        cmd=[
            "ros2",
            "launch",
            "humanoid_robot",
            "multiple_humanoid_spawn.launch.py",
            f"log_level:={log_level}",
            f"humanoids:={humanoid_str}",
        ],
        output="screen",
    )

    ######## Test execution ########
    rclpy.init()

    yield launch.LaunchDescription(
        [
            sim_proc,
            humanoid_proc,
            launch_testing.util.KeepAliveProc(),
            launch_pytest.actions.ReadyToTest(),
        ]
    )
    rclpy.shutdown()


@pytest.mark.launch(fixture=launch_description)
async def test_sim_and_multiple_robots_bringup_Startup_Nodes_and_topics_should_be_visible():
    print(
        "STARTING TEST: sim_and_multiple_robots_bringup_Startup_Nodes_and_topics_should_be_visible "
    )
    time.sleep(5)

    ###### SETUP  ######
    num_retries = 5
    target_topics = [
        "/tf",
        "/humanoid_1/support_whole_body_controller/controller_state",
        "/humanoid_1/support_whole_body_controller/joint_trajectory",
        "/humanoid_1/support_whole_body_controller/transition_event",
        "/humanoid_1/trajectory_execution_event",
        "/humanoid_1/velocity_controller/cmd_vel",
        "/humanoid_1/velocity_controller/cmd_vel_unstamped",
        "/humanoid_1/velocity_controller/odom",
        "/humanoid_1/velocity_controller/transition_event",
    ]
    target_nodes = [
        "/ros_gz_bridge",
        "/humanoid_1/twist_mux",
        "/humanoid_1/twist_stamper_node",
        "/humanoid_1/velocity_controller",
        "/humanoid_1/controller_manager",
        "/humanoid_1/gz_ros_control",
        "/humanoid_1/joint_state_broadcaster",
        "/humanoid_1/move_group",
        "/humanoid_1/moveit",
        "/humanoid_1/moveit_simple_controller_manager",
        "/humanoid_1/robot_state_publisher",
        "/humanoid_1/support_whole_body_controller",
        "/humanoid_1/twist_mux",
        "/humanoid_1/twist_stamper_node",
    ]

    ###### NODE SETUP ######
    test_node = MakeTestNode()

    actual_nodes, actual_topics = [], []

    ###### EXECUTION ######
    for i in range(num_retries):
        actual_topics.extend(
            [topic for topic, _ in test_node.get_topic_names_and_types()]
        )
        actual_nodes.extend(test_node.get_fully_qualified_node_names())
        actual_nodes = list(set(actual_nodes))
        actual_topics = list(set(actual_topics))
        actual_nodes.sort()
        actual_topics.sort()

        found_all_topics = all(
            [True if topic in actual_topics else False for topic in target_topics]
        )
        found_all_nodes = all(
            [True if node in actual_nodes else False for node in target_nodes]
        )
        if found_all_nodes and found_all_topics:
            break
        time.sleep(3)

    ###### ASSERTIONS ######
    for topic in target_topics:
        assert (
            topic in actual_topics
        ), f"Target topic {topic} not found in actual topics"
    for node in target_nodes:
        assert node in actual_nodes, f"Target node {node} not found in actual nodes"

    ###### CLEANUP ######
    test_node.stop()


@pytest.mark.launch(fixture=launch_description, shutdown=True)
async def test_after_shutdown(launch_service, launch_description):
    print("Testing after shutdown command.")
    pass
