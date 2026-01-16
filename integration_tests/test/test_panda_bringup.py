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
import subprocess

sys.path.append(".")


@launch_pytest.fixture(autouse=True)
def launch_description():

    ######## Arguments ########
    world_setup = "default"
    log_level = "error"
    humanoid_str = (
        '[{"namespace": "humanoid_1","initial_pose_x":10,"initial_pose_y":0.0}]'
    )

    ######## Launch-files / Processes  ########

    sim_proc = ExecuteProcess(
        cmd=[
            "ros2",
            "launch",
            "simlan_bringup",
            "sim.launch.py",
            f"log_level:={log_level}",
            f"world_setup:={world_setup}",
            f"headless_gazebo:=true",
        ],
        output="screen",
    )
    panda_arm_proc = ExecuteProcess(
        cmd=[
            "ros2",
            "launch",
            "moveit_resources_panda_moveit_config",
            "demo.launch.py",
            f"log_level:={log_level}",
        ],
        output="screen",
    )

    ######## Test execution ########
    rclpy.init()

    yield launch.LaunchDescription(
        [
            sim_proc,
            panda_arm_proc,
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
        "/panda/controller_manager/activity",
        "/panda/dynamic_joint_states",
        "/panda/joint_states",
        "/panda/monitored_planning_scene",
        "/panda/panda_arm_controller/controller_state",
        "/panda/panda_arm_controller/joint_trajectory",
        "/panda/pipeline_state",
        "/panda/planning_scene",
        "/panda/robot_description",
        "/panda/robot_description_semantic",
    ]
    target_nodes = [
        "/ros_gz_bridge",
        "/panda/controller_manager",
        "/panda/gz_ros_control",
        "/panda/joint_state_broadcaster",
        "/panda/move_group",
        "/panda/moveit",
        "/panda/moveit_simple_controller_manager",
        "/panda/panda_arm_controller",
        "/panda/panda_hand_controller",
        "/panda/robot_state_publisher",
        "/panda/static_transform_publisher",
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
    print("Killing remaining processes.")
    subprocess.run(["pkill", "-9", "-f", "gzserver"])
    subprocess.run(["pkill", "-9", "-f", "ruby"])
    subprocess.run(["pkill", "-9", "-f", "gzclient"])
    subprocess.run(["pkill", "-9", "-f", "gazebo"])
    pass
