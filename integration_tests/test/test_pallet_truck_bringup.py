import os
import sys
import time

import launch_pytest
import launch_testing
import pytest
import rclpy
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from test_node import MakeTestNode

sys.path.append(".")


@launch_pytest.fixture(autouse=True)
def launch_description():
    ######## Arguments ########
    world_setup = "default"
    log_level = "error"
    robots = '[{ "namespace": "robot_agent_1", "initial_pose_x":"10.0", "initial_pose_y":"1.0", "robot_type":"pallet_truck", "aruco_id":"1" }]'
    ######## Launch-files / Processes ########
    os.environ["ROS_DOMAIN_ID"] = str(os.getpid() % 232)

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
    multiple_robot_proc = ExecuteProcess(
        cmd=[
            "ros2",
            "launch",
            "pallet_truck_bringup",
            "multiple_robot_spawn.launch.py",
            f"log_level:={log_level}",
            f"robots:={robots}",
        ],
        output="screen",
    )

    ######## Test execution ########
    rclpy.init()

    yield LaunchDescription(
        [
            sim_proc,
            multiple_robot_proc,
            launch_testing.util.KeepAliveProc(),
            launch_testing.actions.ReadyToTest(),
        ]
    )
    rclpy.shutdown()


@pytest.mark.launch(fixture=launch_description)
async def test_sim_and_multiple_robots_bringup_Startup_Nodes_and_topics_should_be_visible():
    print(
        "STARTING TEST: sim_and_multiple_robots_bringup_Startup_Nodes_and_topics_should_be_visible "
    )
    time.sleep(20)

    ###### SETUP  ######
    num_retries = 5
    target_topics = [
        "/robot_agent_1/key_vel",
        "/robot_agent_1/robot_description",
        "/robot_agent_1/velocity_controller/cmd_vel",
        "/tf",
    ]
    target_nodes = [
        "/robot_agent_1/twist_mux",
        "/robot_agent_1/twist_stamper_node",
        "/ros_gz_bridge",
    ]

    ###### NODE SETUP ######
    test_node = MakeTestNode()

    actual_nodes, actual_topics = [], []

    ###### EXECUTION ######
    for _ in range(num_retries):
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
        print("trying again..")
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
    # --- Cleanup command ---
    print("Testing after shutdown command.")
    pass
