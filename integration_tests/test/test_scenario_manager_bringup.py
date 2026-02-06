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
import pytest


sys.path.append(".")


@launch_pytest.fixture(autouse=True)
def launch_description():

    ######## Arguments ########
    world_setup = "default"
    log_level = "error"
    robots = '[{ "namespace": "robot_agent_1", "initial_pose_x":"10.0", "initial_pose_y":"1.0", "robot_type":"pallet_truck", "aruco_id":"1" }]'
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
    jackal_proc = ExecuteProcess(
        cmd=[
            "ros2",
            "launch",
            "dyno_jackal_bringup",
            "sim.launch.py",
            f"log_level:={log_level}",
        ],
        output="screen",
    )

    scenario_manager_proc = ExecuteProcess(
        cmd=[
            "ros2",
            "launch",
            "scenario_manager",
            "scenario_manager.launch.py",
            f"log_level:={log_level}",
        ],
        output="screen",
    )
    scenario_execution_proc = ExecuteProcess(
        cmd=[
            "ros2",
            "launch",
            "scenario_execution_ros",
            "scenario_launch.py",
            f"scenario:=/home/ros/src/simulation/scenario_manager/scenarios/case1.osc",
            f"log_level:={log_level}",
        ],
        output="screen",
    )
    ######## Test execution ########
    rclpy.init()

    yield launch.LaunchDescription(
        [
            sim_proc,
            multiple_robot_proc,
            jackal_proc,
            scenario_manager_proc,
            scenario_execution_proc,
            launch_pytest.actions.ReadyToTest(),
        ]
    )
    rclpy.shutdown()


@pytest.mark.launch(fixture=launch_description)
async def test_scenario_manager_bringup_Startup_Nodes_and_topics_should_be_visible():
    print(
        "STARTING TEST: scenario_manager_bringup_Startup_Nodes_and_topics_should_be_visible "
    )
    time.sleep(5)

    ###### SETUP  ######
    num_retries = 5
    target_topics = [
        "/tf",
        "/scenario_manager/ttc",
        "/jackal/odom_ground_truth",
        "/jackal/scenario_vel",
        "/jackal/velocity_controller/cmd_vel",
        "/robot_agent_1/odom_ground_truth",
        "/robot_agent_1/scenario_vel",
        "/robot_agent_1/velocity_controller/cmd_vel",
    ]
    target_nodes = [
        "/ros_gz_bridge",
        "/jackal/twist_mux",
        "/jackal/twist_stamper_node",
        "/jackal/velocity_controller",
        "/jackal/controller_manager",
        "/jackal/gz_ros_control",
        "/jackal/joint_state_broadcaster",
        "/jackal/robot_state_publisher",
        "/robot_agent_1/twist_mux",
        "/robot_agent_1/twist_stamper_node",
        "/robot_agent_1/velocity_controller",
        "/robot_agent_1/controller_manager",
        "/robot_agent_1/gz_ros_control",
        "/robot_agent_1/joint_state_broadcaster",
        "/robot_agent_1/robot_state_publisher",
        "/scenario_manager/collision_action_server",
        "/scenario_manager/safety_stop",
        "/scenario_manager/set_speed_action_server",
        "/scenario_manager/teleport_action_server",
        "/scenario_manager/ttc",
        "/scenario_execution",
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
