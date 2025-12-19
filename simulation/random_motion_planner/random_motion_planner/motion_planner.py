#!/usr/bin/env python3
from multiprocessing.util import get_logger
from venv import logger
import rclpy
import numpy as np
from rclpy.node import Node
from moveit.core.robot_state import RobotState
from moveit.planning import MoveItPy
import random
import time
import json
import os
from camera_system.srv import CaptureMotionStatus
import random
import sys
from moveit.core.kinematic_constraints import construct_joint_constraint
from std_msgs.msg import String

from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

random.seed(int(time.time()))
NUM_MOTION = 10000
sys.path.append(".")
import humanoid_utility.humanoid_config as humanoid_config


class WholeBodyMotionPlanner(Node):
    def __init__(self):
        super().__init__("random_motion_planner")
        self.declare_parameter("namespace", "humanoid_1")
        self.namespace = (
            self.get_parameter("namespace").get_parameter_value().string_value
        )

        self.logger = get_logger()  # TODO redundant?
        self.declare_parameter("run_random_generate", False)
        run_random_generate = (
            self.get_parameter("run_random_generate").get_parameter_value().bool_value
        )

        self.declare_parameter("output_dir", "DATASET")
        self.output_dir = (
            self.get_parameter("output_dir").get_parameter_value().string_value
        )

        self.declare_parameter("motion_dir", self.output_dir + "/motion_data")
        self.motion_dir = self.get_parameter("motion_dir").value
        os.makedirs(self.motion_dir, exist_ok=True)
        self.get_logger().info(f"Motion directory: {self.motion_dir}")

        # Client for camera synchronization, only for random motion
        if run_random_generate:
            self.cli = self.create_client(CaptureMotionStatus, "capture_motion")
            while not self.cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().info("Waiting for service...")
            self.req = CaptureMotionStatus.Request()

        self.declare_parameter("max_velocity", 0.8)
        self.declare_parameter("max_acceleration", 0.8)
        self.declare_parameter("motion_amplitude", 0.8)
        self.joint_limit = 0.8  # TO AVOID FULL RANGE
        self.declare_parameter("planner_id", "RRTConnectkConfigDefault")

        self.max_velocity = self.get_parameter("max_velocity").value
        self.max_acceleration = self.get_parameter("max_acceleration").value
        self.motion_amplitude = self.get_parameter("motion_amplitude").value
        self.planner_id = self.get_parameter("planner_id").value

        self.movable_joint_names = humanoid_config.movable_joint_names
        self.movable_joint_upper = humanoid_config.movable_joint_upper
        self.movable_joint_lower = humanoid_config.movable_joint_lower
        # Set all joints to 0.0 as initial position
        self.movable_joint_positions = [0] * len(self.movable_joint_names)

        self.current_motion = None
        self.current_motion_id = 9999
        self.group_name = "support_whole_body"
        self.robot = MoveItPy(
            node_name="moveit_py",
            name_space=self.namespace,
            config_dict=None,  # or your config dictionary
            provide_planning_service=True,
        )
        self.pc = self.robot.get_planning_component(self.group_name)
        self.model = self.robot.get_robot_model()
        # self.state = RobotState(self.model)
        self.psm = self.robot.get_planning_scene_monitor()
        self.state = RobotState(self.model)
        self.planner_id = self.planner_id
        self.max_velocity = self.max_velocity
        self.max_acceleration = self.max_acceleration
        time.sleep(1.0)

        # Two modes, either start generating random motions or we simply run the node and listen to new joint state requests.
        if run_random_generate:
            self.get_logger().info("Random Motion")
            for _ in range(NUM_MOTION):
                self.execute_random_motion()
        else:
            qos_profile = QoSProfile(
                depth=10,
                reliability=ReliabilityPolicy.BEST_AVAILABLE,
                durability=DurabilityPolicy.VOLATILE,
            )
            self.motion_request_subscriber = self.create_subscription(
                String,
                f"{self.namespace}/execute_motion",
                self.execute_motion_callback,
                qos_profile,
            )

    def execute_random_motion(self):
        self.current_motion_id = random.randint(1000000000, 9999999999)
        # self.state = RobotState(self.model)
        self.positions = self.random_joint_positions()
        self.joint_values = dict(zip(self.movable_joint_names, self.positions))
        print(
            f" ===================\n{self.movable_joint_names}/{self.positions}/{self.joint_values}\n ==================="
        )
        self.state.joint_positions = self.joint_values
        self.joint_constraints = construct_joint_constraint(
            robot_state=self.state,
            joint_model_group=self.robot.get_robot_model().get_joint_model_group(
                "support_whole_body"
            ),
        )
        self.pc.set_goal_state(motion_plan_constraints=[self.joint_constraints])
        time.sleep(2)
        self.plan_and_execute_motion()

    def execute_motion_callback(self, msg):
        raw = msg.data.strip()

        # 1. Check if the message is a file path
        if os.path.isfile(raw):
            self.get_logger().info(f"Loading motion from file: {raw}")
            try:
                with open(raw, "r") as f:
                    data = json.load(f)
            except Exception as e:
                self.get_logger().error(f"Failed to load JSON file: {e}")
                return

        else:
            self.get_logger().info("Parsing motion from JSON string")
            try:
                data = json.loads(raw)
            except json.JSONDecodeError:
                self.get_logger().error("Received invalid JSON or invalid filepath")
                return

        # 2. Convert dict to joint names & positions
        joint_names = list(data.keys())
        joint_positions = list(data.values())

        self.joint_values = dict(zip(joint_names, joint_positions))
        self.state.joint_positions = self.joint_values

        # 3. MoveIt2 constraint setup
        self.joint_constraints = construct_joint_constraint(
            robot_state=self.state,
            joint_model_group=self.robot.get_robot_model().get_joint_model_group(
                "support_whole_body"
            ),
        )

        self.pc.set_goal_state(motion_plan_constraints=[self.joint_constraints])

        time.sleep(1)
        self.plan_and_execute_fromfile()

    def random_joint_positions(self):
        positions = []
        for i in range(len(self.movable_joint_names)):
            positions.append(
                random.uniform(
                    self.movable_joint_lower[i] * self.joint_limit,
                    self.movable_joint_upper[i] * self.joint_limit,
                )
            )
        return positions

    def save_current_motion(self):
        # # Save current motion as [motion ID]_motion.json
        current_motion = self.joint_values

        os.makedirs(self.motion_dir, exist_ok=True)
        json_filename = os.path.join(
            self.motion_dir, f"{self.current_motion_id}_motion.json"
        )
        with open(json_filename, "w") as f:
            json.dump(current_motion, f, indent=2)
        self.get_logger().info(f"Motion sequence saved to {json_filename}")

    def send_request_camera(self):
        # Send a request to the camera viewer service server to capture the pose for the current motion
        self.req.bodymotionid = self.current_motion_id
        future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)
        r = future.result().pose_data_capture
        return r

    def plan_and_execute_motion(self, sleep_time=5.0):
        logger = self.logger
        pc = self.pc
        """Helper function to plan and execute a motion (from example)."""
        logger.info("Planning trajectory")
        # pc.set_start_state_to_current_state()
        # pc.set_goal_state(robot_state=self.state)
        plan_result = pc.plan()
        if plan_result and getattr(plan_result, "trajectory", None) is not None:
            logger.info("Executing plan")
            self.robot.execute(plan_result.trajectory, controllers=[])
            self.get_logger().info(f"Motion is successful")
            print("=== Current joint states after execution ===")
            req = self.send_request_camera()
            if req:
                self.save_current_motion()
            else:
                self.get_logger().info(f"Pose not detected")

        else:
            logger.error("Planning failed")

        time.sleep(sleep_time)

    def plan_and_execute_fromfile(self, sleep_time=5.0):
        logger = self.logger
        pc = self.pc
        """Helper function to plan and execute a motion (from example)."""
        logger.info("Planning trajectory")
        plan_result = pc.plan()
        if plan_result and getattr(plan_result, "trajectory", None) is not None:
            logger.info("Executing plan")
            self.robot.execute(plan_result.trajectory, controllers=[])
            self.get_logger().info(f"Motion is successful")
            print("=== Current joint states after execution ===")
        else:
            logger.error("Planning failed")

        time.sleep(sleep_time)


def main(args=None):
    rclpy.init(args=args)
    planner = WholeBodyMotionPlanner()

    rclpy.spin(planner)
    planner.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
