#!/usr/bin/env python3
"""
A script to outline the fundamentals of the moveit_py motion planning API.
"""

import math
import time

# generic ros libraries
import rclpy
from geometry_msgs.msg import PoseStamped

# set constraints message
from moveit.core.kinematic_constraints import construct_joint_constraint

# moveit python library
from moveit.core.robot_state import RobotState
from moveit.planning import MoveItPy, MultiPipelinePlanRequestParameters
from rclpy.logging import get_logger


def make_pose(frame_id, x, y, z, qx, qy, qz, qw):
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    pose.pose.orientation.x = qx
    pose.pose.orientation.y = qy
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw
    return pose


def plan_and_execute(
    robot,
    planning_component,
    logger,
    single_plan_parameters=None,
    multi_plan_parameters=None,
    sleep_time=0.0,
):
    """Helper function to plan and execute a motion."""
    # plan to goal
    logger.info("Planning trajectory")
    if multi_plan_parameters is not None:
        plan_result = planning_component.plan(
            multi_plan_parameters=multi_plan_parameters
        )
    elif single_plan_parameters is not None:
        plan_result = planning_component.plan(
            single_plan_parameters=single_plan_parameters
        )
    else:
        plan_result = planning_component.plan()

    # execute the plan
    if plan_result:
        logger.info("Executing plan")
        robot_trajectory = plan_result.trajectory
        robot.execute(robot_trajectory, controllers=[])
    else:
        logger.error("Planning failed")

    time.sleep(sleep_time)


def plan_and_execute_hand_motion(panda, hand, logger, target_state="open"):
    hand.set_start_state_to_current_state()
    hand.set_goal_state(configuration_name=target_state)
    plan_and_execute(panda, hand, logger, sleep_time=3.0)


def plan_and_execute_arm_motion_joint_config(
    panda, robot_state, panda_arm, logger, target_joints
):
    ## We place it above the cube
    panda_arm.set_start_state_to_current_state()
    robot_state.joint_positions = target_joints
    joint_constraint = construct_joint_constraint(
        robot_state=robot_state,
        joint_model_group=panda.get_robot_model().get_joint_model_group("panda_arm"),
    )
    panda_arm.set_goal_state(motion_plan_constraints=[joint_constraint])
    plan_and_execute(panda, panda_arm, logger, sleep_time=3.0)


def plan_and_execute_arm_motion(
    panda, robot_state, panda_arm, logger, target_state="ready"
):
    ## We place it above the cube
    panda_arm.set_start_state_to_current_state()
    panda_arm.set_goal_state(configuration_name=target_state)
    plan_and_execute(panda, panda_arm, logger, sleep_time=3.0)


def main():
    ###################################################################
    # MoveItPy Setup
    ###################################################################

    rclpy.init()
    logger = get_logger("moveit_py_demo_gripper_pick")

    # instantiate MoveItPy instance and get planning component

    panda = MoveItPy(
        node_name="moveit_py_demo_gripper_pick",
        name_space="/panda",
        config_dict=None,  # or your config dictionary
        provide_planning_service=True,
    )
    panda_arm = panda.get_planning_component("panda_arm")
    hand = panda.get_planning_component("hand")

    # instantiate a RobotState instance using the current robot model
    robot_model = panda.get_robot_model()
    robot_state = RobotState(robot_model)

    logger.info("MoveItPy instance created")

    #######################################################
    # Setup
    #######################################################

    joint_values_start_above_cube = {
        "panda_joint1": math.radians(99),
        "panda_joint2": math.radians(-97),
        "panda_joint3": math.radians(-92),
        "panda_joint4": math.radians(-130),
        "panda_joint5": math.radians(-97),
        "panda_joint6": math.radians(87),
        "panda_joint7": math.radians(96),
    }
    joint_values_start_on_cube = {
        "panda_joint1": math.radians(99),
        "panda_joint2": math.radians(-101),
        "panda_joint3": math.radians(-101),
        "panda_joint4": math.radians(-127),
        "panda_joint5": math.radians(-107),
        "panda_joint6": math.radians(94),
        "panda_joint7": math.radians(95),
    }

    joint_values_end_on_cube = {
        "panda_joint1": math.radians(17),
        "panda_joint2": math.radians(-90),
        "panda_joint3": math.radians(-94),
        "panda_joint4": math.radians(-116),
        "panda_joint5": math.radians(-92),
        "panda_joint6": math.radians(93),
        "panda_joint7": math.radians(0),
    }

    ###########################################################################
    # Execution
    ###########################################################################

    logger.info("::::: Starting scene. :::::")

    # We make sure the panda is at the ready state
    plan_and_execute_arm_motion(
        panda, robot_state, panda_arm, logger, target_state="ready"
    )
    time.sleep(5)

    # We open the gripper
    plan_and_execute_hand_motion(panda, hand, logger, "open")

    ## We place it above the cube
    plan_and_execute_arm_motion_joint_config(
        panda, robot_state, panda_arm, logger, joint_values_start_above_cube
    )

    # We place it on the cube, ready to grab
    plan_and_execute_arm_motion_joint_config(
        panda, robot_state, panda_arm, logger, joint_values_start_on_cube
    )
    time.sleep(5)

    # We close the gripper
    plan_and_execute_hand_motion(panda, hand, logger, "close")

    # We move to the goal pose
    plan_and_execute_arm_motion_joint_config(
        panda, robot_state, panda_arm, logger, joint_values_end_on_cube
    )

    # We open the gripper and release cube
    plan_and_execute_hand_motion(panda, hand, logger, "open")

    # We return to the original pose
    plan_and_execute_arm_motion(
        panda, robot_state, panda_arm, logger, target_state="ready"
    )


if __name__ == "__main__":
    main()
