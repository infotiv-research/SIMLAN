#!/usr/bin/env python3
"""
Simple move/grip demo for UR10 + Robotiq 2F-140 using MoveItPy with PoseStamped targets.
"""

import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from moveit.planning import MoveItPy
from rclpy.logging import get_logger
import tf2_ros
from tf2_geometry_msgs import do_transform_pose
from rclpy.qos import QoSProfile, DurabilityPolicy
from visualization_msgs.msg import Marker


# ---------------------------------------------------------------------------
# Pose constants  (gazebo_world frame)
# ---------------------------------------------------------------------------
def rpy_deg_to_quat(roll_deg, pitch_deg, yaw_deg):
    """Convert roll/pitch/yaw in degrees to a quaternion dict."""
    roll = math.radians(roll_deg)
    pitch = math.radians(pitch_deg)
    yaw = math.radians(yaw_deg)

    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    qw = cr * cp * cy + sr * sp * sy

    return {"qx": qx, "qy": qy, "qz": qz, "qw": qw}


# Orientation: gripper pointing straight down.
GRIPPER_DOWN_ORIENTATION = rpy_deg_to_quat(180.0, 0.0, 0.0)

# The Robotiq 2F-140 extends ~0.24 beyond tool0.
GRIPPER_OFFSET_Z = 0.24

# Move-to-grip location (gazebo_world frame)
GRIP_POSE = {
    "x": 21.29,
    "y": -19.39,
    "z": 0.60 + GRIPPER_OFFSET_Z,
    **GRIPPER_DOWN_ORIENTATION,
}

# Move-away location (gazebo_world frame)
RETRACT_POSE = {
    "x": 21.10,
    "y": -19.70,
    "z": 0.80 + GRIPPER_OFFSET_Z,
    **GRIPPER_DOWN_ORIENTATION,
}


# ---------------------------------------------------------------------------
# Helper functions
# ---------------------------------------------------------------------------
def make_pose(frame_id, x, y, z, qx, qy, qz, qw):
    """Build a PoseStamped message."""
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


def transform_pose(tf_buffer, pose_stamped, target_frame, logger, timeout_sec=2.0):
    """Transform a PoseStamped into the target frame using TF."""
    try:
        transform = tf_buffer.lookup_transform(
            target_frame,
            pose_stamped.header.frame_id,
            rclpy.time.Time(),
            rclpy.duration.Duration(seconds=timeout_sec),
        )
    except Exception as exc:
        logger.error(f"Failed to lookup transform to {target_frame}: {exc}")
        return None
    transformed_pose = do_transform_pose(pose_stamped.pose, transform)
    out = PoseStamped()
    out.header.frame_id = target_frame
    out.header.stamp = pose_stamped.header.stamp
    out.pose = transformed_pose
    return out


def wait_for_transform(
    tf_buffer,
    node,
    target_frame,
    source_frame,
    logger,
    timeout_sec=30.0,
    retry_interval_sec=0.5,
):
    """Wait for a transform to become available, retrying until timeout."""
    chain_desc = f"{source_frame} -> {target_frame}"
    logger.info(f"Waiting for TF chain: {chain_desc}")
    end_time = node.get_clock().now() + rclpy.duration.Duration(seconds=timeout_sec)
    while node.get_clock().now() < end_time:
        if tf_buffer.can_transform(
            target_frame,
            source_frame,
            rclpy.time.Time(),
            timeout=rclpy.duration.Duration(seconds=retry_interval_sec),
        ):
            logger.info(f"TF chain available: {chain_desc}")
            return True
        time.sleep(retry_interval_sec)
    logger.error(f"Timeout waiting for TF chain: {chain_desc}")
    logger.error("Available frames:\n" + tf_buffer.all_frames_as_string())
    return False


def log_pose(logger, label, pose_stamped):
    pos = pose_stamped.pose.position
    ori = pose_stamped.pose.orientation
    logger.info(
        f"{label} frame={pose_stamped.header.frame_id} "
        f"pos=({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f}) "
        f"quat=({ori.x:.3f}, {ori.y:.3f}, {ori.z:.3f}, {ori.w:.3f})"
    )


def publish_marker(marker_pub, frame_id, pose_stamped, marker_id, color):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = pose_stamped.header.stamp
    marker.ns = "ur10_pick_and_place_targets"
    marker.id = marker_id
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.pose = pose_stamped.pose
    marker.scale.x = 0.05
    marker.scale.y = 0.05
    marker.scale.z = 0.05
    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    marker.color.a = 0.9
    marker_pub.publish(marker)


def plan_and_execute(robot, planning_component, logger, sleep_time=0.0, max_retries=3):
    """Plan and execute a trajectory, with retries on planning failure."""
    for attempt in range(1, max_retries + 1):
        logger.info(f"Planning trajectory (attempt {attempt}/{max_retries})")
        plan_result = planning_component.plan()
        if plan_result:
            logger.info("Executing plan")
            robot_trajectory = plan_result.trajectory
            robot.execute(robot_trajectory, controllers=[])
            time.sleep(sleep_time)
            return True
        logger.warning(f"Planning failed (attempt {attempt}/{max_retries})")
        if attempt < max_retries:
            planning_component.set_start_state_to_current_state()
            time.sleep(1.0)
    logger.error(f"Planning failed after {max_retries} attempts")
    time.sleep(sleep_time)
    return False


def move_arm_to_pose(robot, arm, logger, pose):
    """Move the arm so that tool0 reaches the given PoseStamped."""
    arm.set_start_state_to_current_state()
    arm.set_goal_state(pose_stamped_msg=pose, pose_link="tool0")
    return plan_and_execute(robot, arm, logger, sleep_time=3.0)


def move_arm_to_named(robot, arm, logger, config_name):
    """Move the arm to a named SRDF configuration (e.g. 'home')."""
    arm.set_start_state_to_current_state()
    arm.set_goal_state(configuration_name=config_name)
    return plan_and_execute(robot, arm, logger, sleep_time=3.0)


def move_gripper(robot, gripper, logger, state_name):
    """Open or close the gripper via named state ('open' / 'close')."""
    gripper.set_start_state_to_current_state()
    gripper.set_goal_state(configuration_name=state_name)
    return plan_and_execute(robot, gripper, logger, sleep_time=3.0)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    rclpy.init()
    logger = get_logger("ur10_pick_and_place")

    # --- MoveItPy setup ---------------------------------------------------
    ur10 = MoveItPy(
        node_name="moveit_py",
        name_space="/ur10",
        config_dict=None,
        provide_planning_service=True,
    )
    arm = ur10.get_planning_component("ur_manipulator")
    gripper = ur10.get_planning_component("gripper")
    logger.info("MoveItPy instance created")

    logger.info("Waiting for controllers to become available...")
    time.sleep(5)

    # --- Build pose targets ------------------------------------------------
    tf_node = rclpy.create_node(
        "ur10_pick_and_place_tf",
        namespace="/ur10",
        parameter_overrides=[rclpy.parameter.Parameter("use_sim_time", value=True)],
    )
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer, tf_node, spin_thread=True)

    marker_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    marker_pub = tf_node.create_publisher(Marker, "ur10_target_markers", marker_qos)

    if not wait_for_transform(tf_buffer, tf_node, "base_link", "gazebo_world", logger):
        return

    grip_pose_world = make_pose("gazebo_world", **GRIP_POSE)
    retract_pose_world = make_pose("gazebo_world", **RETRACT_POSE)

    grip_pose = transform_pose(tf_buffer, grip_pose_world, "base_link", logger)
    if grip_pose is None:
        logger.error("Failed to transform grip pose to base_link. Aborting sequence.")
        return
    retract_pose = transform_pose(tf_buffer, retract_pose_world, "base_link", logger)
    if retract_pose is None:
        logger.error(
            "Failed to transform retract pose to base_link. Aborting sequence."
        )
        return

    log_pose(logger, "Grip target (world)", grip_pose_world)
    log_pose(logger, "Grip target (base_link)", grip_pose)
    log_pose(logger, "Retract target (world)", retract_pose_world)
    log_pose(logger, "Retract target (base_link)", retract_pose)

    publish_marker(marker_pub, "gazebo_world", grip_pose_world, 1, (0.0, 1.0, 0.0))
    publish_marker(marker_pub, "gazebo_world", retract_pose_world, 2, (1.0, 0.5, 0.0))

    # --- Execute simple sequence -------------------------------------------
    logger.info("===== Starting UR10 move/grip sequence =====")

    # 1. Go to home
    logger.info("Step 1: Moving to home")
    if not move_arm_to_named(ur10, arm, logger, "home"):
        logger.error("Failed to move to home. Aborting sequence.")
        return
    time.sleep(2)

    # 2. Open gripper
    logger.info("Step 2: Opening gripper")
    if not move_gripper(ur10, gripper, logger, "open"):
        logger.error("Failed to open gripper. Aborting sequence.")
        return

    # 3. Move to grip pose
    logger.info("Step 3: Moving to grip pose")
    if not move_arm_to_pose(ur10, arm, logger, grip_pose):
        logger.error("Failed to move to grip pose. Aborting sequence.")
        return
    time.sleep(1)

    # 4. Close gripper
    logger.info("Step 4: Closing gripper")
    if not move_gripper(ur10, gripper, logger, "close"):
        logger.error("Failed to close gripper. Aborting sequence.")
        return
    time.sleep(1)

    # 5. Move to retract pose
    logger.info("Step 5: Moving to retract pose")
    if not move_arm_to_pose(ur10, arm, logger, retract_pose):
        logger.error("Failed to move to retract pose. Aborting sequence.")
        return
    time.sleep(1)

    # 6. Open gripper
    logger.info("Step 6: Opening gripper")
    if not move_gripper(ur10, gripper, logger, "open"):
        logger.error("Failed to open gripper. Aborting sequence.")
        return
    time.sleep(1)

    # 7. Return to home
    logger.info("Step 7: Returning to home")
    if not move_arm_to_named(ur10, arm, logger, "home"):
        logger.error("Failed to return to home. Aborting sequence.")
        return

    logger.info("===== UR10 move/grip sequence complete =====")


if __name__ == "__main__":
    main()
