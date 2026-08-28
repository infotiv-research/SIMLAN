import json
import math
import re
from copy import deepcopy
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
from ros_gz_interfaces.srv import SetEntityPose
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Empty, String
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Twist

from visualize_real_data.camera_polygons_config import (
    CAMERA_COLOR_ENUM,
    CAMERA_CORRIDOR_POLYGONS,
    CAMERA_POLYGONS,
    COLOR_ENUM,
)


def _default_frame_position():
    default = {"x": 15.35, "y": 5.7, "z": -0.2}
    try:
        yaml_path = Path(
            get_package_share_directory("visualize_real_data"), "config", "params.yaml"
        )
        with open(yaml_path, "r") as f:
            data = yaml.safe_load(f) or {}
        shared = data.get("shared", {})
        frame_position = shared.get("frame_position", {})
        return {
            "x": float(frame_position.get("x", default["x"])),
            "y": float(frame_position.get("y", default["y"])),
            "z": float(frame_position.get("z", default["z"])),
        }
    except Exception:
        return default


class SafetyNode(Node):
    def __init__(self):
        super().__init__("safety_node")
        default_frame_position = _default_frame_position()

        self.declare_parameter("frame_id", "real_data")
        self.declare_parameter("odd_topic", "/visualize_real_data/odd_detections")
        self.declare_parameter("entity_topic", "/visualize_real_data/entity_topic")
        self.declare_parameter("loop_topic", "/visualize_real_data/loop")
        self.declare_parameter("allowed_ids", Parameter.Type.INTEGER_ARRAY)
        self.declare_parameter("jackal_groundtruth_topic", "/jackal/odom_ground_truth")
        self.declare_parameter("cmd_vel_topic", "/jackal/safety_vel")
        self.declare_parameter("marker_topic", "/visualize_real_data/camera_polygons")
        self.declare_parameter(
            "corridor_marker_topic", "/visualize_real_data/corridor_polygons"
        )
        self.declare_parameter(
            "lookahead_marker_topic", "/visualize_real_data/lookahead_points"
        )
        self.declare_parameter("lookahead_constant_speed", 3.0)
        self.declare_parameter("marker_ns", "camera_safety")
        self.declare_parameter("line_width", 0.1)
        self.declare_parameter("teleport_on_start", True)
        self.declare_parameter("teleport_service", "/world/default/set_pose")
        self.declare_parameter("teleport_entity_name", "jackal")
        # start_pose order: [x, y, z, yaw]
        self.declare_parameter("start_pose", [0.0, 0.0, 0.05, 0.0])
        self.declare_parameter("frame_position", json.dumps(default_frame_position))

        # Store frame position offset
        self.frame_position = json.loads(
            self.get_parameter("frame_position").get_parameter_value().string_value
        )

        self.camera_polygons = deepcopy(CAMERA_POLYGONS)
        self.corridor_polygons = deepcopy(CAMERA_CORRIDOR_POLYGONS)
        self.COLOR_ENUM = deepcopy(COLOR_ENUM)
        self.camera_color_enum = deepcopy(CAMERA_COLOR_ENUM)

        self.frame_id = (
            self.get_parameter("frame_id").get_parameter_value().string_value
        )
        self.odd_topic = (
            self.get_parameter("odd_topic").get_parameter_value().string_value
        )
        self.entity_topic = (
            self.get_parameter("entity_topic").get_parameter_value().string_value
        )
        self.loop_topic = (
            self.get_parameter("loop_topic").get_parameter_value().string_value
        )
        allowed_ids_param = self.get_parameter_or(
            "allowed_ids",
            Parameter("allowed_ids", type_=Parameter.Type.INTEGER_ARRAY, value=[]),
        )
        self.allowed_ids = set(
            allowed_ids_param.get_parameter_value().integer_array_value
        )
        self.jackal_groundtruth_topic = (
            self.get_parameter("jackal_groundtruth_topic")
            .get_parameter_value()
            .string_value
        )
        self.cmd_vel_topic = (
            self.get_parameter("cmd_vel_topic").get_parameter_value().string_value
        )
        self.marker_topic = (
            self.get_parameter("marker_topic").get_parameter_value().string_value
        )
        self.corridor_marker_topic = (
            self.get_parameter("corridor_marker_topic")
            .get_parameter_value()
            .string_value
        )
        self.lookahead_marker_topic = (
            self.get_parameter("lookahead_marker_topic")
            .get_parameter_value()
            .string_value
        )
        self.lookahead_constant_speed = (
            self.get_parameter("lookahead_constant_speed")
            .get_parameter_value()
            .double_value
        )
        self.marker_ns = (
            self.get_parameter("marker_ns").get_parameter_value().string_value
        )
        self.line_width = (
            self.get_parameter("line_width").get_parameter_value().double_value
        )
        self.teleport_on_start = (
            self.get_parameter("teleport_on_start").get_parameter_value().bool_value
        )
        self.teleport_service = (
            self.get_parameter("teleport_service").get_parameter_value().string_value
        )
        self.teleport_entity_name = (
            self.get_parameter("teleport_entity_name")
            .get_parameter_value()
            .string_value
        )
        start_pose = list(
            self.get_parameter("start_pose").get_parameter_value().double_array_value
        )
        if len(start_pose) < 4:
            self.get_logger().warn(
                "Parameter 'start_pose' must have 4 values [x, y, z, yaw]. Falling back to defaults."
            )
            start_pose = [0.0, 0.0, 0.05, 0.0]
        self.start_pose_x = float(start_pose[0])
        self.start_pose_y = float(start_pose[1])
        self.start_pose_z = float(start_pose[2])
        self.start_pose_yaw = float(start_pose[3])
        self.loading_collision_box_size = 2.0
        self.safety_cmd_period = 0.1
        self.jackal_position = None
        self.jackal_yaw = None
        self.jackal_velocity = None
        self.last_stop_state = None
        self.last_stop_reason = ""
        self.replay_started = False
        self.safety_cmd_active = False
        self.odd_cameras = set()
        self.entity_status_by_id = {}
        self.entity_position_by_id = {}
        self.entity_route_remaining_by_id = {}

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.marker_pub = self.create_publisher(MarkerArray, self.marker_topic, qos)
        self.corridor_marker_pub = self.create_publisher(
            MarkerArray, self.corridor_marker_topic, qos
        )
        self.lookahead_marker_pub = self.create_publisher(
            MarkerArray, self.lookahead_marker_topic, qos
        )
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, qos)
        self.teleport_client = self.create_client(SetEntityPose, self.teleport_service)
        self.odd_sub = self.create_subscription(
            String, self.odd_topic, self._odd_cb, qos
        )
        self.entity_sub = self.create_subscription(
            MarkerArray,
            self.entity_topic,
            self._entity_cb,
            qos,
        )
        self.loop_sub = self.create_subscription(
            Empty, self.loop_topic, self._loop_cb, qos
        )
        self.jackal_gt_sub = self.create_subscription(
            Odometry,
            self.jackal_groundtruth_topic,
            self._jackal_groundtruth_cb,
            qos,
        )
        # Keep markers visible even when odd_detections is temporarily silent.
        self.render_timer = self.create_timer(0.5, self._publish_visualization)
        self.safety_cmd_timer = self.create_timer(
            self.safety_cmd_period, self._publish_safety_cmd
        )

        self.get_logger().info(
            f"Safety node ready. Listening on '{self.odd_topic}' and '{self.jackal_groundtruth_topic}', publishing markers on '{self.marker_topic}'"
        )
        self.get_logger().info(f"Publishing safety cmd_vel on '{self.cmd_vel_topic}'")
        self.get_logger().info(f"Listening for loop events on '{self.loop_topic}'")
        self.get_logger().info(
            "Waiting for first odd_detections message before startup teleport and safety cmd publishing"
        )

    def _loop_cb(self, _msg: Empty):
        if not self.replay_started:
            return
        self.get_logger().info(
            "Replay loop event received. Teleporting jackal to startup pose."
        )
        self.safety_cmd_active = False
        self._teleport_jackal_to_start()

    def _teleport_jackal_to_start(self):
        if not self.teleport_on_start:
            self.safety_cmd_active = True
            return

        if not self.teleport_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn(
                f"Teleport service '{self.teleport_service}' unavailable. Skipping startup teleport."
            )
            self.safety_cmd_active = True
            return

        req = SetEntityPose.Request()
        req.entity.name = self.teleport_entity_name
        req.entity.type = 1  # MODEL
        req.pose.position.x = float(self.start_pose_x)
        req.pose.position.y = float(self.start_pose_y)
        req.pose.position.z = float(self.start_pose_z)
        req.pose.orientation.x = 0.0
        req.pose.orientation.y = 0.0
        req.pose.orientation.z = math.sin(float(self.start_pose_yaw) * 0.5)
        req.pose.orientation.w = math.cos(float(self.start_pose_yaw) * 0.5)

        future = self.teleport_client.call_async(req)
        future.add_done_callback(self._on_startup_teleport_done)

    def _on_startup_teleport_done(self, future):
        try:
            resp = future.result()
            if resp and resp.success:
                self.get_logger().info(
                    f"Teleported '{self.teleport_entity_name}' to startup pose x={self.start_pose_x:.2f}, y={self.start_pose_y:.2f}, z={self.start_pose_z:.2f}, yaw={self.start_pose_yaw:.2f}"
                )
            else:
                self.get_logger().warn(
                    f"Startup teleport failed for '{self.teleport_entity_name}'"
                )
        except Exception as exc:
            self.get_logger().warn(f"Startup teleport error: {exc}")
        finally:
            self.safety_cmd_active = True

    def _jackal_groundtruth_cb(self, msg):
        ox = float(self.frame_position.get("x", 0.0))
        oy = float(self.frame_position.get("y", 0.0))
        self.jackal_position = (
            float(msg.pose.pose.position.x) - ox,
            float(msg.pose.pose.position.y) - oy,
        )
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.jackal_yaw = math.atan2(siny_cosp, cosy_cosp)
        self.jackal_velocity = (
            float(msg.twist.twist.linear.x),
            float(msg.twist.twist.linear.y),
            float(msg.twist.twist.linear.z),
            float(msg.twist.twist.angular.z),
        )

    def _odd_cb(self, msg):
        if not self.replay_started:
            self.replay_started = True
            self.get_logger().info(
                "First odd_detections message received. Teleporting jackal to startup pose."
            )
            self._teleport_jackal_to_start()

        self.odd_cameras = self._extract_cameras_with_odds(msg.data)
        self._publish_visualization()

    def _publish_visualization(self):
        lookahead_points = self._compute_lookahead_points(
            horizon_seconds=3.0, step_seconds=0.5
        )
        future_inside_cameras = self._get_future_inside_camera_names(lookahead_points)
        occupied_corridors = self._compute_occupied_corridors()
        future_inside_corridors = self._get_future_inside_corridor_names(
            lookahead_points
        )
        markers = self._build_polygon_markers(self.odd_cameras, future_inside_cameras)
        corridor_markers = self._build_corridor_polygon_markers(
            occupied_corridors,
            future_inside_corridors,
        )
        stationary_collision_markers = self._build_stationary_collision_markers()
        corridor_markers.markers.extend(stationary_collision_markers.markers)
        lookahead_markers = self._build_lookahead_markers(lookahead_points)
        self.marker_pub.publish(markers)
        self.corridor_marker_pub.publish(corridor_markers)
        self.lookahead_marker_pub.publish(lookahead_markers)

    def _publish_safety_cmd(self):
        if not self.replay_started or not self.safety_cmd_active:
            return

        lookahead_points = self._compute_lookahead_points(
            horizon_seconds=3.0, step_seconds=0.5
        )
        future_inside_cameras = self._get_future_inside_camera_names(lookahead_points)
        occupied_corridors = self._compute_occupied_corridors()
        future_inside_corridors = self._get_future_inside_corridor_names(
            lookahead_points
        )
        lookahead_hits_stationary_box = self._lookahead_hits_stationary_collision_box(
            lookahead_points
        )

        camera_red_collision = (
            len(set(future_inside_cameras).intersection(self.odd_cameras)) > 0
        )
        corridor_red_collision = (
            len(set(future_inside_corridors).intersection(occupied_corridors)) > 0
        )

        must_stop = (
            camera_red_collision
            or corridor_red_collision
            or lookahead_hits_stationary_box
        )
        reasons = []
        if corridor_red_collision:
            reasons.append("red corridor")
        if lookahead_hits_stationary_box:
            reasons.append("stationary collision box")
        if camera_red_collision:
            reasons.append("red camera polygon")

        cmd = Twist()
        if must_stop:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        else:
            cmd.linear.x = float(self.lookahead_constant_speed)
            cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)

        reason_text = ", ".join(reasons) if reasons else "clear"
        if (
            self.last_stop_state is None
            or self.last_stop_state != must_stop
            or self.last_stop_reason != reason_text
        ):
            if must_stop:
                self.get_logger().warn(f"Safety stop active: {reason_text}")
            else:
                self.get_logger().info("Safety stop cleared")
            self.last_stop_state = must_stop
            self.last_stop_reason = reason_text

    def _entity_cb(self, msg: MarkerArray):
        for marker in msg.markers:
            if marker.action == Marker.DELETE:
                if marker.id >= 1000:
                    entity_id = marker.id - 1000
                    if not self._is_entity_id_allowed(entity_id):
                        continue
                    self.entity_status_by_id.pop(entity_id, None)
                    self.entity_route_remaining_by_id.pop(entity_id, None)
                else:
                    if not self._is_entity_id_allowed(marker.id):
                        continue
                    self.entity_position_by_id.pop(marker.id, None)
                continue

            if marker.type == Marker.TEXT_VIEW_FACING:
                entity_id = marker.id - 1000 if marker.id >= 1000 else None
                if entity_id is None:
                    continue
                if not self._is_entity_id_allowed(entity_id):
                    continue
                status = self._parse_entity_status(marker.text)
                if status is not None:
                    self.entity_status_by_id[entity_id] = status
                    if status[0] != "route":
                        self.entity_route_remaining_by_id.pop(entity_id, None)
            else:
                if not self._is_entity_id_allowed(marker.id):
                    continue
                self.entity_position_by_id[marker.id] = (
                    float(marker.pose.position.x),
                    float(marker.pose.position.y),
                )

    def _is_entity_id_allowed(self, entity_id: int):
        if not self.allowed_ids:
            return True
        return int(entity_id) in self.allowed_ids

    def _parse_entity_status(self, text: str):
        if not text:
            return None

        text_l = text.lower()
        if "stationary" in text_l:
            return ("stationary", None)
        if "loading" in text_l:
            return ("loading", None)

        m = re.search(r"route\s*:\s*([0-9iI]+)\s*->\s*([0-9iI]+)", text)
        if m:
            return ("route", (m.group(1).upper(), m.group(2).upper()))

        return None

    def _compute_occupied_corridors(self):
        occupied = set()

        for entity_id, status in self.entity_status_by_id.items():
            status_type, payload = status
            if status_type == "route" and payload is not None:
                desired_route = self._expand_route_corridors(payload)
                remaining_route = self.entity_route_remaining_by_id.get(entity_id)
                if remaining_route != desired_route:
                    remaining_route = list(desired_route)

                position = self.entity_position_by_id.get(entity_id)
                current_corridor = self._find_corridor_for_point(position)
                if current_corridor is not None and current_corridor in remaining_route:
                    idx = remaining_route.index(current_corridor)
                    if idx > 0:
                        remaining_route = remaining_route[idx:]

                self.entity_route_remaining_by_id[entity_id] = remaining_route
                occupied.update(remaining_route)
                continue

            if status_type in ("loading", "stationary"):
                position = self.entity_position_by_id.get(entity_id)
                if position is None:
                    continue
                for corridor_name, polygon in self.corridor_polygons.items():
                    if self.is_inside_polygon(position, polygon):
                        occupied.add(str(corridor_name))

        return occupied

    def _expand_route_corridors(self, payload):
        start = str(payload[0]).upper()
        end = str(payload[1]).upper()
        if start == end:
            return [start]
        route = [start, "I", end]
        # Keep order, remove duplicates if start/end are already I or equal.
        seen = set()
        ordered_unique = []
        for item in route:
            if item in seen:
                continue
            seen.add(item)
            ordered_unique.append(item)
        return ordered_unique

    def _find_corridor_for_point(self, point_xy):
        if point_xy is None:
            return None
        for corridor_name, polygon in self.corridor_polygons.items():
            if self.is_inside_polygon(point_xy, polygon):
                return str(corridor_name)
        return None

    def _extract_cameras_with_odds(self, raw_json):
        odd_cameras = set()

        try:
            payload = json.loads(raw_json)
        except json.JSONDecodeError:
            self.get_logger().warn("Received odd_detections is not valid JSON")
            return odd_cameras

        for item in payload:
            camera = item.get("cam")
            odd_cameras.add(str(camera))

        return odd_cameras

    def _build_polygon_markers(self, odd_cameras, future_inside_cameras):
        marker_array = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        for idx, (camera_name, polygon) in enumerate(self.camera_polygons.items()):
            marker = Marker()
            marker.header.frame_id = self.frame_id
            marker.header.stamp = stamp
            marker.ns = self.marker_ns
            marker.id = idx
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD

            marker.scale.x = self.line_width
            marker.scale.y = self.line_width
            marker.scale.z = 0.05

            future_inside = str(camera_name) in future_inside_cameras

            # Red if camera has odd detections, otherwise green.
            if str(camera_name) in odd_cameras:
                marker.color.r = 1.0
                marker.color.g = 0.0
            else:
                marker.color.r = 0.0
                marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0

            for point in polygon:
                p = self._point(point)
                if p is None:
                    continue
                marker.points.append(p)

            # Close the polygon.
            if marker.points:
                marker.points.append(marker.points[0])

            marker_array.markers.append(marker)

        return marker_array

    def _build_corridor_polygon_markers(
        self, occupied_corridors, future_inside_corridors
    ):
        marker_array = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        # Corridor polygons are shown in blue by default.
        for idx, (corridor_name, polygon) in enumerate(self.corridor_polygons.items()):
            marker = Marker()
            marker.header.frame_id = self.frame_id
            marker.header.stamp = stamp
            marker.ns = f"{self.marker_ns}_corridor"
            marker.id = idx
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD

            marker.scale.x = self.line_width
            marker.scale.y = self.line_width
            marker.scale.z = 0.05

            if (
                str(corridor_name) in occupied_corridors
                and str(corridor_name) in future_inside_corridors
            ):
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
            elif str(corridor_name) in occupied_corridors:
                marker.color.r = 0.98
                marker.color.g = 0.50
                marker.color.b = 0.05
            else:
                marker.color.r = 0.10
                marker.color.g = 0.35
                marker.color.b = 0.95
            marker.color.a = 1.0

            for point in polygon:
                p = self._point(point)
                if p is None:
                    continue
                marker.points.append(p)

            if marker.points:
                marker.points.append(marker.points[0])

            marker_array.markers.append(marker)

            # Add corridor name label at polygon centroid.
            bg_marker = Marker()
            bg_marker.header.frame_id = self.frame_id
            bg_marker.header.stamp = stamp
            bg_marker.ns = f"{self.marker_ns}_corridor_label_bg"
            bg_marker.id = 20000 + idx
            bg_marker.type = Marker.CYLINDER
            bg_marker.action = Marker.ADD
            bg_marker.pose.orientation.w = 1.0
            bg_marker.scale.x = 0.9
            bg_marker.scale.y = 0.9
            bg_marker.scale.z = 0.02
            bg_marker.color.r = 0.0
            bg_marker.color.g = 0.0
            bg_marker.color.b = 0.0
            bg_marker.color.a = 0.75

            text_marker = Marker()
            text_marker.header.frame_id = self.frame_id
            text_marker.header.stamp = stamp
            text_marker.ns = f"{self.marker_ns}_corridor_label"
            text_marker.id = 10000 + idx
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.text = str(corridor_name)
            text_marker.pose.orientation.w = 1.0
            text_marker.scale.z = 0.7
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0

            if polygon:
                sum_x = 0.0
                sum_y = 0.0
                count = 0
                for point in polygon:
                    p = self._point(point)
                    if p is None:
                        continue
                    sum_x += p.x
                    sum_y += p.y
                    count += 1
                if count > 0:
                    center_x = sum_x / count
                    center_y = sum_y / count
                    bg_marker.pose.position.x = center_x
                    bg_marker.pose.position.y = center_y
                    bg_marker.pose.position.z = 0.18
                    text_marker.pose.position.x = center_x
                    text_marker.pose.position.y = center_y
                    text_marker.pose.position.z = 0.35
                else:
                    bg_marker.action = Marker.DELETE
                    text_marker.action = Marker.DELETE
            else:
                bg_marker.action = Marker.DELETE
                text_marker.action = Marker.DELETE

            marker_array.markers.append(bg_marker)
            marker_array.markers.append(text_marker)

        return marker_array

    def is_inside_polygon(self, point_xy, polygon):
        if point_xy is None:
            return False

        vertices = []
        for point in polygon:
            p = self._point(point)
            if p is None:
                continue
            vertices.append((p.x, p.y))

        if len(vertices) < 3:
            return False

        x, y = point_xy
        inside = False
        j = len(vertices) - 1

        for i in range(len(vertices)):
            xi, yi = vertices[i]
            xj, yj = vertices[j]

            # Ray-casting: toggle when edge crosses horizontal ray to +x.
            if ((yi > y) != (yj > y)) and (
                x
                < (xj - xi) * (y - yi) / ((yj - yi) if (yj - yi) != 0.0 else 1e-12) + xi
            ):
                inside = not inside
            j = i

        return inside

    def _compute_lookahead_points(self, horizon_seconds=3.0, step_seconds=0.5):
        if self.jackal_position is None or self.jackal_yaw is None:
            return []

        x, y = self.jackal_position
        yaw = self.jackal_yaw
        v_x = float(self.lookahead_constant_speed)
        w_z = (
            float(self.jackal_velocity[3]) if self.jackal_velocity is not None else 0.0
        )

        points = []
        steps = int(horizon_seconds / step_seconds)
        for i in range(1, steps + 1):
            dt = step_seconds
            x += v_x * math.cos(yaw) * dt
            y += v_x * math.sin(yaw) * dt
            yaw += w_z * dt
            points.append((x, y, i * step_seconds))

        return points

    def _get_future_inside_camera_names(self, lookahead_points):
        inside = set()
        for camera_name, polygon in self.camera_polygons.items():
            for point_x, point_y, _ in lookahead_points:
                if self.is_inside_polygon((point_x, point_y), polygon):
                    inside.add(str(camera_name))
                    break
        return inside

    def _get_future_inside_corridor_names(self, lookahead_points):
        inside = set()
        for corridor_name, polygon in self.corridor_polygons.items():
            for point_x, point_y, _ in lookahead_points:
                if self.is_inside_polygon((point_x, point_y), polygon):
                    inside.add(str(corridor_name))
                    break
        return inside

    def _build_lookahead_markers(self, lookahead_points):
        marker_array = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        total = max(len(lookahead_points), 1)
        for idx, (x, y, _) in enumerate(lookahead_points):
            marker = Marker()
            marker.header.frame_id = self.frame_id
            marker.header.stamp = stamp
            marker.ns = "lookahead_points"
            marker.id = idx
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD

            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.02
            marker.pose.orientation.w = 1.0

            marker.scale.x = 0.25
            marker.scale.y = 0.25
            marker.scale.z = 0.03

            ratio = float(idx) / float(total - 1) if total > 1 else 0.0
            marker.color.r = 0.15 + 0.45 * ratio
            marker.color.g = 0.05 + 0.10 * (1.0 - ratio)
            marker.color.b = 0.95 - 0.20 * ratio
            marker.color.a = 1.0

            marker_array.markers.append(marker)

        return marker_array

    def _build_stationary_collision_markers(self):
        marker_array = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        half_size = 0.5 * float(self.loading_collision_box_size)

        for entity_id, status in self.entity_status_by_id.items():
            if not status or status[0] != "stationary":
                continue

            position = self.entity_position_by_id.get(entity_id)
            if position is None:
                continue

            x, y = position
            marker = Marker()
            marker.header.frame_id = self.frame_id
            marker.header.stamp = stamp
            marker.ns = f"{self.marker_ns}_collision_stationary"
            marker.id = 30000 + int(entity_id)
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = max(self.line_width, 0.12)
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.95
            marker.lifetime.sec = 1

            corners = [
                (x - half_size, y - half_size),
                (x + half_size, y - half_size),
                (x + half_size, y + half_size),
                (x - half_size, y + half_size),
                (x - half_size, y - half_size),
            ]

            for px, py in corners:
                p = self._point((px, py, 0.08))
                if p is not None:
                    marker.points.append(p)

            marker_array.markers.append(marker)

        return marker_array

    def _lookahead_hits_stationary_collision_box(self, lookahead_points):
        half_size = 0.5 * float(self.loading_collision_box_size)
        for entity_id, status in self.entity_status_by_id.items():
            if not status or status[0] != "stationary":
                continue

            position = self.entity_position_by_id.get(entity_id)
            if position is None:
                continue

            cx, cy = position
            min_x = cx - half_size
            max_x = cx + half_size
            min_y = cy - half_size
            max_y = cy + half_size

            for px, py, _ in lookahead_points:
                if min_x <= px <= max_x and min_y <= py <= max_y:
                    return True

        return False

    def _point(self, xyz):
        from geometry_msgs.msg import Point

        if isinstance(xyz, dict):
            if "x" not in xyz or "y" not in xyz:
                return None
            x = xyz["x"]
            y = xyz["y"]
            z = xyz.get("z", 0.01)
        elif isinstance(xyz, (list, tuple)) and len(xyz) >= 2:
            x = xyz[0]
            y = xyz[1]
            z = xyz[2] if len(xyz) > 2 else 0.01
        else:
            return None

        p = Point()
        p.x = float(x)
        p.y = float(y)
        p.z = float(z)
        return p


def main(args=None):
    rclpy.init(args=args)
    node = SafetyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
