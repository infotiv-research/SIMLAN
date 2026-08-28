import json
import math
import os
import select
import sys
import termios
import threading
import tty
from copy import deepcopy
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray

from visualize_real_data.camera_polygons_config import (
    CAMERA_COLOR_ENUM,
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


class UpdatePolygons(Node):
    def __init__(self):
        super().__init__("update_polygons")
        default_frame_position = _default_frame_position()

        self.declare_parameter("frame_id", "real_data")
        self.declare_parameter("marker_topic", "/visualize_real_data/camera_polygons")
        self.declare_parameter("marker_ns", "camera_safety")
        self.declare_parameter("line_width", 0.1)
        self.declare_parameter("frame_position", json.dumps(default_frame_position))
        self.declare_parameter("edit_dash_length", 0.25)
        self.declare_parameter("edit_dash_gap", 0.14)

        self.frame_id = (
            self.get_parameter("frame_id").get_parameter_value().string_value
        )
        self.marker_topic = (
            self.get_parameter("marker_topic").get_parameter_value().string_value
        )
        self.marker_ns = (
            self.get_parameter("marker_ns").get_parameter_value().string_value
        )
        self.line_width = (
            self.get_parameter("line_width").get_parameter_value().double_value
        )
        self.edit_dash_length = (
            self.get_parameter("edit_dash_length").get_parameter_value().double_value
        )
        self.edit_dash_gap = (
            self.get_parameter("edit_dash_gap").get_parameter_value().double_value
        )
        self.frame_position = json.loads(
            self.get_parameter("frame_position").get_parameter_value().string_value
        )

        self.camera_polygons = deepcopy(CAMERA_POLYGONS)
        self.camera_color_enum = deepcopy(CAMERA_COLOR_ENUM)
        self.current_highlighted_camera_ids = set(self.camera_polygons.keys())
        self.edit_camera_id = None
        self.edit_old_polygon = []
        self.edit_click_points = []

        self._click_condition = threading.Condition()
        self._click_queue = []

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.marker_pub = self.create_publisher(MarkerArray, self.marker_topic, qos)
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            "/initialpose",
            self._initialpose_cb,
            qos,
        )
        self.publish_timer = self.create_timer(0.2, self._publish_current_polygons)

    def _publish_current_polygons(self):
        self.publish_polygons(
            highlighted_camera_ids=self.current_highlighted_camera_ids
        )

    def _initialpose_cb(self, msg: PoseWithCovarianceStamped):
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        with self._click_condition:
            self._click_queue.append((x, y, msg.header.frame_id))
            self._click_condition.notify_all()

    def _raw_to_display_xy(self, raw_x: float, raw_y: float):
        x = raw_x
        y = raw_y
        return x, y

    def _display_to_raw_xy(self, disp_x: float, disp_y: float):
        # Convert click in map frame to polygon local frame.
        ox = float(self.frame_position.get("x", 0.0))
        oy = float(self.frame_position.get("y", 0.0))
        raw_x = disp_x - ox
        raw_y = disp_y - oy
        return raw_x, raw_y

    def publish_polygons(self, highlighted_camera_ids=None):
        if highlighted_camera_ids is None:
            highlighted_camera_ids = set(self.camera_polygons.keys())

        self.current_highlighted_camera_ids = set(highlighted_camera_ids)
        selected_ids = {str(cid) for cid in highlighted_camera_ids}

        marker_array = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        for idx, (camera_id, polygon) in enumerate(self.camera_polygons.items()):
            marker = Marker()
            marker.header.frame_id = self.frame_id
            marker.header.stamp = stamp
            marker.ns = self.marker_ns
            marker.id = idx
            marker.type = Marker.LINE_STRIP

            # Only selected polygons are visible; unselected ones are explicitly deleted.
            if str(camera_id) not in selected_ids:
                marker.action = Marker.DELETE
                marker_array.markers.append(marker)
                continue

            # While editing, hide the solid line if it is still the old polygon.
            # The old shape is shown as dashed overlay instead.
            if (
                self.edit_camera_id is not None
                and str(camera_id) == str(self.edit_camera_id)
                and polygon == self.edit_old_polygon
            ):
                marker.action = Marker.DELETE
                marker_array.markers.append(marker)
                continue

            marker.action = Marker.ADD
            marker.scale.x = self.line_width
            marker.scale.y = self.line_width
            marker.scale.z = 0.05

            color_name = self.camera_color_enum.get(str(camera_id), "GRAY")
            color = COLOR_ENUM.get(color_name, (0.8, 0.8, 0.8))
            marker.color.r = float(color[0])
            marker.color.g = float(color[1])
            marker.color.b = float(color[2])
            marker.color.a = 1.0

            for point in polygon:
                if "x" not in point or "y" not in point:
                    continue

                disp_x, disp_y = self._raw_to_display_xy(
                    float(point["x"]), float(point["y"])
                )

                p = Point()
                p.x = disp_x
                p.y = disp_y
                p.z = float(point.get("z", 0.01))
                marker.points.append(p)

            if marker.points:
                marker.points.append(marker.points[0])

            marker_array.markers.append(marker)

        self._append_edit_overlays(marker_array, stamp)

        self.marker_pub.publish(marker_array)

    def _append_edit_overlays(self, marker_array: MarkerArray, stamp):
        ns = f"{self.marker_ns}_edit"

        dashed_marker = Marker()
        dashed_marker.header.frame_id = self.frame_id
        dashed_marker.header.stamp = stamp
        dashed_marker.ns = ns
        dashed_marker.id = 900000
        dashed_marker.type = Marker.LINE_LIST

        click_points_marker = Marker()
        click_points_marker.header.frame_id = self.frame_id
        click_points_marker.header.stamp = stamp
        click_points_marker.ns = ns
        click_points_marker.id = 900001
        click_points_marker.type = Marker.SPHERE_LIST

        click_path_marker = Marker()
        click_path_marker.header.frame_id = self.frame_id
        click_path_marker.header.stamp = stamp
        click_path_marker.ns = ns
        click_path_marker.id = 900002
        click_path_marker.action = Marker.DELETE

        if self.edit_camera_id is None:
            dashed_marker.action = Marker.DELETE
            click_points_marker.action = Marker.DELETE
            marker_array.markers.extend(
                [dashed_marker, click_points_marker, click_path_marker]
            )

            for i in range(4):
                t = Marker()
                t.header.frame_id = self.frame_id
                t.header.stamp = stamp
                t.ns = ns
                t.id = 900100 + i
                t.action = Marker.DELETE
                marker_array.markers.append(t)
            return

        old_points = []
        for point in self.edit_old_polygon:
            if "x" not in point or "y" not in point:
                continue
            disp_x, disp_y = self._raw_to_display_xy(
                float(point["x"]), float(point["y"])
            )
            old_points.append((disp_x, disp_y))

        if len(old_points) >= 2:
            dashed_marker.action = Marker.ADD
            dashed_marker.scale.x = max(self.line_width * 0.75, 0.03)
            color_name = self.camera_color_enum.get(str(self.edit_camera_id), "GRAY")
            color = COLOR_ENUM.get(color_name, (0.8, 0.8, 0.8))
            dashed_marker.color.r = float(color[0])
            dashed_marker.color.g = float(color[1])
            dashed_marker.color.b = float(color[2])
            dashed_marker.color.a = 0.95

            closed_points = old_points + [old_points[0]]
            dash_len = max(self.edit_dash_length, 0.01)
            gap_len = max(self.edit_dash_gap, 0.0)
            step = dash_len + gap_len

            for i in range(len(closed_points) - 1):
                x0, y0 = closed_points[i]
                x1, y1 = closed_points[i + 1]
                dx = x1 - x0
                dy = y1 - y0
                dist = math.hypot(dx, dy)
                if dist <= 1e-9:
                    continue

                ux = dx / dist
                uy = dy / dist
                t = 0.0
                while t < dist:
                    seg_start = t
                    seg_end = min(t + dash_len, dist)

                    p0 = Point()
                    p0.x = x0 + ux * seg_start
                    p0.y = y0 + uy * seg_start
                    p0.z = 0.04

                    p1 = Point()
                    p1.x = x0 + ux * seg_end
                    p1.y = y0 + uy * seg_end
                    p1.z = 0.04

                    dashed_marker.points.append(p0)
                    dashed_marker.points.append(p1)
                    t += step
        else:
            dashed_marker.action = Marker.DELETE

        # Already-placed click points for the new polygon.
        click_points_marker.action = (
            Marker.ADD if self.edit_click_points else Marker.DELETE
        )
        click_points_marker.scale.x = 0.22
        click_points_marker.scale.y = 0.22
        click_points_marker.scale.z = 0.22
        click_points_marker.color.r = 0.1
        click_points_marker.color.g = 1.0
        click_points_marker.color.b = 0.1
        click_points_marker.color.a = 1.0

        for point in self.edit_click_points:
            p = Point()
            p.x = float(point["x"])
            p.y = float(point["y"])
            p.z = 0.05
            click_points_marker.points.append(p)

        marker_array.markers.extend(
            [dashed_marker, click_points_marker, click_path_marker]
        )

        # Point index labels (1..4)
        for i in range(4):
            t = Marker()
            t.header.frame_id = self.frame_id
            t.header.stamp = stamp
            t.ns = ns
            t.id = 900100 + i
            t.type = Marker.TEXT_VIEW_FACING
            if i < len(self.edit_click_points):
                t.action = Marker.ADD
                t.text = str(i + 1)
                t.pose.position.x = float(self.edit_click_points[i]["x"])
                t.pose.position.y = float(self.edit_click_points[i]["y"])
                t.pose.position.z = 0.22
                t.pose.orientation.w = 1.0
                t.scale.z = 0.25
                t.color.r = 1.0
                t.color.g = 1.0
                t.color.b = 1.0
                t.color.a = 1.0
            else:
                t.action = Marker.DELETE
            marker_array.markers.append(t)

    def wait_for_initialpose_click(self):
        with self._click_condition:
            while not self._click_queue:
                self._click_condition.wait()
            return self._click_queue.pop(0)

    def wait_for_initialpose_click_or_back(self):
        # While editing, allow keyboard 'b' to cancel current polygon edit immediately.
        if not sys.stdin.isatty():
            return ("click", self.wait_for_initialpose_click())

        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            while True:
                with self._click_condition:
                    if self._click_queue:
                        return ("click", self._click_queue.pop(0))

                readable, _, _ = select.select([sys.stdin], [], [], 0.1)
                if not readable:
                    continue

                ch = sys.stdin.read(1)
                if ch == "\x03":
                    raise KeyboardInterrupt
                if ch.lower() == "b":
                    print("\nBack requested for current polygon edit.")
                    return ("back", None)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    def _camera_menu(self, selected):
        cameras = list(self.camera_polygons.keys())
        print("\nSelect cameras to remap. Press key to toggle; press ENTER when done.")
        all_mark = "X" if len(selected) == len(cameras) else " "
        print(f"(0) - TOGGLE ALL [{all_mark}]")
        for i, camera_id in enumerate(cameras, start=1):
            color_name = self.camera_color_enum.get(camera_id, "GRAY")
            mark = "X" if camera_id in selected else " "
            key = str(i) if i <= 9 else chr(ord("a") + (i - 10))
            print(f"({key}) - {camera_id} {color_name} [{mark}]")

    def _read_single_key(self):
        if not sys.stdin.isatty():
            return input("Selection: ").strip().lower()

        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

        if ch in ("\r", "\n"):
            print("")
            return ""

        if ch == "\x03":
            raise KeyboardInterrupt

        print(ch)
        return ch.lower()

    def _drain_stdin_buffer(self):
        if not sys.stdin.isatty():
            return

        fd = sys.stdin.fileno()
        while True:
            readable, _, _ = select.select([sys.stdin], [], [], 0)
            if not readable:
                break
            try:
                os.read(fd, 1024)
            except OSError:
                break

    def _prompt_line(self, prompt: str):
        # Avoid auto-skipping input() prompts due to trailing newlines from raw single-key input.
        self._drain_stdin_buffer()
        return input(prompt).strip().lower()

    def select_cameras(self):
        cameras = list(self.camera_polygons.keys())
        # Default: all cameras enabled.
        selected = set(cameras)

        # Draw enabled polygons immediately when selection starts.
        self.publish_polygons(highlighted_camera_ids=selected)

        while True:
            self.publish_polygons(highlighted_camera_ids=selected)
            self._camera_menu(selected)
            print("Selection key: ", end="", flush=True)
            key = self._read_single_key()

            if key == "":
                break

            if key == "0":
                if len(selected) == len(cameras):
                    selected.clear()
                else:
                    selected = set(cameras)
                continue

            idx = None
            if key.isdigit():
                idx = int(key) - 1
            elif "a" <= key <= "z":
                idx = 9 + (ord(key) - ord("a"))

            if idx is None or idx < 0 or idx >= len(cameras):
                continue

            camera_id = cameras[idx]
            if camera_id in selected:
                selected.remove(camera_id)
            else:
                selected.add(camera_id)

        return [c for c in cameras if c in selected]

    def print_copy_paste_block(self):
        print("\nCopy/paste-ready camera polygon dictionary:")
        print("{")
        for camera_id, polygon in self.camera_polygons.items():
            print(f'    "{camera_id}": [')
            for point in polygon:
                print(
                    "        "
                    + '{"x": '
                    + f"{point['x']:.15f}"
                    + ', "y": '
                    + f"{point['y']:.15f}"
                    + "},"
                )
            print("    ],")
        print("}")

    def _next_camera_menu(self, selected_order, updated):
        print("\nChoose next camera to update. Press ENTER when done.")
        for i, camera_id in enumerate(selected_order, start=1):
            key = str(i) if i <= 9 else chr(ord("a") + (i - 10))
            color_name = self.camera_color_enum.get(camera_id, "GRAY")
            status = " (UPDATED)" if camera_id in updated else ""
            print(f"({key}) - {camera_id} {color_name}{status}")

    def _choose_next_camera(self, selected_order, updated):
        while True:
            self._next_camera_menu(selected_order, updated)
            print("Next camera key: ", end="", flush=True)
            key = self._read_single_key()

            if key == "":
                return None

            idx = None
            if key.isdigit():
                idx = int(key) - 1
            elif "a" <= key <= "z":
                idx = 9 + (ord(key) - ord("a"))

            if idx is None or idx < 0 or idx >= len(selected_order):
                continue

            return selected_order[idx]

    def remap_selected_cameras(self, selected_cameras):
        if not selected_cameras:
            print("No cameras selected. Nothing to update.")
            return

        selected_set = set(selected_cameras)
        selected_order = list(selected_cameras)
        updated = set()

        while True:
            self.publish_polygons(highlighted_camera_ids=selected_set)
            camera_id = self._choose_next_camera(selected_order, updated)
            if camera_id is None:
                break

            original_polygon = deepcopy(self.camera_polygons[camera_id])
            while True:
                color_name = self.camera_color_enum.get(camera_id, "GRAY")
                print(f"\nRemapping color {color_name} (camera {camera_id})")
                print("Click/publish 4 points on /initialpose in RViz.")
                print("Tip: use '2D Pose Estimate' tool for each corner.")

                self.edit_camera_id = camera_id
                self.edit_old_polygon = deepcopy(original_polygon)
                self.edit_click_points = []

                with self._click_condition:
                    self._click_queue.clear()

                updated_polygon = []
                self.publish_polygons(highlighted_camera_ids=selected_set)

                for i in range(4):
                    print(
                        f"Waiting for point {i + 1}/4 for color {color_name} "
                        f"(camera {camera_id})... (press 'b' to go back)"
                    )
                    click_result, payload = self.wait_for_initialpose_click_or_back()
                    if click_result == "back":
                        self.camera_polygons[camera_id] = deepcopy(original_polygon)
                        self.edit_camera_id = None
                        self.edit_old_polygon = []
                        self.edit_click_points = []
                        self.publish_polygons(highlighted_camera_ids=selected_set)
                        break

                    x, y, frame = payload
                    raw_x, raw_y = self._display_to_raw_xy(x, y)
                    updated_polygon.append({"x": raw_x, "y": raw_y})
                    self.edit_click_points = deepcopy(updated_polygon)
                    self.publish_polygons(highlighted_camera_ids=selected_set)
                    print(
                        f"  Got point {i + 1}: display=({x:.4f}, {y:.4f}) frame='{frame}'"
                        f" -> raw=({raw_x:.4f}, {raw_y:.4f})"
                    )

                if len(updated_polygon) < 4:
                    # Back was requested while placing points.
                    break

                self.camera_polygons[camera_id] = updated_polygon
                # Full polygon is now placed; hide temporary click dots/labels for inspection.
                self.edit_click_points = []
                self.publish_polygons(highlighted_camera_ids=selected_set)

                confirm = self._prompt_line(
                    f"Accept new polygon for {color_name} (camera {camera_id})? "
                    "[y=accept / n=redo / b=back / q=quit]: "
                )

                if confirm in ("y", "yes", ""):
                    original_polygon = deepcopy(updated_polygon)
                    updated.add(camera_id)
                    self.edit_camera_id = None
                    self.edit_old_polygon = []
                    self.edit_click_points = []
                    break
                if confirm in ("q", "quit"):
                    self.camera_polygons[camera_id] = original_polygon
                    self.edit_camera_id = None
                    self.edit_old_polygon = []
                    self.edit_click_points = []
                    self.publish_polygons(highlighted_camera_ids=selected_set)
                    return

                if confirm in ("b", "back"):
                    self.camera_polygons[camera_id] = deepcopy(original_polygon)
                    self.edit_camera_id = None
                    self.edit_old_polygon = []
                    self.edit_click_points = []
                    self.publish_polygons(highlighted_camera_ids=selected_set)
                    break

                # Redo requested: discard tentative polygon and show last accepted one.
                self.camera_polygons[camera_id] = deepcopy(original_polygon)
                self.edit_camera_id = None
                self.edit_old_polygon = []
                self.edit_click_points = []
                self.publish_polygons(highlighted_camera_ids=selected_set)


def main(args=None):
    rclpy.init(args=args)
    node = UpdatePolygons()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        node.publish_polygons()
        selected = node.select_cameras()
        node.remap_selected_cameras(selected)
        node.publish_polygons()
        node.print_copy_paste_block()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
