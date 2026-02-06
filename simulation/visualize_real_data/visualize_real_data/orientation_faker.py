from rclpy.node import Node
import rclpy
import json
import math
import std_msgs.msg
import rclpy.qos
import os
import glob
from datetime import datetime
import time

"""Node to fake object orientations in a JSON file based on their movement trajectories.
Calculates orientations based on movement direction, applies low-pass filtering and rate limiting,
and saves the updated JSON file. Can also list available JSON files and their message counts."""


class OrientationFaker(Node):
    def __init__(self):
        super().__init__("orientation_faker")
        self.get_logger().info("Faking orientation in json file...")
        # Declare parameters
        self.declare_parameter("json_file_name", "trajectories.json")
        self.declare_parameter(
            "max_angular_change", 0.5
        )  # Maximum angular change per timestep (radians)
        self.declare_parameter(
            "angular_smoothing_factor", 0.3
        )  # Low-pass filter coefficient (0-1)
        self.declare_parameter(
            "min_movement_distance", 0.02
        )  # Minimum distance to calculate new orientation
        self.declare_parameter(
            "list_json_files", False
        )  # List available JSON files and their message counts before processing
        self.declare_parameter(
            "start_time", ""
        )  # Start time filter (YYYY-MM-DD HH:MM:SS format)
        self.declare_parameter(
            "end_time", ""
        )  # End time filter (YYYY-MM-DD HH:MM:SS format)
        self.declare_parameter(
            "initial_heading", 0.0
        )  # Initial heading/orientation in degrees (0 = pointing left). Positive heading is counter-clockwise, i.e. 90 deg = DOWN.

        self.json_file_path = (
            self.get_parameter("json_file_name").get_parameter_value().string_value
        )
        self.max_angular_change = (
            self.get_parameter("max_angular_change").get_parameter_value().double_value
        )
        self.smoothing_factor = (
            self.get_parameter("angular_smoothing_factor")
            .get_parameter_value()
            .double_value
        )
        self.min_movement_distance = (
            self.get_parameter("min_movement_distance")
            .get_parameter_value()
            .double_value
        )
        self.list_json_files = (
            self.get_parameter("list_json_files").get_parameter_value().bool_value
        )
        self.start_time = (
            self.get_parameter("start_time").get_parameter_value().string_value
        )
        self.end_time = (
            self.get_parameter("end_time").get_parameter_value().string_value
        )
        self.initial_heading_degrees = (
            self.get_parameter("initial_heading").get_parameter_value().double_value
        )

        # Convert degrees to radians for internal use
        self.initial_heading = self.degrees_to_radians(self.initial_heading_degrees)

        self.done_pub = self.create_publisher(
            std_msgs.msg.Empty,
            "orientation_done",
            rclpy.qos.QoSProfile(
                depth=10, durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL
            ),
        )

        # List JSON files if requested
        if self.list_json_files:
            self.list_available_json_files()
            return  # Exit after listing files

        # Convert start/end times to milliseconds if provided
        self.start_timestamp_ms = None
        self.end_timestamp_ms = None

        if self.start_time:
            self.start_timestamp_ms = self.parse_time_input(
                self.start_time, "start_time"
            )
            if self.start_timestamp_ms:
                readable_time = self.timestamp_to_readable(self.start_timestamp_ms)
                self.get_logger().info(
                    f"Start time filter: {readable_time} ({self.start_timestamp_ms} ms)"
                )

        if self.end_time:
            self.end_timestamp_ms = self.parse_time_input(self.end_time, "end_time")
            if self.end_timestamp_ms:
                readable_time = self.timestamp_to_readable(self.end_timestamp_ms)
                self.get_logger().info(
                    f"End time filter: {readable_time} ({self.end_timestamp_ms} ms)"
                )

        self.get_logger().info(f"Angular filtering parameters:")
        self.get_logger().info(
            f"  Max angular change: {self.max_angular_change:.3f} rad ({math.degrees(self.max_angular_change):.1f}°)"
        )
        self.get_logger().info(f"  Smoothing factor: {self.smoothing_factor:.3f}")
        self.get_logger().info(
            f"  Min movement distance: {self.min_movement_distance:.3f} m"
        )
        self.get_logger().info(
            f"  Initial heading: {self.initial_heading_degrees:.1f}° ({self.initial_heading:.3f} rad)"
        )

        # Process the JSON file
        self.process_json_with_orientations()

    def parse_time_input(self, time_input, param_name):
        """Parse time input that can be either Unix timestamp (ms) or datetime string format"""
        if not time_input:
            return None

        # Try to parse as Unix timestamp (milliseconds) first
        try:
            timestamp_ms = int(time_input)
            # Validate that it's a reasonable timestamp (between 2000-2100)
            if 946684800000 <= timestamp_ms <= 4102444800000:  # Year 2000 to 2100 in ms
                return timestamp_ms
        except ValueError:
            pass  # Not a valid integer, try datetime format

        # Try to parse as datetime string
        try:
            dt = datetime.strptime(time_input, "%Y-%m-%d %H:%M:%S")
            timestamp_ms = int(dt.timestamp() * 1000)
            return timestamp_ms
        except ValueError:
            self.get_logger().error(
                f"Invalid {param_name} format: {time_input}. Use either Unix timestamp (ms) or YYYY-MM-DD HH:MM:SS format"
            )
            return None

    def timestamp_to_readable(self, timestamp_ms):
        """Convert Unix timestamp in milliseconds to readable datetime string"""
        try:
            dt = datetime.fromtimestamp(timestamp_ms / 1000)
            return dt.strftime("%Y-%m-%d %H:%M:%S")
        except (ValueError, OSError):
            return f"Invalid timestamp: {timestamp_ms}"

    def list_available_json_files(self):
        """List all available JSON files in the directory of json_file_path and show message counts"""
        # Use the directory of the json_file_path parameter

        # If empty string is given as json, this needs special handling.
        if self.json_file_path.split("/")[-1] == "replay_data":
            target_dir = self.json_file_path
        else:
            target_dir = os.path.dirname(os.path.abspath(self.json_file_path))
        json_pattern = os.path.join(target_dir, "*.json")
        json_files = glob.glob(json_pattern)

        total_files_found = 0

        # Build the entire table as a single string
        table_lines = ["Listing available JSON files and their message counts:"]
        table_lines.append("╔" + "═" * 118 + "╗")
        table_lines.append(
            "║" + " AVAILABLE JSON FILES IN TARGET DIRECTORY".center(118) + "║"
        )
        table_lines.append("╚" + "═" * 118 + "╝")
        table_lines.append("")
        table_lines.append(f"📁 Directory: {target_dir}")

        if json_files:
            # Calculate maximum filename length for dynamic column sizing
            max_filename_len = max(len(os.path.basename(f)) for f in json_files)
            filename_col_width = max(max_filename_len + 2, 20)  # At least 20 chars

            table_lines.append(
                "┌"
                + "─" * filename_col_width
                + "┬"
                + "─" * 10
                + "┬"
                + "─" * 10
                + "┬"
                + "─" * 12
                + "┬"
                + "─" * 14
                + "┐"
            )
            table_lines.append(
                f'│ {"File Name":<{filename_col_width-1}}│ {"Messages":>8} │ {"Objects":>8} │ {"Duration":>10} │ {"Size":>12} │'
            )
            table_lines.append(
                "├"
                + "─" * filename_col_width
                + "┼"
                + "─" * 10
                + "┼"
                + "─" * 10
                + "┼"
                + "─" * 12
                + "┼"
                + "─" * 14
                + "┤"
            )

            for json_file in sorted(json_files):
                file_name = os.path.basename(json_file)
                file_size_bytes = os.path.getsize(json_file)
                message_count, object_count, time_span = self.analyze_json_file(
                    json_file
                )

                if message_count is not None:
                    # Format file size
                    if file_size_bytes >= 1024 * 1024:
                        size_str = f"{file_size_bytes / (1024 * 1024):.1f} MB"
                    elif file_size_bytes >= 1024:
                        size_str = f"{file_size_bytes / 1024:.1f} KB"
                    else:
                        size_str = f"{file_size_bytes} B"

                    # Format duration
                    if time_span >= 60:
                        duration_str = f"{time_span / 60:.1f} min"
                    else:
                        duration_str = f"{time_span:.1f} s"

                    table_lines.append(
                        f"│ {file_name:<{filename_col_width-1}}│ {message_count:>8} │ {object_count:>8} │ {duration_str:>10} │ {size_str:>12} │"
                    )
                    total_files_found += 1
                else:
                    table_lines.append(
                        f'│ ❌ {file_name:<{filename_col_width-3}}│ {"Invalid/corrupted JSON file":>46} │'
                    )

            table_lines.append(
                "└"
                + "─" * filename_col_width
                + "┴"
                + "─" * 10
                + "┴"
                + "─" * 10
                + "┴"
                + "─" * 12
                + "┴"
                + "─" * 14
                + "┘"
            )

        if total_files_found == 0:
            table_lines.append("❌ No valid JSON files found in target directory.")
        else:
            table_lines.append("")
            table_lines.append(f"✅ Found {total_files_found} valid JSON file(s)")

        table_lines.append(
            "Exiting... Disable list_json_files parameter to process a specific JSON file."
        )
        # Log the entire table as a single multi-line string
        self.get_logger().info("\n".join(table_lines))

    def analyze_json_file(self, file_path):
        """Analyze JSON file and return message count, unique object count, and time span (in milliseconds)"""
        try:
            with open(file_path, "r") as file:
                data = json.load(file)

            if not isinstance(data, list):
                return None, None, None

            message_count = len(data)
            unique_objects = set()
            timestamps = []

            for frame_data in data:
                if not isinstance(frame_data, dict):
                    continue

                # Extract timestamp
                timestamp = frame_data.get("time_stamp", 0)
                if timestamp > 0:
                    timestamps.append(timestamp)

                # Count unique objects
                objects = frame_data.get("object_list", [])
                for obj_array in objects:
                    if isinstance(obj_array, list) and len(obj_array) >= 1:
                        unique_objects.add(obj_array[0])  # Object ID

            time_span = 0.0
            if len(timestamps) >= 2:
                time_span = (max(timestamps) - min(timestamps)) / 1e3
                # Log timestamp range in both formats
                min_readable = self.timestamp_to_readable(min(timestamps))
                max_readable = self.timestamp_to_readable(max(timestamps))
                self.get_logger().debug(
                    f"  Time range: {min_readable} ({min(timestamps)}) to {max_readable} ({max(timestamps)})"
                )

            return message_count, len(unique_objects), time_span

        except (FileNotFoundError, json.JSONDecodeError, KeyError, IndexError):
            return None, None, None
        except Exception:
            return None, None, None

    def process_json_with_orientations(self):
        """Read JSON file, calculate orientations, and save updated JSON"""
        try:
            # Read the JSON file
            self.get_logger().info(f"Reading JSON file: {self.json_file_path}")
            with open(self.json_file_path, "r") as file:
                data = json.load(file)

            # Process the data to add orientations (filtering happens inside the processing)
            updated_data = self.add_orientations_to_data(data)

            # Save the updated JSON file
            output_path = self.json_file_path
            self.get_logger().info(f"Saving updated JSON file: {output_path}")
            with open(output_path, "w") as file:
                json.dump(updated_data, file, indent=2)

            self.get_logger().info("Successfully added orientations to JSON file")

            # Publish done 3 times just in case so that the "done" publish is not missed by rest of the launch file.
            pubs = 0
            while pubs < 3:
                self.done_pub.publish(std_msgs.msg.Empty())
                pubs += 1
                time.sleep(0.5)

        except FileNotFoundError:
            self.get_logger().error(f"JSON file not found: {self.json_file_path}")
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Error parsing JSON file: {e}")
        except Exception as e:
            self.get_logger().error(f"Error processing JSON file: {e}")

    def add_orientations_to_data(self, data):
        """Add orientation field to each object based on next position with angular filtering
        Only processes objects within the specified time range, but returns the complete dataset
        """

        # Determine which frames are within the time filter
        frames_in_range = set()
        if self.start_timestamp_ms or self.end_timestamp_ms:
            for i, frame_data in enumerate(data):
                timestamp = frame_data.get("time_stamp", 0)

                # Check if frame is within time range
                within_range = True
                if self.start_timestamp_ms and timestamp < self.start_timestamp_ms:
                    within_range = False
                if self.end_timestamp_ms and timestamp > self.end_timestamp_ms:
                    within_range = False

                if within_range:
                    frames_in_range.add(i)
            self.get_logger().info(
                f"Processing orientations for {len(frames_in_range)} frames within time range (out of {len(data)} total frames)"
            )
        else:
            # No time filtering - process all frames
            frames_in_range = set(range(len(data)))
            self.get_logger().info("No time filters applied - processing all frames")

        # Create a dictionary to organize data by object ID and timestamp (only for frames in range)
        objects_by_id = {}

        # Group objects by ID (only for frames within time range)
        for i, frame_data in enumerate(data):
            if i not in frames_in_range:
                continue  # Skip frames outside time range

            timestamp = frame_data.get("time_stamp", 0)
            objects = frame_data.get("object_list", [])

            for obj_array in objects:
                if len(obj_array) >= 4:  # [id, x, y, z]
                    obj_id = obj_array[0]
                    x = obj_array[1]
                    y = obj_array[2]
                    z = obj_array[3]

                    if obj_id not in objects_by_id:
                        objects_by_id[obj_id] = []

                    # Create object entry with timestamp for sorting
                    obj_entry = {
                        "id": obj_id,
                        "x": x,
                        "y": y,
                        "z": z,
                        "_timestamp": timestamp,
                        "_frame_data": frame_data,
                        "_original_array": obj_array,  # Keep reference to original array
                    }
                    objects_by_id[obj_id].append(obj_entry)

        # Sort objects by timestamp for each ID
        for obj_id in objects_by_id:
            objects_by_id[obj_id].sort(key=lambda x: x["_timestamp"])

        objects_processed = 0
        # Calculate orientations for each object and update original arrays
        for obj_id, obj_list in objects_by_id.items():
            current_orientation = self.normalize_angle(
                self.initial_heading
            )  # Start with normalized initial heading
            orientation_updates = 0
            orientation_smoothed = 0

            for i, obj in enumerate(obj_list):
                new_orientation = self.calculate_filtered_orientation(
                    obj_list, i, current_orientation
                )

                # Track statistics
                if (
                    abs(self.normalize_angle(new_orientation - current_orientation))
                    > 0.001
                ):
                    if (
                        abs(self.normalize_angle(new_orientation - current_orientation))
                        < self.max_angular_change
                    ):
                        orientation_smoothed += 1
                    else:
                        orientation_updates += 1

                current_orientation = new_orientation

                # Add orientation to the original array: [id, x, y, z, orientation]
                original_array = obj["_original_array"]
                if len(original_array) == 4:
                    original_array.append(current_orientation)
                elif len(original_array) == 5:
                    original_array[4] = (
                        current_orientation  # Update existing orientation
                    )

            self.get_logger().info(
                f"Object {obj_id}: {orientation_updates} major changes, {orientation_smoothed} smoothed transitions"
            )
            objects_processed += 1

        if objects_processed > 0:
            self.get_logger().info(
                f"Processed orientations for {objects_processed} unique object IDs within time range"
            )
        else:
            self.get_logger().warn(
                "No objects found within the specified time range for orientation processing"
            )

        # Return the complete original dataset (some objects will have 4 elements, some 5)
        return data

    def calculate_raw_orientation_to_next_point(self, obj_list, current_index):
        """Calculate raw orientation based on movement to next significant point"""
        current_obj = obj_list[current_index]
        current_x = current_obj["x"]
        current_y = current_obj["y"]

        # Look for next position with significant movement
        for i in range(current_index + 1, len(obj_list)):
            candidate = obj_list[i]
            next_x = candidate["x"]
            next_y = candidate["y"]

            dx = next_x - current_x
            dy = next_y - current_y
            distance = math.sqrt(dx**2 + dy**2)

            if distance >= self.min_movement_distance:
                # Calculate raw orientation to this point
                theta = math.atan2(dy, dx) + math.pi  # Add π to make 0° point left
                return self.normalize_angle(theta), True

        # No significant movement found
        return 0.0, False

    def calculate_filtered_orientation(
        self, obj_list, current_index, previous_orientation
    ):
        """Calculate orientation with low-pass filtering and rate limiting"""
        # Get raw target orientation
        target_orientation, has_target = self.calculate_raw_orientation_to_next_point(
            obj_list, current_index
        )

        if not has_target:
            # No movement detected, keep previous orientation
            self.get_logger().debug(
                f'Object {obj_list[current_index]["id"]}: No significant movement, keeping orientation: {previous_orientation:.3f} rad'
            )
            return previous_orientation

        # Calculate the angular difference
        angle_diff = self.normalize_angle(target_orientation - previous_orientation)

        # Apply rate limiting (maximum change per timestep)
        if abs(angle_diff) > self.max_angular_change:
            # Limit the change to maximum allowed
            limited_diff = math.copysign(self.max_angular_change, angle_diff)
            new_orientation = self.normalize_angle(previous_orientation + limited_diff)

            self.get_logger().debug(
                f'Object {obj_list[current_index]["id"]}: Rate limited angular change from {math.degrees(angle_diff):.1f}° to {math.degrees(limited_diff):.1f}°'
            )
        else:
            # Apply low-pass filtering for smooth transitions
            # new_angle = previous_angle + smoothing_factor * (target_angle - previous_angle)
            filtered_diff = self.smoothing_factor * angle_diff
            new_orientation = self.normalize_angle(previous_orientation + filtered_diff)

            self.get_logger().debug(
                f'Object {obj_list[current_index]["id"]}: Smoothed angular change from {math.degrees(angle_diff):.1f}° to {math.degrees(filtered_diff):.1f}°'
            )

        current_obj = obj_list[current_index]
        self.get_logger().debug(
            f'  Position: ({current_obj["x"]:.3f}, {current_obj["y"]:.3f})'
        )
        self.get_logger().debug(
            f"  Orientation: {math.degrees(previous_orientation):.1f}° -> {math.degrees(new_orientation):.1f}°"
        )

        return new_orientation

    def normalize_angle(self, angle):
        """Normalize angle to [-π, π] range"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def degrees_to_radians(self, degrees):
        """Convert degrees to radians"""
        return degrees * math.pi / 180.0

    def radians_to_degrees(self, radians):
        """Convert radians to degrees"""
        return radians * 180.0 / math.pi


def main(args=None):
    rclpy.init(args=args)
    node = OrientationFaker()

    try:
        # Keep the node alive briefly to complete processing
        rclpy.spin_once(node, timeout_sec=1.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
