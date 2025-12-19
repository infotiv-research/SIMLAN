import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
from datetime import datetime
from rclpy.executors import MultiThreadedExecutor
import threading


class CameraRecorder(Node):
    def __init__(self):
        super().__init__("camera_recorder")
        self.declare_parameter("camera_ids", "164")

        ids_str = self.get_parameter("camera_ids").get_parameter_value().string_value
        self.camera_ids_array = [int(x) for x in ids_str.split()]
        print("HAMID", self.camera_ids_array)
        self.declare_parameter("camera_update_rate")
        self.camera_update_rate = (
            self.get_parameter("camera_update_rate")
            .get_parameter_value()
            ._integer_value
        )
        self.output_dir = os.path.expanduser(
            "simulation/camera_bird_eye_view/recordings"
        )
        os.makedirs(self.output_dir, exist_ok=True)
        self.timestamp = datetime.now().strftime("%Y%m%d-%H%M%S")
        os.makedirs(f"{self.output_dir}/{self.timestamp}", exist_ok=True)
        self.bridge = CvBridge()

        self.image_locks = {}
        self.semantic_locks = {}
        self.depth_locks = {}

        self.image_writers = {}
        self.semantic_writers = {}
        self.depth_writers = {}

        self.image_sizes = {}
        self.semantic_sizes = {}
        self.depth_sizes = {}
        self.all_subs = []

        for cid in self.camera_ids_array:
            self.image_locks[cid] = threading.Lock()
            self.semantic_locks[cid] = threading.Lock()
            self.depth_locks[cid] = threading.Lock()
            self.get_logger().info(f"Starting subscriptions for camera {cid}")
            image_topic = f"/static_agents/camera_{cid}/image_raw"
            semantic_topic = f"/static_agents/semantic_camera_{cid}/labels_map"
            depth_topic = f"/static_agents/depth_camera_{cid}/image_raw"

            # Subscribers
            self.all_subs.append(
                self.create_subscription(
                    Image,
                    image_topic,
                    lambda msg, cid=cid: self.image_callback(msg, cid),
                    20,
                )
            )
            self.all_subs.append(
                self.create_subscription(
                    Image,
                    semantic_topic,
                    lambda msg, cid=cid: self.semantic_callback(msg, cid),
                    20,
                )
            )
            self.all_subs.append(
                self.create_subscription(
                    Image,
                    depth_topic,
                    lambda msg, cid=cid: self.depth_callback(msg, cid),
                    20,
                )
            )

    #### CALLBACKS ####
    def image_callback(self, msg, cid):
        # Convert ROS image to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        # Ensure 8-bit BGR format
        if cv_image.dtype != np.uint8:
            cv_image = cv_image.astype(np.uint8)

        # Initialize writer for RGB images
        if cid not in self.image_writers:
            size = (cv_image.shape[1], cv_image.shape[0])
            self.image_sizes[cid] = size

            filename = (
                f"{self.output_dir}/{self.timestamp}/image_recording_cam{cid}.mp4"
            )

            writer = cv2.VideoWriter(
                filename,
                cv2.VideoWriter_fourcc(*"mp4v"),
                self.camera_update_rate,
                size,
            )
            self.image_writers[cid] = (writer, filename)
            self.get_logger().info(f"Started recording RGB for cam {cid}")

        writer, _ = self.image_writers[cid]
        # Write frame
        with self.image_locks[cid]:
            writer.write(cv_image)

    def semantic_callback(self, msg, cid):
        # Convert ROS image to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        cv_image = cv_image.astype(np.float32)

        # Normalize safely
        max_val = np.max(cv_image)
        if max_val > 0:
            cv_image = cv_image / max_val
        cv_image = (cv_image * 255).astype(np.uint8)
        cv_image = cv2.applyColorMap(cv_image, cv2.COLORMAP_JET)

        # Initialize video writer
        if cid not in self.semantic_writers:
            size = (cv_image.shape[1], cv_image.shape[0])
            filename = (
                f"{self.output_dir}/{self.timestamp}/semantic_recording_cam{cid}.mp4"
            )

            writer = cv2.VideoWriter(
                filename,
                cv2.VideoWriter_fourcc(*"mp4v"),
                self.camera_update_rate,
                size,
            )
            self.semantic_writers[cid] = (writer, filename)
            self.get_logger().info(f"Started recording semantic for cam {cid}")

        writer, _ = self.semantic_writers[cid]
        # Write frame
        with self.semantic_locks[cid]:
            writer.write(cv_image)

    def depth_callback(self, msg, cid):
        # Convert ROS image to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        cv_image = cv_image.astype(np.float32)

        # Normalize safely
        max_val = np.max(cv_image)
        if max_val > 0:
            cv_image = cv_image / max_val
        cv_image = (cv_image * 255).astype(np.uint8)
        cv_image = cv2.applyColorMap(cv_image, cv2.COLORMAP_JET)

        # Initialize video writer
        if cid not in self.depth_writers:
            size = (cv_image.shape[1], cv_image.shape[0])
            filename = (
                f"{self.output_dir}/{self.timestamp}/depth_recording_cam{cid}.mp4"
            )

            writer = cv2.VideoWriter(
                filename,
                cv2.VideoWriter_fourcc(*"mp4v"),
                self.camera_update_rate,
                size,
            )
            self.depth_writers[cid] = (writer, filename)
            self.get_logger().info(f"Started recording depth for cam {cid}")

        writer, _ = self.depth_writers[cid]
        # Write frame
        with self.depth_locks[cid]:
            writer.write(cv_image)

    def destroy_node(self):
        # Release writers
        for cid, (writer, filename) in self.image_writers.items():
            writer.release()
            self.get_logger().info(f"Saved {filename}")
        for cid, (writer, filename) in self.semantic_writers.items():
            writer.release()
            self.get_logger().info(f"Saved {filename}")
        for cid, (writer, filename) in self.depth_writers.items():
            writer.release()
            self.get_logger().info(f"Saved {filename}")

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    recorder = CameraRecorder()
    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(recorder)
    try:
        executor.spin()
    except KeyboardInterrupt:
        recorder.get_logger().info("Shutting down...")
    finally:
        recorder.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
