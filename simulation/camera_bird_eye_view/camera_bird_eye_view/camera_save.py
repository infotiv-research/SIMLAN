import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import numpy as np
from datetime import datetime


class CameraSaver(Node):
    def __init__(self):
        super().__init__("camera_saver")
        self.declare_parameter("camera_ids", "163")  # default value

        # Read the parameter
        camera_ids_param = (
            self.get_parameter("camera_ids").get_parameter_value().string_value
        )
        camera_ids = [
            int(x.strip())
            for x in camera_ids_param.replace(",", " ").split()
            if x.strip()
        ]

        self.get_logger().info(f"Using camera IDs: {camera_ids}")

        self.bridge = CvBridge()
        self.save_interval = 5.0  # seconds

        # Output folder
        self.output_dir = os.path.expanduser(
            "simulation/camera_bird_eye_view/camera_images"
        )
        os.makedirs(self.output_dir, exist_ok=True)

        # Dynamically generate camera configs
        self.cameras = []
        for cid in camera_ids:
            cam_base_dir = os.path.join(self.output_dir, f"camera_{cid}")
            depth_dir = os.path.join(cam_base_dir, "depth")
            seg_dir = os.path.join(cam_base_dir, "semantic_segmentation")
            image_dir = os.path.join(cam_base_dir, "image")
            os.makedirs(depth_dir, exist_ok=True)
            os.makedirs(seg_dir, exist_ok=True)
            os.makedirs(image_dir, exist_ok=True)

            self.cameras.append(
                {
                    "id": cid,
                    "name": f"depth_{cid}",
                    "topic": f"/static_agents/depth_camera_{cid}/image_raw",
                    "type": "depth",
                    "save_dir": depth_dir,
                }
            )
            self.cameras.append(
                {
                    "id": cid,
                    "name": f"semantic_{cid}",
                    "topic": f"/static_agents/semantic_camera_{cid}/colored_map",
                    "type": "segmentation",
                    "save_dir": seg_dir,
                }
            )
            self.cameras.append(
                {
                    "id": cid,
                    "name": f"camera_{cid}",
                    "topic": f"/static_agents/camera_{cid}/image_raw",
                    "type": "image",
                    "save_dir": image_dir,
                }
            )

        self.last_saved = 0.0
        self.subscribers = []
        for cam in self.cameras:
            sub = self.create_subscription(
                Image, cam["topic"], lambda msg, c=cam: self.image_callback(msg, c), 10
            )
            self.subscribers.append(sub)

        self.timer = self.create_timer(0.5, self.timer_callback)  # check every 0.5s
        self.latest_images = {}

    def image_callback(self, msg, cam):
        self.latest_images[cam["name"]] = (msg, cam)

    def timer_callback(self):
        now = self.get_clock().now().nanoseconds / 1e9  # seconds
        if now - self.last_saved >= self.save_interval:
            for cam_name, (msg, cam) in self.latest_images.items():
                try:
                    cv_image = self.bridge.imgmsg_to_cv2(
                        msg, desired_encoding="passthrough"
                    )
                    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                    filename = os.path.join(
                        cam["save_dir"], f"{cam_name}_{timestamp}.png"
                    )

                    if cam["type"] == "depth":
                        # Convert depth image to grayscale
                        cv_image = np.nan_to_num(
                            cv_image, nan=0.0, posinf=0.0, neginf=0.0
                        )
                        max_depth = 100.0  # meters
                        cv_image_clipped = np.clip(cv_image, 0, max_depth)
                        cv_image_norm = cv2.normalize(
                            cv_image_clipped, None, 0, 255, cv2.NORM_MINMAX
                        )
                        depth_gray = cv_image_norm.astype("uint8")
                        cv2.imwrite(filename, depth_gray)
                        self.get_logger().info(
                            f"[{cam_name}] Saved grayscale depth: {filename}"
                        )

                    elif cam["type"] == "segmentation":
                        cv2.imwrite(filename, cv_image)
                        self.get_logger().info(
                            f"[{cam_name}] Saved segmentation: {filename}"
                        )
                    elif cam["type"] == "image":
                        cv_image = self.bridge.imgmsg_to_cv2(
                            msg, desired_encoding="bgr8"
                        )
                        cv2.imwrite(filename, cv_image)
                        self.get_logger().info(
                            f"[{cam_name}] Saved raw image: {filename}"
                        )

                except Exception as e:
                    self.get_logger().error(f"Error saving image from {cam_name}: {e}")

            self.last_saved = now


def main(args=None):
    rclpy.init(args=args)
    node = CameraSaver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
