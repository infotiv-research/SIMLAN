#!/usr/bin/env python3
import os
from pathlib import Path
import json
import numpy as np
import cv2
import mediapipe as mp
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from camera_system.srv import CaptureMotionStatus


os.environ["MEDIAPIPE_MODEL_PATH"] = str(Path.home() / ".mediapipe/models")
os.makedirs(os.environ["MEDIAPIPE_MODEL_PATH"], exist_ok=True)


class PoseDetector(Node):
    def __init__(self):
        super().__init__("pose_detector")

        self.declare_parameter("output_dir", "DATASET")
        self.output_dir = (
            self.get_parameter("output_dir").get_parameter_value().string_value
        )

        self.camera_ids = [500, 501, 502, 503]  # List of camera IDs to subscribe to

        # Create separate directories for each camera
        self.camera_dirs = {}
        for camera_id in self.camera_ids:
            pose_dir = os.path.join(self.output_dir, f"camera_{camera_id}", "pose_data")
            image_dir = os.path.join(
                self.output_dir, f"camera_{camera_id}", "pose_images"
            )
            os.makedirs(pose_dir, exist_ok=True)
            os.makedirs(image_dir, exist_ok=True)
            self.camera_dirs[camera_id] = {"pose_dir": pose_dir, "image_dir": image_dir}
            self.get_logger().info(
                f"Camera {camera_id} - Pose: {pose_dir}, Images: {image_dir}"
            )

        # self.declare_parameter('camera_id', 502)
        # camera_id = self.get_parameter('camera_id').get_parameter_value().integer_value
        # self.camera_topic = f"/static_agents/camera_{camera_id}/image_raw"

        # self.declare_parameter('pose_dir', self.output_dir + '/pose_data')
        # self.declare_parameter('image_dir', self.output_dir + '/pose_images')

        # self.pose_dir = self.get_parameter('pose_dir').value
        # self.image_dir = self.get_parameter('image_dir').value

        # os.makedirs(self.pose_dir, exist_ok=True)
        # os.makedirs(self.image_dir, exist_ok=True)
        # self.get_logger().info(f"Pose directory: {self.pose_dir} and image directory: {self.image_dir}")

        self.current_motion_id = None
        self.should_capture = False

        self.bridge = CvBridge()
        self.mp_pose = mp.solutions.pose
        self.mp_drawing = mp.solutions.drawing_utils

        self.pose = self.mp_pose.Pose(
            static_image_mode=True,  # TODO: Hamid, Only those that you can detect by looking at the image and not the stream
            model_complexity=2,
            min_detection_confidence=0.2,  # higher confidence
            min_tracking_confidence=0.8,
        )

        # self.latest_image = None
        self.latest_image = {}
        for camera_id in self.camera_ids:
            topic = f"/static_agents/camera_{camera_id}/image_raw"
            self.latest_image[camera_id] = None
            self.create_subscription(
                Image,
                topic,
                lambda msg, cam_id=camera_id: self.store_image(msg, cam_id),
                1,
            )

        self.get_logger().info(f"Subscribed to camera topics: {topic}")
        # self.get_logger().info(f"Subscribed to camera topic: {self.camera_topic}")
        # Subscribe to camera image
        # self.image_sub = self.create_subscription(
        #     Image,
        #     # '/static_agents/camera_502/image_raw',
        #     self.camera_topic,
        #     self.store_image,
        #     1 # only the most recent image
        # )
        # self.get_logger().info(f"Subscribed to camera topic: {self.camera_topic}")
        # Service for synchronization with motion planner
        self.srv = self.create_service(
            CaptureMotionStatus, "capture_motion", self.motion_status_callback
        )

        self.timer = self.create_timer(
            0.2, lambda: self.process_latest_image(save=False)
        )
        self.get_logger().info(
            f"Detector node started with output directory: {self.output_dir}"
        )

    def store_image(self, msg, cam_id):
        # Store latest image read from camera
        self.latest_image[cam_id] = msg

    def motion_status_callback(self, request, response):
        """
        Handle motion capture requests from the motion planner
        If landmarks are detected: save pose data and image, return True
        If no landmarks are detected: return False
        """
        motion_id = str(request.bodymotionid)
        self.get_logger().info(f"Received take picture for {motion_id}")
        # self.results = None
        detection_success = False

        # Process cameras in order: 500 first, then 501
        for camera_id in self.camera_ids:
            if self.latest_image[camera_id] is not None:
                self.get_logger().info(
                    f"Processing camera {camera_id} for motion {motion_id}"
                )

                # Process this specific camera
                success = self.process_single_camera(camera_id, motion_id, save=True)
                if success:
                    detection_success = True
                    self.get_logger().info(
                        f"Camera {camera_id}: Detection and save completed"
                    )
                else:
                    self.get_logger().warn(
                        f"Camera {camera_id}: No pose landmarks detected"
                    )
            else:
                self.get_logger().warn(f"Camera {camera_id}: No image available")

        response.pose_data_capture = detection_success
        return response

    def process_single_camera(self, camera_id, motion_id, save=True):
        """
        Process a single camera's image for pose detection
        Returns True if pose landmarks were detected, False otherwise
        """
        if self.latest_image[camera_id] is None:
            return False

        print(f"Camera Processing from camera {camera_id}")
        cv_image = self.bridge.imgmsg_to_cv2(self.latest_image[camera_id], "bgr8")

        # Process with MediaPipe

        results = self.pose.process(cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB))

        if save and results.pose_landmarks:
            # Save pose data to camera-specific folder
            # TODO: Siyu, it is possible to use pose_world_landmarks for 3D coordinates
            self.save_pose_data(results.pose_landmarks, motion_id, camera_id)

            # Create display image with landmarks
            display_image = cv_image.copy()
            drawing_spec = self.mp_drawing.DrawingSpec(
                color=(0, 0, 255), thickness=1, circle_radius=2
            )
            self.mp_drawing.draw_landmarks(
                image=display_image,
                landmark_list=results.pose_landmarks,
                connections=self.mp_pose.POSE_CONNECTIONS,
                landmark_drawing_spec=drawing_spec,
            )

            # Save images to camera-specific folder
            self.save_pose_image(display_image, cv_image, motion_id, camera_id)

            # Show window with camera ID in title
            cv2.imshow(f"Camera View - {camera_id}", display_image)
            cv2.waitKey(1)

            return True

        return results.pose_landmarks is not None if save else True

    def process_latest_image(self, save=False):
        """
        Process latest images for display (non-save mode)
        """
        if not save:
            # Just process for display, don't save
            for camera_id in self.camera_ids:
                if self.latest_image[camera_id] is not None:
                    self.process_single_camera(camera_id, None, save=False)

    def save_pose_image(self, display_image, original_image, motionid, camera_id):
        """Save pose images to camera-specific directory"""
        image_dir = self.camera_dirs[camera_id]["image_dir"]

        display_filename = f"{motionid}_display.jpg"
        display_path = os.path.join(image_dir, display_filename)
        cv2.imwrite(display_path, display_image)

        original_filename = f"{motionid}_original.jpg"
        original_path = os.path.join(image_dir, original_filename)
        cv2.imwrite(original_path, original_image)

        self.get_logger().info(f"Camera {camera_id}: Saved pose images to {image_dir}")

    def save_pose_data(self, pose_landmarks, motionid, camera_id):
        """Save pose data to camera-specific directory"""
        pose_dir = self.camera_dirs[camera_id]["pose_dir"]

        # Save pose data as JSON file
        landmarks_data = []
        for idx, landmark in enumerate(pose_landmarks.landmark):
            landmarks_data.append(
                {
                    "index": idx,
                    "x": landmark.x,
                    "y": landmark.y,
                    "z": landmark.z,
                    "visibility": landmark.visibility,
                }
            )

        json_filename = f"{motionid}_pose.json"
        json_path = os.path.join(pose_dir, json_filename)

        with open(json_path, "w") as f:
            json.dump(landmarks_data, f, indent=2)

        self.get_logger().info(f"Camera {camera_id}: Saved pose data to {json_path}")


def main(args=None):
    rclpy.init(args=args)
    detector = PoseDetector()
    rclpy.spin(detector)
    detector.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
