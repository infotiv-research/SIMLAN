# pip install opencv-contrib-python
# pip install transforms3d==0.4.1

import os
import calibration
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
import numpy as np
import os
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from time import time
import argparse
import cv2 as cv
import shutil
from rclpy.qos import qos_profile_system_default
from rcl_interfaces.srv import GetParameters

# Authors: Hamid Ebadi
# Run this after spawn-static-agents.sh script as below:
# python3 ./camera_viewer.py
# Background subtraction: https://docs.opencv.org/4.x/d1/dc5/tutorial_background_subtraction.html


# You may need to update
# - camera 'update_rate'
# - physics real_time_factor
# - physics real_time_update_rate


class StatusClient(Node):
    def __init__(self):
        super().__init__("status_client")
        self.client = self.create_client(GetParameters, "status_server")
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for status service...")
        self.request = GetParameters.Request()

    def send_request(self):
        future = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            # Extract and log the `double_value` field from each ParameterValue
            values = [param.double_value for param in future.result().values]
            self.get_logger().info(f"status received: {values}")
        else:
            self.get_logger().error("Failed to call service")
        return values


class MultiCameraSubscriber(Node):
    def __init__(self, camera_id):
        super().__init__("image_subscriber")
        # self.node = StatusClient()

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.background_sub = None
        camera_conf = calibration.Camera_config(
            "intrinsic/" + camera_id + ".yaml",
            "extrinsic/" + camera_id + ".yaml",
            camera_id,
        )
        # create Background Subtractor objects
        if args.algo == "MOG2":
            self.background_sub = cv.createBackgroundSubtractorMOG2(
                history=500, varThreshold=8, detectShadows=False
            )
        elif args.algo == "KNN":
            self.background_sub = cv.createBackgroundSubtractorKNN(
                history=500, dist2Threshold=400.0, detectShadows=False
            )
        else:
            print("No BackgroundSubtractor for camera: ", camera_id)
        self.subscription = self.create_subscription(
            Image,
            "/static_agents/camera_" + camera_id + "/image_raw",
            lambda msg, camera_id=camera_id: self.image_callback(
                msg, camera_id, camera_conf
            ),
            qos_profile,  # QoS
        )
        self.cv_bridge = CvBridge()

    def image_callback(self, msg, camera_id, camera_conf):
        timestamp = str(int(time()))
        # Convert ROS Image message to OpenCV format imgmsg_to_cv
        cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        cv_image_rotated = cv.rotate(cv_image, cv.ROTATE_90_CLOCKWISE)
        np_image = np.array(cv_image_rotated)

        # status_response = self.node.send_request()
        # print(status_response)
        # if (
        #    status_response[0] == 41.0
        #    and status_response[1] == 41.0
        #    and status_response[2] == 41.0
        # ):
        #    print("Received End Sequence")
        #    raise Exception("End Sequence")

        if args.action == "save":
            # cv.imshow('raw-image-'  + camera_id , cv_image)
            cv.imwrite(
                os.path.join(
                    DATA_DIR,
                    "raw_" + timestamp + "_" + camera_id + "_"
                    # + (",".join(map(str, status_response)))
                    + ".png",
                ),
                cv_image,
            )
            self.destroy_subscription(self.subscription)
            # cv.waitKey(1)
        elif args.action == "screenshot":
            print("Screeshot timers", timestamp, start_timestamp, args.shottime)
            cv.imwrite(
                os.path.join(DATA_DIR, "screenshot_" + camera_id + ".png"), cv_image
            )
            if int(timestamp) > start_timestamp + args.shottime:
                exit(0)
        elif args.action == "removebg":
            fgMask = self.background_sub.apply(cv_image)
            # cv.imshow('mask-image-'  + camera_id , fgMask)
            cv.imwrite(
                os.path.join(DATA_DIR, "mask_" + timestamp + "_" + camera_id + ".png"),
                fgMask,
            )
            # cv.waitKey(1)

        else:
            print("unknown action" + args.action)


start_timestamp = int(time())


def main(camera):

    rclpy.init()

    try:
        node = MultiCameraSubscriber(camera)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--action", help="continuously save camera images (for background extraction)"
    )
    parser.add_argument("--camera", help="camera id")
    parser.add_argument(
        "--shottime",
        type=int,
        help="after how many seconds take camera shots",
        default=5,
    )

    parser.add_argument(
        "--algo", help="Background subtraction method (KNN, MOG2).", default="MOG2"
    )
    args = parser.parse_args()

    DATA_DIR = (
        os.path.dirname(os.path.abspath(__file__)) + "/camera_data/" + args.camera
    )
    shutil.rmtree(DATA_DIR, ignore_errors=True)
    os.mkdir(DATA_DIR)

    main(args.camera)
