import calibration
import cv2
import matplotlib.pyplot as plt
import os
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import Image
from functools import partial
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import time
import numpy as np

def convert_float64_to_mono8(float_img):
    # Normalize to 0–255
    normalized = cv2.normalize(float_img, None, 0, 255, cv2.NORM_MINMAX)

    # Convert to uint8
    mono8_img = normalized.astype(np.uint8)

    return mono8_img


def undistortPoint(point, D):
    """
    Shifts a point in Camera normalized camera coordinates to new position according to distortion factor D
    :param Tuple[x,y,1] point:  Undistorted point with x,y,1 coordinates in camera frame
    :param Tuple[dx, dy] D:     Distortion factor in x and y direction
    :return: Distorted point
    :rtype: Tuple[x,y,1]
    """
    dist_x, dist_y = D[0], D[1]

    # r^2 = (x^2 + y^2)
    r_squared = point[0] ** 2 + point[1] ** 2

    # Calculate undistortion. PUndist = PCamFrame * (1 + PDistX * r² + PDistY * r² )
    point_undistorted = point * (1 + dist_x * r_squared + dist_y * r_squared**2)

    # Ensure that last factor is still 1
    point_undistorted[2] = 1

    # Return distorted value
    return point_undistorted


def inside_image(width, height, coordinates):
    # TODO: double check height and width order
    in_range = (
        (coordinates[0] > 0)
        & (coordinates[0] < width)
        & (coordinates[1] > 0)
        & (coordinates[1] < height)
    )
    # Convert boolean result to an integer array (1 for True, 0 for False)
    in_range = in_range.astype(int)

    # Example: returning the difference between the two elements
    return in_range

def world2camera(Pw, K, R, t, width, height):
    # build projection matrix
    T = np.concatenate((R, t), axis=1)

    # project world points into camera coordinates
    Pc = np.matmul(T, Pw)  # shape (3, N)

    # normalize each column by its own z
    Pc_norm = Pc / Pc[2, :]   # broadcast division

    # intrinsics
    p = np.matmul(K, Pc_norm)
    uv = p[:-1]

    # check if inside image
    inside_image_partial = partial(inside_image, width, height)
    inside_column = np.apply_along_axis(inside_image_partial, axis=0, arr=uv)
    inside_column = inside_column.reshape(1, -1)

    new_uv = np.vstack([uv, inside_column])
    return new_uv

class CameraBirdEyeViewPublisher(Node):

    def __init__(self):

        super().__init__("camera_bird_eye_view_publisher")

        self.declare_parameter("camera_id", "164")
        self.declare_parameter("point_start", "4, -4")
        self.declare_parameter("point_end", "24, 6")
        self.declare_parameter("resolution", 0.02)
        self.declare_parameter("input_img", "image_raw")
        self.input_img = self.get_parameter("input_img").get_parameter_value().string_value
        

        self.camera_id = self.get_parameter("camera_id").value
        self.point_start = [float(value) for value in self.get_parameter("point_start").value.split(" ")]
        self.point_end = [float(value) for value in self.get_parameter("point_end").value.split(" ")]
        self.resolution = self.get_parameter("resolution").value

        self.cv_bridge = CvBridge()
        self.Pw = np.array(
            np.meshgrid(
                np.arange(self.point_start[0], self.point_end[0], step=self.resolution),
                np.arange(self.point_start[1], self.point_end[1], step=self.resolution),
                [0],
                [1],
            )
        ).reshape(4, -1)
        self.Pw_width = int(
            np.abs(self.point_end[0] - self.point_start[0]) / self.resolution
        )
        self.Pw_height = int(
            np.abs(self.point_end[1] - self.point_start[1]) / self.resolution
        )
        name = f"{self.camera_id}.yaml"
        print(
            "Reading calibration file from CALIB_PATH: ", os.environ.get("CALIB_PATH")
        )
        intrinsic_yaml_file = os.environ.get("CALIB_PATH", ".") + f"/intrinsic/{name}"
        extrinsic_yaml_file = os.environ.get("CALIB_PATH", ".") + f"/extrinsic/{name}"
        calib = calibration.Camera_config(
            intrinsic_yaml_file, extrinsic_yaml_file, name
        )

        self.K = calib.in_K
        self.t_vec = calib.ex_t_vec 
        self.rot_matrix = calib.ex_rot_mat 
        self.width = calib.width
        self.height = calib.height

        self.uv_s = np.empty((3, self.Pw.shape[1]))
        self.uv_s = world2camera(
            self.Pw, self.K, self.rot_matrix, self.t_vec, self.width, self.height
        )

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        # Image callback
        if self.input_img in ["colored_map", "labels_map"]:
            topic_name = f"static_agents/semantic_camera_{self.camera_id}/{self.input_img}"
        else:
            topic_name = f"static_agents/camera_{self.camera_id}/{self.input_img}"

        self.image_sub = self.create_subscription(
            Image,
            topic_name,
            self.project_area_callback,
            10,
        )
        # Image publisher
        self.projection_publisher = self.create_publisher(
            Image, f"static_agents/camera_{self.camera_id}/image_projected", qos_profile
        )

    def project_area_callback(self, msg: Image):
        cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        

        num_coords = self.uv_s.shape[1]
        coords_color = np.zeros(num_coords)
        image = self.uv_s.astype(int)
        for i in range(num_coords):
            x = image[0][i]
            y = image[1][i]
            valid = image[2][i]
            if valid:
                coords_color[i] = cv_image[y][x]
            else:
                coords_color[i] = 0

        image_cropped = np.reshape(coords_color, (self.Pw_height, self.Pw_width))
        mono_img = convert_float64_to_mono8(image_cropped)

        image_msg = self.cv_bridge.cv2_to_imgmsg(mono_img, encoding="mono8")
        image_msg.header = msg.header
        self.projection_publisher.publish(image_msg)

def main():

    rclpy.init()
    node = CameraBirdEyeViewPublisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
