import sys
import cv2
import transforms3d  # pip3 install transforms3d==0.4.1
import math
import numpy
import numpy as np


# https://learnopencv.com/rotation-matrix-to-euler-angles/
# Checks if a matrix is a valid rotation matrix.
def isRotationMatrix(R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6


# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).
def rotationMatrixToEulerAngles(R):

    assert isRotationMatrix(R)

    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0

    return np.array([x, y, z])


class Camera_config:
    def __init__(self, intrinsic_yaml_file, extrinsic_yaml_file, name="camera_name"):

        self.camera_name = name
        intrinsic_yaml = cv2.FileStorage(intrinsic_yaml_file, cv2.FILE_STORAGE_READ)
        extrinsic_yaml = cv2.FileStorage(extrinsic_yaml_file, cv2.FILE_STORAGE_READ)

        # https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html
        # https://www.geeksforgeeks.org/calibratecamera-opencv-in-python/

        # 9 items: Camera matrix
        self.in_K = intrinsic_yaml.getNode("K").mat()
        # 5 items: Distortion coefficients
        self.in_D = intrinsic_yaml.getNode("D").mat()

        self.in_xi = intrinsic_yaml.getNode("xi").mat()  # 0

        # 3 items
        self.in_image_shape = intrinsic_yaml.getNode("image_shape").mat()

        # 9 items: rotation vectors
        self.ex_rot_mat = extrinsic_yaml.getNode("rot_mat").mat()

        # 3 items:  translation vectors
        self.ex_t_vec = extrinsic_yaml.getNode("t_vec").mat()

        # 12 items:
        self.ex_camera_matrix_p = extrinsic_yaml.getNode("camera_matrix_p").mat()

        # 3 items: xyz position
        self.ex_position = extrinsic_yaml.getNode("position").mat()

        # http://sdformat.org/tutorials?tut=specify_pose
        # The elements x y z specify the position vector (in meters), and the elements roll pitch yaw are Euler angles (in radians) that specify the orientation, which can be computed by an extrinsic X-Y-Z rotation

        self.x_ = self.ex_position[0][0]
        self.y_ = self.ex_position[1][0]
        self.z_ = self.ex_position[2][0]

        invR = numpy.linalg.inv(self.ex_rot_mat)
        neg_invR = numpy.negative(invR)
        neg_invR_t = numpy.dot(neg_invR, self.ex_t_vec)
        self.x = neg_invR_t[0][0]
        self.y = neg_invR_t[1][0]
        self.z = neg_invR_t[2][0]

        # other possible _AXES2TUPLE : https://github.com/matthew-brett/transforms3d/blob/main/transforms3d/euler.py#L148
        # https://www.andre-gaschler.com/rotationconverter/

        self.height = int(self.in_image_shape[0][0])
        self.width = int(self.in_image_shape[1][0])

        # 84 degree to 1.46607657 radians
        # http://sdformat.org/spec?ver=1.11&elem=sensor#camera_distortion

        self.k1 = self.in_D[0][0]
        self.k2 = self.in_D[1][0]
        self.p1 = self.in_D[2][0]
        self.p2 = self.in_D[3][0]
        self.k3 = self.in_D[4][0]

        # [OpenCV](https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html) uses five parameters, known as distortion coefficients given by like this: `k1, k2, p1, p2 , k3 # pay attention to the order`

        self.cx = self.in_K[0][2]
        self.cy = self.in_K[1][2]
        self.fx = self.in_K[0][0]
        self.fy = self.in_K[1][1]
        self.fov_x = 2 * math.atan(self.width / (2 * self.fx))
        self.fov_y = 2 * math.atan(self.height / (2 * self.fy))
        # The FoV is
        # ~84 degrees horizontal
        # ~53 degrees vertical
        # print(self.fov_x, self.fov_y) # 53.550787735526455 83.92250895589785

        self.fovhor = self.fov_x
        self.aspect_ratio = self.width / self.height

        # rpy, R, Q, Qx, Qy, Qz = cv2.RQDecomp3x3(self.ex_rot_mat)

        # R_temp = np.array([
        #     [0, -1 ,0],
        #     [0, 0, -1],
        #     [1, 0, 0]
        # ])
        R_world_to_cam = self.ex_rot_mat
        R_cam_to_gazebocam = np.array(
            [[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]]
        )
        R_world_to_gazebocam = np.matmul(R_cam_to_gazebocam, R_world_to_cam)
        R_gazebocam_to_world = np.linalg.inv(R_world_to_gazebocam)

        # First, it turns out that the cameras in gazebo use another coordinate system than what is usually used for cameras (e.g., assumed by the calibration methods in opencv).
        # Therefore, a rotation matrix R_cam_to_gazebocam to account is introduced.
        # Second, rather than computing the Euler angles from R_world_to_gazebocam, (which was our initial attempt), the Euler angles must be computed from R_gazebocam_to_world.

        self.r, self.p, self.w = rotationMatrixToEulerAngles(R_gazebocam_to_world)

    def __str__(self):
        return f"""<xacro:camera number="{self.camera_name}" x="{self.x}" y="{self.y}" z="{self.z}" r="{self.r}" p="{self.p}" w="{self.w}" width="{int(self.width)}" height="{int(self.height)}" k1="{self.k1}" k2="{self.k2}" k3="{self.k3}" p1="{self.p1}" p2="{self.p2}" horizontal_fov="{self.fovhor}" aspect_ratio="{self.aspect_ratio}" fx="{self.fx}" fy="{self.fy}" cx="{self.cx}" cy="{self.cy}"/>"""


if __name__ == "__main__":

    if len(sys.argv) != 4:
        print("incorrect number of arguments:", len(sys.argv))
        print(
            "first argument: [opencv_intrinsic.yaml] [opencv_extrinsic.yaml] [camera_name]"
        )

    intrinsic_yaml_file = sys.argv[1]
    extrinsic_yaml_file = sys.argv[2]
    name = sys.argv[3]

    print(Camera_config(intrinsic_yaml_file, extrinsic_yaml_file, name))
