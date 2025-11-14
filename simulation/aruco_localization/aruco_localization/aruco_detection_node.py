from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
import numpy as np
import cv2 as cv
import tf2_ros
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from rclpy.qos import qos_profile_sensor_data

"""_summary_
This node is used to listen to a camera and subscribe to its intrinsic values and raw camera data and detect potential aruco markers.
When a camera finds a marker, we run a function that get its pose and rotation relative to the camera and publish it to TF.
"""

# ### CV2 ARUCO DETECTION CONFIG PARAMETERS ###
aruco_dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_5X5_1000)
#aruco_parameters = cv.aruco.DetectorParameters()
aruco_parameters = cv.aruco.DetectorParameters_create()  
# These are key for low-res or off-center markers
aruco_parameters.adaptiveThreshWinSizeMin = 3
aruco_parameters.adaptiveThreshWinSizeMax = 23
aruco_parameters.adaptiveThreshWinSizeStep = 10
aruco_parameters.adaptiveThreshConstant = 4  # Lower = more sensitive (default is 7)
# aruco_parameters.minMarkerPerimeterRate *= 1 / 20  # Lower to detect smaller markers
aruco_parameters.maxMarkerPerimeterRate = 4.0
aruco_parameters.polygonalApproxAccuracyRate = 0.03
# aruco_parameters.minCornerDistanceRate *= 1 / 20
aruco_parameters.minDistanceToBorder = 1  # Very important for edge cases
aruco_parameters.cornerRefinementMethod = cv.aruco.CORNER_REFINE_SUBPIX
# aruco_parameters.minMarkerDistanceRate *= 1 / 20
aruco_marker_size = 0.5

R_cam_to_gazebocam = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])


class ArucoDetectionNode(Node):

    def __init__(self):
        super().__init__("aruco_detection_node")

        self.declare_parameter("camera_id", 163)
        self.camera_id = self.get_parameter("camera_id").value
        self.last_stamp = self.get_clock().now()
        self.get_logger().info(f"Initiated Camera: {self.camera_id}")

        # node attributes
        self.intrinsic_mat = np.array([])
        self.cv_bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Subscribers
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            "/static_agents/camera_" + str(self.camera_id) + "/camera_info",
            self.camera_info_callback,
            10,
        )
        self.image_sub = self.create_subscription(
            Image,
            "/static_agents/camera_" + str(self.camera_id) + "/image_raw",
            lambda msg, camera_id=self.camera_id: self.update_aruco_poses(
                msg, camera_id
            ),
            qos_profile_sensor_data,
        )

        # Publishers
        self.aruco_image_pub = self.create_publisher(Image, "/aruco_image", 10)

    # Callbacks
    def camera_info_callback(self, msg: CameraInfo):
        self.intrinsic_mat = np.array(msg.k).reshape(3, 3)
        self.get_logger().info("Camera info received")
        self.destroy_subscription(self.camera_info_sub)
        self.camera_info_sub = None

    # Source for this logic: https://automaticaddison.com/how-to-publish-tf-between-an-aruco-marker-and-a-camera/
    def update_aruco_poses(self, msg: Image, camera_id: int):

        if not self.intrinsic_mat.any():
            self.get_logger().warn(
                f"Skipping image from camera_{camera_id} — no intrinsics yet."
            )
            return
        
        cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        # Detect corners of an aruco
        corners, marker_ids, _ = cv.aruco.detectMarkers(
            cv_image, aruco_dictionary, parameters=aruco_parameters
        )

        if marker_ids is not None:

            # We receive the rvecs and tvecs of the aruco marker relative to the camera
            aruco_rvecs, aruco_tvecs, _ = cv.aruco.estimatePoseSingleMarkers(
                corners, aruco_marker_size, self.intrinsic_mat, None
            )

            for i, marker_id in enumerate(marker_ids):
                cv.aruco.drawDetectedMarkers(cv_image, corners, marker_ids)

                DEBUG = True
                if DEBUG:
                    # Draw axes on the detected markers
                    cv.drawFrameAxes(
                        cv_image,
                        self.intrinsic_mat,
                        None,
                        aruco_rvecs[i],
                        aruco_tvecs[i],
                        0.5,
                    )
                    aruco_image_msg = self.cv_bridge.cv2_to_imgmsg(
                        cv_image, encoding="bgr8"
                    )
                    # Publish the image to /aruco_image topic
                    self.aruco_image_pub.publish(aruco_image_msg)

                # We check if the z-axis is negative and so we flip the axis to correct this.
                aruco_rvec = aruco_rvecs[i][0]
                aruco_tvec = aruco_tvecs[i][0]

                rotation_matrix = np.eye(3)
                rotation_matrix = cv.Rodrigues(np.array(aruco_rvec))[0]

                # Correct the rotation of the openCV camera to Gazebo camera.
                aruco_tvec = np.dot(R_cam_to_gazebocam, aruco_tvec)

                # We do the same rotation with the rvecs to get it in gazebo space
                rotation_matrix = np.dot(R_cam_to_gazebocam, rotation_matrix)

                r = R.from_matrix(rotation_matrix)
                quat = r.as_quat()

                # Create the coordinate transform
                t = TransformStamped()
                t.header.stamp = self.get_clock().now().to_msg()

                t.header.frame_id = f"static_agents/camera_{camera_id}_link"
                t.child_frame_id = f"static_agents/camera_{camera_id}_marker_{marker_id[0]}"

                # Store the translation (i.e. position) information
                t.transform.translation.x = float(aruco_tvec[0])
                t.transform.translation.y = float(aruco_tvec[1])
                t.transform.translation.z = float(aruco_tvec[2])

                # Quaternion format
                t.transform.rotation.x = quat[0]
                t.transform.rotation.y = quat[1]
                t.transform.rotation.z = quat[2]
                t.transform.rotation.w = quat[3]
                # Send the transform
                self.tf_broadcaster.sendTransform(t)


def main():

    rclpy.init()
    node = ArucoDetectionNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
