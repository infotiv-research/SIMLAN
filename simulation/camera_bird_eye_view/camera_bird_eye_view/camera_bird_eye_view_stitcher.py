import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import cv2

class CameraBirdEyeViewStitcher(Node):

    def __init__(self):
        super().__init__("camera_bird_eye_view_stitcher")
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.declare_parameter("camera_ids", "163 164 165")
        self.declare_parameter("update_rate", 30)
        self.declare_parameter("grid_size", "20 10" )

        self.camera_ids = [int(s) for s in self.get_parameter("camera_ids").value.split(" ")]
        self.grid_size = [ int(s) for s in self.get_parameter("grid_size").value.split(" ")]
        self.update_rate = self.get_parameter("update_rate").value

        self.retrieved_images = {}
        self.cv_bridge = CvBridge()
        
        # Subscribers
        # Creating the list of image subscribers
        self.image_subscribers = []
        for camera_id in self.camera_ids:
            image_sub = self.create_subscription(
                            Image,
                            f"static_agents/camera_{camera_id}/image_projected",
                            lambda msg, camera_id=camera_id: self.store_image_callback(msg, camera_id),
                            10
                        )
            self.image_subscribers.append(image_sub)

        # timer for generating a stitched image
        self.timer = self.create_timer(
            1 / self.update_rate, self.generate_stitched_image_callback
        )
        
        # Publishers
        self.stitched_image_publisher = self.create_publisher(Image, "projected_images_stitched",qos_profile)

    def store_image_callback(self, msg, camera_id):
        cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
        self.retrieved_images[camera_id] = cv_image
    
    def generate_stitched_image_callback(self):
        if(self.retrieved_images == {}):
            return
        means = np.mean(list((self.retrieved_images.values())), axis=0)
        stitch = means.reshape((self.grid_size[1] ,self.grid_size[0]))
        stitch = cv2.normalize(stitch, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        stitch = cv2.flip(stitch, 0)
        msg = self.cv_bridge.cv2_to_imgmsg(stitch, encoding="mono8")
        self.stitched_image_publisher.publish(msg)

def main():

    rclpy.init()
    node = CameraBirdEyeViewStitcher()
    rclpy.spin(node)
    rclpy.shutdown()



if __name__ == "__main__":
    main()
