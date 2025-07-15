import rclpy
from rclpy.node import Node
from sensor_msgs_py import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from pathlib import Path
import cv2
import time
from ament_index_python.packages import get_package_share_directory
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


class PreparePointCloud2Data(Node):
    def __init__(self):
        super().__init__('prepare_pointcloud2_data')

        self.declare_parameter('topic_name','_internal_rpi_topic')
        topic_name = self.get_parameter('topic_name').get_parameter_value().string_value

        qos_profile = QoSProfile(depth=10,
                                 reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.VOLATILE)

        self.publisher_ = self.create_publisher(PointCloud2, topic_name, qos_profile)


        self.declare_parameter('image_scale', 0.04)
        self.image_scale = self.get_parameter('image_scale').get_parameter_value().double_value


        self.images_folder_path = Path(
            get_package_share_directory('realize_pointcloud_images'),
            'bev_img'
        )

        # TODO: Possibly a more reliable solution, 'handshake' for example,
        #       to make sure it is the correct subscriber. This is good enough though.
        while self.publisher_.get_subscription_count() != 1:
            self.get_logger().info('Waiting for recorder to subscribe...')
            time.sleep(1)

        self.get_logger().info('Subscriber detected')
        self.execute_callback()

    def execute_callback(self):
        # TODO: Introduce the ability to check wether conversion is needed or not.

        self.get_logger().info('Starting process...')

        sorted_images = sorted([f for f in self.images_folder_path.iterdir() if f.suffix==".jpg"])
        num_imgs = len(sorted_images)
        for iter, image_path in enumerate(sorted_images):
            cloud_msg = self.convert_to_pc2(image_path)
            self.publisher_.publish(cloud_msg)
            self.get_logger().info(f'published {iter+1}/{num_imgs} images')

        self.publisher_.wait_for_all_acked()
        self.get_logger().info('All images published, cancel recording by pressing ctrl+c')

    # ===========================================================================

    def extract_jpg_data(self, image_path):
        # Extract image data from file
        image = cv2.imread(str(image_path))
        height, width, _ = image.shape

        data_package = []
        for h in range(height):
            for w in range(width):
                # Define positional data
                x = (w - width / 2) * self.image_scale
                y = (h - height / 2) * self.image_scale
                z = 0.0

                # Define color data
                b, g, r = image[h, w]
                rgb = (int(r) << 16) | (int(g) << 8) | int(b)

                # Save data
                data_package.append((x, y, z, rgb))
        return data_package


    def convert_to_pc2(self, image_path):

        points = self.extract_jpg_data(image_path)
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1)
        ]

        header = Header()
        header.frame_id = 'real_images'
        header.stamp = self.get_clock().now().to_msg()

        cloud_msg = point_cloud2.create_cloud(header, fields, points)
        return cloud_msg
        
            
def main(args=None):
    rclpy.init(args=None)
    node = PreparePointCloud2Data()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()