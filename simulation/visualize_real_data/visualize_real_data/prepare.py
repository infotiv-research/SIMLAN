from visualization_msgs.msg import MarkerArray, Marker
from rclpy.node import Node
import rclpy
import json
import time
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs_py import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from pathlib import Path
import cv2
import yaml

class PrepareRealData(Node):
    def __init__(self):
        super().__init__('prepare_real_data')

        # Declare parameters
        # - possibly overridden in launch file, otherwise set
        #   to let process run independently in test mode.
        self.declare_parameter('image_scale', 0.04)
        self.declare_parameter('pointcloud_topic','factory_pointcloud')
        self.declare_parameter('images_folder', 'bev_img')
        self.declare_parameter('entity_topic', 'entity_marker')
        self.declare_parameter('json_file_name', 'trajectories.json')
        self.declare_parameter('set_frames', True)
        self.declare_parameter('frames_to_process', 1)
        self.declare_parameter('processing_time_limit', 0.8)
        self.declare_parameter('frame_id', 'real_images')
        self.declare_parameter('config_file_path', 'params.yaml')


        # Get and assign parameters
        # - First 3 are needed for PointCloud2 creation
        # - Next 2 are needed for MarkerArray creation
        # - set_frames is used limit the amount of frames processed
        # - frames_to_process sets the number of frames the process is limited to
        # - frame_id
        pointcloud_topic = self.get_parameter('pointcloud_topic').get_parameter_value().string_value
        self.image_scale = self.get_parameter('image_scale').get_parameter_value().double_value
        self.images_folder_path = self.get_parameter('images_folder').get_parameter_value().string_value
        entity_topic = self.get_parameter('entity_topic').get_parameter_value().string_value
        self.json_file_path = self.get_parameter('json_file_name').get_parameter_value().string_value
        self.set_frames = self.get_parameter('set_frames').get_parameter_value().bool_value
        self.frames_to_process = self.get_parameter('frames_to_process').get_parameter_value().integer_value
        self.processing_time = self.get_parameter('processing_time_limit').get_parameter_value().double_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.config_path = self.get_parameter('config_file_path').get_parameter_value().string_value


        # Create publishers
        qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
        self.pc_publisher_ = self.create_publisher(PointCloud2, pointcloud_topic, qos_profile)
        self.ent_publisher_ = self.create_publisher(MarkerArray, entity_topic, qos_profile)


        self._wait_for_subscriber()

        self.get_logger().info('Executing callback...')
        self.execute_callbacks()


    def execute_callbacks(self):

        # Extract trajectory data
        # TODO: Handle File/Path not found error
        with open(self.json_file_path, 'r') as file:
            data_dict = json.load(file)

        # Sort and collect image-paths
        # TODO: Handle File/Path not found error
        sorted_images = sorted(
            [f for f in Path(self.images_folder_path).iterdir() if f.suffix==".jpg"]
            )

        self._check_timestamps(sorted_images, data_dict)

        num_frames = len(sorted_images)
        if self.set_frames:

            self.get_logger().warn(f"set_frames = 'true',")

            if self.frames_to_process <= num_frames:
                num_frames = self.frames_to_process
            else:
                self.get_logger().warn(f"frames_to_process assigned value larger than available data,")

            self.get_logger().warn(f"{num_frames}/{len(sorted_images)} available frames will be processed")

        max_limit = self.processing_time
        for frame in range(num_frames):

            start = time.time()

            # TODO: match time stamp
            image_path = sorted_images[frame]
            cloud_msg = self._convert_to_pc2(image_path)

            marker_array = MarkerArray()
            frame_info = data_dict[frame]
            for object_info in frame_info['object_list']:

                # Create and save marker message
                entity_msg, text_msg = self._create_marker_message(object_info[0], object_info[1:])

                marker_array.markers.append(entity_msg)
                marker_array.markers.append(text_msg)

            # Check if publishing interval is consistent
            dt = time.time() - start
            if (dt <= max_limit):
                time.sleep( max_limit - dt )
            else:
                self.get_logger().warn(f'dt larger than {max_limit}, dt: {round(dt,3)}')

            # Publish messages
            self.pc_publisher_.publish(cloud_msg)
            self.ent_publisher_.publish(marker_array)
            self.get_logger().info(f'published {frame+1}/{num_frames} messages')

        # Make sure all images have been published before shutting down
        self.get_logger().info('awaiting publication verification')
        self.pc_publisher_.wait_for_all_acked()
        self.ent_publisher_.wait_for_all_acked()
        self.get_logger().info('All frames published. Shutting down...')


    # USED BY BOTH OR ALL ================================================

    def _wait_for_subscriber(self):

        while self.pc_publisher_.get_subscription_count() + \
                                    self.ent_publisher_.get_subscription_count() < 2:

            self.get_logger().info(f'Waiting for recorder to subscribe')
            time.sleep(1)


    def _check_timestamps(self, sorted_images, data_dict):
        
        saved_stamps = []
        for frame, frame_data in enumerate(data_dict):

            if frame >= len(sorted_images):
                break

            # Retrieve current timestamps according to data
            try:
                img_ts = int(Path(sorted_images[frame]).stem)
                dict_ts = int(frame_data['time_stamp'])

            except ValueError as e_msg:
                self.get_logger().error(
                    f"{e_msg}, at frame {frame}: Timestamp not convertable to int"
                )
                continue

            if img_ts != dict_ts:
                self.get_logger().error(
                    f"Mismatch in timestamps at frame {frame}, please check data"
                )    
                continue
            
            saved_stamps.append(img_ts)


        dt_array = [
            saved_stamps[idx+1]-saved_stamps[idx] for idx in range( len(saved_stamps) - 1 )
            ]
        

        for idx in range( len(dt_array) - 1 ):

            expected_dt = dt_array[idx]

            if dt_array[idx+1] != expected_dt:
                self.get_logger().warn(
                    f"Inconsistent dt between frames {idx} and {idx+1}"
                    )
                continue

            # Calculate fps, assuming dt in ms
            dt_fps = 1/expected_dt/0.001
            
        if len(dt_array) == 0: dt_fps = 1

        # Write to file:
        # TODO: Don't force this if the user don't want to?
        with open(self.config_path, 'r') as file:
            doc = yaml.safe_load(file)
        
        if doc['send_data']['extracted_fps'] != dt_fps:

            doc['send_data']['extracted_fps'] = dt_fps

            with open(self.config_path, 'w') as file:
                yaml.safe_dump(doc, file)
        
            self.get_logger().info(
                f"'extracted_fps' overridden to {dt_fps} in config and " \
                + "will be used when bagged messages are published. " \
                + "To decide fps yourself, please change this value in " \
                + "the config file manually."
                )
    
    # USED BY ONE ========================================================

    def _extract_jpg_data(self, image_path):

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

    def _convert_to_pc2(self, image_path):

        points = self._extract_jpg_data(image_path)

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1)
        ]

        header = Header()
        header.frame_id = self.frame_id
        header.stamp = self.get_clock().now().to_msg()

        return point_cloud2.create_cloud(header, fields, points)

# --

    def _create_marker_message(self, object_id, object_pos):
        """_Explanation_:
        Creates a marker message for one entity and its id-badge as
        a separate marker message
        """
        if object_id > 999:
            self.get_logger().error(f"Too many objects created, object_id will be overridden")

        # Used to align coord. system with image's.
        x_coef = -8
        y_coef = -0.2

        # Create entity and text markers
        entity_msg = Marker()
        entity_msg.type = Marker.SPHERE
        entity_msg.action = Marker.ADD

        entity_msg.id = object_id

        entity_msg.pose.position.x = object_pos[0] + x_coef
        entity_msg.pose.position.y = -object_pos[1] + y_coef
        entity_msg.pose.position.z = 0.5

        entity_msg.scale.x = 0.2  # Length
        entity_msg.scale.y = 0.2  # Width
        entity_msg.scale.z = 0.2  # Height

        text_msg = Marker()
        text_msg.type = Marker.TEXT_VIEW_FACING
        text_msg.action = Marker.ADD
        text_msg.text = "id:" + str(object_id)

        text_msg.pose.position.x = object_pos[0] + x_coef
        text_msg.pose.position.y = -object_pos[1] + y_coef + (-0.5)
        text_msg.pose.position.z = 0.5

        # Arbitrary number 1000, but enough so that entity-id isn't overridden
        text_msg.id = 1000 + object_id

        text_msg.scale.x = 0.5  # Length
        text_msg.scale.y = 0.5  # Width
        text_msg.scale.z = 0.5  # Height

        # The rest of their respective set-up is exactly the same,
        # so loop to preserve space
        for marker in [entity_msg, text_msg]:

            marker.header.frame_id = self.frame_id
            marker.header.stamp = self.get_clock().now().to_msg()

            # Set orientation
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0

            # Set color (yellow)
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0  # Fully opaque

        return entity_msg, text_msg


def main(args=None):
    rclpy.init(args=None)
    node = PrepareRealData()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()