from visualization_msgs.msg import MarkerArray, Marker
from rclpy.node import Node
import rclpy
import json
import time
import math
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs_py import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from pathlib import Path
import cv2
# import yaml

from ruamel.yaml import YAML # If module error - this is probably the cause.

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
        self.declare_parameter('preprocess_all_data', False)  # New parameter

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
        self.preprocess_all_data = self.get_parameter('preprocess_all_data').get_parameter_value().bool_value

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
        # # sorted_images = sorted(
        # #     [f for f in Path(self.images_folder_path).iterdir() if f.suffix==".jpg"]
        # # )

        self._check_timestamps(data_dict)

        num_frames = len(data_dict) # num_frames = len(sorted_images)

        if self.set_frames:
            self.get_logger().warn(f"set_frames = 'true',")

            if self.frames_to_process <= num_frames:
                num_frames = self.frames_to_process
            else:
                self.get_logger().warn(f"frames_to_process assigned value larger than available data,")

            self.get_logger().warn(f"{num_frames}/{len(data_dict)} available frames will be processed")
        
        max_limit = self.processing_time

        if self.preprocess_all_data:
            self.get_logger().warn(
                'Preprocessing all data before publishing, '
                'this may take a while depending on data size...'
            )
            preprocessed_clouds = []
            preprocessed_markers = []

        for frame in range(num_frames):
            start = time.time()

            image_path = Path(self.images_folder_path, str(data_dict[frame]['time_stamp']) + '.jpg') #sorted_images[frame]
            cloud_msg = self._convert_to_pc2(image_path)

            marker_array = MarkerArray()
            frame_info = data_dict[frame]
            for object_info in frame_info['object_list']:
                # Create and save marker message
                entity_msg, text_msg = self._create_marker_message(object_info[0], object_info[1:])

                marker_array.markers.append(entity_msg)
                marker_array.markers.append(text_msg)

            # Only have a wait if publishing is being done in real-time while processing
            if not self.preprocess_all_data:
                dt = time.time() - start
                if (dt <= 1/self.dt_fps):
                    time.sleep(1/self.dt_fps - dt)
                else:
                    self.get_logger().warn(
                        f'dt larger than frame interval {1/self.dt_fps}, dt: {round(dt,3)}'
                    )

            if self.preprocess_all_data:
                preprocessed_clouds.append(cloud_msg)
                preprocessed_markers.append(marker_array)
            else:
                # Publish messages
                self.pc_publisher_.publish(cloud_msg)
                self.ent_publisher_.publish(marker_array)

            self.get_logger().info(f'Processed {frame+1}/{num_frames} messages')

        if self.preprocess_all_data:
            self.get_logger().info('Preprocessing complete, publishing all preprocessed data using original timing...')

            # Publish remaining frames using original intervals
            for frame in range(num_frames-1):
                start = time.time()
                
                # Update timestamps to current time for published messages
                current_ros_time = self.get_clock().now().to_msg()
                preprocessed_clouds[frame].header.stamp = current_ros_time
                for marker in preprocessed_markers[frame].markers:
                    marker.header.stamp = current_ros_time

                # Publish messages
                self.pc_publisher_.publish(preprocessed_clouds[frame])
                self.ent_publisher_.publish(preprocessed_markers[frame])
                

                # Use the actual time interval from the original data
                frame_interval = self.dt_intervals[frame]  # dt_intervals[0] is the interval between frame 0 and 1
                self.get_logger().info(f'published {frame+1}/{num_frames} messages (interval: {frame_interval:.3f}s)')
                dt = time.time() - start
                if (dt <= frame_interval):
                    time.sleep(frame_interval - dt)
                else:
                    self.get_logger().warn(
                        f'dt larger than frame interval {frame_interval}, dt: {round(dt,3)}'
                    )
            
            # Publish the last frame (needs to be done outside of loop due to indexing the intervals)
            self.pc_publisher_.publish(preprocessed_clouds[-1])
            self.ent_publisher_.publish(preprocessed_markers[-1])
            self.get_logger().info(f'published {num_frames}/{num_frames} messages (interval: {frame_interval:.3f}s)')
            
                
        # Make sure all images have been published before shutting down
        self.get_logger().info('awaiting publication verification')

        self.pc_publisher_.wait_for_all_acked()
        self.ent_publisher_.wait_for_all_acked()

        self.get_logger().info(
            'All frames published. Rosbag will complete the recording, then shut down.'
        )

    # USED BY BOTH OR ALL ================================================

    def _wait_for_subscriber(self):
        while self.pc_publisher_.get_subscription_count() + \
                                    self.ent_publisher_.get_subscription_count() < 2:

            self.get_logger().info(f'Waiting for recorder to subscribe')
            time.sleep(1)

    def _check_timestamps(self, data_dict):
        saved_stamps = []
        for frame_data in data_dict:

            # Retrieve current timestamps according to data
            try:
                dict_ts = int(frame_data['time_stamp'])

                img_path = Path(self.images_folder_path, str(dict_ts) + '.jpg')
                if not img_path.exists():
                    self.get_logger().error(
                        f"Image file not found for timestamp {dict_ts}"
                    )
                    continue

            except ValueError as e_msg:
                self.get_logger().error(
                    f"{e_msg}, at frame {frame_data['time_stamp']}: Timestamp not convertable to int"
                )
                continue
            
            saved_stamps.append(dict_ts)

        dt_array = [
            saved_stamps[idx+1]-saved_stamps[idx] for idx in range( len(saved_stamps) - 1 )
        ]
        
        # Convert dt_array from milliseconds to seconds and store for later use
        self.dt_intervals = [dt / 1000.0 for dt in dt_array]
        
        # Calculate average dt and fps from all intervals
        if len(dt_array) > 0:
            # Calculate statistics
            avg_dt = sum(dt_array) / len(dt_array)
            min_dt = min(dt_array)
            max_dt = max(dt_array)
            
            # Calculate fps using average dt, assuming dt in ms
            self.dt_fps = 1 / (avg_dt * 0.001)
            
            # Log timing statistics
            self.get_logger().info(f"Timestamp analysis:")
            self.get_logger().info(f"  Average dt: {avg_dt:.1f} ms")
            self.get_logger().info(f"  Min dt: {min_dt} ms, Max dt: {max_dt} ms")
            self.get_logger().info(f"  Calculated average fps: {self.dt_fps:.2f} Hz")
            self.get_logger().info(f"  Individual intervals (s): {[f'{dt:.3f}' for dt in self.dt_intervals[:5]]}{'...' if len(self.dt_intervals) > 5 else ''}")
            
            # Warn about significant variations
            dt_variation = max_dt - min_dt
            if dt_variation > avg_dt * 0.1:  # More than 10% variation
                self.get_logger().warn(
                    f"Significant timing variation detected: {dt_variation} ms "
                    f"({dt_variation/avg_dt*100:.1f}% of average)"
                )
        else:
            self.dt_fps = 1
            self.dt_intervals = [1.0]  # Default 1 second interval
            self.get_logger().warn("No valid timestamp intervals found, defaulting to 1 fps")

        # Write average fps to file
        yaml = YAML()
        yaml.preserve_quotes = True

        with open(self.config_path, 'r') as file:
            doc = yaml.load(file)
        
        if doc['shared']['extracted_fps'] != self.dt_fps:
            doc['shared']['extracted_fps'] = self.dt_fps

            with open(self.config_path, 'w') as file:
                yaml.dump(doc, file)
        
            self.get_logger().info((
                f'extracted_fps set to {self.dt_fps:.2f} (average) in config and '
                 'will be used when bagged messages are published. '
                 'To decide fps yourself, please change this value in '
                 'the config file manually.'
            ))
        else:
            self.get_logger().info(f'Config already contains correct average fps: {self.dt_fps:.2f}')
    
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

    def _yaw_to_quaternion(self, yaw):
        """Convert yaw angle (radians) to quaternion"""
        return {
            'x': 0.0,
            'y': 0.0,
            'z': math.sin(yaw / 2.0),
            'w': math.cos(yaw / 2.0)
        }

    def _create_marker_message(self, object_id, object_pos):
        """_Explanation_:
        Creates a marker message for one entity and its id-badge as
        a separate marker message
        """

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

        entity_msg.text = "id:" + str(object_id) # Used to teleport the robots in gazebo

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

            # Set orientation - use yaw if available, otherwise default
            if len(object_pos) >= 4:
                # Convert yaw angle to quaternion
                yaw = object_pos[3]  # Orientation in radians
                quat = self._yaw_to_quaternion(yaw)
                marker.pose.orientation.x = quat['x']
                marker.pose.orientation.y = quat['y']
                marker.pose.orientation.z = quat['z']
                marker.pose.orientation.w = quat['w']
            else:
                # Default orientation (no rotation)
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