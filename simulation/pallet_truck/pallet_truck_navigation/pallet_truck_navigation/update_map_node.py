import cv2
import yaml
import os
import rclpy
from rclpy.node import Node
import numpy as np
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy
from PIL import Image as PILImage
from tf2_ros import Buffer, TransformListener
from nav_msgs.msg import OccupancyGrid

RADIUS=1
MAP_PATH = "/home/ros/src/simulation/pallet_truck/pallet_truck_navigation/maps/warehouse.yaml" #path to warehouse.pgm
MAP_PUBLISHER_FREQUENCY=1 #hz
OBSTACLE_VALUE=100
FREE_SPACE_VALUE=0
UNKNOWN_SPACE_VALUE=-1
MAX_PGM_VALUE=255

class MapUpdater(Node):
    def __init__(self):
        super().__init__('map_updater', namespace='')
 
        self.declare_parameter("namespace", "robot_agent_1")
        self.declare_parameter("all_namespaces", ["robot_agent_1"])
        self.namespace = self.get_parameter("namespace").get_parameter_value().string_value
        self.all_namespace=self.get_parameter("all_namespaces").value

        # Load map
        try:
            with open(MAP_PATH) as file:
                map_metadata=yaml.safe_load(file)
        except Exception as e:
            self.get_logger().error(f"Failed to load map YAML: {e}")
            raise e

        self.radius=RADIUS
        

        # Load original map data from yaml or fill with default values
        image_file = map_metadata["image"]
        origin=map_metadata.get("origin", [0.0,0.0])
        self.resolution=map_metadata.get("resolution", 0.01)
        occupied_thresh = map_metadata.get("occupied_thresh", 0.65)
        free_thresh = map_metadata.get("free_thresh", 0.25)
        negate = map_metadata.get("negate", 0)

        self.x_shift=-origin[0]
        self.y_shift=-origin[1]

        # Load map image.pgm
        self.original_map_path = os.path.join(os.path.dirname(MAP_PATH),image_file)
        self.original_image = PILImage.open(self.original_map_path).convert('L')
        self.original_array = np.array(self.original_image)

        # Format image.pgm to ros2 /map topic
        normalized = self.original_array.astype(np.float32) / MAX_PGM_VALUE
        if negate:
            normalized = 1.0 - normalized
        converted = np.full(normalized.shape, UNKNOWN_SPACE_VALUE, dtype=np.int8)
        converted[normalized >= free_thresh] = FREE_SPACE_VALUE
        converted[normalized <= occupied_thresh] = OBSTACLE_VALUE
        self.original_array = converted[::-1, :]
 
        # TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publisher with QoS compatible with Nav2 map subscriber
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
 
        self.create_timer(1/MAP_PUBLISHER_FREQUENCY, self.publish_map)

        self.map_pub = self.create_publisher(OccupancyGrid, f'/{self.namespace}/map', qos)

        self.get_logger().info('Map updater started...')
        
    def publish_map(self):
        occupancy_array=self.original_array.copy()

        for ns in self.all_namespace:
            
            parent_frame = "world"
            child_frame = f"{ns}/base_link"
 
            try:
                # Look up the latest transform
                t = self.tf_buffer.lookup_transform(
                    parent_frame,
                    child_frame,
                    rclpy.time.Time()
                )

                # Extract x and y positions and add offset to the size of the warehouse_map.pgm
                x = t.transform.translation.x + self.x_shift
                y = t.transform.translation.y + self.y_shift
 
            except Exception as e:
                self.get_logger().warn(f"Transform not available yet: {e}")
                continue
 
            # Create a white canvas (255 = white in mono8)
            obstacle = np.zeros(occupancy_array.shape, dtype=np.uint8)
            center_x = int(x / self.resolution)
            center_y = int(y / self.resolution)
            radius = int((self.radius) / self.resolution)
            cv2.circle(obstacle, (center_x, center_y), radius, OBSTACLE_VALUE, thickness=-1)
 
            # if namespace equals itself dont update map
            if ns != self.namespace:
                occupancy_array[obstacle==OBSTACLE_VALUE] = OBSTACLE_VALUE
 
        # Convert to int8 for OccupancyGrid
        occupancy_array = occupancy_array.astype(np.int8)
 
        # Prepare OccupancyGrid message
        grid = OccupancyGrid()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = f'{self.namespace}/map'
        grid.info.width = occupancy_array.shape[1]
        grid.info.height = occupancy_array.shape[0]
        grid.info.resolution = self.resolution  # adjust to your map scale hardcoded from warehouse.yaml
        grid.info.origin.position.x = -self.x_shift
        grid.info.origin.position.y = -self.y_shift
        grid.data = occupancy_array.flatten().tolist()
 
        # Publish map
        self.map_pub.publish(grid)
        self.get_logger().debug('Published updated occupancy map.')
 
def main(args=None):
    rclpy.init(args=args)
    node = MapUpdater()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
 
if __name__ == '__main__':
    main()
 
 