from visualization_msgs.msg import MarkerArray, Marker
from rclpy.node import Node
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Pose, PoseStamped, Twist
from ros_gz_interfaces.srv import SetEntityPose
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import rclpy.time
import rclpy.duration
import tf2_ros
import tf2_geometry_msgs
from tf2_ros import TransformException
import math

class ScenarioReplayCmdVel(Node):
    def __init__(self):
        super().__init__('scenario_replay_cmd_vel')

        # Parameters
        self.declare_parameter('robot_namespaces', ['robot_agent_1',  'robot_agent_2', 'robot_agent_3', 'robot_agent_4'])
        self.declare_parameter('entity_topic', 'entity_topic')
        self.declare_parameter('gazebo_teleport_service', '/world/default/set_pose')
        self.declare_parameter('target_frame_id', 'map')  # Target frame for transformations
        self.declare_parameter('parking_corner_x', -20.0)
        self.declare_parameter('parking_corner_y', -20.0)
        self.declare_parameter('parking_corner_z', 0.5)
        self.declare_parameter('parking_spacing', 2.0)
        self.declare_parameter('transform_timeout', 1.0)  # Timeout for TF lookups
        self.declare_parameter('extracted_fps', 10.0)  # FPS from bag data
        self.declare_parameter('max_linear_velocity', 2.0)  # Maximum linear velocity m/s
        self.declare_parameter('max_angular_velocity', 1.0)  # Maximum angular velocity rad/s

        # Get parameters
        self.available_robots = self.get_parameter('robot_namespaces').get_parameter_value().string_array_value
        self.entity_topic_name = self.get_parameter('entity_topic').get_parameter_value().string_value
        self.teleport_service_name = self.get_parameter('gazebo_teleport_service').get_parameter_value().string_value
        self.target_frame_id = self.get_parameter('target_frame_id').get_parameter_value().string_value
        
        self.parking_corner_x = self.get_parameter('parking_corner_x').get_parameter_value().double_value
        self.parking_corner_y = self.get_parameter('parking_corner_y').get_parameter_value().double_value
        self.parking_corner_z = self.get_parameter('parking_corner_z').get_parameter_value().double_value
        self.parking_spacing = self.get_parameter('parking_spacing').get_parameter_value().double_value
        self.transform_timeout = self.get_parameter('transform_timeout').get_parameter_value().double_value
        self.extracted_fps = self.get_parameter('extracted_fps').get_parameter_value().double_value
        self.max_linear_velocity = self.get_parameter('max_linear_velocity').get_parameter_value().double_value
        self.max_angular_velocity = self.get_parameter('max_angular_velocity').get_parameter_value().double_value

        # Calculate time step between bag messages
        self.time_step = 1.0 / self.extracted_fps if self.extracted_fps > 0 else 0.1

        # TF2 setup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Mapping from marker IDs to robot namespaces (permanent once assigned)
        self.id_to_robot_mapping = {}
        self.robot_to_id_mapping = {}
        
        # Available robots pool (robots not yet assigned to an ID)
        self.available_robots_pool = set(self.available_robots)
        
        # Track which marker IDs are currently active
        self.active_marker_ids = set()
        
        # Store previous positions for each robot to calculate cmd_vel
        self.robot_previous_positions = {}  # robot_name -> Pose
        self.robot_current_positions = {}   # robot_name -> Pose
        
        # Track if robot needs initial teleportation
        self.robot_needs_teleport = set()

        # Log available robots and configuration
        self.get_logger().info(f'Available robots: {self.available_robots}')
        self.get_logger().info(f'Target frame: {self.target_frame_id}')
        self.get_logger().info(f'Transform timeout: {self.transform_timeout}s')
        self.get_logger().info(f'Extracted FPS: {self.extracted_fps:.2f} Hz')
        self.get_logger().info(f'Time step: {self.time_step:.3f}s')
        self.get_logger().info(f'Max velocities: linear={self.max_linear_velocity:.1f} m/s, angular={self.max_angular_velocity:.1f} rad/s')

        # QoS profile for reliable communication
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        # Callback group for service calls
        self.callback_group = ReentrantCallbackGroup()

        # Subscriber to entity topic
        self.entity_subscriber = self.create_subscription(
            MarkerArray,
            self.entity_topic_name,
            self.entity_callback,
            qos_profile,
            callback_group=self.callback_group
        )

        # Service client for Gazebo teleportation (for initial positioning)
        self.teleport_client = self.create_client(
            SetEntityPose,
            self.teleport_service_name,
            callback_group=self.callback_group
        )
        
        # Create cmd_vel publishers for each robot
        self.cmd_vel_publishers = {}
        for robot_name in self.available_robots:
            topic_name = f'/{robot_name}/replay_vel'
            publisher = self.create_publisher(Twist, topic_name, qos_profile)
            self.cmd_vel_publishers[robot_name] = publisher
            self.get_logger().info(f'Created cmd_vel publisher for {robot_name} on {topic_name}')
        
        # Wait for service to be available
        self.get_logger().info('Waiting for Gazebo teleport service...')
        self.teleport_client.wait_for_service(timeout_sec=10.0)
        self.get_logger().info('Gazebo teleport service available!')
        
        # Initialize by parking all robots in the corner
        self.park_all_robots()

    def is_text_marker(self, marker: Marker) -> bool:
        """Check if a marker is a text marker"""
        return marker.type == Marker.TEXT_VIEW_FACING

    def transform_pose_to_target_frame(self, pose: Pose, source_frame: str) -> Pose:
        """Transform a pose from source frame to target frame using TF2"""
        try:
            # Create PoseStamped message
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = source_frame
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            pose_stamped.pose = pose

            # Get the transform
            transform = self.tf_buffer.lookup_transform(
                self.target_frame_id,
                source_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=self.transform_timeout)
            )

            # Transform the pose
            transformed_pose_stamped = tf2_geometry_msgs.do_transform_pose_stamped(pose_stamped, transform)
            
            self.get_logger().debug(
                f'Transformed pose: {source_frame}({pose.position.x:.3f}, {pose.position.y:.3f}, {pose.position.z:.3f}) '
                f'-> {self.target_frame_id}({transformed_pose_stamped.pose.position.x:.3f}, '
                f'{transformed_pose_stamped.pose.position.y:.3f}, {transformed_pose_stamped.pose.position.z:.3f})'
            )
            
            return transformed_pose_stamped.pose

        except TransformException as e:
            self.get_logger().error(f'Failed to transform pose from {source_frame} to {self.target_frame_id}: {e}')
            # Return original pose as fallback
            return pose
        except Exception as e:
            self.get_logger().error(f'Unexpected error during pose transformation: {e}')
            return pose

    def calculate_cmd_vel(self, robot_name: str, target_pose: Pose) -> Twist:
        """Calculate cmd_vel for differential drive robot to reach target position"""
        twist = Twist()
        
        if robot_name not in self.robot_previous_positions:
            # No previous position, can't calculate velocity
            self.get_logger().debug(f'No previous position for {robot_name}, sending zero velocity')
            return twist
        
        prev_pose = self.robot_previous_positions[robot_name]
        
        # Calculate displacement in global frame
        dx_global = target_pose.position.x - prev_pose.position.x
        dy_global = target_pose.position.y - prev_pose.position.y
        
        # Get current robot orientation
        current_yaw = self.get_yaw_from_quaternion(prev_pose.orientation)
        
        # Transform displacement to robot's local frame
        dx_local = dx_global * math.cos(current_yaw) + dy_global * math.sin(current_yaw)
        dy_local = -dx_global * math.sin(current_yaw) + dy_global * math.cos(current_yaw)
        
        # For differential drive: linear.x is forward/backward, angular.z is rotation
        # Calculate required linear velocity (forward/backward in robot frame)
        linear_velocity = dx_local / self.time_step
        
        # Calculate required angular velocity to point towards target
        # If we have significant lateral displacement, we need to turn
        if abs(dy_local) > 0.01:  # Small threshold to avoid jitter
            # Calculate the angle to the target in robot frame
            target_angle_local = math.atan2(dy_local, dx_local)
            angular_velocity = target_angle_local / self.time_step
        else:
            angular_velocity = 0.0
        
        # Limit velocities
        if abs(linear_velocity) > self.max_linear_velocity:
            linear_velocity = math.copysign(self.max_linear_velocity, linear_velocity)
            self.get_logger().debug(f'{robot_name}: Limited linear velocity to {self.max_linear_velocity:.2f} m/s')
        
        if abs(angular_velocity) > self.max_angular_velocity:
            angular_velocity = math.copysign(self.max_angular_velocity, angular_velocity)
            self.get_logger().debug(f'{robot_name}: Limited angular velocity to {self.max_angular_velocity:.2f} rad/s')
        
        # Set velocities
        twist.linear.x = linear_velocity
        twist.angular.z = angular_velocity
        
        self.get_logger().debug(
            f'{robot_name}: diff-drive cmd_vel - '
            f'linear: {twist.linear.x:.3f} m/s, angular: {twist.angular.z:.3f} rad/s '
            f'(dx_global: {dx_global:.3f}, dy_global: {dy_global:.3f}, '
            f'dx_local: {dx_local:.3f}, dy_local: {dy_local:.3f}, '
            f'yaw: {math.degrees(current_yaw):.1f}°)'
        )
        
        return twist

    def get_yaw_from_quaternion(self, q) -> float:
        """Extract yaw angle from quaternion"""
        # Convert quaternion to yaw angle
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle: float) -> float:
        """Normalize angle to [-pi, pi] range"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def park_all_robots(self):
        """Park all available robots in the corner initially"""
        for i, robot_name in enumerate(self.available_robots):
            parking_pose = self.get_parking_pose(i)
            self.teleport_robot_direct(robot_name, parking_pose)
            # Send zero velocity to parked robots
            self.send_cmd_vel(robot_name, Twist())
            self.get_logger().info(f'Parked robot {robot_name} in corner')

    def get_parking_pose(self, index: int) -> Pose:
        """Get parking pose for robot at given index (already in target frame coordinates)"""
        pose = Pose()
        pose.position.x = self.parking_corner_x + (index * self.parking_spacing)
        pose.position.y = self.parking_corner_y
        pose.position.z = self.parking_corner_z
        pose.orientation.w = 1.0  # No rotation
        return pose

    def get_available_robot(self) -> str:
        """Get the first available robot from the pool (ascending order)"""
        if self.available_robots_pool:
            # Get robots in ascending order by finding the one with the lowest index
            available_list = list(self.available_robots_pool)
            
            # Sort by the original index in self.available_robots to maintain order
            available_list.sort(key=lambda robot: self.available_robots.index(robot))
            
            # Take the first (lowest index) robot
            robot = available_list[0]
            self.available_robots_pool.remove(robot)
            
            self.get_logger().info(f'Assigning robot {robot} (index {self.available_robots.index(robot)}) from available pool. Remaining: {sorted(list(self.available_robots_pool), key=lambda r: self.available_robots.index(r))}')
            return robot
        
        self.get_logger().warn('No robots available in pool!')
        return None

    def park_robot(self, robot_name: str):
        """Park a robot in the corner and stop it"""
        # Find the robot's original index for parking position
        if robot_name in self.available_robots:
            index = self.available_robots.index(robot_name)
            parking_pose = self.get_parking_pose(index)
            self.teleport_robot_direct(robot_name, parking_pose)
            
            # Send zero velocity
            self.send_cmd_vel(robot_name, Twist())
            
            # Clear position history but DO NOT return robot to available pool
            # The robot remains permanently assigned to its marker ID
            if robot_name in self.robot_previous_positions:
                del self.robot_previous_positions[robot_name]
            if robot_name in self.robot_current_positions:
                del self.robot_current_positions[robot_name]
            
            self.get_logger().info(f'Parked robot {robot_name} (remains assigned to ID {self.robot_to_id_mapping.get(robot_name, "unknown")})')

    def send_cmd_vel(self, robot_name: str, twist: Twist):
        """Send cmd_vel message to robot"""
        if robot_name in self.cmd_vel_publishers:
            self.cmd_vel_publishers[robot_name].publish(twist)
            self.get_logger().debug(f'Sent cmd_vel to {robot_name}: linear={twist.linear.x:.3f}, angular={twist.angular.z:.3f}')
        else:
            self.get_logger().warn(f'No cmd_vel publisher found for robot {robot_name}')

    def entity_callback(self, msg: MarkerArray):
        """Process MarkerArray messages and send cmd_vel to robots"""
        current_marker_ids = set()
        
        # Log incoming markers for debugging - sort for consistent logging
        non_text_markers = [m for m in msg.markers if not self.is_text_marker(m)]
        marker_ids_in_msg = sorted([m.id for m in non_text_markers])
        self.get_logger().debug(f'Processing {len(non_text_markers)} markers with IDs: {marker_ids_in_msg}')
        
        # Process markers in sorted order by ID to ensure deterministic assignment
        sorted_markers = sorted([m for m in msg.markers if not self.is_text_marker(m)], key=lambda m: m.id)
        
        for marker in sorted_markers:
            marker_id = marker.id
            current_marker_ids.add(marker_id)
            
            # Transform pose from marker's frame to target frame
            world_pose = self.transform_pose_to_target_frame(marker.pose, marker.header.frame_id)
            
            # Check if this marker ID is already permanently assigned to a robot
            if marker_id in self.id_to_robot_mapping:
                robot_name = self.id_to_robot_mapping[marker_id]
                self.get_logger().debug(f'Marker ID {marker_id} already assigned to robot {robot_name}')
                
                # Update position history
                if robot_name in self.robot_current_positions:
                    self.robot_previous_positions[robot_name] = self.robot_current_positions[robot_name]
                
                self.robot_current_positions[robot_name] = world_pose
                
                # Calculate and send cmd_vel
                cmd_vel = self.calculate_cmd_vel(robot_name, world_pose)
                self.send_cmd_vel(robot_name, cmd_vel)
                
                # Mark as active if it wasn't before
                if marker_id not in self.active_marker_ids:
                    self.active_marker_ids.add(marker_id)
                    self.get_logger().info(f'Reactivated robot {robot_name} for marker ID {marker_id}')
            else:
                # New marker ID - assign a robot permanently in ascending order
                self.get_logger().info(f'New marker ID {marker_id} detected, assigning robot in ascending order...')
                self.get_logger().info(f'Current ID->Robot mapping: {dict(sorted(self.id_to_robot_mapping.items()))}')
                
                # Show available robots in order
                available_ordered = sorted(list(self.available_robots_pool), key=lambda r: self.available_robots.index(r))
                self.get_logger().info(f'Available robots pool (ordered): {available_ordered}')
                
                available_robot = self.get_available_robot()
                if available_robot:
                    # Check for duplicate assignment (debugging)
                    if available_robot in self.robot_to_id_mapping:
                        self.get_logger().error(f'ERROR: Robot {available_robot} is already assigned to ID {self.robot_to_id_mapping[available_robot]}!')
                        self.get_logger().error(f'Attempting to assign it to ID {marker_id}')
                        continue  # Skip this assignment to prevent duplicate
                    
                    # Create permanent bidirectional mapping
                    self.id_to_robot_mapping[marker_id] = available_robot
                    self.robot_to_id_mapping[available_robot] = marker_id
                    
                    # Teleport to initial position (first occurrence)
                    self.teleport_robot_direct(available_robot, world_pose)
                    
                    # Initialize position history
                    self.robot_current_positions[available_robot] = world_pose
                    # No previous position yet, so cmd_vel will be zero for first step
                    
                    # Send zero velocity for this timestep
                    self.send_cmd_vel(available_robot, Twist())
                    
                    # Mark as active
                    self.active_marker_ids.add(marker_id)
                    
                    robot_index = self.available_robots.index(available_robot)
                    self.get_logger().info(f'Successfully assigned robot {available_robot} (index {robot_index}) to marker ID {marker_id}')
                    self.get_logger().info(f'Updated mappings - ID->Robot: {dict(sorted(self.id_to_robot_mapping.items()))}')
                    self.get_logger().info(f'Updated mappings - Robot->ID: {dict(sorted(self.robot_to_id_mapping.items()))}')
                    
                    # Show remaining available robots in order
                    remaining_ordered = sorted(list(self.available_robots_pool), key=lambda r: self.available_robots.index(r))
                    self.get_logger().info(f'Available robots remaining (ordered): {remaining_ordered}')
                else:
                    self.get_logger().warn(f'No available robots to assign to marker ID {marker_id}')
        
        # Find marker IDs that are no longer present
        inactive_marker_ids = self.active_marker_ids - current_marker_ids
        
        # Stop robots whose markers are no longer active
        for marker_id in inactive_marker_ids:
            if marker_id in self.id_to_robot_mapping:
                robot_name = self.id_to_robot_mapping[marker_id]
                
                # Send zero velocity to stop the robot
                self.send_cmd_vel(robot_name, Twist())
                
                self.active_marker_ids.discard(marker_id)
                self.get_logger().info(f'Stopped robot {robot_name} - marker ID {marker_id} no longer active')
                
                # DO NOT remove from permanent mapping - robot stays assigned to this ID
                # The robot will just be inactive until the marker reappears

    def teleport_robot_direct(self, robot_name: str, pose: Pose):
        """Teleport a robot to a new pose in Gazebo with full pose including orientation"""
        if not self.teleport_client.service_is_ready():
            self.get_logger().warn('Teleport service not ready')
            return

        request = SetEntityPose.Request()
        request.entity.name = robot_name
        request.entity.type = 1  # MODEL type
        request.pose = pose  # Use the full pose including orientation
        
        # Extract yaw for logging
        yaw = self.get_yaw_from_quaternion(pose.orientation)
        
        self.get_logger().debug(
            f'Teleporting {robot_name} to {self.target_frame_id} pose: '
            f'x={pose.position.x:.3f}, y={pose.position.y:.3f}, z={pose.position.z:.3f}, '
            f'yaw={math.degrees(yaw):.1f}°'
        )

        # Call service asynchronously
        future = self.teleport_client.call_async(request)
        future.add_done_callback(
            lambda f: self.teleport_callback(f, robot_name)
        )

    def teleport_callback(self, future, robot_name: str):
        """Handle teleport service response"""
        try:
            response = future.result()
            if response.success:
                self.get_logger().debug(f'Successfully teleported {robot_name}')
            else:
                self.get_logger().warn(f'Failed to teleport {robot_name}')
        except Exception as e:
            self.get_logger().error(f'Teleport service call failed for {robot_name}: {e}')

    def get_robot_status(self):
        """Debug method to print current robot status"""
        active_robots = [self.id_to_robot_mapping[marker_id] for marker_id in sorted(self.active_marker_ids)]
        assigned_robots = sorted(list(self.robot_to_id_mapping.keys()), key=lambda r: self.available_robots.index(r))
        available_robots = sorted(list(self.available_robots_pool), key=lambda r: self.available_robots.index(r))
        
        self.get_logger().info(f'Active robots (ordered): {active_robots}')
        self.get_logger().info(f'Assigned robots (ordered): {assigned_robots}')
        self.get_logger().info(f'Available robots (ordered): {available_robots}')
        self.get_logger().info(f'ID to robot mapping (sorted): {dict(sorted(self.id_to_robot_mapping.items()))}')


def main(args=None):
    rclpy.init(args=args)
    
    node = ScenarioReplayCmdVel()
    
    # Use MultiThreadedExecutor for handling async service calls
    executor = MultiThreadedExecutor()
    
    try:
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()