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

class ScenarioReplayTeleport(Node):
    def __init__(self):
        super().__init__('scenario_replay_teleport')

        # Parameters
        self.declare_parameter('robot_namespaces', ['robot_agent_1',  'robot_agent_2', 'robot_agent_3', 'robot_agent_4'])
        self.declare_parameter('entity_topic', 'entity_topic')
        self.declare_parameter('target_frame_id', 'map')  # Target frame for transformations
        self.declare_parameter('extracted_fps', 10.0)  # FPS of the extracted data for cmd_vel calculation
        self.declare_parameter('use_cmd_vel', False)

        # Get parameters
        self.available_robots = self.get_parameter('robot_namespaces').get_parameter_value().string_array_value
        self.entity_topic_name = self.get_parameter('entity_topic').get_parameter_value().string_value
        self.target_frame_id = self.get_parameter('target_frame_id').get_parameter_value().string_value
        self.extracted_fps = self.get_parameter('extracted_fps').get_parameter_value().double_value
        self.use_cmd_vel = self.get_parameter('use_cmd_vel').get_parameter_value().bool_value
        
        # Use these to only allow certain marker IDs to be assigned to robots
        # For example if a robot gets teleported into a wall or shelf use this to exclude that robot, since it will absolutely kill the FPS of the simulation.
        self.allowed_marker_ids = {}  # Empty set means allow all IDs
        
        self.parking_corner_x = -20.0
        self.parking_corner_y = -20.0
        self.parking_corner_z = -0.5
        self.parking_spacing = 2 # x-axis
        self.transform_timeout = 1.0
        self.max_linear_velocity = 2.0
        self.max_angular_velocity = 2.0

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
        
        # Log available robots and TF configuration
        self.get_logger().info(f'Available robots: {self.available_robots}')
        self.get_logger().info(f'Target frame: {self.target_frame_id}')
        self.get_logger().info(f'Transform timeout: {self.transform_timeout}s')
        
        # Log allowed marker IDs filter
        if not self.allowed_marker_ids:
            self.get_logger().info('Allowed marker IDs: ALL (no filter)')
        else:
            self.get_logger().info(f'Allowed marker IDs: {sorted(list(self.allowed_marker_ids))}')

        if self.use_cmd_vel:
            # Store previous positions for each robot to calculate cmd_vel
            self.robot_previous_positions = {}  # robot_name -> Pose
            self.robot_current_positions = {}   # robot_name -> Pose
            
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

        # Service client for Gazebo teleportation
        self.teleport_client = self.create_client(
            SetEntityPose,
            '/world/default/set_pose',
            callback_group=self.callback_group
        )

        if self.use_cmd_vel:
            # Create cmd_vel publishers for each robot
            self.cmd_vel_publishers = {}
            for robot_name in self.available_robots:
                topic_name = f'/{robot_name}/scenario_vel'
                publisher = self.create_publisher(Twist, topic_name, qos_profile)
                self.cmd_vel_publishers[robot_name] = publisher
                self.get_logger().info(f'Created cmd_vel publisher for {robot_name} on {topic_name}')

            # Used to detect when a rosbag loops so the robots can be teleported back to their starting positions
            self.timestamp_jump_threshold = 5.0 # Seconds
            self.get_logger().info(f'Timestamp jump threshold: {self.timestamp_jump_threshold:.1f}s')
        
        # Wait for service to be available
        self.get_logger().info('Waiting for Gazebo teleport service...')
        self.teleport_client.wait_for_service(timeout_sec=10.0)
        self.get_logger().info('Gazebo teleport service available!')
        
        # Initialize by parking all robots in the corner
        self.park_all_robots()

        
        # Loop detection state
        self.last_message_timestamp = None
        self.loop_count = 0

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
            self.get_logger().error(f'Error details: transformed_pose_stamped type = {type(transformed_pose_stamped) if "transformed_pose_stamped" in locals() else "not defined"}')
            return pose


    def park_all_robots(self):
        """Park all available robots in the corner initially"""
        for i, robot_name in enumerate(self.available_robots):
            parking_pose = self.get_parking_pose(i)
            self.teleport_robot_direct(robot_name, parking_pose)
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
        """Get an available robot from the pool"""
        if self.available_robots_pool:
            return self.available_robots_pool.pop()
        return None

    def park_robot(self, robot_name: str):
        """Park a robot in the corner"""
        # Find the robot's original index for parking position
        if robot_name in self.available_robots:
            index = self.available_robots.index(robot_name)
            parking_pose = self.get_parking_pose(index)
            self.teleport_robot_direct(robot_name, parking_pose)
            self.get_logger().info(f'Parked robot {robot_name}')

    def is_marker_id_allowed(self, marker_id: int) -> bool:
        """Check if a marker ID is allowed to be assigned to robots"""
        if self.allowed_marker_ids == {}:
            return True  # No filter, allow all
        return marker_id in self.allowed_marker_ids

    def detect_bag_loop(self, current_timestamp_sec: float) -> bool:
        """Detect if the rosbag has looped by checking for timestamp jumps backwards"""
        if self.last_message_timestamp is None:
            self.last_message_timestamp = current_timestamp_sec
            return False
        
        # Calculate time difference
        time_diff = current_timestamp_sec - self.last_message_timestamp
        
        # Check for significant backward jump (indicating loop)
        if time_diff < -self.timestamp_jump_threshold:
            self.loop_count += 1
            self.get_logger().info(
                f'Bag loop detected! Loop #{self.loop_count} '
                f'(timestamp jumped from {self.last_message_timestamp:.3f}s to {current_timestamp_sec:.3f}s, '
                f'diff: {time_diff:.3f}s)'
            )
            self.last_message_timestamp = current_timestamp_sec
            return True
        
        self.last_message_timestamp = current_timestamp_sec
        return False

    def reset_robots_to_start(self):
        """Reset all robots to their starting positions when bag loops"""
        self.get_logger().info('Resetting all robots due to bag loop...')
        
        # Clear all robot assignments and position history
        self.id_to_robot_mapping.clear()
        self.robot_to_id_mapping.clear()
        self.active_marker_ids.clear()
        
        self.robot_previous_positions.clear()
        self.robot_current_positions.clear()
        
        # Return all robots to available pool
        self.available_robots_pool = set(self.available_robots)
        
        # Park all robots
        self.park_all_robots()
        
        self.get_logger().info(f'Reset complete. {len(self.available_robots)} robots available for assignment.')

    def entity_callback(self, msg: MarkerArray):
        """Process MarkerArray messages and update robot positions in Gazebo"""
        # Extract timestamp from message header (only relevant if using cmd_vel)
        if self.use_cmd_vel and msg.markers:
            # Use timestamp from first marker (they should all have same timestamp)
            message_timestamp = msg.markers[0].header.stamp.sec + msg.markers[0].header.stamp.nanosec * 1e-9
            
            # Check for bag loop
            if self.detect_bag_loop(message_timestamp):
                self.reset_robots_to_start()
                return  # Skip processing this frame to allow reset to complete
        
        current_marker_ids = set()
        
        # Process all markers in the array, but skip text markers
        for marker in msg.markers:
            # Skip text markers - they shouldn't control robot teleportation
            if self.is_text_marker(marker):
                self.get_logger().debug(f'Skipping text marker with ID {marker.id}')
                continue
            
            marker_id = marker.id
            
            # Check if this marker ID is allowed
            if not self.is_marker_id_allowed(marker_id):
                self.get_logger().info(f'Skipping marker ID {marker_id} - not in allowed list')
                continue
            
            current_marker_ids.add(marker_id)
            
            # Transform pose from marker's frame to target frame
            world_pose = self.transform_pose_to_target_frame(marker.pose, marker.header.frame_id)

            # Check if this marker ID is already permanently assigned to a robot
            if marker_id in self.id_to_robot_mapping:
                robot_name = self.id_to_robot_mapping[marker_id]
                
                if self.use_cmd_vel:
                    if robot_name in self.robot_current_positions:
                        self.robot_previous_positions[robot_name] = self.robot_current_positions[robot_name]
                    self.robot_current_positions[robot_name] = world_pose
                    
                    # Calculate and send cmd_vel
                    cmd_vel = self.calculate_cmd_vel(robot_name, world_pose)
                    self.send_cmd_vel(robot_name, cmd_vel)
                else:
                    self.teleport_robot_direct(robot_name, world_pose)


                # Mark as active if it wasn't before
                if marker_id not in self.active_marker_ids:
                    self.active_marker_ids.add(marker_id)
                    self.get_logger().info(f'Reactivated robot {robot_name} for marker ID {marker_id}')
            else:
                # New marker ID - assign a robot permanently
                available_robot = self.get_available_robot()
                if available_robot:
                    # Create permanent bidirectional mapping
                    self.id_to_robot_mapping[marker_id] = available_robot
                    self.robot_to_id_mapping[available_robot] = marker_id

                    # Move robot to marker position
                    self.teleport_robot_direct(available_robot, world_pose)

                    if self.use_cmd_vel:
                        self.robot_current_positions[available_robot] = world_pose

                        # Send zero velocity for this timestep
                        self.send_cmd_vel(available_robot, Twist())
                    
                    # Mark as active
                    self.active_marker_ids.add(marker_id)
                    
                    self.get_logger().info(f'Permanently assigned robot {available_robot} to marker ID {marker_id}')
                else:
                    self.get_logger().warn(f'No available robots to assign to marker ID {marker_id}')
        
        # Find marker IDs that are no longer present
        inactive_marker_ids = self.active_marker_ids - current_marker_ids
        
        # Park robots whose markers are no longer active
        for marker_id in inactive_marker_ids:
            if marker_id in self.id_to_robot_mapping:
                robot_name = self.id_to_robot_mapping[marker_id]
                self.park_robot(robot_name)
                self.active_marker_ids.discard(marker_id)
                self.get_logger().info(f'Parked robot {robot_name} - marker ID {marker_id} no longer active')

    def teleport_robot_direct(self, robot_name: str, pose: Pose):
        """Teleport a robot to a new pose in Gazebo (pose already in target frame coordinates)"""
        if not self.teleport_client.service_is_ready():
            self.get_logger().warn('Teleport service not ready')
            return

        request = SetEntityPose.Request()
        request.entity.name = robot_name
        request.entity.type = 1  # MODEL type
        request.pose = pose
        request.pose.position.z = 0.01  # Slight offset to avoid ground collision
        
        self.get_logger().debug(f'Teleporting {robot_name} to {self.target_frame_id} pose: x={pose.position.x:.3f}, y={pose.position.y:.3f}, z={pose.position.z:.3f}')

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
        active_robots = [self.id_to_robot_mapping[marker_id] for marker_id in self.active_marker_ids]
        assigned_robots = list(self.robot_to_id_mapping.keys())
        available_robots = list(self.available_robots_pool)
        
        self.get_logger().info(f'Active robots: {active_robots}')
        self.get_logger().info(f'Assigned robots: {assigned_robots}')
        self.get_logger().info(f'Available robots: {available_robots}')
        self.get_logger().info(f'ID to robot mapping: {self.id_to_robot_mapping}')


    ## CMD VEL SPECIFIC (not used if teleporting directly) ##
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
            self.get_logger().info(f'{robot_name}: Limited linear velocity to {self.max_linear_velocity:.2f} m/s')
        
        if abs(angular_velocity) > self.max_angular_velocity:
            angular_velocity = math.copysign(self.max_angular_velocity, angular_velocity)
            self.get_logger().info(f'{robot_name}: Limited angular velocity to {self.max_angular_velocity:.2f} rad/s')
        
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

    def send_cmd_vel(self, robot_name: str, twist: Twist):
        """Send cmd_vel message to robot"""
        if robot_name in self.cmd_vel_publishers:
            self.cmd_vel_publishers[robot_name].publish(twist)
            self.get_logger().debug(f'Sent cmd_vel to {robot_name}: linear={twist.linear.x:.3f}, angular={twist.angular.z:.3f}')
        else:
            self.get_logger().warn(f'No cmd_vel publisher found for robot {robot_name}')

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

def main(args=None):
    rclpy.init(args=args)
    
    node = ScenarioReplayTeleport()
    
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