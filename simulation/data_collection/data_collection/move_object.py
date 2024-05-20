import rclpy
from rclpy.node import Node
import transforms3d

import geometry_msgs.msg
from gazebo_msgs.srv import SetEntityState

class MoveObject(Node):

    def __init__(self):
        super().__init__('move_object')

        self.cli = self.create_client(SetEntityState , 'gazebo/set_entity_state')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.get_logger().info('service available')
        
        # set some default values for the request
        self.req = SetEntityState.Request()
        self.req.state.reference_frame = 'world'

        self.pose = [0.0, 0.0, 0.0]
        self.orientation = [0.0, 0.0, 0.0, 0.0]

        # create lists to loop through with objects, poses and orientations
        self.orientations = 225 # Request from Jesper: 225 orientations in each cell
        self.grid_x = [21.0, 26.6]
        self.grid_y = [-0.5, 3.8]
        self.grid_count = 15 # Request from Jesper: 15x15 grid

        self.objects = ['jackal', 'infotiv', 'eur_pallet', 'box_group_pickup']
        self.yaws = [iter * 2*3.1415/self.orientations for iter in range(self.orientations)]

        # make a grid of poses from grid_x and grid_y
        self.poses_x = [self.grid_x[0] + i * ((self.grid_x[1] - self.grid_x[0]) / self.grid_count) for i in range(self.grid_count)]
        self.poses_y = [self.grid_y[0] + i * ((self.grid_y[1] - self.grid_y[0]) / self.grid_count) for i in range(self.grid_count)]
        self.poses = [(x, y, 0.0) for x in self.poses_x for y in self.poses_y]
        # add a final pose to move the object out of camera view
        self.poses.append([20.0, 20.0, 1.0])

        self.object_iter = 0
        self.pose_iter = 0
        self.yaw_iter = 0

        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def send_request(self):
        
        # add new position and orientation to the request
        self.req.state.pose.position.x = self.pose[0]
        self.req.state.pose.position.y = self.pose[1]
        self.req.state.pose.position.z = self.pose[2]
        self.req.state.pose.orientation.x = self.orientation[0]
        self.req.state.pose.orientation.y = self.orientation[1]
        self.req.state.pose.orientation.z = self.orientation[2]
        self.req.state.pose.orientation.w = self.orientation[3]

        self._logger.info("Sending request to {} with pose: {}".format(self.objects[self.object_iter], self.req.state.pose))
        future = self.cli.call_async(self.req)
        
        if future.done():
                self._logger.info("Asynchronous call finished")
                try:
                    response = future.result()
                except Exception as e:
                    self.get_logger().error("Service call failed %r" % (e,))
                else:
                    self._logger.info("Got response: {}".format(response.value))
        return future.result()
    
    def timer_callback(self):
        self.send_request()
        self.get_next_state()
        self.update_pose()

    def get_next_state(self):
        self.yaw_iter += 1
        if(self.yaw_iter == self.orientations):
            self.yaw_iter = 0
            self.pose_iter += 1
        if(self.pose_iter == len(self.poses)):
            self.pose_iter = 0
            self.object_iter += 1
        if (self.pose_iter == len(self.poses)-1 and self.yaw_iter != 0): # no need to rotate the last pose
            self.yaw_iter = 0
            self.pose_iter = 0
            self.object_iter += 1
        if(self.object_iter == len(self.objects)):
            self.object_iter = 0
            ## end all iterations
            self.get_logger().info("Finished all iterations")
            self.destroy_node()
        
    def update_pose(self):
        self.req.state.name = self.objects[self.object_iter]
        self.pose = self.poses[self.pose_iter]
        self.orientation = quaternion_from_euler(0, 0, self.yaws[self.yaw_iter])

def quaternion_from_euler(roll: float, pitch: float, yaw: float) -> list[float]:
    w, x, y, z = transforms3d.euler.euler2quat(roll, pitch, yaw, "rxyz")
    return [x, y, z, w]

def main(args=None):
    rclpy.init(args=args)

    executor = rclpy.executors.MultiThreadedExecutor()

    move_object = MoveObject()

    executor.add_node(move_object)
    executor.spin()
    executor.shutdown()
    move_object.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()