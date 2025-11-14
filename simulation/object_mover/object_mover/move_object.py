import rclpy
from rclpy.node import Node
import transforms3d

import geometry_msgs.msg
from gazebo_msgs.srv import SetEntityState
import subprocess
import random
import numpy as np

PI = 3.14

import rclpy
from rclpy.node import Node
from rclpy.service import Service
from rclpy.qos import qos_profile_system_default
from rcl_interfaces.srv import GetParameters
from rcl_interfaces.msg import ParameterValue

"""
Object: forklift
Camera: 1
(#Parameter:  orientation of forklift)
Annotation: Position and rotation of the forklift. Annotation in filename. Images can be identical if camera FPS is higher than movements.
Resolution: Small steps (0.1 degree, 1 small grid 5% of image size?)
10,000 (14,400 as per below steps)
Rotation: 360 (10 degree each)
X: 20
Y: 20
Image size < 1MB (png)
256x256 RGB non compression
"""


class MoveObject(Node):
    def __init__(self):
        super().__init__("move_object")
        self.srv = self.srv = self.create_service(
            GetParameters, "status_server", self.status_server_callback
        )
        self.MODE = "one_object_deterministic"  # pick between "one_object_deterministic", "normal" and "abnormal" (tilted)
        self.cli = self.create_client(SetEntityState, "gazebo/set_entity_state")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")
        self.get_logger().info("service available")

        # set some default values for the request
        self.req = SetEntityState.Request()
        self.req.state.reference_frame = "world"
        self.req.state.name = "infobot"
        self.pose = [0.0, 0.0, 0.0]
        self.orientation = [0.0, 0.0, 0.0, 0.0]
        self.rotation_in_degree = 0.0
        self.grid_x = [8.0, 22.0]  # 8, 20
        self.grid_y = [0.5, 2.5]  # 0, 3
        self.index_state = 0
        self.all_states = []
        for x in np.arange(self.grid_x[0], self.grid_x[1], 0.57):  # 21
            for y in np.arange(
                self.grid_y[0],
                self.grid_y[1],
                (self.grid_y[1] - self.grid_y[0]) / 11,  # 11
            ):
                for rot in np.arange(0, 2 * 3.14, 2 * 3.14 / 50):  # 2 * 3.14/50
                    self.all_states.append(
                        (round(x, 2), round(y, 2), round(rot, 2))
                    )  # [19.43, 2.32, 6.15]

        self.all_states.append([41.0, 41.0, 41.0])  # Adding End Sequence

        print(self.all_states)
        # exit(0)
        self.objects_default_position = {
            "jackal": (40.0, 40.0),
            "infobot": (43.0, 43.0),
            "eur_pallet_move": (46.0, 46.0),
            "box_move": (50.0, 50.0),
            "traffic_cone": (53.0, 53.0),
            "cameras": (0.0, 0.0),
            "user_spot_light_move": (30.0, 30.0),
            "support_pole_move": (50.0, 50.0),
        }

        if self.MODE == "normal":
            self.objects = ["jackal", "infobot", "eur_pallet_move", "box_move"]
        elif self.MODE == "abnormal":
            self.objects = [
                "jackal",
                "infobot",
                "eur_pallet_move",
                "box_move",
                "traffic_cone",
                "user_spot_light_move",
                "support_pole_move",
            ]
        elif self.MODE == "one_object_deterministic":
            self.objects = ["infobot"]
        else:
            print("ERROR: INVALID MODE")

        self.timer_period = 2  # seconds

    def status_server_callback(self, request, response):
        response.values = []
        data = [self.pose[0], self.pose[1], self.rotation_in_degree]  # self.orientation
        for value in data:
            param_value = ParameterValue()
            param_value.double_value = value  # Assign the float value
            response.values.append(param_value)
        print("sending:", data)
        return response

    def start(self):
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def send_request(self):
        # add new position and orientation to the request
        self.req.state.pose.position.x = self.pose[0]
        self.req.state.pose.position.y = self.pose[1]
        self.req.state.pose.position.z = self.pose[2]
        self.req.state.pose.orientation.x = self.orientation[0]
        self.req.state.pose.orientation.y = self.orientation[1]
        self.req.state.pose.orientation.z = self.orientation[2]
        self.req.state.pose.orientation.w = self.orientation[3]

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
        if self.MODE == "normal":
            self.reset_pose()
            self.send_request()
            self.update_pose()
            self.send_request()
        elif self.MODE == "one_object_deterministic":
            self.update_pose()
            self.send_request()
        elif self.MODE == "abnormal":
            # Make it possible to have two object in the scene by not moving the old object back to the default position
            prob_not_moving = [True, True, True, True, True, False]
            if random.choice(prob_not_moving):
                self.reset_pose()
            self.send_request()
            self.update_pose()
            self.send_request()
        else:
            print("ERROR: INVALID MODE")

    def reset_pose(self):
        self.pose = [
            self.objects_default_position[self.req.state.name][0],
            self.objects_default_position[self.req.state.name][1],
            0.0,
        ]
        self.orientation = quaternion_from_euler(0, 0, 0)

    def update_pose(self):
        self.req.state.name = random.choice(self.objects)
        if self.MODE == "normal":
            self.pose = [
                random.uniform(self.grid_x[0], self.grid_x[1]),
                random.uniform(self.grid_y[0], self.grid_y[1]),
                0.0,
            ]
            self.orientation = quaternion_from_euler(0, 0, random.uniform(-1, 1))
        elif self.MODE == "abnormal":
            self.pose = [
                random.uniform(self.grid_x[0], self.grid_x[1]),
                random.uniform(self.grid_y[0], self.grid_y[1]),
                random.uniform(2, 3),
            ]
            # PI/6 = 30 degree
            # PI/4 = 45 degree
            if self.req.state.name == "user_spot_light_move":
                # make sure not major rotation is done, < 30 degree
                self.orientation = quaternion_from_euler(
                    random.uniform(-PI / 6, PI / 6),
                    random.uniform(PI / 6, PI / 6),
                    random.uniform(PI / 6, PI / 6),
                )
            else:
                # make sure there is a visible rotation, > 45 degree
                self.orientation = quaternion_from_euler(
                    random.uniform(PI / 4, (2 * PI) - PI / 4),
                    random.uniform(PI / 4, (2 * PI) - PI / 4),
                    random.uniform(PI / 4, (2 * PI) - PI / 4),
                )
        elif self.MODE == "one_object_deterministic":
            x, y, rot = self.all_states[self.index_state]
            self.index_state = self.index_state + 1
            if self.index_state >= len(self.all_states):
                raise Exception("End Sequence")
            self.pose = [
                x,
                y,
                0.0,
            ]
            self.rotation_in_degree = rot
            self.orientation = quaternion_from_euler(0, 0, rot)


def quaternion_from_euler(roll: float, pitch: float, yaw: float) -> list[float]:
    w, x, y, z = transforms3d.euler.euler2quat(roll, pitch, yaw, "rxyz")
    return [x, y, z, w]


def main(args=None):
    rclpy.init(args=args)

    executor = rclpy.executors.MultiThreadedExecutor()

    move_object = MoveObject()
    move_object.start()

    executor.add_node(move_object)
    executor.spin()
    executor.shutdown()
    move_object.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
