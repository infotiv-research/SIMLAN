## Pallet Truck bringup.

This package handles initilization and status of spawned robots. 


- [gazebo.launch.py](launch/gazebo.launch.py) - Spawns robot, handles robot_state_publisher, robot_description 
- [keyboard_steering.launch.py](launch/keyboard_steering.launch.py) - keyboard steering. 
- [multiple_robot_spawn.launch.py](launch/multiple_robot_spawn.launch.py) - contains configurable list of robots you spawn in the sim. <br>
- [sim.launch.py](launch/sim.launch.py) - main launch file for single robot. Make sure gazebo is running, control, twist_mux, keyboard_steering as all launched.
- [rviz.launch.py](launch/rviz.launch.py) - runs rviz


The package focuses on pallet_truck for now but could be made modular to spawn other types of robots.

## Configuring and spawning robots inside the sim.

Below is the structure which we use to spawn robots inside of the sim. Please read these notes before setting up a new or editing a robot.  



Attributes for spawning a robot: 
| Attribute        | Description                                                                                           | Example             |
|------------------|-------------------------------------------------------------------------------------------------------|---------------------|
| `namespace`      | Used to differentiate between multiple robots. Follows the format `robot_agent_N`, where `N` is the robot ID. | `"robot_agent_1"`   |
| `initial_pose_x` | The robot's initial x-coordinate position. Float value wrapped as a string.                          | `"10.0"`            |
| `initial_pose_y` | The robot's initial y-coordinate position. Float value wrapped as a string.                          | `"1.0"`             |
| `robot_type`     | Selects the robot mesh or appearance. Options are `"pallet_truck"` or `"forklift"`.                 | `"pallet_truck"`    |
| `aruco_id`       | Sets the ID shown on the robot's ArUco marker. Must match the ID in the `namespace`.                | `"1"` if namespace ID is `1`               |

```
Example robot setup:
{
    "namespace": "robot_agent_1", 
    "initial_pose_x":"10.0", 
    "initial_pose_y":"1.0", 
    "robot_type":"pallet_truck", 
    "aruco_id":"1"
}

```

---

## automatically generated parameter files:

When launching multiple robots, a script to automatically generate the `gz_bridge.yaml` file was introduce to dynamically remap topics for the spawned robots. This is done in the `/launch/generate_gz_bridge.py` file which sets both the static world topics as well as robot specific topics such as the `/namespace/contact` topics of the spawned `pallet_truck`s. This is run everytime `multiple_robot_spawn.launch.py` is executed.

This is also implemented for the `control.yaml`file which is the ros2_controller. This is done in the `/launch/generate_control_yaml.py` with the same reasoning as before to dynamically add control for all robots that are spawned. The `control.yaml` is re-created every time `multiple_robot_spawn.launch.py` is executed

