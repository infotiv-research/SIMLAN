# Pallet Truck bringup

This package handles initialization and status of spawned robots. 


- [gazebo.launch.py](launch/gazebo.launch.py) - Spawns robot, handles robot_state_publisher, robot_description 
- [keyboard_steering.launch.py](launch/keyboard_steering.launch.py) - keyboard steering. 
- [multiple_robot_spawn.launch.py](launch/multiple_robot_spawn.launch.py) - contains configurable list of robots you spawn in the sim. <br>
- [sim.launch.py](launch/sim.launch.py) - main launch file for single robot. Make sure Gazebo is running; control, twist_mux, and keyboard_steering are all launched.
- [rviz.launch.py](launch/rviz.launch.py) - runs RViz


The package focuses on pallet_truck and forklift for now, but could be made modular to spawn other types of robots.

## Configuring and spawning robots inside the sim

Below is the structure which we use to spawn robots inside of the sim. Please read these notes before setting up a new or editing a robot.  



Here is the information from the table presented as a markdown list:

## Robot Spawning Attributes

* **`namespace`**
    * **Description:** Used to differentiate between multiple robots. Follows the format `robot_agent_N`, where `N` is the robot ID.
    * **Example:** `"robot_agent_1"`
* **`initial_pose_x`**
    * **Description:** The robot's initial x-coordinate position. **Float value** wrapped as a string.
    * **Example:** `"10.0"`
* **`initial_pose_y`**
    * **Description:** The robot's initial y-coordinate position. **Float value** wrapped as a string.
    * **Example:** `"1.0"`
* **`robot_type`**
    * **Description:** Selects the robot mesh or appearance. Options are **`"pallet_truck"`** or **`"forklift"`**.
    * **Example:** `"pallet_truck"`
* **`aruco_id`**
    * **Description:** Sets the ID shown on the robot's ArUco marker. Must match the ID in the `namespace`.
    * **Example:** `"1"` if namespace ID is `1`


Would you like me to use these attributes to create a sample robot configuration?
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



## Automatically generated parameter files

Some parameter files are automatically generated for the pallet_truck_bringup package. The generation scripts can be found in the config_generation/ directory. The automatically generated files include the `nav2_params.yaml`, `gz_bridge.yaml` and more. To look at all the automatically generated files, build the workspace and the generated files will be printed in the terminal or look manually in the config_generation/generate.py.

