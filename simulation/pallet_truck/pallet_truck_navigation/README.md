

# Navigation (Robot agent and Humanoid) and Map server

the pallet truck navigation consist of:
- [map_server.launch.py](launch/map_server.launch.py) - starts the map server and creates necessary transforms for all of the robot agents.
- [nav2.launch.py](launch/nav2.launch.py) - starts the navigation stack and the update_map.py for each robot.
- [cartography.launch.py](launch/cartography.launch.py)  - starts cartography.
- [update_map.py](pallet_truck_navigation/update_map_node.py) - adds the dynamic obstacles (other robot_agents) to each robot_agent's map.

To run the navigation stack do the following either with the input=ROBOTS or HUMANOIDS
```bash
./control.sh nav ROBOTS
```
## Map server

Instead of starting with an empty map, to get the general layout of the environment and detect the static object in the environment (walls, cooridors and hallways) you can **optionally** build a map.
To start mapping (otherwise known as cartography), you can use `cartography.launch.py`. It requires a lidar to be mounted on the `robot_agent` and `odometry` to exist, which can be activated from the  `camera_utility/aruco_localization` package. 

Launch the following package, run gazebo, rviz, aruco_localization, and start moving around the simulation with the robot_agent. When you are finished you need to save it using this command: `ros2 run nav2_map_server map_saver_cli -f simulation/pallet_truck/pallet_truck_navigation/maps/YOUR_MAP_NAME`. Keep in mind that this step needs to be done only once and the map assumed to be static.

The navigation stack uses a base map which is updated dynamically with other robot_agents for each robot. This done to add the static environment and update the dynamic moving obstacles later.

```python
Node(  # Manually setting the joint between map and odom to initial_pose_x initial_pose_y 0, i.e. the location of where the robot spawn. map -> odom
    package="tf2_ros",
    executable="static_transform_publisher",
    namespace=robot["namespace"],
    name="static_map_to_odom",
    arguments=[f"{robot["initial_pose_x"]}",
               f"{robot["initial_pose_y"]}",
              "0",
              "0",
              "0",
              "0",
              f"{robot["namespace"]}/map",
              f"{robot["namespace"]}/odom"],
)
```

## Robot_agent navigation

This package covers the functionality of mapping, localizing, and navigation for the robot_agent. Each package is built using code from the nav2 packages, thus you are able to modify the launch files but to change the node's behaviour you need to modify the config files. The code is tailored for the robot_agent and using the `aruco_detection` package that supplies `/TF` chain for the truck.

- The `aruco_localization` package runs the aruco localisation.
- The `map_server.py` creates a map for each agent
- The `nav2.launch.py` uses the localisation (from the previous steps) for the  navigation of the robot_agent.

Keep in mind that the same namespace has to be used when launching the navigation stack which is done automatically:  `ros2 launch pallet_truck_navigation nav2.launch.py robots:=ROBOTS`

For navigation to be able to automatically find the robot agent, we have to set a namespace that specified when launching the robot_agent. (see above)

To do navigation for both `humanoids and pallet_trucks` at the same time you need to create a merged variable of the 2 and sent that as a input to the map_server.launch.py and nav2.launch.py to make sure the obstacles are detected.

Each robot has it own navigation configuration in `nav2_params.yaml` which is dynamically generated on build based on the `ROBOTS` and `HUMANOIDS` variables in config.sh. the pallet_trucks and forklifts also have an individual `navigate_w_replaning_and_recovery_robot_agent_x.xml` which is supposed to stop the navigation of the agents if their aruco_markers are not detected. To visualize which aruco markers are seen, first run the gpss and nav operations and then echo the `/aruco_marker_seen` topic.


We use following `/TF` structure:

![view_frames.png](view_frames.png)

**Note** that `world` and `map` are static at the same position. `robot_agent_X/odom` is static at the position from where the robots are spawned and the new position of the robots are then determined by `robot_agent_X/base_link` which is a dynamic transform from `robot_agent_X/odom`.

Good video to watch for multi agent navigation: https://www.youtube.com/watch?v=cGUueuIAFgw

## Humanoid_navigation

The humanoid navigation is implemented in the same way as for the robot_agents with the difference of calling the function `./control.sh nav HUMANOIDS`. Then the same principles apply but with a different nav2_parameter file being used as an argument to nav2 nodes.

There is one major difference between the navigation of the humanoids and the robot_agents. The robot _agents get their `odom` frame from the aruco_localization pkg which find aruco_markers and publishes their orientation and position. The humanoids on the other hand get their `odom` frame from the humanoid_odom_pub node which subscribes to the ground truth of the humanoids pose from gazebo and creates a dynamic transform between the `namespace/odom` -> `namespace/base_link` frames. This means the robot_agents get their odom frame from what the cameras can see and the humanoids get their odom frame from the actual pose in the simulation.

## Obstacles detection (update_map_node)

The obstacle detection is based on a static map which is dynamically updated with all other robot agents except itself. Therefore every robot_agent has its own map to not map itself. To not depend on the orientation of the robots, the obstacle is depicted as a circle. With this setup the map_server is gets an action call that updates the moving obstacles

---

The `/map_updater/update_map_node.py` node works as following:

First it makes a copy of the `warehouse.pgm` map which is a long array of pixel values ranging from (0-255). Since the map nav2 needs has a different setup than the `.pgm` file, a conversion is needed.

The .pgm file has these pixels ranges

|value  |meaning                                  |
|-----  |-----------------------                  |
|0      |black obstacle                           |
|255    |White Free space                         |
|1-254  |ranges of gray, not defined at the moment|


The map nav2 needs is set up with pixels ranging from (-1-100) where

|value|meaning                |
|-----|-----------------------|
|-1   |Unknown                |
|0    |Free space             |
|100  |obstacle               |
|1-99 |probabilistic occupancy|

therefore this has to be parsed to match by doing the following:

```python
current_array = self.original_array.copy()
occupancy_array = np.zeros(current_array.shape, dtype=np.uint8)
occupancy_array[current_array.copy()<100] = 100
```

The map is a 2D array, typically stored row-major from bottom-left, but many implementations flip it vertically ([::-1]) to match how image viewers treat top-left as (0,0). and therefore the following is done

``` python
occupancy_array = occupancy_array[::-1, :]
```

The position of all other robot_agents are found by their transforms between robot_agent_x/base_link and world and an obstacle is set at that position which is updated with 1 Hz to continuously update their positions. This can be tweaked but to limit CPU usage it was set to 1 Hz.

This concept is repeated for all namespaces sent through the ./control.sh nav function.

## Future improvements

**How often the new path the pallet_trucks can take be update**
Add a speed limiter instead of obstacle, in those cases the robot will slow down instead of replanning the paths. Could probably mix with the inflation radius's of the costmaps to make robot take wider turns

**Collision**
Two robots that are approaching each other (from left and right) don't know where each one is planning to go. Therefore they might both plan to the same path and both trie to out turn each other resulting in a race condition.

Possible solution: block future path on every other robots map!


### DEBUG

Getting this warning below is a sign that the nav2 does not work as intended. If this appears repeatedly, you can check the TF tree to see if there is a connection between the `robot_agent` link and `world` link. If there is no connection, it probably means that no camera is viewing the robot. If so then either move the robot into a camera's view or add more cameras to the simulation.  
```bash
[update_map_node-6] [WARN] [robot_agent_1.update_map] [1767949972.586691493]: Transform not available yet: Could not find a connection between 'world' and 'robot_agent_2/base_link' because they are not part of the same tree.Tf has two or more unconnected trees.
```