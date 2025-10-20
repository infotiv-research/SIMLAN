## Description

This is a pkg to create a map with obstacles around all other existing robot_agents in the simulation.

Included in this pkg as well is a namespace publisher, which publishes all of the robot namespaces so other nodes can use them.

each robot is assigned a map_server and this node send an updated map with all other robots_agents as obstacle to the correct namespaced map_server.

The node is launched in src/simulation/pallet_truck/pallet_truck_navigation/launch/nav2.launch.py



---

The /map_updater/update_map_node.py node works by the following steps:

First it makes a copy of the warehouse.pgm map which is a long array of pixel values ranging from (0-255) and saves it as `original_array`

Then since the map nav2 needs has a different setup than the .pgm file some convertions are needed.

The .pgm file has these pixels ranges

|value  |meaning                                  |
|-----  |-----------------------                  |
|0      |black obstacle                           |
|255    |White Free space                         |
|1-254  |ranges of gray, not defined at the moment|


The map nav2 needs is set up with pixels ranging from (-1-100) where

|value|meaning                |
|-----|-----------------------|
|-1   |Unkown                 |
|0    |Free space             |
|100  |obstacle               |
|1-99 |probibalistic occupancy|

therefore this has to be parsed to match by doing the following:

```python
current_array = self.original_array.copy()
occupancy_array = np.zeros(current_array.shape, dtype=np.uint8)
occupancy_array[current_array.copy()<100] = 100
```

So everything that is supposed to be a wall in the .pgm file is parsed as a wall in the map context

The map is a 2D array, typically stored row-major from bottom-left, but many implementations flip it vertically ([::-1]) to match how image viewers treat top-left as (0,0). and therefore the following is done

``` python
occupancy_array = occupancy_array[::-1, :]
```

Then the position of all other robot_agents are found by their transforms and an obstacle is set at their position which is updated with 10Hz to continuously update their positions.

This concept is repeated for all namespaces sent through the ./control.sh nav function.

---
## Future work


##### problem is how often the new path the pallet_trucks can take can update
<pre>
possible solutions:
    add a speed limiter instead of obstacle, in those cases the robot will slow down instead of replanning and avoiding completely update slower
    could probably mix with the inflation radius's of the costmaps to make robot take wider turns
</pre>

