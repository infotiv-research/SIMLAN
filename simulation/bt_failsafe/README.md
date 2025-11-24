# Failsafe for the navigation

The SIMLAN systems support a failsafe and geo-fencing mechanism for the pallet trucks navigation. The main idea is that the pallet trucks should stop the navigation as soon as a dangerous situation below are detected:

In this approach for a new Behavior Tree, the **condition node** named [`StopRobotCondition`](simulation/bt_failsafe/include/bt_failsafe/stop_robot.hpp) is defined and can be triggered when a safety issue is occurs. We then added a [Behavior Tree plugging](simulation/bt_failsafe/bt_failsafe_plugins.xml) that calls an acction called [CancelControl](https://docs.nav2.org/configuration/packages/bt-plugins/actions/CancelControl.html) when the given condition is triggered.

Currently this safety controller triggers `StopRobotCondition`:

### Activation of collision sensor

When a collision is detected by the simulator, the collision's physical properties (such as the force and the object that pallet truck collided with) are published to the pallet trucks `/contact` topic.

### Loss of Observability

To implement geofencing and the safety situation in which a pallet truck is not observable in any camera. `aruco_localization` pkg under `aruco_localization/aruco_pose_pub.py` continuously published the list of pallet truck that are observable in  `/aruco_marker_seen` topic. Then if one stops being observable, the BT condition passes.

## Behavior tree and direct implementation in aruco_localization pkg

At first we define a custom `behavior_tree.xml` in `simulation/pallet_truck/pallet_truck_navigation/config/navigate_w_replanning_and_recovery_robot_agent_X.xml`. This xml files defines which bt plugins we want to use during the navigation and which run continuously during the navigation. In this xml-file the `StopRobotCondition` and `CancelControl` plugins are defined in a fallback function. A fallback function works as its run the first stated plugin, and when that plugin fails it moves over to the second one an executes that plugin. So in this case we first run the `StopRobotCondition` plugin until the robot is out of bounds. Once the plugin fails the `CancelControl` plugin executes and stops the pallet truck.

In order to know whether a pallet truck is out of bounds or not is determined by looking at the aruco marker on the pallet truck and decide if its seen by any of the cameras or not. This is implemented in the `aruco_localization` pkg under `aruco_localization/aruco_pose_pub.py`. As long as the aruco marker on the pallet truck is seen by any of the cameras, the node publishes the robot namespace in a list to the topic `/aruco_marker_seen`.

Same for the collision sensor. It publishes to the topic `/robot_agent_X/contact` when the pallet trucks collide with something.

Step two in the failsafe is to stop the pallet truck when it gets out of bounds or collide with something. This is done by a custom made behavior tree plugin called StopRobotCondition which is found in `simulation/bt_failsafe/src/stop_robot.cpp`. This plugin listen to the `/aruco_marker_lost` and `/robot_agent_X/contact` topics. As soon as a contact is detected or a robots namespace is lost, it fails and the `CancelControl` behavior tree starts. This in turn stops the pallet truck. The `CancelControl` plugin is an existing built in plugin in Nav2.

### Why Nav2 behavior tree plugins was not suitable

We tried to use only built in plugins which most likely is the most robust and secure way to do it on as the plugins is updated accordingly to Ros2 and Nav2.

We tried to use several plugins to detect if the pallet trucks are out of bounds, but non of them really suited this purpose of this project.

The first one we tried to use is the [`TransformAvailable` plugin](https://github.com/ros-navigation/navigation2/blob/main/nav2_behavior_tree/plugins/condition/transform_available_condition.cpp). This plugin checks if a certain TF exists. If the TF is missing the plugin returns `FAILURE`. So in this case we thought we could look at the TFs between the cameras and the aruco marker and if none of them exists, it should fail and stop the pallet truck. Unfortunately we couldn't use this plugin because it only looks at the tf from the very beginning of the navigation. And if it exists from the beginning it will always succeed and will never return `FAILURE`. Therefore it cannot be used in our case because we have TF from the beginning, and our TF disappears after some time during the navigation. You could probably modify the plugin to work for our case as well, but if modifications are needed it could be implemented in a better way instead.

The second plugin we tried is the [`IsStuckCondition`](https://github.com/ros-navigation/navigation2/blob/main/nav2_behavior_tree/plugins/condition/is_stuck_condition.cpp) This plugin checks if the robot is stuck by calculating the deceleration of the robot. If the deceleration is to big it returns `FAILURE`. This wasn't anything we could use because the acceleration is set to zero as soon as the robot get out of bounds. So this is probably not suitable for this case.
