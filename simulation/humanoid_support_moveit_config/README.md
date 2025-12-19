# Humanoid

In this file, the basics of the humanoid structure will be described. In the future, all nodes will be namespaced to add the functionality of spawning and controlling multiple humanoids.

## Moveit

For MoveIt to work, 3 components must have correct information:

- URDF link names
- SRDF joint definitions
- TF frames published by robot_state_publisher

There was an issue when trying to add namespaces to the humanoid project. Since the humanoid is made of 40+ links, it was decided to add the `"frame_prefix": f"{namespace}"` in the robot_state_publisher node. This, however, made a conflict since the URDF, SRDF, and TF frames no longer matched. This was solved by adding `base_link` in between the world and namespace/base_link frames and creating a static transform between them. **This means that all further humanoids will be spawned under the `base_link` frame.** (otherwise, the warning below again appears)

## Dynamically updated URDFs

To make the multiple_humanoid_spawn work, we need to be able to launch different namespaces in the robot_description. This is done by running:

```python
moveit_config = (
    MoveItConfigsBuilder("human_support", package_name="humanoid_support_moveit_config")
    .robot_description(
        file_path="config/human_support.urdf.xacro", mappings={"namespace": namespace}
    )
    .to_moveit_configs()
)
```

This updates the `namespace` argument inside the .xacro files.

## Planning scene monitor

When working with the MoveIt package, some bugs or undesired features were found. When MoveIt is launched, the PlanningSceneMonitor and PlanningFrame subscribe to all TF frames that exist and assume they can transform any known object or sensor data into its planning frame, which is `base_link` in this case. If this cannot be done, it will throw a warning saying the following:

```
[WARN] [humanoid.moveit.moveit.ros.planning_scene_monitor] [id]: Unable to transform object from frame 'unconnected_frame' to planning frame 'base_link' (Could not find a connection between 'base_link' and 'unconnected_frame' because they are not part of the same tree. TF has two or more unconnected trees)
```

To limit this, a wait function was added: `ld.add_action(TimerAction(period=5.0, actions=[rsp_node]))` to make sure the transform is published before the rsp_node starts.

## Figures

![My Robot Diagram](/resources/humanoid-static_agents-frames.png)
*Figure 1: Humanoid frames visualization*

## Rviz2 visualization

If you want to visualize the movement in Rviz, you need to configure the `config/moveit.rviz` file and change all `/humanoid_X` instances to the namespace you want to visualize.

## Bugs: TF visualization and Gazebo simulation not matching

When visualizing and comparing the actual TF data and simulated locations of the humanoids when doing navigation and teleop, it can be seen that these do not match. This is a major issue since that means the location of where the humanoids think they are in the map and the location of where the humanoids actually are in Gazebo will be different. This can be visualized by turning the humanoid 360 degrees in Gazebo with the `./control.sh teleop humanoid_1` command and comparing it to Rviz. There, the humanoid has turned closer to 300 degrees.

This problem probably comes from some URDF descriptions not matching the actual geometry or specifications the humanoid has in `/home/ros/src/simulation/humanoid_support_moveit_config/config/human_support_wheels.urdf.xacro`. By tuning the `mass`, `mu`, and `inertial` values in the `<xacro:macro name="wheel" params="wheel_prefix *joint_pose">` xacro tag, it was possible to tune the degree mismatch.

For proper navigation and control, this needs to be fixed!
