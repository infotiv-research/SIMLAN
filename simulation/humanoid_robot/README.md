# Humanoid Robot

There needs to be better documentation of the difference between this pkg and the humanoid_support_moveit_config pkg. Is this supposed to be the bring-up of that pkg? Why is the humanoid model in this pkg and the URDF that spawns the robot in humanoid_support_moveit_config? Are the humanSubject01-08 used, or is only humanSubjectWithMeshes used? Clarification is needed. All in all, a restructure of the entire humanoid_robot and humanoid_support_moveit_config should be done.

## Configuring and spawning humanoids inside the sim

Humanoids are configured in `config.sh` as a string in the HUMANOIDS variable. It is possible to add more humanoids and they are defined as:

```
{
    "namespace": "humanoid_1",
    "initial_pose_x":5.5,
    "initial_pose_y":-10.0
},
```

## model and URDF

Package created based on this [tutorial](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html)\
Note that only the human with mesh model is installed in the package; see /model/CMakeLists.txt for how to install the non-mesh models.

In the URDF, the fixed links and joints are (mostly) named \[muscle name\]\_\[location\], for example, BicBrac_RUA = biceps brachii_right upper arm. In humanSubjectWithMesh_simplified.urdf, all fixed joints have been removed; only the skeleton joints remain.

To open Rviz with the human model:

`ros2 launch urdf_tutorial display.launch.py model:=/home/ros/src/simulation/humanoid_robot/model/human-gazebo/humanSubjectWithMeshes/humanSubjectWithMesh_simplified.urdf`

TODO: Explain what are the parameters (`config.sh`). Add this to resources/build-documentation
