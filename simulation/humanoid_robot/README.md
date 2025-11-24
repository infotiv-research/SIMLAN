# Humanoid Robot

There needs to be better documentation of the difference between this pkg and the humanoid_support_moveit_config pkg. Is this supposed to be the bringup of that pkg? why is the humanoid model in this pkg and the urdf that spawns the robot in humanoid_support_moveit_config. Are the humanSubject01-08 used or is only humanSubjectWithMeshes used? Clarification is needed. All in all a restructure of the entire humanoid_robot and humanoid_support_moveit_config should be done.

Package created based on this [tutorial](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html)\
Note that only the human with mesh model is installed in the package, see /model/CMakeLists.txt for how to install the non-mesh models.

In the URDF, the fixed links and joints are (mostly) named \[muscle name\]\_\[location\], for example BicBrac_RUA = biceps brachii_right upper arm. In humanSubjectWithMesh_simplified.urdf all fixed joints have been removed, only the skeleton joints remain.

To open Rviz with the human model:

`ros2 launch urdf_tutorial display.launch.py model:=/home/ros/src/simulation/humanoid_robot/model/human-gazebo/humanSubjectWithMeshes/humanSubjectWithMesh_simplified.urdf`
