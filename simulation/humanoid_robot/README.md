### 
Package created based on this [tutorial](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html)\
Note that only the human with mesh model is installed in the package, see /model/CMakeLists.txt for how to install the non-mesh models.

In the URDF, the fixed links and joints are (mostly) named [muscle name]_[location], for example BicBrac_RUA = biceps brachii_right upper arm. In humanSubjectWithMesh_simplified.urdf all fixed joints have been removed, only the skeleton joints remain.




To open Rviz with the human model:

`ros2 launch urdf_tutorial display.launch.py model:=/home/ros/src/simulation/humanoid_robot/model/human-gazebo/humanSubjectWithMeshes/humanSubjectWithMesh_simplified.urdf`
