# Pallet_truck_description
 
In addition to the change_log, this description is added to give vital information that may help other developers in the future to debug or update features faster.


Here additions to the urdf.xacro files are mentioned

### Collision sensor


A collision_sensor was added and linked to the mesh tag of the pallet_trucks. This is integrated in the pallet_truck.urdf.xacro.

To visualize the topic subscribe to: /namespace/collision e.g. /robot_agent_1/contact.

The topic publishes information about the position, torque and which models are in contact with each other.

The topic name is defined by the automatically generated gz_bridge() which can be found in /home/ros/src/simulation/pallet_truck/pallet_truck_bringup/launch/generate_gz_bridge.py



### xacro/urdf/sdf files Bugs
 
Gazebo only reads sdf files and if .xacro or .urdf files are used, they are later parsed to .sdf's before being used by Gazebo.
 
A "bug" for Gazebo harmonic using ros2 jazzy was that when adding the collision sensor for the pallet_trucks, Gazebo was unable to find the collision tag for the mesh that we were using.
 
After investigation it was found that when the .xacro file was parsed to an .sdf file, the name of the collision tag was updated by the parse plugin. Therefore this lumped renaming of the collision tag was "hard coded" instead of using its original name which is in the pallet_truck.urdf.xacro.

---
 
original: "\$(arg prefix)/chassis_link::collision" 
  
hard coded: "\${prefix}_base_link_fixed_joint_lump__collision_collision"
 
---

When adding sensors in the xacro file, make sure the plugins are also loaded. There are different world and robot plugins so if system  plugins are used they should most likely be added to the .world file