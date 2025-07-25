# pallet_truck

Common packages for pallet_truck, including messages and robot description. These are packages relevant
to all pallet_truck workspaces, whether simulation, desktop, or on the robot's own headless PC.


# SIMLAN project
`prefix` is used to publishing unique base link names of each robot agent to `/tf`. This way they all visible in rviz and probably it is used for odometry done by aruco localisation.  
`namespace` is used to separate nodes and topic related to each robot agent. This way we can control each robot separately and run a separate nav2 stack.  


**Have in mind that the value of `namespace` is used for `prefix`** but they are different concept and usecases.