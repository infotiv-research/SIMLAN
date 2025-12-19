## Pallet Truck Package
Common packages for pallet_truck, including messages and robot description. These are packages relevant
to all pallet_truck workspaces, whether simulation, desktop, or on the robot's own headless PC.

Links to read about each individual pkg
- [pallet_truck_bringup](pallet_truck_bringup/README.md)
- [pallet_truck_control](pallet_truck_control/README.md)
- [pallet_truck_description](pallet_truck_description/README.md)
- [pallet_truck_navigation](pallet_truck_navigation/README.md)


`prefix` is used to publish unique base link names of each robot agent to `/tf`. This way they are all visible in rviz and probably it is used for odometry done by aruco localization.  
`namespace` is used to separate nodes and topics related to each robot agent. This way we can control each robot separately and run a separate nav2 stack.  

**Keep in mind that the value of `namespace` is used for `prefix`** but they are different concepts and use cases.