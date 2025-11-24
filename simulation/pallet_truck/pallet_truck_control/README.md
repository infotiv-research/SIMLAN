# Pallet Truck Control

The `pallet_pallet_truck_control` package includes all nodes necessary for the control of the robots. These include the following:

- **twist_mux**: Takes in velocity topics and orders them in order of relevance. from highest to lowest: /key_vel, /safety_vel, / scenario_vel, /nav_vel, /cmd_vel
- **twist_stamper**: As of ros2 jazzy the ros2_controllers need the twist messages to be stamped, therefore the stamper was introduced
- **ros2_control_node**: Main node which uses the control.yaml file and maps "hardware" to controller actions. 
- **spawner-joint_state_broadcaster**: Publishes the joint_state so the robot moves in gazebo and rviz.
- **spawner-velocity_controller**: Accepts and publishes velocity commands

 With this setup, the robot_agents can be controlled by running the teleop command in control.sh.