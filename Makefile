camera_enabled_ids=164

gpss:
	ros2 launch simlan_bringup full_sim.launch.py camera_enabled_ids:="${camera_enabled_ids}"

nav:
	ros2 launch pallet_truck_navigation map_server.launch.py &
	sleep 10
	ros2 launch pallet_truck_navigation nav2.launch.py namespace:=robot_agent_1 &
	ros2 launch pallet_truck_navigation nav2.launch.py namespace:=robot_agent_2

send_goals:
	ros2 action send_goal /robot_agent_1/navigate_to_pose nav2_msgs/action/NavigateToPose '{pose: {header: {frame_id: "map"}, pose: {position: {x: 18.0, y: 2.0, z: 0.0}, orientation: {w: 1.0}}}}' &
	pid1=$$!
	ros2 action send_goal /robot_agent_2/navigate_to_pose nav2_msgs/action/NavigateToPose '{pose: {header: {frame_id: "map"}, pose: {position: {x: 18.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}' &
	pid2=$$!
	wait $$pid1
	wait $$pid2

teleop_pallet:
	ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r __ns:=/robot_agent_1 -r cmd_vel:=key_vel

scenario_replay:
	ros2 launch visualize_real_data scenario_replayer.launch.py