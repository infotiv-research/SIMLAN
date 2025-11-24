#!/bin/bash
#################################################################
#          SOURCING AND SETTING ENVIRONMENT VARIABLES           #
#################################################################
#region environment variables
source install/setup.bash 2>/dev/null
source /opt/dependencies_ws/install/setup.bash 2>/dev/null

export GZ_SIM_RESOURCE_PATH=/usr/share/ignition/
export IGN_GAZEBO_RESOURCE_PATH=usr/share/ignition/
export PYTHONPATH="$(pwd)/camera_utility/:$PYTHONPATH"
export CALIB_PATH="$(pwd)/camera_utility/"
export RCUTILS_LOGGING_SEVERITY_THRESHOLD=WARN
unset ROS_LOCALHOST_ONLY
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] [{name}] [{time}]: {message}"
CONFIG_FILE="$(dirname "$0")/config.sh"
source $CONFIG_FILE
CONTROL_FILE="$(dirname "$0")/control.sh"
source $CONTROL_FILE
#endregion

#region robot and humanoid setup (if you want to launch from config setup remove these but be sure to merge humanoids and robots somehow)
HUMANOIDS='[
        {
            "namespace": "humanoid_1",
            "initial_pose_x":26,
            "initial_pose_y":-7.0,
            "cam_ns": "camera0"
        }
    ]'
ROBOTS='[
        {
            "namespace": "robot_agent_1",
            "initial_pose_x":"10.0",
            "initial_pose_y":"-0.8",
            "robot_type":"pallet_truck",
            "aruco_id":"1"
        },
        {
             "namespace": "robot_agent_2",
             "initial_pose_x":"30.0",
             "initial_pose_y":"8.0",
             "robot_type":"forklift",
             "aruco_id":"2"
        }
    ]'
ROBOTS_AND_HUMANOIDS='[
        {
            "namespace": "robot_agent_1",
            "initial_pose_x":"10.0",
            "initial_pose_y":"-0.8",
            "robot_type":"pallet_truck",
            "aruco_id":"1"
        },
        {
             "namespace": "robot_agent_2",
             "initial_pose_x":"30.0",
             "initial_pose_y":"8.0",
             "robot_type":"forklift",
             "aruco_id":"2"
        },
        {
            "namespace": "humanoid_1",
            "initial_pose_x":26,
            "initial_pose_y":-7.0,
            "cam_ns": "camera0"
        }
    ]'
#endregion
################################################################################
## First scenario dont forget to change real time factor in                   ##
## simulation/simlan_gazebo_environment/worlds/ign_simlan_factory.world.xacro ##
## to about 0.05-0.1                                                          ##
################################################################################
#region scenario
#brings up environment, cameras, aruco detection
sim &
static_agent &
sleep 10 &
sleep 10 ; ros2 launch aruco_localization multi_detection.launch.py use_sim_time:=true "camera_enabled_ids:=${CAMERA_ENABLED_IDS}" "robots:=${ROBOTS}" "log_level:=${log_level}" &

#spawn robot_agents, humanoids, panda_arm
sleep 10 ; ros2 launch pallet_truck_bringup multiple_robot_spawn.launch.py "robots:=${ROBOTS}" "log_level:=${log_level}" &
sleep 10 ; humanoid &
sleep 20; ros2 launch moveit_resources_panda_moveit_config demo.launch.py "log_level:=${log_level}" &

#start navigation stack
#for the navigation stack it is important that both robots and humanoids are passed as a single list of dicts, otherwise they will not detect each other as obstacles on the map.
sleep 10; ros2 launch pallet_truck_navigation map_server.launch.py "robots:=${ROBOTS_AND_HUMANOIDS}" "log_level:=${log_level}" &
sleep 5; ros2 launch pallet_truck_navigation nav2.launch.py "robots:=${ROBOTS_AND_HUMANOIDS}" "log_level:=${log_level}" &

#send goal humanoid_1 robot_agents_1 robot_agents_2 panda_arm
#this where all the actions should go, it is possible to add pids and make the actions wait for each other to send more goals.
sleep 20; ros2 action send_goal /robot_agent_2/navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: 'robot_agent_2/map'}, pose: {position: {x: 23, y: 0, z: 0.0}, orientation: {w: 1.0}}}}" &
sleep 100; ros2 action send_goal /robot_agent_1/navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: 'robot_agent_1/map'}, pose: {position: {x: 30, y: 8.50, z: 0.0}, orientation: {w: 1.0}}}}" &
sleep 100; ros2 action send_goal /humanoid_1/navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: 'humanoid_1/map'}, pose: {position: {x: 13, y: 9, z: 0.0}, orientation: {z: 1.0}}}}" &
sleep 5; ros2 launch motion_planning_python_api motion_planning_python_api_tutorial.launch.py "log_level:=${log_level}" &
#endregion
###########################################################################################################################
## to use no cameras dont forget to update all the robot_agent_X's                                                        ##
## simulation/pallet_truck/pallet_truck_navigation/config/navigate_w_replanning_and_recovery_robot_agent_X.xml           ##
## with the contents in : simulation/pallet_truck/pallet_truck_navigation/config/navigate_w_replanning_and_recovery.xml  ##
## otherwise they will be out of bounds and stop moving                                                                  ##
###########################################################################################################################
#region no camera

# sim &
# ros2 launch pallet_truck_bringup multiple_robot_spawn.launch.py "robots:=${ROBOTS}" "log_level:=${log_level}" &

# sleep 5; ros2 run humanoid_odom_pub humanoid_odom_pub --ros-args \
#   -p namespace:=robot_agent_1 \
#   -p initial_pose_x:=10.0 \
#   -p initial_pose_y:=-0.8 &
# sleep 5; ros2 run humanoid_odom_pub humanoid_odom_pub --ros-args \
#   -p namespace:=robot_agent_2 \
#   -p initial_pose_x:=30.0 \
#   -p initial_pose_y:=8.5 &

# sleep 5 ; humanoid &

# sleep 10; ros2 launch moveit_resources_panda_moveit_config demo.launch.py "log_level:=${log_level}" &


# sleep 10; ros2 launch pallet_truck_navigation map_server.launch.py "robots:=${ROBOTS_AND_HUMANOIDS}" "log_level:=${log_level}" &

# sleep 3; ros2 launch pallet_truck_navigation nav2.launch.py "robots:=${ROBOTS_AND_HUMANOIDS}" "log_level:=${log_level}" &

# sleep 10; ros2 action send_goal /humanoid_1/navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: 'humanoid_1/map'}, pose: {position: {x: 13, y: 9, z: 0.0}, orientation: {z: 1.0}}}}" &
# sleep 15; ros2 action send_goal /robot_agent_2/navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: 'robot_agent_2/map'}, pose: {position: {x: 23, y: 0, z: 0.0}, orientation: {w: 1.0}}}}" &
# sleep 10; ros2 action send_goal /robot_agent_1/navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: 'robot_agent_1/map'}, pose: {position: {x: 30, y: 8.50, z: 0.0}, orientation: {w: 1.0}}}}" &
# sleep 5; ros2 launch motion_planning_python_api motion_planning_python_api_tutorial.launch.py "log_level:=${log_level}" &

#endregion