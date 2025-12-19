#!/bin/bash


#################################################################
#          SOURCING AND SETTING ENVIRONMENT VARIABLES           #
#################################################################
#region environment variables
CONFIG_FILE="$(dirname "$0")/config.sh"
source $CONFIG_FILE
CONTROL_FILE="$(dirname "$0")/control.sh"
source $CONTROL_FILE

kill
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
             "robot_type":"pallet_truck",
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
##########################################################################################
## Scenario 1 - Running multiple agents sequentially with nav2.                         ##
## - Runs robot_agents, humanoid, and panda arm sequentially using moveit2 and nav2     ##
## - Brings up environment, cameras, aruco detection                                    ##
## - Spawns robot_agent_1, robot_agent_2, humanoid_1, panda_arm                         ##
## - Runs nav2 on "robot_agent_1, robot_agent_2, humanoid_1".                           ##
## - Runs moveit2 on Humanoid_1 and panda_arm                                           ##
## NOTE: Don't forget to change real time factor in                                     ##
## simulation/simlan_gazebo_environment/worlds/ign_simlan_factory.world.xacro           ##
## to about 0.05-0.1                                                                    ##
##########################################################################################
#region scenario 1
scenario_1 () {
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
}
#endregion
######################################################################################
## Scenario 2 - Running nav2 and moveit2 together on humanoid.                      ##
## - Brings up environment, cameras, aruco detection                                ##
## - Spawns humanoid_1, panda_arm                                                   ##
## - Runs nav2 on "humanoid_1".                                                     ##
## - Runs moveit2 on Humanoid_1 and panda_arm                                       ##
## NOTE: Don't forget to change real time factor in                                 ##
## simulation/simlan_gazebo_environment/worlds/ign_simlan_factory.world.xacro       ##
## to about 0.05-0.1                                                                ##
######################################################################################
#region scenario 2
scenario_2 () {
    replay_motion_namespace="humanoid_1"

    sim &
    static_agent &
    sleep 10 &

    #spawn humanoids, panda_arm
    sleep 10 ; humanoid & moveit &
    sleep 20; ros2 launch moveit_resources_panda_moveit_config demo.launch.py "log_level:=${log_level}" &

    #start navigation stack for humanoid
    sleep 10; ros2 launch pallet_truck_navigation map_server.launch.py "robots:=${HUMANOIDS}" "log_level:=${log_level}" &
    sleep 5; ros2 launch pallet_truck_navigation nav2.launch.py "robots:=${HUMANOIDS}" "log_level:=${log_level}" &

    #send goal to humanoid_1 panda_arm
    #this where all the actions should go, it is possible to add pids and make the actions wait for each other to send more goals.
    sleep 10; ros2 action send_goal /humanoid_1/navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: 'humanoid_1/map'}, pose: {position: {x: 13, y: 8, z: 0.0}, orientation: {z: 1.0}}}}" &
    sleep 2; ros2 topic pub --once $replay_motion_namespace/execute_motion std_msgs/String "{data: '/home/ros/src/humanoid_utility/input/humanoid_scenario/random_motion_1.json'"} &
    sleep 2; ros2 topic pub --once $replay_motion_namespace/execute_motion std_msgs/String "{data: '/home/ros/src/humanoid_utility/input/humanoid_scenario/default_motion.json'"} &
    sleep 2; ros2 topic pub --once $replay_motion_namespace/execute_motion std_msgs/String "{data: '/home/ros/src/humanoid_utility/input/humanoid_scenario/random_motion_2.json'"} &
    sleep 2; ros2 topic pub --once $replay_motion_namespace/execute_motion std_msgs/String "{data: '/home/ros/src/humanoid_utility/input/humanoid_scenario/default_motion.json'"} &

    sleep 5; ros2 launch motion_planning_python_api motion_planning_python_api_tutorial.launch.py "log_level:=${log_level}" &
}
#endregion
###########################################################################################################################
## to use no cameras dont forget to update all the robot_agent_X's                                                        ##
## simulation/pallet_truck/pallet_truck_navigation/config/navigate_w_replanning_and_recovery_robot_agent_X.xml           ##
## with the contents in : simulation/pallet_truck/pallet_truck_navigation/config/navigate_w_replanning_and_recovery.xml  ##
## otherwise they will be out of bounds and stop moving                                                                  ##
###########################################################################################################################
#region scenario 3
scenario_3 () {
    sim &
    ros2 launch pallet_truck_bringup multiple_robot_spawn.launch.py "robots:=${ROBOTS}" "log_level:=${log_level}" &

    sleep 5; ros2 run humanoid_odom_pub humanoid_odom_pub --ros-args \
    -p namespace:=robot_agent_1 \
    -p initial_pose_x:=10.0 \
    -p initial_pose_y:=-0.8 &
    sleep 5; ros2 run humanoid_odom_pub humanoid_odom_pub --ros-args \
    -p namespace:=robot_agent_2 \
    -p initial_pose_x:=30.0 \
    -p initial_pose_y:=8.5 &

    sleep 5 ; humanoid &

    sleep 10; ros2 launch moveit_resources_panda_moveit_config demo.launch.py "log_level:=${log_level}" &

    sleep 10; ros2 launch pallet_truck_navigation map_server.launch.py "robots:=${ROBOTS_AND_HUMANOIDS}" "log_level:=${log_level}" &

    sleep 3; ros2 launch pallet_truck_navigation nav2.launch.py "robots:=${ROBOTS_AND_HUMANOIDS}" "log_level:=${log_level}" &

    sleep 10; ros2 action send_goal /humanoid_1/navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: 'humanoid_1/map'}, pose: {position: {x: 13, y: 9, z: 0.0}, orientation: {z: 1.0}}}}" &
    sleep 15; ros2 action send_goal /robot_agent_2/navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: 'robot_agent_2/map'}, pose: {position: {x: 23, y: 0, z: 0.0}, orientation: {w: 1.0}}}}" &
    sleep 10; ros2 action send_goal /robot_agent_1/navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: 'robot_agent_1/map'}, pose: {position: {x: 30, y: 8.50, z: 0.0}, orientation: {w: 1.0}}}}" &
    sleep 5; ros2 launch motion_planning_python_api motion_planning_python_api_tutorial.launch.py "log_level:=${log_level}" &
}
#endregion
scenario_video_demo () {
###########################################################################################################################
## to use no cameras dont forget to update all the robot_agent_X's                                                        ##
## simulation/pallet_truck/pallet_truck_navigation/config/navigate_w_replanning_and_recovery_robot_agent_X.xml           ##
## with the contents in : simulation/pallet_truck/pallet_truck_navigation/config/navigate_w_replanning_and_recovery.xml  ##
## otherwise they will be out of bounds and stop moving                                                                  ##
## NOTE: Don't forget to change real time factor in                                 ##
## simulation/simlan_gazebo_environment/worlds/ign_simlan_factory.world.xacro       ##
## to about 0.05-0.1                                                                ##
###########################################################################################################################
    sim &
    static_agent &
    sleep 10 &
    sleep 10 ; ros2 launch aruco_localization multi_detection.launch.py use_sim_time:=true "camera_enabled_ids:=${CAMERA_ENABLED_IDS}" "robots:=${ROBOTS}" "log_level:=${log_level}" &

    #spawn robot_agents, humanoids, panda_arm
    sleep 10 ; ros2 launch pallet_truck_bringup multiple_robot_spawn.launch.py "robots:=${ROBOTS}" "log_level:=${log_level}" &
    sleep 10 ; humanoid & moveit &
    # sleep 20; ros2 launch moveit_resources_panda_moveit_config demo.launch.py "log_level:=${log_level}" &

    #start navigation stack
    #for the navigation stack it is important that both robots and humanoids are passed as a single list of dicts, otherwise they will not detect each other as obstacles on the map.
    sleep 10; ros2 launch pallet_truck_navigation map_server.launch.py "robots:=${ROBOTS_AND_HUMANOIDS}" "log_level:=${log_level}" &
    sleep 5; ros2 launch pallet_truck_navigation nav2.launch.py "robots:=${ROBOTS_AND_HUMANOIDS}" "log_level:=${log_level}" &

    #send goal humanoid_1 robot_agents_1 robot_agents_2 panda_arm
    #this where all the actions should go, it is possible to add pids and make the actions wait for each other to send more goals.
    sleep 20; ros2 action send_goal /robot_agent_2/navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: 'robot_agent_2/map'}, pose: {position: {x: 23, y: 0, z: 0.0}, orientation: {w: 1.0}}}}" &
    sleep 100; ros2 action send_goal /robot_agent_1/navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: 'robot_agent_1/map'}, pose: {position: {x: 30, y: 8.50, z: 0.0}, orientation: {w: 1.0}}}}" &
    sleep 100; ros2 action send_goal /humanoid_1/navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: 'humanoid_1/map'}, pose: {position: {x: 13, y: 9, z: 0.0}, orientation: {z: 1.0}}}}" &

    sleep 15; ros2 topic pub --once $replay_motion_namespace/execute_motion std_msgs/String "{data: '/home/ros/src/humanoid_utility/input/humanoid_scenario/random_motion_1.json'"} &
    sleep 15; ros2 topic pub --once $replay_motion_namespace/execute_motion std_msgs/String "{data: '/home/ros/src/humanoid_utility/input/humanoid_scenario/default_motion.json'"} &

}

  ######################################
 ##                                  ##
##     Commands and execution       ##
 ##                                  ##
  ######################################
#region commands
if [[ "$1" == *"1"* ]]
then
    echo "###### Executing scenario 1 ######"
    scenario_1
elif [[ "$1" == *"2"* ]]
then
    echo "###### Executing scenario 2 ######"
    scenario_2
elif [[ "$1" == *"3"* ]]
then
    echo "###### Executing scenario 2 ######"
    scenario_3

elif [[ "$1" == *"4"* ]]
then
    echo "###### Executing scenario_video_demo ######"
    scenario_video_demo
fi

#endregion