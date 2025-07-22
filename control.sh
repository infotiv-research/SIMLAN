#!/bin/bash

export ROS_DOMAIN_ID=$(cat ROS_DOMAIN_ID.txt)
echo "ROS_DOMAIN_ID.txt:" $ROS_DOMAIN_ID
export GZ_SIM_RESOURCE_PATH=/usr/share/ignition/
export IGN_GAZEBO_RESOURCE_PATH=usr/share/ignition/

# Camera Ids listed here are enabled in the simulation.
camera_enabled_ids='163 164 165 166'
source install/setup.bash
source /opt/dependencies_ws/install/setup.bash


export PYTHONPATH="/home/ros/src/camera_utility/:$PYTHONPATH"
export CALIB_PATH="/home/ros/src/camera_utility/"
echo "Available camera IDs: ${camera_enabled_ids[@]}"

if [ $# -eq 0 ]
  then
    echo "No arguments supplied. ** Try passing 'sim' as an argument ** "
fi

kill (){
    echo "--- killing processes ---"
    pkill -9 -f gzserver
    pkill -9 -f gzclient
    pkill -9 -f gazebo
    pkill -9 -f rviz
    pkill -9 -f humble
    pkill -9 -f object_mover
    pkill -9 -f camera_subscriber
    pkill -9 -f ign
    pkill -9 -f gz
}
clean () {
    kill
    echo "--- removing build files"
    rm -rf ./build ./install ./log
    rm -rf rosbag* ; rm -rf camera_utility/camera_data/*
    rm ROS_DOMAIN_ID.txt
}

build () {
    clean
    ROS_DOMAIN_ID=$((RANDOM % 102))
    echo $ROS_DOMAIN_ID > ROS_DOMAIN_ID.txt
    export ROS_DOMAIN_ID=$ROS_DOMAIN_ID
    mkdir camera_utility/camera_data/
    # Generate camera_config.xacro from camera_config.yaml
    (cd camera_utility/; ./camconf2xacro.sh ${camera_enabled_ids} > ../simulation/static_agent_launcher/description/camera_config.xacro )
    colcon build --merge-install --symlink-install --cmake-args " -Wno-dev "
    git checkout camera_utility/camera_data/ # undo changes
    #git checkout simulation/static_agent_launcher/description/camera_config.xacro # undo changes
    echo "successful build"
}

sim () {
    ros2 launch simlan_bringup sim.launch.py "camera_enabled_ids:=${camera_enabled_ids}"

}

# basic operation
if [[ "$*" == *"clean"* ]]
then
    clean
elif [[ "$*" == *"kill"* ]]
then
    kill
elif [[ "$*" == *"build"* ]]
then
    build
elif [[ "$*" == *"cmd"* ]]
then
    shift 1
    echo "running ::: $@"
    exec "$@"
elif [[ "$*" == *"sim"* ]]
then
    sim

## Teleop
elif [[ "$*" == *"jackal_teleop"* ]]
then
    ros2 launch dyno_jackal_bringup keyboard_steering.launch.py
elif [[ "$*" == *"pallet_truck_teleop"* ]]
then
    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r __ns:=/robot_agent_1 -r cmd_vel:=key_vel
## cartography, localization, navigation

elif [[ "$*" == *"cartographer"* ]]
then
    ros2 launch pallet_truck_navigation cartography.launch.py
    # ros2 run nav2_map_server map_saver_cli -f simulation/mapname

elif [[ "$*" == *"birdeye"* ]]
then
    for camera_id in $camera_enabled_ids; do
        ros2 run camera_bird_eye_view camera_bird_eye_view_publisher \
            --ros-args -p camera_id:=$camera_id \
            --ros-args -p point_start:="[4, -4]" \
            --ros-args -p point_end:="[24, 6]" \
            --ros-args -p resolution:=0.02 &
    done
    wait

elif [[ "$*" == *"gpss"* ]]
then
    sim &
    sleep 10
    ros2 launch aruco_localization multi_detection.launch.py use_sim_time:=true "camera_enabled_ids:=${camera_enabled_ids}" &
    sleep 10
    ros2 launch pallet_truck_bringup multiple_robot_spawn.launch.py

elif [[ "$*" == *"nav"* ]]
then
    ros2 launch pallet_truck_navigation map_server.launch.py &
    sleep 10
    ros2 launch pallet_truck_navigation nav2.launch.py namespace:=robot_agent_1 &
    ros2 launch pallet_truck_navigation nav2.launch.py namespace:=robot_agent_2

elif [[ "$*" == *"send_goals"* ]]
then
    ros2 action send_goal /robot_agent_1/navigate_to_pose nav2_msgs/action/NavigateToPose '{pose: {header: {frame_id: "map"}, pose: {position: {x: 18.0, y: 2.0, z: 0.0}, orientation: {w: 1.0}}}}' &
    pid1=$!

    ros2 action send_goal /robot_agent_2/navigate_to_pose nav2_msgs/action/NavigateToPose '{pose: {header: {frame_id: "map"}, pose: {position: {x: 18.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}' &
    pid2=$!

    wait $pid1
    wait $pid2

## Scenario
elif [[ "$*" == *"scenario"* ]]
then
    ros2 launch scenario_execution_ros scenario_launch.py scenario:=simulation/scenario_manager/scenarios/test.osc


## Store and replay

elif [[ "$*" == *"ros_record"* ]]
then
    ros2 bag record /cmd_vel
elif [[ "$*" == *"ros_replay"* ]]
then
    # replay last recording
    LAST_ROSBAG_DIR=$(ls -td rosbag* | head -1)
    ros2 bag info $LAST_ROSBAG_DIR
    ros2 bag play $LAST_ROSBAG_DIR

elif [[ "$*" == *"screenshot"* ]]
then
    # python3 ./camera_subscriber.py
    # python3 ./camera_subscriber.py  --action save
    # python3 ./camera_subscriber.py  --action removebg  --algo KNN
    # algo: MOG2, KNN
    cd ./camera_utility ;
    python3 camera_subscriber.py --action screenshot --camera $2 --shottime 4

elif [[ "$*" == *"aruco_detection"* ]]
then
    # assigns an aruco detection node to each camera listed in the argument
    ros2 launch aruco_localization multi_detection.launch.py use_sim_time:=true "camera_enabled_ids:=${camera_enabled_ids}"

elif [[ "$*" == *"camera_dump"* ]]
then
    cd ./camera_utility ;
    echo "Starting camera dump for cameras: ${camera_enabled_ids}"
    for camera_id in $camera_enabled_ids; do
        echo "Starting camera $camera_id"
        python3 camera_subscriber.py --action save --camera $camera_id &
    done
    # wait  # Wait for all background processes to complete

elif [[ "$*" == *"test"* ]]
then
    # Running python unittest, https://docs.ros.org/en/humble/Tutorials/Intermediate/Testing/Python.html
    # https://docs.ros.org/en/humble/Tutorials/Intermediate/Testing/Python.html
    colcon test --packages-select ros2_test --pytest-args --verbose
elif [[ "$*" == *"move_object"* ]]
then
    ros2 run object_mover move_object
fi
