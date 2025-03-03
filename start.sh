#! /bin/bash

export GAZEBO_RESOURCE_PATH=/usr/share/gazebo-11
export GAZEBO_MODEL_PATH=/usr/share/gazebo-11/models:/usr/share/gazebo-11/models
source install/setup.bash
source /opt/dependencies_ws/install/setup.bash
if [ $# -eq 0 ]
  then
    echo "No arguments supplied. ** Try passing 'sim' as an argument ** "
fi

clean () {
    echo "--- killing processes ---"
    pkill -9 -f gzserver
    pkill -9 -f gzclient
    pkill -9 -f gazebo
    pkill -9 -f rviz
    pkill -9 -f humble
    pkill -9 -f object_mover
    pkill -9 -f camera_subscriber
    echo "--- removing build files"
    rm -rf ./build ./install ./log
    rm -rf rosbag* ; rm -rf processing/camera_data/*
}

build () {
    clean
    mkdir processing/camera_data/
    (cd processing/; ./camconf2xacro.sh > ../simulation/static_agent_launcher/description/camera_config.xacro )
    colcon build --merge-install --symlink-install --cmake-args " -Wno-dev "
    echo "successful build"
}


if [[ "$*" == *"clean"* ]]
then
    clean
elif [[ "$*" == *"build"* ]]
then
    build
elif [[ "$*" == *"rviz"* ]]
then
    rviz2 -d ./processing/rviz_config.rviz
elif [[ "$*" == *"jackal_teleop"* ]]
then
    ros2 launch dyno_jackal_bringup keyboard_steering.launch.py
elif [[ "$*" == *"pallet_truck_teleop"* ]]
then
    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r __ns:=/pallet_truck -r cmd_vel:=key_vel
elif [[ "$*" == *"infobot_teleop"* ]]
then
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
elif [[ "$*" == *"ros_record"* ]]
then
    ros2 bag record /cmd_vel
elif [[ "$*" == *"ros_replay"* ]]
then
    # replay last recording
    LAST_ROSBAG_DIR=$(ls -td rosbag* | head -1)
    ros2 bag info $LAST_ROSBAG_DIR
    ros2 bag play $LAST_ROSBAG_DIR
elif [[ "$*" == *"slam"* ]]
then
    ros2 launch slam_toolbox online_sync_launch.py use_sim_time:=True
    # ros2 run nav2_map_server map_saver_cli -f maps/mapname
elif [[ "$*" == *"commander"* ]]
then
    ros2 run nav2_commander commander
elif [[ "$*" == *"nav"* ]]
then
    ros2 launch infobot_navigation2 navigation2.launch.py use_sim_time:=True map:=maps/emptywarehouse.yaml
elif [[ "$*" == *"screenshot"* ]]
then
    # python3 ./camera_subscriber.py
    # python3 ./camera_subscriber.py  --action save
    # python3 ./camera_subscriber.py  --action removebg  --algo KNN
    # algo: MOG2, KNN
    cd ./processing ;
    python3 camera_subscriber.py --action screenshot --camera $2 --shottime 4

elif [[ "$*" == *"sim"* ]]
then
    ros2 launch simlan_bringup full_sim.launch.py

elif [[ "$*" == *"scenario"* ]]
then
    ros2 launch scenario_execution_ros scenario_launch.py scenario:=simulation/scenario_manager/scenarios/test.osc


elif [[ "$*" == *"camera_dump"* ]]
then
    cd ./processing ;
    # python3 camera_subscriber.py --action save --camera 160 &
    # python3 camera_subscriber.py --action save --camera 161 &
    # python3 camera_subscriber.py --action save --camera 162 &
    # python3 camera_subscriber.py --action save --camera 163 &
    python3 camera_subscriber.py --action save --camera 164
    # python3 camera_subscriber.py --action save --camera 165 &
    # python3 camera_subscriber.py --action save --camera 166 &
    # python3 camera_subscriber.py --action save --camera 167 &
    # python3 camera_subscriber.py --action save --camera 168 &
    # python3 camera_subscriber.py --action save --camera 169 &
    # python3 camera_subscriber.py --action save --camera 170 &
    # python3 camera_subscriber.py --action save --camera 171 &



    # python3 camera_subscriber.py --action save --camera 264 &
    # python3 camera_subscriber.py --action save --camera 265 &
    # python3 camera_subscriber.py --action save --camera 266 &
    # python3 camera_subscriber.py --action save --camera 267 &
    # python3 camera_subscriber.py --action save --camera 268 &

    # python3 camera_subscriber.py --action save --camera 364 &
    # python3 camera_subscriber.py --action save --camera 365 &
    # python3 camera_subscriber.py --action save --camera 366 &
    # python3 camera_subscriber.py --action save --camera 367 &
    # python3 camera_subscriber.py --action save --camera 368 &

    # python3 camera_subscriber.py --action save --camera 464 &
    # python3 camera_subscriber.py --action save --camera 465 &
    # python3 camera_subscriber.py --action save --camera 466 &
    # python3 camera_subscriber.py --action save --camera 467 &
    # python3 camera_subscriber.py --action save --camera 468 &


elif [[ "$*" == *"test"* ]]
then
    # Running python unittest, https://docs.ros.org/en/humble/Tutorials/Intermediate/Testing/Python.html
    # https://docs.ros.org/en/humble/Tutorials/Intermediate/Testing/Python.html
    colcon test --packages-select ros2_test --pytest-args --verbose
elif [[ "$*" == *"move_object"* ]]
then
    ros2 run object_mover move_object
fi
