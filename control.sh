#!/bin/bash
#################################################################
#          SOURCING AND SETTING ENVIRONMENT VARIABLES           #
#################################################################
#region environment variables
source install/setup.bash 2>/dev/null
source /opt/dependencies_ws/install/setup.bash 2>/dev/null
source /opt/venv/bin/activate  2>/dev/null
export GZ_SIM_RESOURCE_PATH=/usr/share/ignition/
export IGN_GAZEBO_RESOURCE_PATH=usr/share/ignition/
export PYTHONPATH="$(pwd)/camera_utility/:$PYTHONPATH"
export CALIB_PATH="$(pwd)/camera_utility/"
export RCUTILS_LOGGING_SEVERITY_THRESHOLD=WARN
unset ROS_LOCALHOST_ONLY
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] [{name}] [{time}]: {message}"
#endregion
#################################################################
#                  CONFIGURATIONS VARIABLES                     #
#       here lies all variables that can be configured          #
#################################################################
#region configuration variables
# CONFIG FILE
CONFIG_FILE="$(dirname "$0")/config.sh"
source $CONFIG_FILE
echo "---[ ROS_DOMAIN_ID: $ROS_DOMAIN_ID  ]---"

#endregion
#################################################################
#                        SAFETY CHECKS                          #
#################################################################
#region safety checks

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CURRENT_DIR="$(pwd)"

if [[ "$CURRENT_DIR" != "$SCRIPT_DIR" ]]; then
    echo "---[ Error: Please run this script from its own directory: $SCRIPT_DIR ]---"
    exit 1
fi

echo "---[ Available camera IDs in simulation: ${CAMERA_ENABLED_IDS[@]} ]---"
if [ $# -eq 0 ]
then
    echo "---[ No arguments supplied. ** Try passing 'sim' as an argument **  ]---"
fi

if [[ "$WORLD_SETUP" != "default" && "$WORLD_SETUP" != "medium" && "$WORLD_SETUP" != "light" && "$WORLD_SETUP" != "empty" ]]
then
    echo "---[ WORLD_SETUP is not set to any of the valid selections: default, medium, light or empty ]---"
    WORLD_SETUP="default"
fi
echo "---[ WORLD_SETUP = ${WORLD_SETUP} ]---"
echo "---[ CAMERA_STREAMS = ${CAMERA_STREAMS} ]---"
echo "SPAWN_JACKAL = ${SPAWN_JACKAL}"

#endregion

#################################################################
#                          FUNCTIONS                            #
#################################################################
#region functions
kill (){
    echo "---[ killing processes ]---"
    pkill -9 -f gzserver
    pkill -9 -f gzclient
    pkill -9 -f gazebo
    pkill -9 -f rviz
    pkill -9 -f jazzy
    pkill -9 -f object_mover
    pkill -9 -f ign
    pkill -9 -f gz
    pkill -9 -f humanoid_camera_system
    pkill -9 -f humanoid_motion_planner
    pkill -9 -f humble
    pkill -9 -f move_
    pkill -9 -f ros2
    pkill -9 -f ruby
    pkill -9 -f eog
    pkill -9 -f update_map_node
    pkill -9 -f aruco_detection_node
    pkill -9 -f humanoid_odom_pub
    pkill -9 -f aruco_pose_pub_node
}
clean () {
    echo "---[ removing build files ]---"
    kill
    rm -rf ./build ./install ./log ./output
    rm -rf rosbag* ; rm -rf camera_utility/camera_data/* ; touch camera_utility/camera_data/empty.txt
    sed -i -E "s/^ROS_DOMAIN_ID=.*/ROS_DOMAIN_ID=/" "$CONFIG_FILE"

    rm -rf /mlenv /models
    rm -rf *.csv
}
generate_configs(){
    echo "---[ generating config ]---"
    python3 config_generation/generate.py --robots "${ROBOTS}" --humanoids "${HUMANOIDS}"
}
build () {
    clean
    echo "---[ building ]---"
    ROS_DOMAIN_ID=$((RANDOM % 102))
    export AMENT_PREFIX_PATH="${AMENT_PREFIX_PATH/#$PWD\/install:/}"

    sed -i -E "s/^ROS_DOMAIN_ID=.*/ROS_DOMAIN_ID=$ROS_DOMAIN_ID/" "$CONFIG_FILE"
    export ROS_DOMAIN_ID=$ROS_DOMAIN_ID
    mkdir -p camera_utility/camera_data/ ; touch camera_utility/camera_data/empty.txt
    (cd camera_utility/; ./camconf2xacro.sh ${CAMERA_ENABLED_IDS} > ../simulation/static_agent_launcher/description/camera_config.xacro )
    generate_configs
    colcon build --merge-install --symlink-install --cmake-args " -Wno-dev "
    echo "---[ successful build ]---"
}
sim () {
    echo "---[ launching gazebo ]---"
    ros2 launch simlan_bringup sim.launch.py "world_setup:=${WORLD_SETUP}" "log_level:=${log_level}" "headless_gazebo:=$headless_gazebo" "spawn_jackal:=${SPAWN_JACKAL}"

}
static_agent() {
    echo "---[ launching cameras (static agents) ]---"
    ros2 launch static_agent_launcher static-agent.launch.py "camera_enabled_ids:=${CAMERA_ENABLED_IDS}" "camera_streams:=${CAMERA_STREAMS}" "camera_update_rate":=${CAMERA_UPDATE_RATE} "log_level:=${log_level}"
}
#endregion
#################################################################
#                       HUMANDOID FUNCTIONS                     #
#################################################################
#region humanoid functions
camera () {
    echo "---[ CAMERA ]---"
    source install/setup.bash ; ros2 run humanoid_camera_system camera_viewer.py  --ros-args -p output_dir:=$1 -p camera_ids:="'${dataset_cameras}'"
}
humanoid () {
    echo "---[ launching humanoids ]---"
    ros2 launch humanoid_robot multiple_humanoid_spawn.launch.py "log_level:=${log_level}" "humanoids:=${HUMANOIDS}"

}

random_motion () {
    echo "---[ random motion]---"
    ros2 launch humanoid_motion_planner random_motion.launch.py run_random_generate:=true output_dir:=$1 "log_level:=${log_level}"
}

dataset () {
    echo "---[ dataset creation ]---" in $1 with camera $2
    while true; do
        echo "---[ new iteration ]---"
        sim &
        sleep 5 ; static_agent &
        sleep 6 ; camera $1 &
        sleep 7 ; humanoid &
        sleep 10   ; random_motion $1 &
        sleep 1200; kill
    done
}

mlenv () {
    python3 -m venv ~/mlenv
    unset PYTHONPATH ROS_PACKAGE_PATH AMENT_PREFIX_PATH COLCON_PREFIX_PATH
    export PYTHONNOUSERSITE=1
    source ~/mlenv/bin/activate
    pip install --upgrade pip
    pip install -r $humanoid_utility_dir/pose_to_motion/humanoid_ml_requirements.txt
    #pip install torch torchvision --index-url https://download.pytorch.org/whl/cu130
    deactivate
}

humanoid_moveit () {
    echo "---[ humanoid moveit ]---"
    ros2 launch humanoid_motion_planner random_motion.launch.py namespace:=$replay_motion_namespace run_random_generate:=false "log_level:=${log_level}"
}

execute_motion () {
    echo "---[ replaying $1 for $replay_motion_namespace ]---"
    ros2 topic pub --once $replay_motion_namespace/execute_motion std_msgs/String "{data: '$1'"}
}

save_depth_seg_videos (){
    echo "---[ save_depth_seg_videos ]---"
    ros2 run camera_bird_eye_view record_camera_stream --ros-args -p camera_ids:="${CAMERA_ENABLED_IDS}" -p camera_update_rate:=${CAMERA_UPDATE_RATE}
}

save_depth_seg_images(){
    echo "---[ save_depth_seg_images ]---"
    ros2 run camera_bird_eye_view camera_save --ros-args -p camera_ids:="${CAMERA_ENABLED_IDS}"
}


#endregion
#################################################################
#                         OPERATIONS                            #
#################################################################
#region all operations
########################  CORE ###################################
#region core and GPSS related operations
if [[ "$1" == "clean" ]]
then
    clean
elif [[ "$1" == "kill" ]]
then
    kill
elif [[ "$1" == "build" ]]
then
    build
elif [[ "$1" == "static_agent" ]]
then
    static_agent
elif [[ "$1" == "cmd" ]]
then
    shift 1
    echo "running ::: $@"
    exec "$@"

elif [[ "$1" == "sim" ]]
then
    sim

## Teleop, Input argument: the namespace specifying what robot you want to control. i.e. to run humanoid run: ./control.sh teleop humanoid
elif [[ "$1" == "teleop" ]]
then
    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r __ns:=/$2 -r cmd_vel:=key_vel

elif [[ "$1" == "birdeye" ]]
then
    # input_img: choose between image_raw (regular camera image), labels_map (the segmentation without color) or colored_map (colored segmentation)
    ros2 launch camera_bird_eye_view bird_eye_view.launch.py "camera_ids:=${CAMERA_ENABLED_IDS}" point_start:="-8 -10" point_end:="34 10" input_img:="image_raw" "log_level:=${log_level}"

## Run this command to save the segmentation and depth images from the simulation ####
elif [[ "$1" == "save_depth_seg_images" ]]
then
    save_depth_seg_images

elif [[ "$1" == "save_depth_seg_videos" ]]
then
    save_depth_seg_videos
elif [[ "$1" == "gpss" ]]
then
    sim &
    static_agent &
    sleep 10 ; ros2 launch aruco_localization multi_detection.launch.py use_sim_time:=true "camera_enabled_ids:=${CAMERA_ENABLED_IDS}" "robots:=${ROBOTS}" "log_level:=${log_level}" &
    sleep 10 ; ros2 launch pallet_truck_bringup multiple_robot_spawn.launch.py "robots:=${ROBOTS}" "log_level:=${log_level}"
#endregion
######################## NAV2 ###################################
#region nav2 related operations
elif [[ "$1" == "nav" ]]
then
    # navigate: ./control.sh nav HUMANOIDS or ROBOTS
    ros2 launch pallet_truck_navigation map_server.launch.py "robots:=${!2}" "log_level:=${log_level}" &
    sleep 3
    ros2 launch pallet_truck_navigation nav2.launch.py "robots:=${!2}" "log_level:=${log_level}"

elif [[ "$1" == "send_goal" ]]
then

    #send goal humanoids or pallet truck
    ros2 action send_goal /${2}/navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: '${2}/map'}, pose: {position: {x: ${3}, y: ${4}, z: 0.0}, orientation: {w: 1.0}}}}" &
    pid1=$!
    wait $pid1

#endregion
######################## PANDA MOVEIT2 ##########################
#region panda robot_arm_moveit2 related operations
elif [[ "$1" == "panda" ]]
then
    ros2 launch moveit_resources_panda_moveit_config demo.launch.py "log_level:=${log_level}"

elif [[ "$1" == "plan_panda_motion" ]]
then
    ros2 launch motion_planning_python_api motion_planning_python_api_tutorial.launch.py "log_level:=${log_level}"
#endregion
######################## HUMANOID ##############################
#region humanoid
elif [[ "$1" == "dataset" ]]
then
    dataset $humanoid_dataset/$2
elif [[ "$1" == "humanoid" ]]
then
    humanoid
elif [[ "$1" == "random_motion" ]]
then
    random_motion $2
elif [[ "$1" == "mlenv" ]]
then
    mlenv
elif [[ "$1" == "humanoid_moveit" ]]
then
    humanoid_moveit
elif [[ "$1" == "train" ]]
then
    echo "---[ Available camera IDs for machine learning: ${dataset_cameras}]---"
    source ~/mlenv/bin/activate
    time python3 $humanoid_utility_dir/pose_to_motion/model.py \
        --mode train \
        --input_dir $humanoid_dataset/$2 \
        --model_instance $model_instance \
        --model_type $model_type \
        --camera_ids "$dataset_cameras" \
         2>&1 | tee training.log


elif [[ "$1" == "eval" ]]
then
    source ~/mlenv/bin/activate
    python3 $humanoid_utility_dir/pose_to_motion/model.py \
        --mode eval \
        --input_dir $humanoid_dataset/$2 \
        --model_instance $model_instance \
        --model_type $model_type \
        --camera_ids "$dataset_cameras"

elif [[ "$1" == "predict" ]]
then
    kill
    sim &
    sleep 5 ; humanoid &
    sleep 8 ; humanoid_moveit &

    source ~/mlenv/bin/activate
    sleep 11; python3 $humanoid_utility_dir/pose_to_motion/model.py \
            --mode predict \
            --input_dir "$2" \
            --camera_ids "$dataset_cameras" \
            --model_type "$model_type" \
            --model_instance "$model_instance"\
            --humanoid_namespace $replay_motion_namespace\
            --save_output $save_prediction_output

elif [[ "$1" == "execute_motion" ]]
then
    execute_motion $2
elif [[ "$1" == "convert2csv" ]]
then
    echo "---[ Convert dataset to CSV, please wait ]---"
    python3 $humanoid_utility_dir/pre_processing/dataframe_json_bridge.py \
    --dataset_dir $humanoid_dataset/$2 \
    --csv_filename $humanoid_dataset/$2/flattened_data.csv \
    --camera_ids "$dataset_cameras"
#endregion
######################## TESTS ##################################
#region tests
elif [[ "$1" == "test" ]]
then
    colcon test --packages-select integration_tests --event-handlers console_direct+ --merge-install --pytest-args "-s"

#################### dyno
elif [[ "$1" == "visualize" ]]
then
    	ros2 launch visualize_real_data scenario_replayer.launch.py
#endregion

#endregion
fi