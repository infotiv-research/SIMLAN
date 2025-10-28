#!/bin/bash
#################################################################
#          SOURCING AND SETTING ENVIRONMENT VARIABLES           #
#################################################################

source install/setup.bash 2>/dev/null
source /opt/dependencies_ws/install/setup.bash 2>/dev/null

export ROS_DOMAIN_ID=$(cat ROS_DOMAIN_ID.txt)
export GZ_SIM_RESOURCE_PATH=/usr/share/ignition/
export IGN_GAZEBO_RESOURCE_PATH=usr/share/ignition/
export PYTHONPATH="$(pwd)/camera_utility/:$PYTHONPATH"
export CALIB_PATH="$(pwd)/camera_utility/"
export RCUTILS_LOGGING_SEVERITY_THRESHOLD=WARN
unset ROS_LOCALHOST_ONLY
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] [{name}] [{time}]: {message}"
#################################################################
#                  CONFIGURATIONS VARIABLES                     #
#       here lies all variables that can be configured          #
#################################################################

# CONFIG FILE 
CONFIG_FILE="$(dirname "$0")/config.sh"
source $CONFIG_FILE
echo "ROS_DOMAIN_ID.txt:" $ROS_DOMAIN_ID

#################################################################
#                        SAFETY CHECKS                          #
#################################################################

echo "Available camera IDs: ${CAMERA_ENABLED_IDS[@]}"
if [ $# -eq 0 ]
then
    echo "No arguments supplied. ** Try passing 'sim' as an argument ** "
fi

if [[ "$WORLD_SETUP" != "default" && "$WORLD_SETUP" != "medium" && "$WORLD_SETUP" != "light" && "$WORLD_SETUP" != "empty" ]]
then
    echo "[WARN] WORLD_SETUP is not set to any of the valid selections: default, medium, light or empty"
    WORLD_SETUP="default"
fi
echo "WORLD_SETUP = ${WORLD_SETUP}"


#################################################################
#                       BLACKLIST LOGS                          #
#################################################################

BLACKLIST_FILE="log_blacklist.txt"

# Remove Windows carriage returns (^M)
sed -i 's/\r$//' "$BLACKLIST_FILE"

# Remove trailing spaces and tabs at end of each line
sed -i 's/[[:space:]]*$//' "$BLACKLIST_FILE"

# Optionally remove empty lines (if you want)
sed -i '/^$/d' "$BLACKLIST_FILE"

if [[ -f "$BLACKLIST_FILE" && -s "$BLACKLIST_FILE" && ( "$log_level" == "warn" || "$log_level" == "error" ) ]]; then

    exec 3>&1 4>&2

    exec > >(
        while IFS= read -r line; do
            # Make a shadow copy without color for matching
            cleaned_line=$(echo -e "$line" | sed -r 's/\x1B\[[0-9;]*[mK]//g')

            # Check against each blacklist entry
            skip_line=0
            while IFS= read -r blacklist_pattern; do
                [[ -z "$blacklist_pattern" ]] && continue
                if [[ "$cleaned_line" == *"$blacklist_pattern"* ]]; then
                    skip_line=1
                    break
                fi
            done < "$BLACKLIST_FILE"

            # Only print lines that don’t match
            [[ $skip_line -eq 0 ]] && echo -e "$line"
        done
    ) 2>&1
fi

#################################################################
#                          FUNCTIONS                            #
#################################################################

kill (){
    echo "--- killing processes ---"
    pkill -9 -f gzserver
    pkill -9 -f gzclient
    pkill -9 -f gazebo
    pkill -9 -f rviz
    pkill -9 -f jazzy
    pkill -9 -f object_mover
    pkill -9 -f camera_subscriber
    pkill -9 -f ign
    pkill -9 -f gz
    pkill -9 -f camera_system
    pkill -9 -f motion_planner
    pkill -9 -f humble
    pkill -9 -f move_
    pkill -9 -f ros2
    pkill -9 -f ruby
    pkill -9 -f eog
}
clean () {
    kill
    echo "--- removing build files ---"
    rm -rf ./build ./install ./log
    rm -rf rosbag* ; rm -rf camera_utility/camera_data/* ; git checkout camera_utility/camera_data/ ;
    sed -i -E "s/^ROS_DOMAIN_ID=.*/ROS_DOMAIN_ID=/" "$CONFIG_FILE"

    rm -rf /mlenv /models
    rm -rf *.csv
}
generate_configs(){
    python3 config_generation/generate.py --robots "${ROBOTS}"
}
build () {
    clean
    ROS_DOMAIN_ID=$((RANDOM % 102))
    export AMENT_PREFIX_PATH="${AMENT_PREFIX_PATH/#$PWD\/install:/}"
    
    sed -i -E "s/^ROS_DOMAIN_ID=.*/ROS_DOMAIN_ID=$ROS_DOMAIN_ID/" "$CONFIG_FILE"
    export ROS_DOMAIN_ID=$ROS_DOMAIN_ID
    mkdir -p camera_utility/camera_data/
    # Generate camera_config.xacro from camera_config.yaml
    (cd camera_utility/; ./camconf2xacro.sh ${CAMERA_ENABLED_IDS} > ../simulation/static_agent_launcher/description/camera_config.xacro )
    colcon build --merge-install --symlink-install --cmake-args " -Wno-dev "
    git checkout camera_utility/camera_data/ # undo changes
    echo "successful build"
    generate_configs
}
sim () {
    ros2 launch simlan_bringup sim.launch.py "camera_enabled_ids:=${CAMERA_ENABLED_IDS}" "world_setup:=${WORLD_SETUP}" "log_level:=${log_level}"  

}
######################### HUMANDOID FUNCTIONS #########################
camera () {
    echo "::: CAMERA  :::"
    source install/setup.bash ; ros2 run camera_system camera_viewer.py  --ros-args -p output_dir:=$1 
}
humanoid () {
    echo "::: HUMANOID  :::"
    source install/setup.bash ; ros2 launch humanoid_robot launch_humanoid.launch.py "log_level:=${log_level}"  

}

random () {
    echo "::: RANDOM  :::"
    source install/setup.bash ; ros2 launch random_motion_planner random_motion.launch.py output_dir:=$1 "log_level:=${log_level}"
}

dataset () {
    echo "::: CREATING DATASET :::" in $1 with camera $2
    while true; do
        sim &
        sleep 10
        humanoid &
        sleep 5
        camera $1 &
        sleep 5
        random $1 &
        sleep 1200
        kill
        sleep 15
    done
}

mlenv () {
    python3 -m venv ~/mlenv
    unset PYTHONPATH ROS_PACKAGE_PATH AMENT_PREFIX_PATH COLCON_PREFIX_PATH
    export PYTHONNOUSERSITE=1
    source ~/mlenv/bin/activate
    pip install --upgrade pip
    pip install -r $humanoid_utility_dir/pose_to_motion/humanoid_ml_requirements.txt
    deactivate
}

moveit () {
    echo "::: MOVEIT  :::"
    source install/setup.bash ; ros2 launch random_motion_planner random_motion.launch.py motion_filename:=random output_dir:=$humanoid_dataset/motion_data "log_level:=${log_level}"
}

clean_humanoid_output () {
    echo "::: DELETE AND REBUILD :::"
    rm -rf $humanoid_output_dir ; mkdir $humanoid_output_dir ;
    mkdir  $humanoid_output_dir/motion_data 
    mkdir  $humanoid_output_dir/pose_data 
    mkdir  $humanoid_output_dir/pose_images 
}

replay_motion_helper () {
    echo "::: REPAYING $1 :::"
    source install/setup.bash ; ros2 launch random_motion_planner random_motion.launch.py motion_filename:="$1" "log_level:=${log_level}"
}

#################################################################
#                         OPERATIONS                            #
#################################################################

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

## Teleop, Input argument: the namespace specifying what robot you want to control. i.e. to run humanoid run: ./control.sh teleop humanoid
elif [[ "$*" == *"teleop"* ]]
then
    echo $2
    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r __ns:=/$2 -r cmd_vel:=key_vel

## cartography, localization, navigation
elif [[ "$*" == *"cartographer"* ]]
then
    ros2 launch pallet_truck_navigation cartography.launch.py "log_level:=${log_level}" 
    # ros2 run nav2_map_server map_saver_cli -f simulation/mapname

elif [[ "$*" == *"birdeye"* ]]
then
    ros2 launch camera_bird_eye_view bird_eye_view.launch.py "camera_ids:=${CAMERA_ENABLED_IDS}" point_start:="-8 -10" point_end:="34 10" input_img:="image_raw" "log_level:=${log_level}"   ##input_img: choose between image_raw (regular camera image), labels_map (the segmentation without color) or colored_map (colored segmentation)


## Run this command to save the segmentation and depth images from the simulation #### 
elif [[ "$*" == *"save_depth_seg_images"* ]]
then
    ros2 run camera_bird_eye_view camera_save --ros-args -p camera_ids:="${CAMERA_ENABLED_IDS}"


elif [[ "$*" == *"gpss"* ]]
then
    sim &
    sleep 10
    ros2 launch aruco_localization multi_detection.launch.py use_sim_time:=true "camera_enabled_ids:=${CAMERA_ENABLED_IDS}" "robots:=${ROBOTS}" "log_level:=${log_level}" &
    sleep 10
    ros2 launch pallet_truck_bringup multiple_robot_spawn.launch.py "robots:=${ROBOTS}" "log_level:=${log_level}"


elif [[ "$*" == *"nav"* ]]
then
    ros2 launch pallet_truck_navigation map_server.launch.py "robots:=${ROBOTS}" "log_level:=${log_level}" &


    ros2 launch pallet_truck_navigation nav2.launch.py "robots:=${ROBOTS}" "log_level:=${log_level}"


elif [[ "$*" == *"send_goal"* ]]
then
    ros2 action send_goal /robot_agent_1/navigate_to_pose nav2_msgs/action/NavigateToPose '{pose: {header: {frame_id: "robot_agent_1/map"}, pose: {position: {x: 15.0, y: -0.0, z: 0.0}, orientation: {w: 1.0}}}}' &
    pid1=$!

    ros2 action send_goal /robot_agent_2/navigate_to_pose nav2_msgs/action/NavigateToPose '{pose: {header: {frame_id: "robot_agent_2/map"}, pose: {position: {x: 15.0, y: -0.0, z: 0.0}, orientation: {w: 1.0}}}}' &
    pid2=$!

    wait $pid1
    wait $pid2

elif [[ "$*" == *"panda"* ]]
then
    #sim &
    #sleep 10
    ros2 launch moveit_resources_panda_moveit_config demo.launch.py "log_level:=${log_level}"

elif [[ "$*" == *"plan_motion"* ]]
then
    ros2 launch motion_planning_python_api motion_planning_python_api_tutorial.launch.py "log_level:=${log_level}"

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
    ros2 launch aruco_localization multi_detection.launch.py use_sim_time:=true "camera_enabled_ids:=${CAMERA_ENABLED_IDS}"

elif [[ "$*" == *"camera_dump"* ]]
then
    cd ./camera_utility ;
    echo "Starting camera dump for cameras: ${CAMERA_ENABLED_IDS}"
    for camera_id in $CAMERA_ENABLED_IDS; do
        echo "Starting camera $camera_id"
        python3 camera_subscriber.py --action save --camera $camera_id &
    done
    # wait  # Wait for all background processes to complete
# Example usage: ./control.sh dataset TRAIN



#################################HUMNAOID####################################
elif [[ "$*" == *"dataset"* ]]
then
    dataset $humanoid_dataset/$2 # 2nd terminal argument is TRAIN or EVAL, 3rd is camera id
elif [[ "$*" == *"humanoid"* ]]
then
    humanoid
elif [[ "$*" == *"random"* ]]
then
    random $2
elif [[ "$*" == *"camera"* ]]
then
    camera $2
elif [[ "$*" == *"mlenv"* ]]
then
    mlenv
elif [[ "$*" == *"moveit"* ]]
then
    moveit
elif [[ "$*" == *"convert2csv"* ]]
then
    echo "::: Convert dataset to CSV, please wait :::"
    python3 $humanoid_utility_dir/pose_to_motion/load_json_data.py $humanoid_dataset/EVAL/pose_data/  $humanoid_dataset/EVAL/motion_data/   $humanoid_dataset/eval.csv
    python3 $humanoid_utility_dir/pose_to_motion/load_json_data.py $humanoid_dataset/TRAIN/pose_data/ $humanoid_dataset/TRAIN/motion_data/  $humanoid_dataset/train.csv
elif [[ "$*" == *"train"* ]]
then
    source ~/mlenv/bin/activate
    echo "TRAIN MODEL, OVERWRITING PREVIOUS MODEL"
    time python3 $humanoid_utility_dir/pose_to_motion/$model_script --mode train --filename $humanoid_dataset/train_wide.csv 2>&1 | tee training.log
elif [[ "$*" == *"wide"* ]]
then
   echo "TRANSFER CSV TO WIDE FORMAT"
   python3 $humanoid_utility_dir/pose_to_motion/csvtowide.py $humanoid_dataset/eval.csv $humanoid_dataset/eval_wide.csv
elif [[ "$*" == *"eval"* ]]
then
    source ~/mlenv/bin/activate
    python3 $humanoid_utility_dir/pose_to_motion/$model_script --mode eval --filename $humanoid_dataset/eval_wide.csv
elif [[ "$*" == *"replay_motion"* ]]
then
    humanoid &
    sleep 5
    replay_motion_helper $2 # 2nd terminal argument
# TODO example
elif [[ "$*" == *"predict"* ]]
then
    echo "::: PREDICT MOTION FROM POSE DATA :::"
    if [ "$#" -ne 2 ]; then
    echo "Error: The second argument is missing" >&2
    echo "Usage: $0 predict PATH_TO_DIR that has pose_data" >&2
    exit 1
    fi
    source ~/mlenv/bin/activate
    dataset_root="$2"
    motion_dir="$2/motion_data" # output motion
    cams=(500 501 502 503)
    sim &
    sleep 15
    humanoid &
    sleep 10
    echo "START TO FIND DATA IN $dataset_root"
    find "$motion_dir" -maxdepth 1 -type f -name '*_motion.json' -print0 \
    | sort -z \
    | while IFS= read -r -d '' motion_file; do
        fname="$(basename "$motion_file")"
        number="${fname%_motion.json}"
        full_motion_path=$motion_dir/${number}_motion.json
        gt_opened=0
        for cam in "${cams[@]}"; do
            gt_img="$dataset_root/camera_${cam}/pose_images/${number}_display.jpg"
            if [ -f "$gt_img" ]; then
                echo ":::::::::::: GROUND TRUTH {$gt_img}"
                eog "$gt_img" & # open pose ground truth image
                gt_opened=1
                break
            fi
        done
        if [ $gt_opened -eq 0 ]; then
            echo "NOT FOUND: Ground truth image for $number"
        fi
        echo "Processing: $fname --- $number ----"
        python3 "$humanoid_utility_dir/pose_to_motion/AUTOGLUON_model.py" --mode predict --sample_root "$dataset_root" --sample_id "$number" --motionfile "${full_motion_path}_PREDICTED"
        replay_motion_helper "${full_motion_path}_PREDICTED" &         
        sleep 10         
        pkill -9 -f motion_planner || true         
        pkill -9 -f eog || true 
    done
elif [[ "$*" == *"image_pipeline"* ]]
then
    clean_humanoid_output
    python3 $humanoid_utility_dir/pose_to_motion/mp_detection.py --action image $humanoid_input_dir/image.png $humanoid_output_dir
elif [[ "$*" == *"video_pipeline"* ]]
then
    clean_humanoid_output
    python3 $humanoid_utility_dir/pose_to_motion/mp_detection.py --action video $humanoid_input_dir/20250611video.mp4 $humanoid_output_dir
elif [[ "$*" == *"test"* ]]
then
    colcon test --packages-select integration_tests --event-handlers console_direct+ --merge-install --pytest-args "-s"

elif [[ "$*" == *"actionlist"* ]]
then
    ros2 action list 
#################### dyno
elif [[ "$*" == *"visualize"* ]]
then
    	ros2 launch visualize_real_data scenario_replayer.launch.py



fi



