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
#endregion
#################################################################
#                  CONFIGURATIONS VARIABLES                     #
#       here lies all variables that can be configured          #
#################################################################
#region configuration variables
# CONFIG FILE
CONFIG_FILE="$(dirname "$0")/config.sh"
source $CONFIG_FILE
echo "ROS_DOMAIN_ID:" $ROS_DOMAIN_ID

#endregion
#################################################################
#                        SAFETY CHECKS                          #
#################################################################
#region safety checks

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CURRENT_DIR="$(pwd)"

if [[ "$CURRENT_DIR" != "$SCRIPT_DIR" ]]; then
    echo "Error: Please run this script from its own directory: $SCRIPT_DIR"
    exit 1
fi

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
echo "CAMERA_STREAMS = ${CAMERA_STREAMS}"

#endregion
#################################################################
#                       BLACKLIST LOGS                          #
#################################################################
#region blacklist logs
BLACKLIST_FILE="resources/log_blacklist.txt"

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
#endregion
#################################################################
#                          FUNCTIONS                            #
#################################################################
#region functions
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
    pkill -9 -f update_map_node
    pkill -9 -f aruco_detection_node
    pkill -9 -f humanoid_odom_pub
    pkill -9 -f aruco_pose_pub_node

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
    python3 config_generation/generate.py --robots "${ROBOTS}" --humanoids "${HUMANOIDS}"
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
    # Generate configs before build
    generate_configs

    colcon build --merge-install --symlink-install --cmake-args " -Wno-dev "
    git checkout camera_utility/camera_data/ # undo changes
    echo "successful build"

}
sim () {
    ros2 launch simlan_bringup sim.launch.py "world_setup:=${WORLD_SETUP}" "log_level:=${log_level}"

}
static_agent() {
    ros2 launch static_agent_launcher static-agent.launch.py "camera_enabled_ids:=${CAMERA_ENABLED_IDS}" "camera_streams:=${CAMERA_STREAMS}" "log_level:=${log_level}"
}
#endregion
#################################################################
#                       HUMANDOID FUNCTIONS                     #
#################################################################
#region humanoid functions
camera () {
    echo "::: CAMERA  :::"
    source install/setup.bash ; ros2 run camera_system camera_viewer.py  --ros-args -p output_dir:=$1
}
humanoid () {
    echo "::: HUMANOID  :::"
    source install/setup.bash ; ros2 launch humanoid_robot multiple_humanoid_spawn.launch.py "log_level:=${log_level}" "humanoids:=${HUMANOIDS}"

}

random () {
    echo "::: RANDOM  :::"
    source install/setup.bash ; ros2 launch random_motion_planner random_motion.launch.py output_dir:=$1 "log_level:=${log_level}"
}

dataset () {
    echo "::: CREATING DATASET :::" in $1 with camera $2
    while true; do
        sleep 15  ; echo "STARTING A NEW ITERATION"
        sim &
        static_agent &
        sleep 10  ; humanoid &
        sleep 5   ; camera $1 &
        sleep 5   ; random $1 &
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
multi_ground_truth_viewer () {
    echo "START TO FIND DATA IN  $humanoid_dataset/$dataset_root"
    cams=(500 501 502 503)
    find "$motion_dir" -maxdepth 1 -type f -name '*_motion.json' | sort | while read -r motion_file; do
        fname="$(basename "$motion_file")"
        number="${fname%_motion.json}"
        predict_file="$predict_dir/${number}_predict.json"

        gt_opened=0
        for cam in "${cams[@]}"; do
            gt_img="$humanoid_dataset/$dataset_root/camera_${cam}/pose_images/${number}_display.jpg"
            if [ -f "$gt_img" ]; then
                echo ":::::::::::: GROUND TRUTH {$gt_img}"
                eog "$gt_img" & # open pose ground truth image
                gt_opened=1
            fi
        done
        if [ $gt_opened -eq 0 ]; then
            echo "NOT FOUND: Ground truth image for $number"
        fi
        echo "Processing: $fname --- $number ----"
        echo "Saving prediction to: $predict_file"
        python3 $humanoid_utility_dir/pose_to_motion/$model_type/model.py \
        --mode multi_predict \
        --sample_root "$dataset_root" \
        --sample_id "$number" \
        --motion_file "$predict_file" \
        --model_instance $model_instance \

        replay_motion_helper "$predict_file" &
        sleep 30
        pkill -9 -f motion_planner || true
        pkill -9 -f eog || true
    done

}
single_ground_truth_viewer () {
    echo "START TO FIND DATA IN  $humanoid_dataset/$dataset_root"
    find "$pose_dir" -maxdepth 1 -type f -name '*_pose.json' | sort | while read -r pose_file; do
        fname="$(basename "$pose_file")"
        number="${fname%_pose.json}"
        full_pose_path="$pose_dir/${number}_pose.json"
        pose_image_path="$pose_image_dir/${number}_display.jpg"
        predict_file="$predict_dir/${number}_predict_cam${cam_id}.json"
        if [ -f "$pose_image_path" ]; then
                echo "Ground truth image: $pose_image_path"
                eog "$pose_image_path" &
        else
                echo "WARNING: No ground truth image found at $pose_image_path"
        fi
        echo "Processing: $fname --- $number ----"
        echo "Saving prediction to: $predict_file"
        python3 $humanoid_utility_dir/pose_to_motion/$model_type/model.py \
        --mode single_predict \
        --sample_root "$dataset_root" \
        --sample_id "$number" \
        --model_instance $model_instance \
        --pose_file "$full_pose_path" \
        --motion_file "$predict_file"

        replay_motion_helper "$predict_file" &
        sleep 30
        pkill -9 -f motion_planner || true
        pkill -9 -f eog || true
    done
}
#endregion
#################################################################
#                         OPERATIONS                            #
#################################################################
#region all operations
########################  CORE ###################################
#region core and GPSS related operations
if [[ "$1" == *"clean"* ]]
then
    clean
elif [[ "$1" == *"kill"* ]]
then
    kill
elif [[ "$1" == *"build"* ]]
then
    build
elif [[ "$1" == *"static_agent"* ]]
then
    static_agent
elif [[ "$1" == *"cmd"* ]]
then
    shift 1
    echo "running ::: $@"
    exec "$@"

elif [[ "$1" == *"sim"* ]]
then
    sim

## Teleop, Input argument: the namespace specifying what robot you want to control. i.e. to run humanoid run: ./control.sh teleop humanoid
elif [[ "$1" == *"teleop"* ]]
then
    echo $2
    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r __ns:=/$2 -r cmd_vel:=key_vel


elif [[ "$1" == *"birdeye"* ]]
then
    ros2 launch camera_bird_eye_view bird_eye_view.launch.py "camera_ids:=${CAMERA_ENABLED_IDS}" point_start:="-8 -10" point_end:="34 10" input_img:="image_raw" "log_level:=${log_level}"   ##input_img: choose between image_raw (regular camera image), labels_map (the segmentation without color) or colored_map (colored segmentation)


## Run this command to save the segmentation and depth images from the simulation ####
elif [[ "$1" == *"save_depth_seg_images"* ]]
then
    ros2 run camera_bird_eye_view camera_save --ros-args -p camera_ids:="${CAMERA_ENABLED_IDS}"


elif [[ "$1" == *"gpss"* ]]
then
    sim &
    static_agent &
    sleep 10 ; ros2 launch aruco_localization multi_detection.launch.py use_sim_time:=true "camera_enabled_ids:=${CAMERA_ENABLED_IDS}" "robots:=${ROBOTS}" "log_level:=${log_level}" &
    sleep 10 ; ros2 launch pallet_truck_bringup multiple_robot_spawn.launch.py "robots:=${ROBOTS}" "log_level:=${log_level}"
#endregion
######################## NAV2 ###################################
#region nav2 related operations
elif [[ "$1" == *"nav"* ]]
then
    # navigate: ./control.sh nav HUMANOIDS or ROBOTS
    ros2 launch pallet_truck_navigation map_server.launch.py "robots:=${!2}" "log_level:=${log_level}" &
    sleep 3
    ros2 launch pallet_truck_navigation nav2.launch.py "robots:=${!2}" "log_level:=${log_level}"


elif [[ "$1" == *"send_goal"* ]]
then

    #send goal humanoids
    ros2 action send_goal /${2}/navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: '${2}/map'}, pose: {position: {x: ${3}, y: ${4}, z: 0.0}, orientation: {w: 1.0}}}}" &
    pid1=$!

    wait $pid1


## cartography, localization, navigation
elif [[ "$1" == *"cartographer"* ]]
then
    ros2 launch pallet_truck_navigation cartography.launch.py "log_level:=${log_level}"
    # ros2 run nav2_map_server map_saver_cli -f simulation/mapname
#endregion
######################## PANDA MOVEIT2 ##########################
#region panda moveit2 related operations
elif [[ "$1" == *"panda"* ]]
then
    ros2 launch moveit_resources_panda_moveit_config demo.launch.py "log_level:=${log_level}"

elif [[ "$1" == *"plan_panda_motion"* ]]
then
    ros2 launch motion_planning_python_api motion_planning_python_api_tutorial.launch.py "log_level:=${log_level}"
#endregion
######################## HUMANOID ##############################
#region humanoid
# Example usage: ./control.sh dataset TRAIN
elif [[ "$1" == *"dataset"* ]]
then
    dataset $humanoid_dataset/$2
elif [[ "$1" == *"humanoid"* ]]
then
    sim &
    humanoid
elif [[ "$1" == *"random"* ]]
then
    random $2
elif [[ "$1" == *"camera"* ]]
then
    camera $2
elif [[ "$1" == *"mlenv"* ]]
then
    mlenv
elif [[ "$1" == *"moveit"* ]]
then
    moveit
elif [[ "$1" == *"convert2csv"* ]]
then
    echo "::: Convert dataset to CSV, please wait :::"
    python3 $humanoid_utility_dir/pre_processing/load_json_data.py $humanoid_dataset/EVAL/pose_data/  $humanoid_dataset/EVAL/motion_data/   $humanoid_preprocessed_dataset/default_eval.csv
    python3 $humanoid_utility_dir/pre_processing/load_json_data.py $humanoid_dataset/TRAIN/pose_data/ $humanoid_dataset/TRAIN/motion_data/  $humanoid_preprocessed_dataset/default_train.csv
    echo "TRANSFER CSV TO WIDE FORMAT"
    python3 $humanoid_utility_dir/pre_processing/csvtowide.py $humanoid_preprocessed_dataset/default_train.csv $humanoid_preprocessed_dataset/TRAIN/multi_train.csv
    python3 $humanoid_utility_dir/pre_processing/csvtowide.py $humanoid_preprocessed_dataset/default_eval.csv $humanoid_preprocessed_dataset/EVAL/multi_eval.csv

elif [[ "$1" == *"multi_train"* ]]
then
    source ~/mlenv/bin/activate
    echo "TRAIN MODEL, OVERWRITING PREVIOUS MODEL"
    time python3 $humanoid_utility_dir/pose_to_motion/$model_type/model.py \
        --mode multi_train \
        --dataset_filename $humanoid_preprocessed_dataset/TRAIN/$humanoid_dataset_training \
        --model_instance $model_instance \
         2>&1 | tee training.log
elif [[ "$1" == *"single_train"* ]] # SINGLE TRAIN: KEEP
then
    source ~/mlenv/bin/activate
    time python3 $humanoid_utility_dir/pose_to_motion/$model_type/model.py \
        --mode single_train \
        --dataset_filename $humanoid_preprocessed_dataset/TRAIN/$humanoid_dataset_training \
        --model_instance $model_instance \
         2>&1 | tee training.log

elif [[ "$1" == *"eval"* ]] # same eval as before
then
    source ~/mlenv/bin/activate
    python3 $humanoid_utility_dir/pose_to_motion/$model_type/model.py \
        --mode eval \
        --dataset_filename $humanoid_preprocessed_dataset/EVAL/$humanoid_dataset_evaluation\
        --model_instance $model_instance \

elif [[ "$1" == *"replay_motion"* ]]
then
    sim &
    sleep 15 ; humanoid &
    sleep 5  ; replay_motion_helper $2 # 2nd terminal argument

elif [[ "$1" == *"multi_predict"* ]]
then
    echo "::: PREDICT MOTION FROM MULTI CAM POSE DATA :::"
    if [ "$#" -ne 2 ]; then
    echo "Error: The second argument is missing" >&2
    echo "Usage: $0 predict PATH_TO_DIR that has pose_data" >&2
    exit 1
    fi
    source ~/mlenv/bin/activate

    dataset_root="$2"
    motion_dir="$humanoid_dataset/$2/motion_data" # output motion
    predict_dir="$humanoid_dataset/$2/predict_data" # prediction output folder
    mkdir -p "$predict_dir"

    sim &
    sleep 15 ; humanoid &
    sleep 10 ; multi_ground_truth_viewer "$dataset_root" "$predict_dir"

elif [[ "$1" == *"single_predict"* ]]
then
    echo "::: PREDICT MOTION FROM SINGLE CAM POSE DATA :::"
    if [ "$#" -ne 3 ]; then
        echo "Error: Missing arguments" >&2
        echo "Usage: $0 single_predict PATH_TO_DIR CAM_ID" >&2
        echo "Example: $0 single_predict TEST 500" >&2
        exit 1
    fi
    source ~/mlenv/bin/activate

    dataset_root="$2"
    cam_id="$3"
    pose_dir="$humanoid_dataset/$dataset_root/camera_${cam_id}/pose_data"
    pose_image_dir="$humanoid_dataset/$dataset_root/camera_${cam_id}/pose_images"
    predict_dir="$humanoid_dataset/$2/predict_data" # prediction output folder
    mkdir -p "$predict_dir"

    sim &
    sleep 15 ; humanoid &
    sleep 10 ; single_ground_truth_viewer "$dataset_root" "$cam_id" "$predict_dir"

elif [[ "$1" == *"image_pipeline"* ]]
then
    clean_humanoid_output
    python3 $humanoid_utility_dir/pre_processing/mp_detection.py --action image $humanoid_input_dir/image.png $humanoid_output_dir
elif [[ "$1" == *"video_pipeline"* ]]
then
    clean_humanoid_output
    python3 $humanoid_utility_dir/pre_processing/mp_detection.py --action video $humanoid_input_dir/20250611video.mp4 $humanoid_output_dir
#endregion
######################## TESTS ##################################
#region tests
elif [[ "$1" == *"test"* ]]
then
    colcon test --packages-select integration_tests --event-handlers console_direct+ --merge-install --pytest-args "-s"
    kill
#################### dyno
elif [[ "$1" == *"visualize"* ]]
then
    	ros2 launch visualize_real_data scenario_replayer.launch.py
#endregion
######################## Other operations #######################
#region other operations
## Store and replay
elif [[ "$1" == *"ros_record"* ]]
then
    ros2 bag record /cmd_vel
elif [[ "$1" == *"ros_replay"* ]]
then
    # replay last recording
    LAST_ROSBAG_DIR=$(ls -td rosbag* | head -1)
    ros2 bag info $LAST_ROSBAG_DIR
    ros2 bag play $LAST_ROSBAG_DIR

elif [[ "$1" == *"screenshot"* ]]
then
    # python3 ./camera_subscriber.py
    # python3 ./camera_subscriber.py  --action save
    # python3 ./camera_subscriber.py  --action removebg  --algo KNN
    # algo: MOG2, KNN
    cd ./camera_utility ;
    python3 camera_subscriber.py --action screenshot --camera $2 --shottime 4

elif [[ "$1" == *"camera_dump"* ]]
then
    cd ./camera_utility ;
    echo "Starting camera dump for cameras: ${CAMERA_ENABLED_IDS}"
    for camera_id in $CAMERA_ENABLED_IDS; do
        echo "Starting camera $camera_id"
        python3 camera_subscriber.py --action save --camera $camera_id &
    done
    # wait  # Wait for all background processes to complete
#endregion
#endregion
fi