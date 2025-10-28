# GENERAL GAZEBO AND ROS2 CONFIG

## ROS_DOMAIN_ID is an auto generated value, modified by control.sh BUILD command
ROS_DOMAIN_ID=
## WORLD_SETUP can be set to either "default", "medium", "light", "empty"
WORLD_SETUP=empty 
## CAMERA_ENABLED_IDS can be set as a string of camera_ids separated by space ' '. valid camera ids are 160-171
CAMERA_ENABLED_IDS='162 163 164 165 166 500 501 502 503'
## DEFAULT: "info". Specify what level of logs you want. Valid log_levels are: "info", "warn", "error", "debug"
log_level="info" 

# HUMANOID CONFIG
humanoid_utility_dir="$(pwd)/humanoid_utility"
humanoid_input_dir="$humanoid_utility_dir/input"
humanoid_output_dir="$humanoid_utility_dir/output"
humanoid_dataset="$humanoid_utility_dir/DATASET"
model_script="AUTOGLUON_model.py" 

# ENABLED ROBOT SETUP CONFIG
ROBOTS='[
        {
            "namespace": "robot_agent_1",
            "initial_pose_x":"10.0",
            "initial_pose_y":"1.0",
            "robot_type":"pallet_truck",
            "aruco_id":"1"
        },
        {
             "namespace": "robot_agent_2",
             "initial_pose_x":"15.0",
             "initial_pose_y":"1.0",
             "robot_type":"pallet_truck",
             "aruco_id":"2"
        },
        {
             "namespace": "robot_agent_3",
             "initial_pose_x":"12.0",
             "initial_pose_y":"1.0",
             "robot_type":"forklift",
             "aruco_id":"3"
         },
         {
             "namespace": "robot_agent_4",
             "initial_pose_x":"10.0",
             "initial_pose_y":"0.0",
             "robot_type":"forklift",
             "aruco_id":"4"
         }
    ]'