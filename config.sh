#################################################################
#         GENERAL GAZEBO AND ROS2 CONFIG                        #
#################################################################
#region GENERAL GAZEBO AND ROS2 CONFIG

## ROS_DOMAIN_ID is an auto generated value, modified by control.sh BUILD command
ROS_DOMAIN_ID=
## WORLD_SETUP can be set to either "default", "medium", "light", "empty"
WORLD_SETUP=medium
## CAMERA_ENABLED_IDS can be set as a string of camera_ids separated by space ' '.
# valid camera ids for GPSS are 160 161 162 163 164 165 166 167 168 169 170
# valid cameras ids for Humanoid are 00 501 502 503
CAMERA_ENABLED_IDS='160 161 162 163 164 165 166 167 168 169 170'
## CAMERA_STREAMS can be set as a string of stream options separated by space ' '. valid camera ids are "image", "depth", "semantic"
CAMERA_STREAMS='image'
CAMERA_UPDATE_RATE=5
## DEFAULT: "info". Specify what level of logs you want. Valid log_levels are: "info", "warn", "error", "debug"
log_level="error"
## DEFAULT: False. So that gazebo window runs.
headless_gazebo=False
rviz_config="rviz_config.rviz"

SPAWN_JACKAL=false

#endregion
#################################################################
#         HUMANOID CONFIG                                       #
#################################################################
#region HUMANOID CONFIG

humanoid_utility_dir="$(pwd)/humanoid_utility"
humanoid_input_dir="$humanoid_utility_dir/input"
humanoid_output_dir="$humanoid_utility_dir/output"
humanoid_dataset="$humanoid_utility_dir/DATASET"

save_prediction_output="true" # default "true", whether to save predicted data. Stored in the output/ folder.
# The cameras you want to train, eval and predict on. This can be a list of cameras i.e. "500 501 502 503". For single training set this to one  ""
dataset_cameras='500 501 502 503'
# Possible selections: pytorch, autogluon
model_type="pytorch"
# If you want to reuse a model, specify its name here. Keep blank if you dont want to save.
model_instance="pytorch_demo_all_cams"
replay_motion_namespace="humanoid_2"
HUMANOIDS='[
        {
            "namespace": "humanoid_1",
            "initial_pose_x":5.5,
            "initial_pose_y":-10.0
        },
        {
            "namespace": "humanoid_2",
            "initial_pose_x":5,
            "initial_pose_y":1.0
        }
    ]'
#endregion
#################################################################
#         ENABLED ROBOT SETUP CONFIG                            #
#################################################################
#region ENABLED ROBOT SETUP CONFIG
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
             "initial_pose_x":"7.0",
             "initial_pose_y":"1.0",
             "robot_type":"forklift",
             "aruco_id":"4"
         }
    ]'