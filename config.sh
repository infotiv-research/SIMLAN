#################################################################
#         GENERAL GAZEBO AND ROS2 CONFIG                        #
#################################################################
#region GENERAL GAZEBO AND ROS2 CONFIG

## ROS_DOMAIN_ID is an auto generated value, modified by control.sh BUILD command
ROS_DOMAIN_ID=
## WORLD_SETUP can be set to either "default", "medium", "light", "empty"
WORLD_SETUP=medium
## CAMERA_ENABLED_IDS can be set as a string of camera_ids separated by space ' '. valid camera ids are 160-171
CAMERA_ENABLED_IDS='164 165 166 167 168'
## CAMERA_STREAMS can be set as a string of stream options separated by space ' '. valid camera ids are image depth semantic
CAMERA_STREAMS='image'
CAMERA_UPDATE_RATE=5
## DEFAULT: "info". Specify what level of logs you want. Valid log_levels are: "info", "warn", "error", "debug"
log_level="info"
#endregion
#################################################################
#         HUMANOID CONFIG                                       #
#################################################################
#region HUMANOID CONFIG

humanoid_utility_dir="$(pwd)/humanoid_utility"
humanoid_input_dir="$humanoid_utility_dir/input"
humanoid_output_dir="$humanoid_utility_dir/output"
humanoid_dataset="$humanoid_utility_dir/DATASET"


# The cameras you want to train on. This can be a list of cameras i.e. "500 501 502". For single training set this to one  ""
dataset_cameras='500'
# Possible selections: pytorch, autogluon
model_type="pytorch"
# If you want to reuse a model, specify its name here. Keep blank if you dont want to save.
model_instance="pytorch_test_pred_500"
replay_motion_namespace="humanoid_2"
HUMANOIDS='[
        {
            "namespace": "humanoid_1",
            "initial_pose_x":5.5,
            "initial_pose_y":-10.0,
            "cam_ns": "camera0"
        },
        {
            "namespace": "humanoid_2",
            "initial_pose_x":5,
            "initial_pose_y":1.0,
            "cam_ns": "camera0"
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