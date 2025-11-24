import os
import yaml
import re

HUMANOID_SUPPORT_MOVEIT_CONFIG_PATH = os.path.expanduser("simulation/humanoid_support_moveit_config")
class FlowStyleList(list):
    pass

class QuotedString(str):
    pass

def flow_style_list_representer(dumper, data):
    return dumper.represent_sequence('tag:yaml.org,2002:seq', data, flow_style=True)

def quoted_str_representer(dumper, data):
    return dumper.represent_scalar('tag:yaml.org,2002:str', data, style='"')

yaml.add_representer(FlowStyleList, flow_style_list_representer)
yaml.add_representer(QuotedString, quoted_str_representer)

def generate_humanoid_control_yaml(robots):
    output = []
    #################################################################################
    # The _blank_x are not needed but added for better visualization in control.yaml#
    #################################################################################
    for robot in robots:
        namespace = robot["namespace"]
        output.append({
            namespace: {
                "controller_manager": {
                    "ros__parameters": {
                        "update_rate": 100,

                        "_blank_1": None,

                        "support_whole_body_controller": {
                            "type": "joint_trajectory_controller/JointTrajectoryController"
                        },
                        "joint_state_broadcaster": {
                            "type": "joint_state_broadcaster/JointStateBroadcaster"
                        },
                        "velocity_controller": {
                            "type": "diff_drive_controller/DiffDriveController"
                        }
                    },

                "_blank_2": None,

                },
                "support_whole_body_controller": {
                    "ros__parameters": {
                        "joints": [
                            "link1_link2_joint", "link2_link3_joint", "link3_link4_joint",
                            "jL5S1_rotx", "jL5S1_roty", "jT9T8_rotx", "jT9T8_roty", "jT9T8_rotz",
                            "jC7LeftShoulder_rotx", "jLeftShoulder_rotx", "jLeftShoulder_roty",
                            "jLeftShoulder_rotz", "jLeftElbow_roty", "jLeftElbow_rotz",
                            "jLeftWrist_rotx", "jLeftWrist_rotz",
                            "jC7RightShoulder_rotx", "jRightShoulder_rotx", "jRightShoulder_roty",
                            "jRightShoulder_rotz", "jRightElbow_roty", "jRightElbow_rotz",
                            "jRightWrist_rotx", "jRightWrist_rotz",
                            "jT1C7_rotx", "jT1C7_roty", "jT1C7_rotz",
                            "jC1Head_rotx", "jC1Head_roty",
                            "jLeftHip_rotx", "jLeftHip_roty", "jLeftHip_rotz",
                            "jLeftKnee_roty", "jLeftKnee_rotz",
                            "jLeftAnkle_rotx", "jLeftAnkle_roty", "jLeftAnkle_rotz",
                            "jLeftBallFoot_roty",
                            "jRightHip_rotx", "jRightHip_roty", "jRightHip_rotz",
                            "jRightKnee_roty", "jRightKnee_rotz",
                            "jRightAnkle_rotx", "jRightAnkle_roty", "jRightAnkle_rotz",
                            "jRightBallFoot_roty"
                        ],
                        "command_interfaces": ["position"],
                        "state_interfaces": ["position", "velocity"]
                    },

                "_blank_3": None,

                },
                "velocity_controller": {
                    "ros__parameters": {
                        "left_wheel_names": ["front_left_wheel_joint", "rear_left_wheel_joint"],
                        "right_wheel_names": ["front_right_wheel_joint", "rear_right_wheel_joint"],
                        "wheel_separation": 0.36,
                        "wheels_per_side": 1,
                        "wheel_radius": 0.098,
                        "wheel_separation_multiplier": 1.5,
                        "left_wheel_radius_multiplier": 1.0,
                        "right_wheel_radius_multiplier": 1.0,

                        "_blank_4": None,

                        "publish_rate": 50.0,
                        "odom_frame_id": "odom",
                        "base_frame_id": "base_link",
                        "pose_covariance_diagonal": [0.001, 0.001, 1000000.0, 1000000.0, 1000000.0, 0.03],
                        "twist_covariance_diagonal": [1000000.0, 1000000.0, 0.03, 1000000.0, 1000000.0, 0.03],
                        "open_loop": False,
                        "enable_odom_tf": False,
                        "cmd_vel_timeout": 0.25,
                        "preserve_turning_radius": True,
                        "publish_cmd": True,
                        "publish_wheel_data": True,

                        "_blank_5": None,

                        "linear.x.has_velocity_limits": True,
                        "linear.x.has_acceleration_limits": True,
                        "linear.x.max_velocity": 2.0,
                        "linear.x.min_velocity": -2.0,
                        "linear.x.max_acceleration": 20.0,
                        "linear.x.min_acceleration": -20.0,
                        "linear.x.max_jerk": "NaN",
                        "linear.x.min_jerk": "NaN",

                        "_blank_6": None,

                        "angular.z.has_velocity_limits": True,
                        "angular.z.has_acceleration_limits": True,
                        "angular.z.max_velocity": 4.0,
                        "angular.z.min_velocity": -4.0,
                        "angular.z.max_acceleration": 25.0,
                        "angular.z.min_acceleration": -25.0,
                        "angular.z.max_jerk": "NaN",
                        "angular.z.min_jerk": "NaN"
                    }
                }
            }
        })
    file = "ros2_controllers.yaml"
    output_path = os.path.join(HUMANOID_SUPPORT_MOVEIT_CONFIG_PATH, "config", file)

    yaml_comment = """\
#####################################################################################################################
# This ros2_controllers.yaml file was auto generated by config_generation/scripts/generate_humanoid_control_yaml.py #
#####################################################################################################################"""

    def dump_with_spacing(data_list):
        dumped_items = []
        for item in data_list:
            dumped_yaml = yaml.dump(item, sort_keys=False, width=float("inf")).strip()
            lines = dumped_yaml.split('\n')
            clean_lines = []
            for line in lines:
                if re.search(r'_blank_\d+: null', line):
                    clean_lines.append('')
                else:
                    clean_lines.append('  ' + line)
            dumped_items.append('\n'.join(clean_lines))
        return '\n\n'.join(dumped_items)

    with open(output_path, 'w') as f:
        f.write(yaml_comment + '\n\n')
        f.write(dump_with_spacing(output))
    
    print(f"Generated config: {file}")
