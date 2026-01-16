import os
import yaml
import re

PALLET_TRUCK_NAVIGATION_PKG_PATH = os.path.expanduser(
    "simulation/pallet_truck/pallet_truck_navigation"
)
FILE = "nav2_params_humanoid.yaml"


class FlowStyleList(list):
    pass


class QuotedString(str):
    pass


def flow_style_list_representer(dumper, data):
    return dumper.represent_sequence("tag:yaml.org,2002:seq", data, flow_style=True)


def quoted_str_representer(dumper, data):
    return dumper.represent_scalar("tag:yaml.org,2002:str", data, style='"')


yaml.add_representer(FlowStyleList, flow_style_list_representer)
yaml.add_representer(QuotedString, quoted_str_representer)


def generate_humanoid_nav2_params(humanoids):
    output = []
    #####################################################################################
    # The _blank_x are not needed but added for better visualization in nav2_params.yaml#
    #####################################################################################
    for humanoid in humanoids:
        namespace = humanoid["namespace"]
        output.append(
            {
                namespace: {
                    "behavior_server": {
                        "ros__parameters": {
                            "use_sim_time": True,
                            "robot_base_frame": f"{namespace}/base_link",
                            "odom_frame": f"{namespace}/odom",
                        }
                    },
                    "_blank_1": None,
                    "bt_navigator": {
                        "ros__parameters": {
                            "use_sim_time": True,
                            # change this hardcoded path
                            "default_nav_to_pose_bt_xml": f"simulation/pallet_truck/pallet_truck_navigation/config/navigate_w_replanning_and_recovery.xml",
                            "robot_base_frame": f"{namespace}/base_link",
                            "global_frame": f"{namespace}/map",
                            "plugin_lib_names": ["bt_failsafe_nodes"],
                            "_blank_2": None,
                            "navigate_to_pose": {
                                "plugin": "nav2_bt_navigator::NavigateToPoseNavigator",
                                "enable_groot_monitoring": False,
                                "groot_server_port": 1667,
                            },
                            "_blank_3": None,
                            "navigate_through_poses": {
                                "plugin": "nav2_bt_navigator::NavigateThroughPosesNavigator",
                                "enable_groot_monitoring": False,
                                "groot_server_port": 1669,
                            },
                            "_blank_4": None,
                            "error_code_name_prefixes": [
                                "assisted_teleop",
                                "backup",
                                "compute_path",
                                "dock_robot",
                                "drive_on_heading",
                                "follow_path",
                                "nav_thru_poses",
                                "nav_to_pose",
                                "route",
                                "spin",
                                "undock_robot",
                                "wait",
                            ],
                        }
                    },
                    "_blank_5": None,
                    "bt_navigator_rclcpp_node": {
                        "ros__parameters": {"use_sim_time": True}
                    },
                    "_blank_6": None,
                    "controller_server": {
                        "ros__parameters": {
                            "robot_base_frame": f"{namespace}/base_link",
                            "odom_frame": f"{namespace}/odom",
                            "global_frame": f"{namespace}/map",
                            "controller_frequency": 20.0,
                            "map_topic": f"/{namespace}/map",
                            "use_sim_time": True,
                            "debug_trajectory_details": True,
                            "controller_plugins": FlowStyleList(
                                [QuotedString("FollowPath")]
                            ),
                            "FollowPath": {
                                "plugin": "dwb_core::DWBLocalPlanner",
                                "min_vel_x": 0.0,
                                "min_vel_y": 0.0,
                                "max_vel_x": 3.26,
                                "max_vel_y": 0.0,
                                "max_vel_theta": 1.5,
                                "min_speed_xy": 0.0,
                                "max_speed_xy": 3.26,
                                "min_speed_theta": 0.0,
                                "min_x_velocity_threshold": 0.001,
                                "min_y_velocity_threshold": 0.001,
                                "min_theta_velocity_threshold": 0.001,
                                "acc_lim_x": 2.5,
                                "acc_lim_y": 0.0,
                                "acc_lim_theta": 3.2,
                                "decel_lim_x": -2.5,
                                "decel_lim_y": 0.0,
                                "decel_lim_theta": -3.2,
                                "vx_samples": 20,
                                "vy_samples": 5,
                                "vtheta_samples": 20,
                                "sim_time": 1.7,
                                "linear_granularity": 0.05,
                                "xy_goal_tolerance": 0.25,
                                "transform_tolerance": 0.5,
                                "critics": FlowStyleList(
                                    [
                                        QuotedString("RotateToGoal"),
                                        QuotedString("Oscillation"),
                                        QuotedString("BaseObstacle"),
                                        QuotedString("GoalAlign"),
                                        QuotedString("PathAlign"),
                                        QuotedString("PathDist"),
                                        QuotedString("GoalDist"),
                                    ]
                                ),
                                "BaseObstacle.scale": 0.02,
                                "PathAlign.scale": 0.0,
                                "GoalAlign.scale": 0.0,
                                "PathDist.scale": 32.0,
                                "GoalDist.scale": 24.0,
                                "RotateToGoal.scale": 32.0,
                            },
                        }
                    },
                    "_blank_7": None,
                    "controller_server_rclcpp_node": {
                        "ros__parameters": {"use_sim_time": True}
                    },
                    "_blank_8": None,
                    "local_costmap": {
                        "local_costmap": {
                            "ros__parameters": {
                                "update_frequency": 5.0,
                                "publish_frequency": 2.0,
                                "global_frame": f"{namespace}/map",
                                "map_topic": f"/{namespace}/map",
                                "robot_base_frame": f"{namespace}/base_link",
                                "odom_frame": f"{namespace}/odom",
                                "use_sim_time": True,
                                "plugin_names": FlowStyleList(
                                    [
                                        QuotedString("obstacle_layer"),
                                        QuotedString("voxel_layer"),
                                        QuotedString("inflation_layer"),
                                    ]
                                ),
                                "plugin_types": FlowStyleList(
                                    [
                                        QuotedString("nav2_costmap_2d::ObstacleLayer"),
                                        QuotedString("nav2_costmap_2d::VoxelLayer"),
                                        QuotedString("nav2_costmap_2d::InflationLayer"),
                                    ]
                                ),
                                "rolling_window": True,
                                "width": 10,
                                "height": 10,
                                "resolution": 0.05,
                                "robot_radius": 2.0,
                                "inflation_layer": {
                                    "cost_scaling_factor": 2.0,
                                    "inflation_radius": 1.3,
                                },
                                "obstacle_layer": {
                                    "enabled": True,
                                    "observation_sources": "scan",
                                    "scan": {
                                        "topic": "/scan",
                                        "max_obstacle_height": 2.0,
                                        "clearing": True,
                                        "marking": True,
                                        "data_type": "LaserScan",
                                    },
                                },
                                "voxel_layer": {
                                    "enabled": True,
                                    "publish_voxel_map": True,
                                    "origin_z": 0.0,
                                    "z_resolution": 0.2,
                                    "z_voxels": 10,
                                    "max_obstacle_height": 2.0,
                                    "mark_threshold": 0,
                                    "observation_sources": "pointcloud",
                                    "pointcloud": {
                                        "topic": "/intel_realsense_r200_depth/points",
                                        "max_obstacle_height": 2.0,
                                        "clearing": True,
                                        "marking": True,
                                        "data_type": "PointCloud2",
                                    },
                                },
                                "static_layer": {
                                    "map_topic": f"/{namespace}/map",
                                    "map_subscribe_transient_local": True,
                                },
                                "always_send_full_costmap": True,
                            }
                        },
                        "local_costmap_client": {
                            "ros__parameters": {"use_sim_time": True}
                        },
                        "local_costmap_rclcpp_node": {
                            "ros__parameters": {"use_sim_time": True}
                        },
                    },
                    "_blank_9": None,
                    "global_costmap": {
                        "global_costmap": {
                            "ros__parameters": {
                                "update_frequency": 1.0,
                                "publish_frequency": 1.0,
                                "global_frame": f"{namespace}/map",
                                "robot_base_frame": f"{namespace}/base_link",
                                "odom_frame": f"{namespace}/odom",
                                "use_sim_time": True,
                                "plugin_names": FlowStyleList(
                                    [
                                        QuotedString("static_layer"),
                                        QuotedString("obstacle_layer"),
                                        QuotedString("voxel_layer"),
                                        QuotedString("inflation_layer"),
                                    ]
                                ),
                                "plugin_types": FlowStyleList(
                                    [
                                        QuotedString("nav2_costmap_2d::StaticLayer"),
                                        QuotedString("nav2_costmap_2d::ObstacleLayer"),
                                        QuotedString("nav2_costmap_2d::VoxelLayer"),
                                        QuotedString("nav2_costmap_2d::InflationLayer"),
                                    ]
                                ),
                                "robot_radius": 0.22,
                                "inflation_layer": {
                                    "cost_scaling_factor": 2.0,
                                    "inflation_radius": 1.3,
                                },
                                "obstacle_layer": {
                                    "enabled": True,
                                    "observation_sources": "scan",
                                    "scan": {
                                        "topic": "/scan",
                                        "max_obstacle_height": 2.0,
                                        "clearing": True,
                                        "marking": True,
                                        "data_type": "LaserScan",
                                    },
                                },
                                "voxel_layer": {
                                    "enabled": True,
                                    "publish_voxel_map": True,
                                    "origin_z": 0.0,
                                    "z_resolution": 0.2,
                                    "z_voxels": 10,
                                    "max_obstacle_height": 2.0,
                                    "mark_threshold": 0,
                                    "observation_sources": "pointcloud",
                                    "pointcloud": {
                                        "topic": "/intel_realsense_r200_depth/points",
                                        "max_obstacle_height": 2.0,
                                        "clearing": True,
                                        "marking": True,
                                        "data_type": "PointCloud2",
                                    },
                                },
                                "static_layer": {
                                    "map_topic": f"/{namespace}/map",
                                    "map_subscribe_transient_local": True,
                                },
                                "always_send_full_costmap": True,
                            }
                        },
                        "global_costmap_client": {
                            "ros__parameters": {"use_sim_time": True}
                        },
                        "global_costmap_rclcpp_node": {
                            "ros__parameters": {"use_sim_time": True}
                        },
                    },
                    "_blank_10": None,
                    "planner_server": {
                        "ros__parameters": {
                            "use_sim_time": True,
                            "planner_plugins": FlowStyleList(
                                [QuotedString("GridBased")]
                            ),
                            "odom_frame": f"{namespace}/odom",
                            "GridBased": {
                                "plugin": "nav2_smac_planner::SmacPlannerHybrid",
                                "motion_model_for_search": "DUBIN",
                                "angle_quantization_bins": 72,
                            },
                        }
                    },
                    "_blank_11": None,
                    "planner_server_rclcpp_node": {
                        "ros__parameters": {"use_sim_time": True}
                    },
                }
            }
        )

    output_path = os.path.join(PALLET_TRUCK_NAVIGATION_PKG_PATH, "config", FILE)

    yaml_comment = """\
###############################################################################################################
# This params.yaml file was auto generated by config_generation/scripts/generate_humanoid_nav2_params_yaml.py #
###############################################################################################################"""

    def dump_with_spacing(data_list):
        dumped_items = []
        for item in data_list:
            dumped_yaml = yaml.dump(item, sort_keys=False, width=float("inf")).strip()
            lines = dumped_yaml.split("\n")
            clean_lines = []
            for line in lines:
                if re.search(r"_blank_\d+: null", line):
                    clean_lines.append("")
                else:
                    clean_lines.append("  " + line)
            dumped_items.append("\n".join(clean_lines))
        return "\n\n".join(dumped_items)

    with open(output_path, "w") as f:
        f.write(yaml_comment + "\n\n")
        f.write(dump_with_spacing(output))

    print(f"Generated config: {FILE}")
