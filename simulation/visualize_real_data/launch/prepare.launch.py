from pathlib import Path
from datetime import datetime
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
import yaml
from dyno_utils.launch_utils import DynoWaitFor
import std_msgs.msg
import rclpy.qos
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


def generate_launch_description():

    # VARIABLES: Doesn't have to be changed, but can be.
    #   -- namespace: Wraps the node
    #   -- parameters: changed through params.yaml config
    parameters = _read_config_file()
    # -----------------------------------------------------------

    # Define parameters for the recorder
    output_folder_path = _prepare_data_folder(parameters["namespace"])

    qos_override_path = Path(
        get_package_share_directory("visualize_real_data"),
        "config",
        "recorder_qos.yaml",
    )

    fake_orientation = LaunchConfiguration(
        "fake_orientation", default=parameters["fake_orientation"]
    )

    rosbag = ExecuteProcess(  # Record the PointCloud2 data
        name="rosbag2_recorder",
        cmd=[
            "ros2",
            "bag",
            "record",
            f"{parameters['namespace']}/{parameters['pointcloud_topic']}",
            f"{parameters['namespace']}/{parameters['entity_topic']}",
            "--include-hidden-topics",
            "--output",
            output_folder_path
            + "/"
            + parameters["json_file_name"].split("/")[-1].replace(".json", "")
            + "__"
            + datetime.now().strftime("%Y%m%d_%H%M%S"),
            "--qos-profile-overrides-path",
            str(qos_override_path),
        ],
        output="screen",
    )

    prepare_data = Node(
        name="prepare_data",
        namespace=parameters["namespace"],
        package="visualize_real_data",
        executable="prepare",
        parameters=[parameters],
        output="screen",
    )

    orientation_faker = Node(
        name="orientation_faker",
        namespace=parameters["namespace"],
        package="visualize_real_data",
        executable="orientation_faker",
        parameters=[parameters],
        output="screen",
        condition=IfCondition(fake_orientation),
    )

    kill_rosbag = RegisterEventHandler(
        OnProcessExit(target_action=prepare_data, on_exit=[EmitEvent(event=Shutdown())])
    )

    ld = LaunchDescription()
    ld.add_action(orientation_faker)
    ld.add_action(
        DynoWaitFor(
            name="wait_for_json_orientation",
            message_on_topics=[
                (
                    parameters["namespace"] + "/orientation_done",
                    std_msgs.msg.Empty,
                    rclpy.qos.QoSProfile(
                        depth=10, durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL
                    ),
                ),
            ],
            actions=[rosbag, prepare_data, kill_rosbag],
        )
    )
    return ld


# ===========================================================================


def _read_config_file():
    """__Explanation__:
    Read a YAML file and return its content.
    """
    # TODO: Change yaml name
    yaml_file_path = Path(
        get_package_share_directory("visualize_real_data"), "config", "params.yaml"
    )

    with open(yaml_file_path, "r") as file:
        data = yaml.safe_load(file)

    # Merge pointcloud and entity parameters
    prepare_params = data.get("prepare_data", {})

    prepare_params["images_folder"] = str(
        Path("/home/ros/src", "replay_data", prepare_params["images_folder"])
    )

    shared_params = data.get("shared", {})

    shared_params["json_file_name"] = str(
        Path("/home/ros/src", "replay_data", shared_params["json_file_name"])
    )

    prepare_params["config_file_path"] = str(yaml_file_path)

    return prepare_params | shared_params


def _prepare_data_folder(namespace):
    """__Explanation__:
    Create output directory if necessary
    and concatenate output folder path.
    Assumes that parent directories exist.
    """
    output_directory = Path("/home/ros/src", "replay_data", "rosbags")
    output_directory.mkdir(exist_ok=True)

    return f"{output_directory}"
