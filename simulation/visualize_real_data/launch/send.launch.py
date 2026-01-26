from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.logging import get_logger
from datetime import datetime
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
import yaml

logger = get_logger("send.launch.py")


def generate_launch_description():

    # TODO: Make it match real speed. Borde gå att använda tidsstämplarna.
    # Sen ska man kunna ändra playbac rate (realtivt den verkliga hastigheten) i yaml
    # Rosbag playback rate
    params = _read_config_file()
    frame_position = params["frame_position"]
    playback_rate = params["playback_rate"]

    if not (playback_rate > 0):
        raise ValueError("Playback rate should always be larger than 0")

    # image position in the map frame
    x = frame_position["x"]
    y = frame_position["y"]
    z = frame_position["z"]

    print(f"\nxyz: {x},{y},{z}", flush=True)
    print(f"frame_id: {params['frame_id']}", flush=True)
    # -----------------------------------------------------------
    _read_config_file()

    base_path = "/home/ros/src"
    saved_bags_path = Path(base_path, "replay_data", "rosbags")

    if params["bag_name"] is not None:
        latest_bag_path = Path(saved_bags_path, params["bag_name"])
        if not latest_bag_path.exists():
            raise FileNotFoundError(
                f"Specified bag file does not exist: {latest_bag_path}"
            )
    else:
        logger.warning(
            f"No bag name specified, retrieving latest recording from {saved_bags_path}"
        )
        latest_bag_path = _retrieve_latest_recording(saved_bags_path)

    logger.info(f"Using rosbag2: {latest_bag_path}")
    logger.info(f"Playback rate: {playback_rate}")
    logger.info(f"Bag FPS: {params['extracted_fps']}")

    return LaunchDescription(
        [
            Node(  # Transform map->real_images
                package="tf2_ros",
                executable="static_transform_publisher",
                name="static_map_to_real_images",
                arguments=[
                    str(x),
                    str(y),
                    str(z),
                    "0",
                    "0",
                    "0",
                    "map",
                    params["frame_id"],
                ],
            ),
            ExecuteProcess(
                name="pointcloud_player",
                cmd=[
                    "xterm",
                    "-title",
                    "ROS2 Bag Player",
                    "-hold",  # Keep window open after process ends
                    "-e",
                    "bash",
                    "-c",
                    f"source /opt/ros/jazzy/setup.bash && "
                    f"ros2 bag play {str(latest_bag_path)} "
                    f"--loop --rate {str(playback_rate)}; exec bash",
                ],
                output="screen",
            ),
        ]
    )


# ===================================================================


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

    return data.get("send_data", {}) | data.get("shared", {})


def _retrieve_latest_recording(saved_bags_path: Path):

    keyword = "__"

    if not saved_bags_path.exists():
        raise FileNotFoundError(
            "Robsags directory does not exist. Have you recorded any data yet?"
        )
    elif not saved_bags_path.is_dir():
        raise NotADirectoryError(
            "Rosbags directory is a file. Delete it and re-record data (and double check your config)."
        )
    elif not any(saved_bags_path.iterdir()):
        raise FileNotFoundError(
            "Rosbag directory is empty. Have you recorded any data yet?"
        )
    for idx, bag_path in enumerate(saved_bags_path.iterdir()):

        if idx == 0:
            latest_bag_path = bag_path
            current = datetime.strptime(
                str(bag_path).split(keyword)[-1], "%Y%m%d_%H%M%S"
            )
            continue

        compare = datetime.strptime(str(bag_path).split(keyword)[-1], "%Y%m%d_%H%M%S")

        if compare > current:
            latest_bag_path = bag_path
            current = compare

    return latest_bag_path
