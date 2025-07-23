from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from datetime import datetime
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
import yaml


def generate_launch_description():

    # VARIABLES: Doesn't have to be changed, but can if you want.
    namespace = "visualize_real_data"

    # TODO: Make it match real speed. Borde gå att använda tidsstämplarna.
    # Sen ska man kunna ändra playbac rate (realtivt den verkliga hastigheten) i yaml
    # Rosbag playback rate
    params = _read_config_file()
    frame_position = params['frame_position']
    playback_rate = params['playback_rate']

    if not (playback_rate > 0):
        raise ValueError("Playback rate should always be larger than 0")
    else:
        # Transform x fps to 10 fps * playback rate
        playback_rate = (1/params['processing_time_limit'])*params['extracted_fps']*playback_rate

    # image position in the map frame
    x = frame_position['x']
    y = frame_position['y']
    z = frame_position['z']

    print(f"\nxyz: {x},{y},{z}", flush=True)
    print(f"frame_id: {params['frame_id']}", flush=True)
    # -----------------------------------------------------------
    _read_config_file()

    saved_bags_path = Path(
        get_package_share_directory(namespace),
        "data",
        "rosbags"
    )
    latest_bag_path = _retrieve_latest_recording(saved_bags_path)


    return LaunchDescription([

        Node( # Transform map->real_images
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_map_to_real_images",
            arguments=[str(x), str(y), str(z),"0", "0", "0",
                       "map", params['frame_id']],
        ),

        ExecuteProcess(
            name='pointcloud_player',
            cmd=['ros2', 'bag', 'play', str(latest_bag_path),
                 '--loop',
                 '--rate', str(playback_rate),
                ],
            output='screen'
        )
    ])

# ===================================================================

def _read_config_file():
    """__Explanation__:
    Read a YAML file and return its content.
    """
    # TODO: Change yaml name
    yaml_file_path = Path(
        get_package_share_directory('visualize_real_data'),
        'config',
        'params.yaml'
    )

    with open(yaml_file_path, 'r') as file:
        data = yaml.safe_load(file)

    return data.get('send_data', {}) | data.get('shared', {})

def _retrieve_latest_recording(saved_bags_path):

    keyword = "rosbag2_"
    keyword_size = len(keyword)

    # TODO: Handle what happens if there are no existing
    #       rosbag2's.
    for idx, bag_path in enumerate(saved_bags_path.iterdir()):

        slice_idx = str(bag_path).find(keyword)+keyword_size

        if idx == 0:
            latest_bag_path = bag_path
            current = datetime.strptime(str(bag_path)[slice_idx:],
                                        "%Y%m%d_%H%M%S")
            continue

        compare = datetime.strptime(str(bag_path)[slice_idx:],
                                    "%Y%m%d_%H%M%S")

        if compare > current:
            latest_bag_path = bag_path
            current = compare

    return latest_bag_path
