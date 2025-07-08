from launch import LaunchDescription
from launch.actions import ExecuteProcess
from datetime import datetime
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():

    namespace = "realize_pointcloud_images"
    playback_topic_name = "pointcloud_image"
    playback_rate = "8"

    saved_bags_path = Path(
        get_package_share_directory(namespace),
        "cached_images"
    )

    latest_bag_path = _retrieve_latest(saved_bags_path)

    return LaunchDescription([

        # TODO: Maybe add a check so rosbag2 don't start sending the
        #       images until someone is actually listening.
        ExecuteProcess(
            name='rosbag2_player',
            cmd=['ros2', 'bag', 'play', str(latest_bag_path),
                 '--loop',
                 '--rate', playback_rate,
                 '--remap',
                 f'{namespace}/_internal_rpi_topic:={namespace}/{playback_topic_name}'
                ],
            output='screen'
            )
    ])

# ===================================================================

def _retrieve_latest(saved_bags_path):

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



