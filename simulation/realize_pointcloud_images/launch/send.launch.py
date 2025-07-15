from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from datetime import datetime
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():

    # VARIABLES: Doesn't have to be changed, but can if you want.
    namespace = "realize_pointcloud_images"
    playback_topic_name = "pointcloud_image"
    playback_rate = "7.5"

    # image position in the map frame
    x = 15.35
    y = 5.8
    z = -0.2
    # -----------------------------------------------------------


    saved_bags_path = Path(
        get_package_share_directory(namespace),
        "cached_images"
    )
    latest_bag_path = _retrieve_latest_recording(saved_bags_path)


    return LaunchDescription([

        Node( # Transform map->real_images
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_map_to_real_images",
            arguments=[str(x), str(y), str(z),"0", "0", "0",
                       "map", "real_images"],
        ),

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
