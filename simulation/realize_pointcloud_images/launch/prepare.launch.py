from pathlib import Path
from datetime import datetime
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():

    # VARIABLES: Doesn't have to be changed, but can if you want.
    namespace = "realize_pointcloud_images"
    topic_name= "_internal_rpi_topic"

    # TODO: Maybe automate the setting to match in rviz?
    image_scale = 0.04
    # -----------------------------------------------------------


    output_directory = Path(
        get_package_share_directory(namespace),
        "cached_images"
    )
    output_folder_path = _prepare_data_folder(output_directory)

    qos_config_path = Path('simulation', namespace,
                           'config', 'qos_config.yaml')


    return LaunchDescription([
        Node( # Prepare the data, i.e. convert images to PointCloud2
            name="prepare",
            namespace=namespace,
            package="realize_pointcloud_images",
            executable="prepare_data",
            parameters=[{'topic_name': topic_name,
                         'image_scale': image_scale}],
            output="screen"
        ),
        ExecuteProcess( # Record the PointCloud2 data
            name='rosbag2_recorder',
            cmd=['ros2', 'bag', 'record', f"{namespace}/{topic_name}",
                 '--output', output_folder_path,
                 '--include-hidden-topics',
                 '--qos-profile-overrides-path', str(qos_config_path),
                ],
            output='screen',
        )
    ])

# ===========================================================================

def _prepare_data_folder(output_directory):
    """__Explanation__:
    Create output directory if necessary
    and concatenate output folder path.
    Assumes that parent directories exist.
    """
    output_directory.mkdir(exist_ok=True)

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_folder_name = f"rosbag2_{timestamp}"

    output_folder_path = f"{output_directory}/{output_folder_name}"
    return output_folder_path