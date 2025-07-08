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
    # -----------------------------------------------------------

    output_directory = Path(
        get_package_share_directory(namespace),
        "cached_images"
    )

    output_folder_path = _prepare_data_folder(output_directory)
    prepare_node, rosbag2_recorder = _declare_nodes(namespace, topic_name,
                                                    output_folder_path)

    return LaunchDescription([
        prepare_node,
        rosbag2_recorder,
    ])

# ===========================================================================

def _declare_nodes(namespace, topic_name, output_folder_path):

    prepare_node = Node(
            name="prepare",
            namespace=namespace,
            package="realize_pointcloud_images",
            executable="prepare_data",
            parameters=[{'topic_name': topic_name}],
            output="screen"
            )
    
    # TODO: Possibly using rosbag2_transport instead for greater control
    #       over start and stop of recording and process in order to avoid
    #       having to rely on the cache when stopping.
    qos_config_path = Path('simulation', namespace,
                           'config', 'qos_config.yaml')

    rosbag2_recorder = ExecuteProcess(
            name='rosbag2_recorder',
            cmd=['ros2', 'bag', 'record', f"{namespace}/{topic_name}",
                 '--output', output_folder_path,
                 '--include-hidden-topics',
                 '--qos-profile-overrides-path', str(qos_config_path),
                ],
            output='screen',
            )

    return prepare_node, rosbag2_recorder


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