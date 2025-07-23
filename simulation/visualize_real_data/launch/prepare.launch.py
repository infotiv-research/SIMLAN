from pathlib import Path
from datetime import datetime
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
import yaml

def generate_launch_description():

    # VARIABLES: Doesn't have to be changed, but can be.
    #   -- namespace: changed bellow. Wraps the node
    #   -- parameters: changed through params.yaml config 
    namespace = "visualize_real_data"
    parameters = _read_config_file()
    # -----------------------------------------------------------  


    # Define parameters for the recorder
    pointcloud_topic = f"{namespace}/{parameters['pointcloud_topic']}"
    entity_topic = f"{namespace}/{parameters['entity_topic']}"

    output_folder_path = _prepare_data_folder(namespace)

    qos_override_path = Path(
            get_package_share_directory('visualize_real_data'),
            "config",
            "recorder_qos.yaml"
        )

    rosbag = ExecuteProcess( # Record the PointCloud2 data
            name='rosbag2_recorder',
            cmd=['ros2', 'bag', 'record',
                 f"{pointcloud_topic}",  f"{entity_topic}",
                 '--include-hidden-topics',
                 '--output', output_folder_path,
                 '--qos-profile-overrides-path', str(qos_override_path),
                ],
            output='screen',
        )

    prepare_data = Node(
            name="prepare_data",
            namespace=namespace,
            package="visualize_real_data",
            executable="prepare",
            parameters=[parameters],
            output="screen"
        )
    
    kill_rosbag = RegisterEventHandler(
        OnProcessExit(
            target_action=prepare_data,
            on_exit=[
                EmitEvent(
                    event=Shutdown()
                )
            ]
        )
    )

    ld = LaunchDescription()
    ld.add_action(rosbag)
    ld.add_action(prepare_data)
    ld.add_action(kill_rosbag)
    return ld


# ===========================================================================

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
    
    # Merge pointcloud and entity parameters
    prepare_params = data.get('prepare_data', {})

    prepare_params['images_folder'] = str(Path(
        get_package_share_directory('visualize_real_data'),
        'data',
        prepare_params['images_folder']
    ))
    prepare_params['json_file_name'] = str(Path(
        get_package_share_directory('visualize_real_data'),
        'data',
        prepare_params['json_file_name']
    ))

    prepare_params['config_file_path'] = str(yaml_file_path)

    return prepare_params | data.get('shared', {})


def _prepare_data_folder(namespace):
    """__Explanation__:
    Create output directory if necessary
    and concatenate output folder path.
    Assumes that parent directories exist.
    """
    output_directory = Path(
        get_package_share_directory(namespace),
        "data",
        "rosbags"
    )
    output_directory.mkdir(exist_ok=True)

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_folder_name = f"rosbag2_{timestamp}"

    return f"{output_directory}/{output_folder_name}"