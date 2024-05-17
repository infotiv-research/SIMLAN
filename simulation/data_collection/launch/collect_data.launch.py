import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_infobot_agent = get_package_share_directory('infobot_agent')
    pkg_static_agents = get_package_share_directory('static_agent_launcher')
    pkg_dyno_jackal = get_package_share_directory('dyno_jackal_bringup')

    
    os.environ['GAZEBO_MODEL_PATH'] = os.environ['GAZEBO_MODEL_PATH'] + os.path.join(get_package_share_directory('infobot_gazebo_environment'), 'models') + ":" + os.path.join(get_package_share_directory('infobot_agent'), 'models') 
    print(os.environ['GAZEBO_MODEL_PATH'])
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    world = os.path.join(
        get_package_share_directory('infobot_gazebo_environment'),
        'worlds',
        'infobot_factory_data_collection.world'
    )
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={
            'world': world, 'use_sim_time': use_sim_time, "verbose": "true", "pause":"false"}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time,
                          "verbose": "true"}.items()
    )

    cameras = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_static_agents, "launch", "static-agent.launch.py")))

    forklift = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_infobot_agent, "launch", "infobot.launch.py")))
    
    jackal = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dyno_jackal, "launch", "sim.launch.py")))

    ld = LaunchDescription()

    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(cameras)
    ld.add_action(forklift)
    # ld.add_action(jackal)

    return ld
