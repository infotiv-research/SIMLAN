import os
import time
from threading import Event, Thread
from geometry_msgs.msg._twist_stamped import TwistStamped
import launch
import launch_pytest
import launch_testing
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from launch.actions import (
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from rclpy.node import Node
from threading import Event

@launch_pytest.fixture(scope="module")
def launch_description():
    ######## Packages ########

    sim_pkg = get_package_share_directory("simlan_bringup")
    robot_pkg = get_package_share_directory("pallet_truck_bringup")
    aruco_localization_pkg = get_package_share_directory("aruco_localization")

    ######## Arguments ########
    camera_enabled_ids = "164 165"
    world_setup = "default"
    log_level = "error"
    robots = '[{ "namespace": "robot_agent_1", "initial_pose_x":"10.0", "initial_pose_y":"1.0", "robot_type":"pallet_truck", "aruco_id":"1"}]'
    ######## Launch-files/nodes to use  ########

    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_pkg, "launch", "multiple_robot_spawn.launch.py")
        ),
        launch_arguments={"log_level": log_level, "robots": robots}.items(),
    )

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(sim_pkg, "launch", "sim.launch.py")),
        launch_arguments={
            "camera_enabled_ids": camera_enabled_ids,
            "world_setup": world_setup,
            "log_level": log_level,
        }.items(),
    )
    aruco_localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(aruco_localization_pkg, "launch", "multi_detection.launch.py")
        ),
        launch_arguments={
            "camera_enabled_ids": camera_enabled_ids,
            "robots": robots,
            "log_level": log_level,
        }.items(),
    )
    
    ######## Test execution ########

    yield launch.LaunchDescription(
        [
            sim_launch,
            robot_launch,
            aruco_localization_launch,
            launch_testing.util.KeepAliveProc(),
            launch_pytest.actions.ReadyToTest(),
        ]
    )
 
@pytest.mark.launch(fixture=launch_description)
async def test_sim_and_multiple_robots_bringup_Startup_Nodes_and_topics_should_be_visible():

    print(
        "STARTING TEST: sim_and_multiple_robots_bringup_Startup_Nodes_and_topics_should_be_visible "
    )

    ###### SETUP ######
    num_retries = 3
    target_topics = [
        "/robot_agent_1/key_vel",
        "/robot_agent_1/robot_description",
        "/robot_agent_1/velocity_controller/cmd_vel",
        "/tf",
    ]
    target_nodes = [
        "/robot_agent_1/twist_mux",
        "/robot_agent_1/twist_stamper_node",
        "/ros_gz_bridge",
        "/gz_server"
    ]

    ###### EXECUTION ######
    rclpy.init()
    node = MakeTestNode("test_node")

    missing_nodes, missing_topics = [],[]
    for i in range(num_retries):

        actual_nodes = node.get_node_names()
        missing_nodes = [n for n in target_nodes if n not in actual_nodes]

        if (len(missing_nodes) == 0):
            break    
        elif i == num_retries:
            AssertionError("Number of attempts exceeded")
            return
        else:
            print("unable to find target nodes, attempting again in 5 secs")
            time.sleep(5)

    for i in range(num_retries):

        actual_topics = [key for (key, _) in node.get_topic_names_and_types()]
        missing_topics = [n for n in target_topics if n not in actual_topics]

        if len(missing_topics) == 0:
            break    
        elif i == num_retries:
            AssertionError("Number of attempts exceeded")
            return
        
        else:
            print("unable to find target topics, attempting again in 5 secs")
            time.sleep(5)
    rclpy.shutdown()
    
    ####### EVALUATION #######
    for target_node in target_nodes:
        assert target_node in actual_nodes, f"{target_node} was not found"
    for target_topic in target_topics:
        assert target_topic in actual_topics, f"{target_topic} was not found"

@pytest.mark.launch(fixture=launch_description, shutdown=True)
async def test_after_shutdown(launch_service, launch_description):
    # --- Cleanup command ---
    print("Final test after shutdown")
    pass

class MakeTestNode(Node):
    def __init__(self, name="test_node", subscription_topic=None, msg_type=None):
        super().__init__(name)
        self.subscription_topic = subscription_topic
        self.msg_type = msg_type
        self.is_event_done = Event()
        self.return_msg = TwistStamped()
    
    def start_subscriber(self):
        self.subscription = self.create_subscription(self.msg_type, self.subscription_topic, self.return_msg_as_string_callback, 10 )
    
        # Add a spin thread. Its important to let it start spinning so it can listen to topics
        self.ros_spin_thread = Thread(target=lambda node: rclpy.spin(node), args=(self,))
        self.ros_spin_thread.start()

    def return_msg_as_string_callback(self, msg):
        print(msg)
        self.return_msg = msg
        self.is_event_done.set()

    def get_return_msg(self,):
        self.return_msg = self.return_msg


    

###########################################################
#                       GRAVEYARD                         #
###########################################################
'''


gz_sim = ExecuteProcess(
        cmd=["gz", "sim", "-v", "4", "-r", "empty.sdf"],
        output="screen"
    )

    control_sim = ExecuteProcess(
        cmd=["bash", "./control.sh", "sim"],
        cwd="/home/ros/src",       # sets working directory
        output="screen",
    )

@pytest.fixture(autouse=True)
def ensure_event_loop():
    """Ensure there is a running event loop for ROS 2."""
    try:
        asyncio.get_running_loop()
    except RuntimeError:
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
    yield
    asyncio.set_event_loop(None)


@pytest.fixture(scope='module')
def order():
    print('once')
    yield []
    print('end')

@pytest.mark.filterwarnings("ignore:event loop")
@launch_pytest.fixture(scope='module', params=['asd', 'bsd'])
def launch_description(request):
    camera_enabled_ids=[164, 165]
    world_setup="default"
    log_level="info" 
    robots='[{"namespace": "robot_agent_1", "initial_pose_x":"10.0", "initial_pose_y":"1.0", "robot_type":"pallet_truck", "aruco_id":"1" }]'

    """Launch both sim and robot bringup."""
    sim_pkg = get_package_share_directory('simlan_bringup')
    robot_pkg = get_package_share_directory('pallet_truck_bringup')

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(sim_pkg, 'launch', 'sim.launch.py')),
        launch_arguments={
            "camera_enabled_ids":camera_enabled_ids,
            "world_setup": world_setup,
            "log_level": log_level,
            }.items(),
    )
    gz_sim = ExecuteProcess(
        cmd=["gz", "sim", "-v", "4", "-r", "empty.sdf"],
        output="screen"
    )
 
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(robot_pkg, 'launch', 'multiple_robot_spawn.launch.py')),
         launch_arguments={
            "log_level": log_level,
            "robots":robots
            }.items(),
    )

    return LaunchDescription(
        [
            gz_sim,
            #sim_launch, 
            #robot_launch,
            launch_testing.util.KeepAliveProc(),
            launch_pytest.actions.ReadyToTest()
        ]), request.param

@pytest.mark.filterwarnings("ignore:event loop")
@pytest.mark.launch(fixture=launch_description)
def test_after_shutdown(proc_output, launch_service, launch_description):
    # Wait for gzserver to start
    proc_output.assertWaitFor("gzserver", timeout=30)

    # Make sure the launch service has completed shutdown after test
    assert launch_service._is_idle()
    assert launch_service.event_loop is None

@pytest.mark.launch(fixture=launch_description, shutdown=True)
def test_after_shutdown(order, launch_service, launch_description):
    param = launch_description[1]
    order.append(f'test_after_shutdown[{param}]')
    assert launch_service._is_idle()
    assert launch_service.event_loop is None

@pytest.mark.filterwarnings("ignore:event loop")
@pytest.mark.launch(fixture=launch_description)
def test_case_1(order, launch_description):
    param = launch_description[1]
    order.append(f'test_case_1[{param}]')
    assert True


@pytest.mark.filterwarnings("ignore:event loop")
@pytest.mark.launch(fixture=launch_description)
def test_case_2(order, launch_description):
    param = launch_description[1]
    order.append(f'test_case_2[{param}]')
    assert True


@pytest.mark.filterwarnings("ignore:event loop")
@pytest.mark.launch(fixture=launch_description)
def test_case_3(order, launch_service, launch_description):
    param = launch_description[1]
    order.append(f'test_case_3[{param}]')
    yield
    assert launch_service._is_idle()
    assert launch_service.event_loop is None
    order.append(f'test_case_3[{param}][shutdown]')


@pytest.mark.filterwarnings("ignore:event loop")
@pytest.mark.launch(fixture=launch_description)
def test_case_4(order, launch_service, launch_description):
    param = launch_description[1]
    order.append(f'test_case_4[{param}]')
    yield
    assert launch_service._is_idle()
    assert launch_service.event_loop is None

    order.append(f'test_case_4[{param}][shutdown]')


def test_order(order):
    assert order == [
        'test_case_1[asd]',
        'test_case_2[asd]',
        'test_case_3[asd]',
        'test_case_4[asd]',
        'test_after_shutdown[asd]',
        'test_case_3[asd][shutdown]',
        'test_case_4[asd][shutdown]',
        'test_case_1[bsd]',
        'test_case_2[bsd]',
        'test_case_3[bsd]',
        'test_case_4[bsd]',
        'test_after_shutdown[bsd]',
        'test_case_3[bsd][shutdown]',
        'test_case_4[bsd][shutdown]',
    ]

# #@pytest.fixture
# @launch_pytest.fixture(scope="module")
# def my_fixture():
#     camera_enabled_ids=[164, 165]
#     world_setup="default"
#     log_level="info" 
#     robots='[{"namespace": "robot_agent_1", "initial_pose_x":"10.0", "initial_pose_y":"1.0", "robot_type":"pallet_truck", "aruco_id":"1" }]'

#     """Launch both sim and robot bringup."""
#     sim_pkg = get_package_share_directory('simlan_bringup')
#     robot_pkg = get_package_share_directory('pallet_truck_bringup')

#     sim_launch = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(os.path.join(sim_pkg, 'launch', 'sim.launch.py')),
#         launch_arguments={
#             "camera_enabled_ids":camera_enabled_ids,
#             "world_setup": world_setup,
#             "log_level": log_level,
#             }.items(),
#     )
#     gz_sim = ExecuteProcess(
#         cmd=["gz", "sim", "-v", "4", "-r", "empty.sdf"],
#         output="screen"
#     )
 
#     robot_launch = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(os.path.join(robot_pkg, 'launch', 'multiple_robot_spawn.launch.py')),
#          launch_arguments={
#             "log_level": log_level,
#             "robots":robots
#             }.items(),
#     )

#     return LaunchDescription(
#         [
#             gz_sim,
#             #sim_launch, 
#             #robot_launch,
#             launch_testing.util.KeepAliveProc(),
#             launch_pytest.actions.ReadyToTest()
#         ]
#     ),




# @pytest.mark.launch(fixture=my_fixture)
# def test_sim_and_robot_startup_running(proc_output):
#     assert True
#     proc_output.assertWaitFor("gzserver", timeout=30)
    #proc_output.assertWaitFor("controller_manager", timeout=30)    # rclpy.init()
    # node = rclpy.create_node("test_checker")
    # # wait up to 60 seconds for expected nodes
    # end_time = time.time() + 60
    # while time.time() < end_time:
    #     nodes = node.get_node_names()
    #     if '/controller_manager' in nodes and '/gazebo' in nodes:
    #         break
    #     rclpy.spin_once(node, timeout_sec=1.0)
    # else:
    #     pytest.fail(f"Nodes not found in ROS graph! Found: {nodes}")
    # node.destroy_node()
    # rclpy.shutdown()


# def test_robot_topics_exist(ros_node):
#     """Assert that robot topics appear in the ROS graph."""
#     # Wait for the system to fully start
#     time.sleep(20)

#     topics = [t[0] for t in ros_node.get_topic_names_and_types()]
#     expected_topics = [
#         '/robot_agent_1/cmd_vel',
#         '/robot_agent_1/key_vel',
#         '/robot_agent_2/cmd_vel',
#         '/robot_agent_2/key_vel',
#     ]

#     for topic in expected_topics:
#         assert topic in topics, f"Expected topic {topic} not found! Available topics: {topics}"
'''
