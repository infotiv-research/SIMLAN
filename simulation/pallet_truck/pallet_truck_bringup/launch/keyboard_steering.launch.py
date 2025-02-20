import os.path

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import launch

# OR RUN THIS COMMAND IN TERMINAL
# ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r __ns:=/pallet_truck -r cmd_vel:=key_vel


def generate_launch_description():

    keyboard_steering = Node(
        package="teleop_twist_keyboard",
        executable="teleop_twist_keyboard",
        output="screen",
        remappings={("cmd_vel", "key_vel")},
        prefix="xterm -e",
    )

    ld = LaunchDescription()
    ld.add_action(keyboard_steering)

    return ld


def main(argv=None):
    launch_service = launch.LaunchService(debug=False)
    launch_service.include_launch_description(generate_launch_description())
    return launch_service.run()


if __name__ == "__main__":
    main()
