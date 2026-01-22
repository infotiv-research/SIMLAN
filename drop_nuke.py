#!/usr/bin/env python3

import os
import time

"""Used to kill all ROS processes running on the computer.
Sometimes the ./control.sh kill command misses some processes,
"""


def kill_gazebo():
    # Kill lingering Gazebo processes from previous runs
    for name in ["gzserver", "gzclient", "gz", "gazebo", "ignition"]:
        while pid := os.popen(f"pidof {name}").read().strip():
            print(f"shutting down {name}...")
            os.system(f"pkill -9 {pid}")
            time.sleep(1)
        print(f"{name} is shutdown!")


def kill_daemon():
    pid = (
        os.popen("ps aux | grep -i ros2-daemon | grep -v grep | awk '{print $2}'")
        .read()
        .strip()
    )
    if pid:
        print(f"killing daemon with pid: {pid}")
        os.system(f"kill -9 {pid}")
        time.sleep(1)


def kill_ros_processes():
    pids = (
        os.popen("ps aux | grep -i ros-args | grep -v grep | awk '{print $2}'")
        .read()
        .strip()
    )
    pids = [pid.strip() for pid in pids.split("\n") if pid.strip() != ""]
    for pid in pids:
        print(f"shutting down ros process with pid: {pid}")
        os.system(f"kill -9 {pid}")
        time.sleep(1)
    if pids:
        time.sleep(3)


if __name__ == "__main__":
    kill_gazebo()
    kill_daemon()
    kill_ros_processes()
