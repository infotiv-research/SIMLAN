from setuptools import setup
from glob import glob

package_name = "visualize_real_data"


setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    data_files=[
        # Marker and package file
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # Launch files
        ("share/" + package_name + "/launch", glob("launch/**")),
        # Config files
        ("share/" + package_name + "/config", glob("config/**")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Sebastian Olsson",
    maintainer_email="sebastian@dynorobotics.se",
    description=[
        "Converts .jpg data to pointcloud message,"
        "trajectory-data to MarkerArray message and"
        "saves it in a rosbag for later playback."
    ],
    license="TODO: License declaration",
    entry_points={
        "console_scripts": [
            "prepare = visualize_real_data.prepare:main",
            "scenario_replayer = visualize_real_data.scenario_replayer:main",
            "scenario_replay_cmd_vel = visualize_real_data.scenario_replay_cmd_vel:main",
            "orientation_faker = visualize_real_data.orientation_faker:main",
        ],
    },
)
