from setuptools import find_packages
from setuptools import setup

package_name = "infobot_teleop"

setup(
    name=package_name,
    version="2.1.5",
    packages=find_packages(exclude=[]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=[
        "setuptools",
    ],
    zip_safe=True,
    author="Hamid Ebadi",
    author_email="hamid.ebadi@gmail.com",
    maintainer="Hamid Ebadi",
    maintainer_email="hamid.ebadi@gmail.com",
    keywords=["ROS"],
    classifiers=[
        "Intended Audience :: Developers",
        "License :: OSI Approved :: Apache Software License",
        "Programming Language :: Python",
        "Topic :: Software Development",
    ],
    description=("Teleoperation node using keyboard for TurtleBot3."),
    license="Apache License, Version 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "teleop_keyboard = infobot_teleop.script.teleop_keyboard:main"
        ],
    },
)
