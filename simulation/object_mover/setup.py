import os
from glob import glob
from setuptools import find_packages, setup

package_name = "object_mover"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Christoffer Johannesson",
    maintainer_email="christoffer@dynorobotics.se",
    description="move objects within the world",
    license="Apache 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "move_object = object_mover.move_object:main",
        ],
    },
)
