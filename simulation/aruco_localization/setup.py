from setuptools import find_packages, setup
from glob import glob

package_name = "aruco_localization"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Pär Aronsson",
    maintainer_email="par.aronsson@infotiv.se",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "aruco_detection_node = aruco_localization.aruco_detection_node:main",
            "aruco_pose_pub = aruco_localization.aruco_pose_pub:main",
        ],
    },
)
