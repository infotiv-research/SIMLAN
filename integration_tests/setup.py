import os
from glob import glob

from setuptools import find_packages, setup

package_name = "integration_tests"


setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
        ("share/" + package_name + "/test", glob("test/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ros",
    maintainer_email="par.aronsson@infotiv.se",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
)
