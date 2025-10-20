from setuptools import setup
from glob import glob
import os

package_name = 'random_motion_planner'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ("share/" + package_name + "/config", glob("config/*")),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='siyu yi',
    maintainer_email='siyu.yi@infotiv.se',
    description='Random motion planner for humanoid robot',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motion_planner = random_motion_planner.motion_planner:main',
        ],
    },
)
