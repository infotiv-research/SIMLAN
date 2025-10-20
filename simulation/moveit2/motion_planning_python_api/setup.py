from setuptools import find_packages, setup
from glob import glob
package_name = 'motion_planning_python_api'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
        ("share/" + package_name + "/config", glob("config/*")),
        ("share/" + package_name + "/models", glob("models/*")),
        ("share/" + package_name + "/scripts", glob("scripts/*")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros',
    maintainer_email='ros@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motion_planning_python_api_tutorial = scripts.motion_planning_python_api_tutorial:main',
            'motion_planning_python_api_planning_scene = scripts.motion_planning_python_api_planning_scene:main',
            'demo_pick_and_place = scripts.demo_pick_and_place:main'
        ],
    },
)
