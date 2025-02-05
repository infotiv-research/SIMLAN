from setuptools import find_packages, setup

package_name = 'scenario_manager'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'py_trees', 'scenario_execution'],
    zip_safe=True,
    maintainer='ros',
    maintainer_email='hjalmar@dynorobotics.se',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'scenario_execution.actions': [
            'teleport_action = scenario_manager.teleport_action_server:TeleportActionServer',
        ],
        'console_scripts': [
            'teleport_action_server = scenario_manager.teleport_action_server:main',
            'set_speen_action_server = scenario_manager.set_speed_action_server:main',
        ],
    },
)
