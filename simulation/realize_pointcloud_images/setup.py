from setuptools import find_packages, setup
# from glob import glob

package_name = 'realize_pointcloud_images'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),

    package_data={package_name: ['data/**']}, # <- verkligen nödvändigt?

    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/prepare.launch.py',
                                               'launch/send.launch.py']),
        # ('share/' + package_name + '/bev_img', ['bev_img/**']),
    ],
    
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Linus Gustafsson',
    maintainer_email='linus@dynorobotics.se',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'build_pointcloud_data = realize_pointcloud_images.build_pointcloud_data:main',
            'prepare_data = realize_pointcloud_images.prepare_data:main',
            'modified_rosbag2 = realize_pointcloud_images.modified_rosbag2:main'
        ],
    },
)
