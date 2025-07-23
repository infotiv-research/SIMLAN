from setuptools import setup
from glob import glob

package_name = 'visualize_real_data'


setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    
    data_files=[
        # Marker and package file
        ('share/ament_index/resource_index/packages',['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Launch files
        ('share/' + package_name + '/launch', glob('launch/**')),

        # Config files
        ('share/' + package_name + '/config', glob('config/**')),

        # Data folder (structure)
        ('share/' + package_name + '/data', ['data/data']),
        ('share/' + package_name + '/data' + '/images', ['data/images/images']),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Linus Gustafsson',
    maintainer_email='linus@dynorobotics.se',
    description=['Converts .jpg data to pointcloud message,'
                'trajectory-data to MarkerArray message and'
                'saves it in a rosbag for later playback.'],
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'prepare = visualize_real_data.prepare:main'
        ],
    },
)
