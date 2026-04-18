from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ros2bag_server'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    package_dir={package_name: package_name},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Include all config files
        (os.path.join('share', package_name, 'config'), glob('config/*')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yc',
    maintainer_email='ng_yung_chuen@i2r.a-star.edu.sg',
    description='ROS2Bag Server for Data Collection',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ros2bag_server = ros2bag_server.ros2bag_server_node:main',
        ],
    },
)
