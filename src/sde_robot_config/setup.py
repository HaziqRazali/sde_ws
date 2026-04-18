from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'sde_robot_config'

data_files = [
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name, ['sde_version_number.txt'])
]

# Walk through config directory
for root, dirs, files in os.walk('config'):
    if files:
        install_dir = os.path.join(
            'share',
            package_name,
            root   # <-- preserves subfolders like booster_t1
        )

        file_paths = [os.path.join(root, f) for f in files]
        data_files.append((install_dir, file_paths))

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    # data_files=[
    #     ('share/ament_index/resource_index/packages',
    #         ['resource/' + package_name]),
    #     ('share/' + package_name, ['package.xml']),
    #     (os.path.join('share', package_name, 'config'), glob('config/**/*.yaml', recursive=True)),
    # ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ngyc',
    maintainer_email='ngyc@a-star.edu.sg',
    description='registering robot configuration for SDE functions',
    license='Apache-2.0',
    tests_require=['pytest'],
)
