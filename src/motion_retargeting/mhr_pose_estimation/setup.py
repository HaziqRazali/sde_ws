import os
from glob import glob

from Cython.Build import cythonize
from Cython.Distutils import build_ext
from setuptools.command.install import install
from setuptools import find_packages, setup, Extension

package_name = 'mhr_pose_estimation'

class InstallWithBuildExt(install):
    def run(self):
        self.run_command("build_ext")
        super().run()

extensions = [
    Extension(
        "mhr_pose_estimation.pose_estimator_node",
        ["mhr_pose_estimation/pose_estimator_node.pyx"],
    ),
]

ext_modules = cythonize(
    extensions,
    language_level=3,
    compiler_directives={
        "boundscheck": False,
        "wraparound": False,
        "cdivision": True,
    },
    quiet=True,
)

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    ext_modules=ext_modules,
    cmdclass={
        "build_ext": build_ext,
        "install": InstallWithBuildExt,
    },
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Include all rviz config files
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    package_data={
        package_name: ['tpose_joint_offsets.npz'],
    },
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='haziq',
    maintainer_email='haziq@a-star.edu.sg',
    description='human pose estimation module',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'demo_ik_collision_solver = mhr_pose_estimation.demo_ik_collision_solver_node:main',
            'mmpose_publisher = mhr_pose_estimation.mmpose_publisher_node:main',
            'mhr_subscriber = mhr_pose_estimation.mhr_subscriber_node:main',
            'pose_estimator = mhr_pose_estimation.pose_estimator_node:main',
            'elbow_echo = mhr_pose_estimation.elbow_echo:main',
        ],
    },
)
