import os
from glob import glob

from Cython.Build import cythonize
from Cython.Distutils import build_ext
from setuptools.command.install import install
from setuptools import find_packages, setup, Extension

package_name = 'teleoperation_interface'

class InstallWithBuildExt(install):
    def run(self):
        self.run_command("build_ext")
        super().run()

extensions = [
    Extension(
        "teleoperation_interface.vr_teleoperation_interface",
        ["teleoperation_interface/vr_teleoperation_interface.pyx"],
    ),
    Extension(
        "teleoperation_interface.interactive_marker_publisher",
        ["teleoperation_interface/interactive_marker_publisher.pyx"],
    ),
    Extension(
        "teleoperation_interface.utils.ros2_handler",
        ["teleoperation_interface/utils/ros2_handler.pyx"],
    ),
    Extension(
        "teleoperation_interface.utils.t1_teleop_wrapper",
        ["teleoperation_interface/utils/t1_teleop_wrapper.pyx"],
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
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='ngyc',
    maintainer_email='ngyc@a-star.edu.sg',
    description='VR teleoperation control for humanoid robots',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vr_teleoperation_interface = teleoperation_interface.vr_teleoperation_interface:main',
            'interactive_marker_publisher = teleoperation_interface.interactive_marker_publisher:main',
        ],
    },
)
