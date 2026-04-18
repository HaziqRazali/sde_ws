import os
from glob import glob

from Cython.Build import cythonize
from Cython.Distutils import build_ext
from setuptools.command.install import install
from setuptools import find_packages, setup, Extension

package_name = 'casadi_optimal_control_ik'

class InstallWithBuildExt(install):
    def run(self):
        self.run_command("build_ext")
        super().run()

extensions = [
    Extension(
        "casadi_optimal_control_ik.ros2_ik_solver_handler",
        ["casadi_optimal_control_ik/ros2_ik_solver_handler.pyx"],
    ),
    Extension(
        "casadi_optimal_control_ik.ik_core",
        ["casadi_optimal_control_ik/ik_core.pyx"],
    ),
    Extension(
        "casadi_optimal_control_ik.utils.ik_config_loader",
        ["casadi_optimal_control_ik/utils/ik_config_loader.pyx"],
    ),
    Extension(
        "casadi_optimal_control_ik.utils.weighted_moving_filter",
        ["casadi_optimal_control_ik/utils/weighted_moving_filter.pyx"],
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
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hari',
    maintainer_email='palanivelu@a-star.edu.sg',
    description='Casadi-based Optimal Control for Humanoid Robots',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_ik_solver = casadi_optimal_control_ik.ros2_ik_solver_handler:main',
        ],
    },
)
