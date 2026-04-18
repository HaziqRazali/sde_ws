import os
from glob import glob

from Cython.Build import cythonize
from Cython.Distutils import build_ext
from setuptools.command.install import install
from setuptools import find_packages, setup, Extension

package_name = 'sde_simulation'

class InstallWithBuildExt(install):
    def run(self):
        self.run_command("build_ext")
        super().run()

extensions = [
    Extension(
        "sde_simulation.sim_main_genesis",
        ["sde_simulation/sim_main_genesis.pyx"],
    ),
    Extension(
        "sde_simulation.utils.genesis_sim_env_builder",
        ["sde_simulation/utils/genesis_sim_env_builder.pyx"],
    ),
    Extension(
        "sde_simulation.utils.ros2_handler",
        ["sde_simulation/utils/ros2_handler.pyx"],
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

    ],
    install_requires=['setuptools'],
    zip_safe=False,         # IMPORTANT for C extensions, required for compiled .so
    maintainer='ngyc',
    maintainer_email='ngyc@a-star.edu.sg',
    description='generic simulation for SDE data collection, validation and augmentation',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sim_main_genesis = sde_simulation.sim_main_genesis:main',
        ],
    },
)
