# Copyright (c) 2026, Agency for Science, Technology and Research (A*STAR) All Rights Reserved.
# License TO-DO: Apache License, Version 2.0  
# Maintainer: Ng Yung Chuen, Email: ng_yung_chuen@a-star.edu.sg

from ament_index_python.packages import get_package_share_directory
import yaml
import os
import time
from termcolor import colored

import genesis as gs
from genesis.utils.geom import trans_R_to_T, euler_to_R
import numpy as np

import rclpy


class SimEnvBuilder():
    def __init__(self, robot_type, sim_env_name, enable_gui, config):
        self.robot_type = robot_type
        self.sim_env_name = sim_env_name
        self.enable_gui = enable_gui
        self.sim_dt = config['simulation']['dt']
        self.robot_description_package_name = config['robot_model']['description_package_name']
        self.robot_urdf_path = config['robot_model']['urdf_path']
        self.robot_cam_info_dict = config['robot_model']['camera']
        self.robot_spawn_pos = config['robot_model']['spawn_pos']
        self.robot_links_to_keep = config['robot_model']['links_to_keep']
        self.robot_suspend_base_link = config['robot_model']['suspend_base_link']
        self.robot_controllable_joint_names = tuple(config['robot_model']['controllable_joint_names'])
        self.robot_reference_configuration = config['robot_model']['reference_configuration']
        self.robot_imu_config = config['robot_model']['imu']
        self.verbose = config['verbose']

    def initialize_camera(self):
        # use function euler_to_R(rpy) for angle (In Yaw-Pitch-Roll convention)
        self.cam_dict = {}
        for key, info in self.robot_cam_info_dict.items():
            cam = self.scene.add_camera(
                    res=tuple(info['resolution']),
                    pos=(0., 0., 0.),
                    # lookat=(0.5, 0., 0.),
                    fov=info['fov'],
                    GUI=info['gui'])
            cam.attach(self.robot.get_link(info['attached_link']), trans_R_to_T(trans=np.array(info['displacement_from_link_centre']), R=euler_to_R(np.array(info['rot_from_link_centre']))))
            self.cam_dict[key] = info
            self.cam_dict[key]['scene_cam_obj'] = cam

    def generate_robot(self):
        """Generate the robot object."""
        # Generate the robot in simulation
        # what can I do with robot entity? https://genesis-world.readthedocs.io/en/latest/api_reference/entity/rigid_entity/index.html
        self.robot = self.scene.add_entity(
            gs.morphs.URDF(file=os.path.join(get_package_share_directory(self.robot_description_package_name), self.robot_urdf_path),
                pos=tuple(self.robot_spawn_pos),
                euler=(0.0, 0.0, 0.0), # or use quat with tuple shape (4,)
                visualization=True,
                collision=True,
                requires_jac_and_IK=True,
                fixed=self.robot_suspend_base_link, 
                links_to_keep=self.robot_links_to_keep
            ),
        )

        # i.e. for Booster T1: [6, 10, 7, 11, 15, 19, 23, 27, 31, 35, 39, 36, 40, 8, 12, 16, 20, 24, 28, 32, 37, 41, 38, 42, 9, 13, 17, 21, 25, 29, 33, 14, 18, 22, 26, 30, 34]
        self.robot_full_motors_dof_idx = [self.robot.get_joint(name).dofs_idx_local[0] for name in self.robot_controllable_joint_names]

        # get id of base_link
        base_link = self.robot.get_link(self.robot_imu_config['parent'])

        # How to use IMU: https://genesis-world.readthedocs.io/en/latest/api_reference/sensor/imu.html OR https://genesis-world.readthedocs.io/en/v0.3.7/user_guide/getting_started/sensors.html
        self.imu = self.scene.add_sensor(
            gs.sensors.IMU(
                entity_idx=self.robot.idx,
                link_idx_local=base_link.idx_local,
                pos_offset=tuple(self.robot_imu_config['pos_offset']),
                # acc_axes_skew=(0.01, 0.01, 0.02),  # simulate sensor misalignment
                # gyro_axes_skew=(0.02, 0.03, 0.04), # simulate sensor misalignment
                # acc_noise=(0.01, 0.01, 0.01),      # add Gaussian noise to measurements
                # gyro_noise=(0.01, 0.01, 0.01),     # add Gaussian noise to measurements
                # acc_random_walk=(0.001, 0.001, 0.001),  # simulate gradual sensor drift over time
                # gyro_random_walk=(0.001, 0.001, 0.001), # simulate gradual sensor drift over time
                # delay=0.01,                   # introduce timing realism
                # jitter=0.01,                  # introduce timing realism
                # interpolate=True,             # smooths delayed measurements
                # draw_debug=True,

                # use in latest ver?
                # link=self.robot.get_link(self.robot_imu_parent),  # use in latest ver?
                # frame="local",                                    # use in latest ver?
                # Accelerometer parameters         # use in latest ver?
                # accel_noise_density=0.0,         # Noise density (m/s^2/sqrt(Hz))
                # accel_random_walk=0.0,           # Random drift  (m/s^3/sqrt(Hz))
                # accel_bias_correlation_time=0.0, # Bias correlation time (s)
                # Gyroscope parameters             # use in latest ver?
                # gyro_noise_density=0.0,          # Noise density (rad/s/sqrt(Hz))
                # gyro_random_walk=0.0,            # Random drift  (rad/s^2/sqrt(Hz))
                # gyro_bias_correlation_time=0.0,  # Bias correlation time (s)
            )
        )

        if self.verbose:
            for link in self.robot.links:
                print("Robot list of links: ", link.name)

    def generate_env(self):
        """Write code to automatically generate env based on env_config.yaml."""
        if self.sim_env_name == "default":
            plane = self.scene.add_entity(
                gs.morphs.Plane(),
            )

            # a random cube to interact with
            cube = self.scene.add_entity(
                gs.morphs.Box(
                    size=(0.1, 0.1, 0.1),
                    pos=(0.45, 0.0, 0.1),
                )
            )
            
            # Building complex geometries: Custom mesh from file
            # mesh = scene.add_entity(
            #     gs.morphs.Mesh(
            #         file="models/object.obj", # can load .obj files
            #         scale=1.0,
            #         pos=(0, 0, 0),
            #         orientation=(0, 0, 0)
            #     )
            # )

    def build_sim_env(self): 
        """ 
        main function to build simulation.
        """

        # Initialize genesis simulation
        gs.init(backend=gs.gpu)
            # seed                = None,
            # precision           = '32',
            # debug               = False,
            # eps                 = 1e-12,
            # logging_level       = None,
            # backend             = gs.gpu,
            # theme               = 'dark',
            # logger_verbose_time = False

        ### NOTE:
        # By default uses rasterizer renderer (faster than raytracing) if not defined
        # Generates simulation with visualization if show_viewer=True. Will take some time to generate
        self.scene = gs.Scene(
            viewer_options=gs.options.ViewerOptions(
                camera_pos=(3, -1, 1.5),
                camera_lookat=(0.0, 0.0, 0.5),
                camera_fov=30,
                max_FPS=60,
            ),
            sim_options=gs.options.SimOptions(
                dt=self.sim_dt,
            ),
            show_viewer=self.enable_gui,
        ) 
        
        self.generate_env()
        self.generate_robot()
        # Initialize Genesis scene camera object to acquire rgbd images (only call after build_sim_env())
        self.initialize_camera()

        self.scene.build() 
        for cam in self.cam_dict:
                rgb, depth, seg, normal = self.cam_dict[cam]['scene_cam_obj'].render(rgb=self.cam_dict[cam]['rgb'], depth=self.cam_dict[cam]['depth'])
            # TO-DO: render pointcloud
        # scene.reset()
        
        # Example config for Booster T1: [6, 10, 7, 11, 15, 19, 23, 27, 31, 35, 39, 36, 40, 8, 12, 16, 20, 24, 28, 32, 37, 41, 38, 42, 9, 13, 17, 21, 25, 29, 33, 14, 18, 22, 26, 30, 34]
        full_motors_dof_idx = [self.robot.get_joint(name).dofs_idx_local[0] for name in self.robot_controllable_joint_names]

        if self.verbose:
            j_kp = self.robot.get_dofs_kp()
            j_kv = self.robot.get_dofs_kv()
            j_force_range = self.robot.get_dofs_force_range()
            j_limits = self.robot.get_dofs_limit()
            print("Joint control info:")
            print("j_kp:", j_kp)
            print("j_kv:", j_kv)
            print("j_force_range:", j_force_range)
            print("j_limits:", j_limits)

        # Hard reset robot to original position
        for i in range(50):
            self.robot.set_dofs_position(self.robot_reference_configuration, full_motors_dof_idx)
            self.scene.step()

        return
