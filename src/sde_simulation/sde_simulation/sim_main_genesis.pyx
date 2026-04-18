# Copyright (c) 2026, Agency for Science, Technology and Research (A*STAR) All Rights Reserved.
# License TO-DO: Apache License, Version 2.0  
# Maintainer: Ng Yung Chuen, Email: ng_yung_chuen@a-star.edu.sg

import os
import subprocess
import signal
import time
from termcolor import colored

from genesis.utils.geom import trans_R_to_T, euler_to_R
import numpy as np
import threading

import rclpy

from sde_simulation.utils.ros2_handler import ROS2Handler
from sde_simulation.utils.genesis_sim_env_builder import SimEnvBuilder


def run_simulation(args): 
    """ 
    main function to build and run main simulation control loop
    """
    # Init ROS2 handler
    rclpy.init(args=args)
    ROS2_handler_node = ROS2Handler()
    ROS2_handler_spin = threading.Thread(target=ROS2_handler_node.spin_thread, daemon=True) # no arguments needed: args=(ROS2_handler_node,)
    ROS2_handler_spin.start()

    # Init genesis simulation
    sim_env_builder = SimEnvBuilder(ROS2_handler_node.robot_type, ROS2_handler_node.sim_env_name, ROS2_handler_node.enable_gui, ROS2_handler_node.config)
    sim_env_builder.build_sim_env()
    input_motors_dof_idx = None
    
    # Init camera rendering timer
    render_img_dt = 1.0 / ROS2_handler_node.render_img_freq
    render_img_timer = ROS2_handler_node.get_clock().now()

    # Wait for callback function data to be received
    print(colored("Waiting for initial joint control data to be received."))
    try: 
        while rclpy.ok():
            if ROS2_handler_node.joint_state_cmd != None:
                joint_names = ROS2_handler_node.joint_state_name
                input_motors_dof_idx = [sim_env_builder.robot.get_joint(name).dofs_idx_local[0] for name in joint_names]
                if ROS2_handler_node.config['verbose']:
                    print("Control input joint state names:", joint_names)
                    print("Control input joint state ids:", input_motors_dof_idx)
                break
    except KeyboardInterrupt:
        print(colored("SIGINT has been called. Manual rclpy shutdown not needed."))
        if ROS2_handler_node.robot_state_publisher_node_process:
            ROS2_handler_node.kill_subprocess()
        # ROS2_handler_spin.join()
    # finally: 
    #     try:
    #         rclpy.shutdown() # this is not needed. This is only for manually calling shutdown
    #     except:
    #         print(colored("SIGINT has been called. Manual rclpy shutdown not needed."))
    #     print(colored("Threads joined."))
    #     ROS2_handler_node.destroy_node()
    #     print(colored("Nodes destroyed."))

    # Main simulation control loop
    try: 
        while rclpy.ok(): 
            with ROS2_handler_node.subscriber['joint_states_callback']['lock']:
                action = ROS2_handler_node.joint_state_cmd
            sim_env_builder.robot.control_dofs_position(action, input_motors_dof_idx)
            # print("action:", action) # DEBUG
            sim_env_builder.scene.step()
            jpos = sim_env_builder.robot.get_qpos(sim_env_builder.robot_full_motors_dof_idx)
            jvel = sim_env_builder.robot.get_dofs_velocity(sim_env_builder.robot_full_motors_dof_idx) # refer to Genesis/tests/test_rigid_physics.py line 1166 - 1169
            jtorque = sim_env_builder.robot.get_dofs_force(sim_env_builder.robot_full_motors_dof_idx) # refer to Genesis/tests/test_rigid_physics.py line 1166 - 1169
            imu_data = sim_env_builder.imu.read()
            # imu_data = sim_env_builder.imu.get_data() # data.linear_acceleration / data.angular_velocity
            # print("IMU data is:", imu_data, "Type: ", type(imu_data[0][0]), "len: ", len(imu_data)) # "lin_acc": [x,y,z], "ang_vel": [...]
            ROS2_handler_node.update_publish_data(jpos=jpos, jvel=jvel, jtorque=jtorque, imu=imu_data)
            if (ROS2_handler_node.get_clock().now() - render_img_timer).nanoseconds * 1e-9 > render_img_dt:
                for cam in sim_env_builder.cam_dict:
                    rgb, depth, seg, normal = sim_env_builder.cam_dict[cam]['scene_cam_obj'].render(rgb=sim_env_builder.cam_dict[cam]['rgb'], depth=sim_env_builder.cam_dict[cam]['depth'])
                    ROS2_handler_node.update_publish_data(rgb=rgb, pub_cam_fn_name=sim_env_builder.cam_dict[cam]['ros2_interface_fn'])
                    ROS2_handler_node.update_publish_data(depth=depth, pub_cam_fn_name=sim_env_builder.cam_dict[cam]['ros2_interface_fn'])
                render_img_timer = ROS2_handler_node.get_clock().now()
    except KeyboardInterrupt:
        pass
    finally: 
        try:
            print(colored("Calling rclpy shutdown."))
            rclpy.shutdown() # this is not needed. This is only for manually calling shutdown
            print(colored("Complete rclpy shutdown."))
        except:
            print(colored("SIGINT has been called. Manual rclpy shutdown not needed."))
            if ROS2_handler_node.robot_state_publisher_node_process:
                ROS2_handler_node.kill_subprocess()
        ROS2_handler_spin.join()
        print(colored("Threads joined."))
        ROS2_handler_node.destroy_node()
        print(colored("Nodes destroyed."))

    # ROS2_handler_node.destroy_node()
    # rclpy.shutdown()
    # ROS2_handler_spin.join()

def main():
    # Handle all parameters and config loading via ROS2
    run_simulation(None) # argparse arguments not required

    try:
        pass
    finally:
        print("Performing final cleanup...")
        
        # Get current process information
        current_pid = os.getpid()
        print(f"Current main process PID: {current_pid}")
        
        try:
            # Find all related Python processes
            result = subprocess.run(['pgrep', '-f', 'sim_main_genesis.py'], 
                                  capture_output=True, text=True)
            if result.returncode == 0:
                pids = result.stdout.strip().split('\n')
                print(f"Found related processes: {pids}")
                
                for pid in pids:
                    if pid and pid != str(current_pid):
                        try:
                            print(f"Terminating child process: {pid}")
                            os.kill(int(pid), signal.SIGTERM)
                        except ProcessLookupError:
                            print(f"Process {pid} does not exist")
                        except Exception as e:
                            print(f"Failed to terminate process {pid}: {e}")
                
                # Wait for processes to exit
                time.sleep(2)
                
                # Check if there are any remaining processes, force kill them
                result2 = subprocess.run(['pgrep', '-f', 'sim_main_genesis.py'], 
                                       capture_output=True, text=True)
                if result2.returncode == 0:
                    remaining_pids = result2.stdout.strip().split('\n')
                    for pid in remaining_pids:
                        if pid and pid != str(current_pid):
                            try:
                                print(f"Force killing process: {pid}")
                                os.kill(int(pid), signal.SIGKILL)
                            except Exception as e:
                                print(f"Failed to force kill process {pid}: {e}")
                                
        except Exception as e:
            print(f"Error during process cleanup: {e}")

        print("Program exit completed")
        
        # Force exit
        os._exit(0)

if __name__ == "__main__":
    main()

