# How to Run I2R Data Engine for Embodied AI

## Install Instruction

Refer to **README.md** in *docker* folder to install the container.

- To collect data in the robot platform itself, copy the file *release/common/<date>/i2r-humble-ros2bag-server_<version_number>.deb* into robot computer. Run ```dpkg -i i2r-humble-ros2bag-server_<version_number>.deb```. After sourcing ROS2 Humble, run ```ros2 launch ros2bag_server ros2bag_server.launch.py robot_type:=booster_t1 config_name:=data_collection_config.yaml output_dir:=bag/ is_run_on_robot:=false is_mcap:=false is_override_qos:=false ```. Your saved bag files will default to *"/opt/ros/humble/share/ros2bag_server"* *bag/* directory. 
	- Do note to clear out space often to avoid clogging up storage on your devices and robots. 

## Run Instruction

### Using TMUX

1. Run **test_sim_control.bash** to control simulated robot in Genesis using interactive marker on Rviz.

2. Run **data_collection/vr_collect_sim_data.bash** to control simulated robot in Genesis using desired VR_DEVICE, defaults to *metaquest3*. 
- Make sure that your ros2bag_server is running on your local device. 
- VR headset ik control TF is visualized on Rviz. 

3. Run **data_collection/vr_collect_sim_data_viz_simTF.bash** to control simulated robot in Genesis using desired VR_DEVICE, defaults to *metaquest3*. 
- Make sure that your ros2bag_server is running on your local device. 
- Simulated robot TF is visualized on Rviz. 

4. Run **test_real_control.bash** to control real robot using interactive marker on Rviz. 
- Please check that the IP ADDRESS in this bash script is set correctly - your system network interface IP for connection to robot. 
- Make sure to set robot in Custom mode first before control. 

5. Run **data_collection/vr_collect_real_data.bash** to control real robot using desired VR_DEVICE, defaults to *metaquest3*. 
- Please check that the IP ADDRESS in this bash script is set correctly - your system network interface IP for connection to robot. 
- Make sure to set robot in Custom mode first before control. 
- Make sure that your *ros2bag_server* is running on the robot or on your local device. 

6. Run **data_validation/validate_ros2bag.bash** to check if collected bag is outputting actionable joint commands to robot. 
- Please check and modify BAG_FILE_PATH to your respective file path of the ros2bag you will like to check. 

### Individual Command Line Interface (CLI)

* Run instruction for a typical data collection setup for a simulated Booster T1.

1. **Simulation** (example with all the launch config defined, else will default): 
```bash
ros2 launch sde_simulation sim_main_genesis.launch.py robot_type:=booster_t1 sim_env_name:=default enable_gui:=true num_ROS2_threads:=8 enable_sim_tf:=true use_sim_time:=false
```
- Always set *robot_type* variable.
- Set *sim_env_name* variable to use your desired environment. 
- Set *enable_gui* to *true* to visualize robot on rviz. Defaults to *false*.
- Set *num_ROS2_threads* to define number of ROS2 threads used for multithreading. 
- Set *enable_sim_tf* variable to *true* if require the simulation to run robot_state_publisher to publish transform of robot. 
- Set *use_sim_time* variable to *true* if require robot_state_publisher to use past /joint_states data. For running in real-time, make sure use_sim_time is *false*.

2. **Hardware interface** for Booster T1 (if do not set *config_name* argument, it defaults to *booster_t1_hardware_interface.yaml*:
```bash
ros2 launch hardware_interfaces hardware_interface.launch.py client_ip:=192.168.10.13 config_name:=booster_t1_hardware_interface.yaml
```
- Always set *client_ip* variable. This IP is your network interface address. 
- Always set *config_name* variable.

3. **Data Recording** within robot computer:
```bash
ros2 launch ros2bag_server ros2bag_server.launch.py robot_type:=booster_t1 config_name:=sim_data_collection_config.yaml path_to_output_dir:=/root/ output_dir:=bag/ is_run_on_robot:=false is_mcap:=false is_override_qos:=false
```
- Always set *robot_type* variable.
- If do not set *config_name* variable, defaults to *data_collection_config.yaml*.
- If set *is_run_on_robot* variable to *true*, the config file will point to the local config directory of *ros2bag_server*, instead of pointing to *sde_robot_config*.
- *path_to_output_dir* defining path to *output_dir* to save ros2 bags in. Defaults to */home/$USER*. 
- *output_dir* variable defining folder to save ros2 bags in. Defaults to *bag/*.
- Set *is_mcap* to *true* if you prefer to save in .mcap format. Change settings in *mcap_config.yaml* file in ros2bag_server config directory.
- Set *is_override_qos* to *true* if you require to overwrite qos of collected being collected due to any misalignment i.e. on qos reliability. Change settings in *qos_override.yaml* file in ros2bag_server config directory.

4. [**CasADi-based inverse kinematics**](https://www.researchgate.net/publication/330553232_CASCLIK_CasADi-Based_Closed-Loop_Inverse_Kinematics)  to control humanoid robot end-effector or hands:
```bash
ros2 launch casadi_optimal_control_ik robot_ik_solver.launch.py robot_type:=booster_t1 sim_or_real:=sim config_file:=ik_solver_config.yaml enable_tf:=false enable_static_world_tf:=true robot_base_link_name:=Trunk use_sim_time:=false
```
- Always set *robot_type* variable.
- Set *sim_or_real* variable depending on whether the computed joint states should publish to sim or real robot, defaults to *sim*.
- Set *config_name* variable, defaults to *ik_solver_config.yaml*.
- Set *enable_tf* to *true* to visualize motion retargeted tf on rviz. Ensure that no other /robot_state_publisher runs at the same time, especially on the same device else there may be conflict. Defaults to *false*.
- Set *enable_static_world_tf* to *true* to visualize robot in world frame on rviz. Ensure there is no odom source to avoid clash. Defaults to *false*.
- Set *robot_base_link_name* if different from *base_link*. This is for the static world transform to *robot_base_link_name*.
- Set *use_sim_time* variable to *true* if require robot_state_publisher to use past /joint_states data. 


5. Run **VR teleoperation interface** to send poses to inverse kinematics module:
```bash
ros2 launch teleoperation_interface teleoperation_interface.launch.py vr_device:=metaquest3 robot_type:=booster_t1 sim_or_real:=sim collect_data:=true teleop_config_file:=teleoperation_interface_config.yaml task_config_file:=task_labelling_config.yaml enable_rviz_gui:=true use_sim_time:=false
```
- Always set *vr_device* variable. Choices: [disable, metaquest3, htc_vive_pro2]. When in *disable*, enables end-effector control via interactive markers on rviz.
- Always set *robot_type* variable.
- Set *sim_or_real* variable depending on whether the feedback joint states should be read from sim or real robot, defaults to *sim*.
- Set *collect_data* variable to *true* to initialize ros2bag_server client for data collection process. Make sure that ros2bag_server server is also running. 
- Set *teleop_config_file* variable, defaults to *teleoperation_interface_config.yaml*.
- Set *task_config_file* variable, defaults to *task_labelling_config.yaml*.
- Set *enable_rviz_gui* to *true* to visualize robot on rviz. Defaults to *false*.
- Set *use_sim_time* variable to *true* if require to get robot to use past /joint_states data and visualize past timestamp and tf data.
