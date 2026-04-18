# Instructions to use sde_ws

This is the new repository that will contain all functions and features needed for **Data Engine** project, some locally within *sde_ws*, some as a Git submodule. 

Packages currently integrated in this version 2026 Feb 6:
- sde_simulation
- ros2bag_server (NOTE THAT THIS PACKAGE WILL NOT BE BINARISED ON RELEASE)
- sde_robot_config
- hardware_interfaces (for Booster T1) (booster_interface is from open_source, aatb-ch)
- t1_description
- televuer (submodule)
- booster_robotics_sdk (submodule)
- motion_retargeting - casadi_optimal_control_ik
- motion_retargeting - teleoperation_interface
- mhr_pose_estimation
- ros2bag_to_lerobot

Improvements to be made:
- To populate individual repos with README and their launch instructions. 
- Make hardware_interfaces launch file to be generic when incorporating other robot embodiments, taking in *robot_type* argument.
- For real robot, add robot_state_publisher for /real/joint_states in hardware_interfaces.py as robot_description from robot is not good
- For booster_t1_hardware_interface_node.py (src/hardware_interfaces), add param to select position mode or force mode and design close-loop with feedback (motion_param.force = 100, motion.param.position = 100, motion_param.speed = 100)
- For t1_teleop_wrapper.py (src/teleoperation_interface), generalise it for use with other robot types based on loaded parameters, and for using with the TeleVuer headset. 

-----

## Installation

### Prepare your device for NVIDIA-accelerated rendering

In order to configure device with docker that runs simulation rendering using GPU i.e. ~RTX5090 and above, setup as follows in host PC:
- **/usr/share/X11/xorg.conf.d/10-amdgpu.conf**
```text
Section "OutputClass"
    Identifier "AMDgpu"
    MatchDriver "amdgpu"
    Driver "modesetting"
    Option "HotplugDriver" "amdgpu"
EndSection
```
- **/usr/share/X11/xorg.conf.d/10-nvidia.conf**
```text
Section "OutputClass"
    Identifier "nvidia"
    MatchDriver "nvidia-drm"
    Driver "nvidia"
    Option "PrimaryGPU" "Yes"
    ModulePath "/usr/lib/x86_64-linux-gnu/nvidia/xorg"
EndSection
```

Then run on terminal:
```bash
echo "xhost +si:localuser:$USER" >> ~/.bashrc
echo "xhost +local:docker" >> ~/.bashrc
echo "xhost +local:root" >> ~/.bashrc
sudo prime-select query # to check 
sudo prime-select nvidia
sudo reboot
```

Once rebooted, run on terminal: 
```bash
glxinfo | grep vendor
```
And you will see output:
```text
server glx vendor string: SGI
client glx vendor string: NVIDIA Corporation
OpenGL vendor string: NVIDIA Corporation
```

### Docker Installation

Refer to README.md [here](docker/). Note that these instructions are catered for installing docker for deployment or licensing of *sde_ws*. If developing, please instead use dockerfile i.e. [Dockerfile-dev-sde-gpu_humble](docker/Dockerfile-dev-sde-gpu_humble).

### Fresh Install

* **Note if you have used Docker installation, please jump to step 6**
* The dependency versions as listed are for **ROS2 Humble**. If you will like to use ROS2 Jazzy **(NOT YET SUPPORTED)**, suggest to follow Docker installation instructions for **ROS2 Jazzy** or its corresponding *requirements.txt* or *[Dockerfile](docker/)*. 

1. Git Clone
Clone by HTTPS:
```bash
git clone --recursive https://gitlab.i2r.a-star.edu.sg/sean/sde/sde_ws.git
```
Clone by SSH:
```bash
git clone --recursive ssh://git@gitlab.i2r.a-star.edu.sg:10022/sean/sde/sde_ws.git
```

2. If using original booster_robotics_sdk
```text
If using latest available booster_robotics_sdk instead of the i2r_fork, please note the following:

To patch:
# booster_robotics_sdk, make sure to patch this directory with COLCON_IGNORE
# 						and comment out line 27, # add_executable(battery_state_subscriber example/low_level/battery_state_subscriber.cpp)

May not work reliably as there are missing messages not provided by Booster Robotics. 
```

3. For compiling Python binaries in workspace_ws:
 ```bash
pip install cython==3.2.4
pip install setuptools==65.5.0 # in ROS2 Humble (original container 82.0.0)

# For making releases in GitLab
curl -sSL "https://raw.githubusercontent.com/upciti/wakemeops/main/assets/install_repository" | sudo bash
sudo apt install -y glab
```

4. Fresh self-install instructions on a GPU ROS2 Humble container / OS:
```bash
# For sde_simulation & teleoperation_interface
sudo apt-get install mesa-utils freeglut3 freeglut3-dev pybind11-stubgen
pip install termcolor PyOpenGL PyOpenGL_accelerate roma vuer
pip install z3-solver
pip install torch torchvision --index-url https://download.pytorch.org/whl/cu128
pip install genesis-world
pip install numpy==1.26.4
pip install transforms3d==0.4.1
sudo apt install ros-humble-tf-transformations

# For casadi_optimal_control_ik
curl http://robotpkg.openrobots.org/packages/debian/robotpkg.asc \
    | sudo tee /etc/apt/keyrings/robotpkg.asc
echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/robotpkg.asc] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -cs) robotpkg" \
    | sudo tee /etc/apt/sources.list.d/robotpkg.list
sudo apt update
sudo apt install robotpkg-pinocchio robotpkg-py310-pinocchio
echo "export PATH=/opt/openrobots/bin:$PATH" >> ~/.bashrc
echo "PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:$PKG_CONFIG_PATH" >> ~/.bashrc
echo "LD_LIBRARY_PATH=/opt/openrobots/lib:$LD_LIBRARY_PATH" >> ~/.bashrc
### ! Use the python version that has been installed in your repository ! ###
echo "PYTHONPATH=/opt/openrobots/lib/python3.10/site-packages:$PYTHONPATH" >> ~/.bashrc
echo "CMAKE_PREFIX_PATH=/opt/openrobots:$CMAKE_PREFIX_PATH" >> ~/.bashrc
# For televuer after colcon build Release
echo "XR_TELEOP_CERT=/root/sde_ws/install/televuer/share/televuer/cert.pem" >> ~/.bashrc
echo "XR_TELEOP_KEY=/root/sde_ws/install/televuer/share/televuer/key.pem" >> ~/.bashrc
```

5. Setup ONNX:
```bash
pip install onnxruntime==1.19.2
COPY docker/config_dev_${ros_distro}/onnxruntime-linux-x64-1.15.1 /root/onnxruntime-linux-x64-1.15.1

echo "export ONNXRUNTIME_DIR=$HOME/onnxruntime-linux-x64-1.15.1 " >> ~/.bashrc
# set env for onnxruntime
echo "export ONNXRUNTIME_DIR=${ONNXRUNTIME_DIR}" >> ~/.bashrc
echo "export LD_LIBRARY_PATH=$ONNXRUNTIME_DIR/lib:$LD_LIBRARY_PATH" >> ~/.bashrc

echo "export MMDEPLOY_DIR=$HOME/mmdeploy" >> ~/.bashrc
echo "export PYTHONPATH=$HOME/mmdeploy/build/lib:$PYTHONPATH" >> ~/.bashrc

cd /root/ && git clone --recursive https://github.com/open-mmlab/mmdeploy.git
source /root/.bashrc && mkdir /root/mmdeploy/build && cd /root/mmdeploy/build && cmake .. \
    -DMMDEPLOY_BUILD_SDK=ON \
    -DMMDEPLOY_BUILD_SDK_PYTHON_API=ON \
    -DMMDEPLOY_BUILD_EXAMPLES=ON \
    -DMMDEPLOY_TARGET_DEVICES=cpu \
    -DMMDEPLOY_TARGET_BACKENDS=ort \
    -DONNXRUNTIME_DIR=${ONNXRUNTIME_DIR} && \
    make -j12 && make install
```

6. (Perform this step if you intend to control real Booster T1 robot from your device, **ELSE** add a *CATKIN_IGNORE* in this repository) Install [booster_robotics_sdk](src/hardware_interface/booster/booster_robotics_sdk/) ThirdParty libraries and build code:
```bash
cd /root/sde_ws/src/hardware_interface/booster/booster_robotics_sdk
./install.sh

# and follow instructions in *booster_robotics_sdk/README.md* similar to below:
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=/opt/ros/humble -DBUILD_PYTHON_BINDING=on ..
make
sudo make install
```

7. Ensure **ros2bag_to_lerobot** scripts are executable:

```bash
cd /root/sde_ws/src/ros2bag_to_lerobot
chmod +x rosbag_to_lerobot.py validate_dataset.py verify_rododm.py

# Do the following if you encounter issue with "python3 validate_dataset.py --frame 100", else ignore below. You may encounter below error:
"""[av1 @ 0x56edc8a14040] Your platform doesn't suppport hardware accelerated AV1 decoding.
[av1 @ 0x56edc8a14040] Failed to get pixel format.
[av1 @ 0x56edc8a14040] Missing Sequence Header."""
# Do the following to resolve the issue: 
pip uninstall -y opencv-python opencv-python-headless
export CMAKE_ARGS="-D WITH_FFMPEG=ON -D CMAKE_BUILD_TYPE=RELEASE"
pip install --no-binary opencv-python opencv-python-headless --force-reinstall opencv-python opencv-python-headless
```
-----

## Build Instruction

To build the repository, navigate to *sde_ws/* and run:
```bash
colcon build --cmake-args -DBUILD_PYTHON_BINDING=on
```

Alternatively, build instructions to make binaries in install directory and more robust:
```bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_PYTHON_BINDING=on
```

### Miscellaneous Bugs or Warning
- If you get this warning, it is due to some pythran module used by mmdeploy_runtime, do not fret, it is harmless:
```text
/usr/lib/python3/dist-packages/pythran/tables.py:4520: FutureWarning: In the future `np.bool` will be defined as the corresponding NumPy scalar.
  if not hasattr(numpy, method):
/usr/lib/python3/dist-packages/pythran/tables.py:4553: FutureWarning: In the future `np.bytes` will be defined as the corresponding NumPy scalar.
  obj = getattr(themodule, elem)
```
To solve this, add the following line '**export PYTHONWARNINGS="ignore::FutureWarning"**' to your *~/.bashrc*.

- If you get an error from casadi, it is likely due to a version mismatch between *robotpkg-casadi 3.6.7* and *robotpkg-py310-casadi 3.7.2*:
```text
from . import _casadi
[robot_ik_solver-1] ImportError: /opt/openrobots/lib/python3.10/site-packages/casadi/_casadi.so: undefined symbol: _ZN6casadi13GlobalOptions21copy_elision_min_sizeE
```
To solve this, match the version by downgrading *robotpkg-py310-casadi 3.7.2* to *3.6.7*. 
```bash
sudo apt install robotpkg-qpoases+doc=3.2.1*
sudo apt install robotpkg-py310-casadi=3.6.7*
```

---

## Run Instruction

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

6. Run **mhr_pose_estimation** for human target pose estimation: 
```bash
ros2 run mhr_pose_estimation pose_estimator
```
- Flags

-----

## Config Quickstart

Modify the config files to gain control over developer settings, or when tuning for a new robot embodiment.

### Simulation
1. sim_config.yaml
```yaml
To be populated. 
```

### 1\. Hardware Interface
1. hardware_interface.yaml
```yaml
To be populated. 
```

