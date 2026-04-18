# Docker support for **sde_ws**

**Note that these Dockerfiles only support GPU devices.**
**Make sure you have installed the following before setting up docker container for *sde_deploy*:**
- [CUDA driver](https://docs.nvidia.com/cuda/cuda-installation-guide-linux/)
- [CUDA toolkit](https://developer.nvidia.com/cuda-downloads?target_os=Linux&target_arch=x86_64&Distribution=Ubuntu&target_version=24.04&target_type=deb_local)
- [Docker container](https://docs.docker.com/engine/install/ubuntu/)
- [NVIDIA-Container-Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)

## Prepare your device for NVIDIA-accelerated rendering

Note that these Dockerfiles only support GPU devices.*

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
sudo prime-select nvidia # if your workstation does not have an integrated GPU, ignore these steps
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

## Build and run docker image

The following basic utilities are installed in the container:
- sublime-text
- nautilus
- tmux
- htop

Note: 

* Ensure all config_${ros_distro}/ros_entrypoint_${ros_distro}.sh, bashrc, and install_dependencies.sh are executable files. 
* Ensure to have setup xhost beforehand for GUI on your device:
```bash
echo "xhost +si:localuser:$USER" >> ~/.bashrc
echo "xhost +local:docker" >> ~/.bashrc
echo "xhost +local:root" >> ~/.bashrc
```
* Ensure that your device has been setup to use NVIDIA driver:
```bash
sudo prime-select query # to check 
sudo prime-select nvidia
sudo reboot
```

### Ubuntu 22.04 ROS2 Humble

This image installs Python3.10. Firstly, navigate to the **sde_deploy** or **/<workspace_ws>/** directory.

Build image:
```bash
    DOCKER_BUILDKIT=1 docker build -t u22_gpu_humble_sde:latest \
        --build-arg UID="$(id -u)" \
        --build-arg GID="$(id -g)" \
        -f docker/Dockerfile-deploy-sde-gpu_humble . 
```

First time running container: 
```bash
    docker run -it --privileged --net=host --ipc=host \
         --name=u22_humble_sde \
         --env="DISPLAY=$DISPLAY" \
         --env="QT_X11_NO_MITSHM=1" \
         --runtime=nvidia \
         --gpus all \
         u22_gpu_humble_sde:latest \
         terminator
```

Run container: 
```bash
    sudo docker start u22_humble_sde
```
OR:
```bash
    ./docker/run.sh u22_humble_sde
```

### Ubuntu 24.04 ROS2 Jazzy (NOT YET SUPPORTED)

This custom image installs Python3.12. Firstly, navigate to the **sde_deploy** or **/<workspace_ws>/** directory.

Build image:
```bash
    docker build -t u24_gpu_jazzy_sde:latest \
        --build-arg UID="$(id -u)" \
        --build-arg GID="$(id -g)" \
        -f docker/Dockerfile-deploy-sde-gpu_jazzy . 
```

First time running container: 
```bash
    docker run -it --privileged --net=host --ipc=host \
         --name=u24_jazzy_sde \
         --env="DISPLAY=$DISPLAY" \
         --env="QT_X11_NO_MITSHM=1" \
         --runtime=nvidia \
         --gpus all \
         u24_gpu_jazzy_sde:latest \
         terminator
```

Run container: 
```bash
    sudo docker start u24_jazzy_sde
```
OR:
```bash
    ./docker/run.sh u24_jazzy_sde
```

## Other Notes or Bugs

- When running step: ```    docker run -it --privileged --net=host --ipc=host \
         --name=u22_humble_sde \
         --env="DISPLAY=$DISPLAY" \
         --env="QT_X11_NO_MITSHM=1" \
         --runtime=nvidia \
         --gpus all \
         u22_gpu_humble_sde:latest \
         terminator```, if you encounter error:
```text
docker: Error response from daemon: failed to create task for container: failed to create shim task: OCI runtime create failed: runc create failed: unable to start container process: error during container init: failed to fulfil mount request: open /usr/lib/x86_64-linux-gnu/libnvidia-egl-gbm.so.1.1.2: no such file or directory
failed to start containers: u22_humble_sde
```
To solve it, do:
```bash
sudo ln -s /usr/lib/x86_64-linux-gnu/libnvidia-egl-gbm.so.1.1.3 /usr/lib/x86_64-linux-gnu/libnvidia-egl-gbm.so.1.1.2
sudo systemctl restart docker
```
This may have happened due to mismatch in nvidia-driver i.e. version 590 with libnvidia-container-toolkit which expects older version *libnvidia-egl-gbm.so.1.1.2*. 

- If you get black screen rendering in Genesis simulation, check your *.bashrc* file for the following: 
```bash
# Ensure the following two lines are commented (these are used on devices with integrated GPUs to ensure NVIDIA GPUs are used):
export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia
# Then run in your terminal: 
unset __NV_PRIME_RENDER_OFFLOAD
unset __GLX_VENDOR_LIBRARY_NAME

# Ensure the following lines are enabled and exported (check using "env | grep NVIDIA"): 
export NVIDIA_VISIBLE_DEVICES=all
export NVIDIA_DRIVER_CAPABILITIES=all
```


- Make sure docker -X permissions are given before launching:
```bash
xhost +si:localuser:$USER
xhost +local:docker
docker start u24_humble_sde
```

- Enter the container to install the remaining pip dependencies within the virtualenv.
```
    . /root/install.sh
```

- Remove container:
```
	docker rm -f u24_humble_sde
```
If you encounter issue where docker containers take up a lot of space, please free up space with docker prune but proceed with caution! 
