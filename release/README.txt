# cd releases # NOT AVAILABLE TODAY
./prepare_release.bash # to filter robot types (but to revise for future usage)

# build sde_ws in release mode first
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_PYTHON_BINDING=on

# release to install
./release/release_deb.bash install /opt/ros/humble

# Add user_instruction.md into release folder for licensee / deployment devices for their usage. 

### When asked for info, feel free to use this template and change version information and description version info / .deb filename: 
# Package name: i2r-sde
# Version: 1.0.0-1
# Section: install
# Priority: required
# Architecture: amd64
# Maintainer name: I2R RAS
# Maintainer email: your_email@a-star.edu.sg
# Description: Data engine and pipeline developed by A*STAR Institute for Infocomm Research (I2R) Robotics and Autonomous Systems (RAS) department for collecting humanoid robot teleoperation datasets for Embodied AI research. To install this package run: dpkg -i sde_1.0.0-1.deb. To uninstall this package run: dpkg -r i2r-sde.