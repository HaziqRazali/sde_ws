#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/jazzy/setup.bash"
# source "/opt/ros/$ROS_DISTRO/setup.bash"
exec "$@"