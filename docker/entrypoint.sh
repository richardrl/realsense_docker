#!/bin/bash
set -e

# setup ros environment
#source "/root/catkin_ws/devel/setup.bash"

#pip3 install -e /home/improbable/isaac_loco

export PYTHONPATH=/root/realsense_docker/:${PYTHONPATH}

export OpenCV_DIR=/root/build

#eval "/usr/sbin/sshd"

eval "bash"

exec "$@"
