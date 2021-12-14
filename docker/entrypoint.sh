#!/bin/bash
set -e

# setup ros environment
#source "/root/catkin_ws/devel/setup.bash"

#pip3 install -e /home/improbable/isaac_loco

export PYTHONPATH=/root/realsense_docker/:${PYTHONPATH}

# the below line is CRITICAL for the headers in opencv
# as well as adding things in the cmakelists to add the opencv headers
export OpenCV_DIR=/root/build

#eval "/usr/sbin/sshd"

eval "bash"

exec "$@"
