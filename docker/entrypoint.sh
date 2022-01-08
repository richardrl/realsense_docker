#!/bin/bash
set -e

# setup ros environment
#source "/root/catkin_ws/devel/setup.bash"

# gosu magic start
# If "-e uid={custom/local user id}" flag is not set for "docker run" command, use 9999 as default
CURRENT_UID=${uid:-9999}

# Notify user about the UID selected
echo "Current UID : $CURRENT_UID"
# Create user called "docker" with selected UID
useradd --shell /bin/bash -u $CURRENT_UID -o -c "" -m docker
# Set "HOME" ENV variable for user's home directory
export HOME=/home/docker

export PYTHONPATH=/home/docker/realsense_docker/:${PYTHONPATH}

# the below line is CRITICAL for the headers in opencv
# as well as adding things in the cmakelists to add the opencv headers
export OpenCV_DIR=/home/docker/opencv/build

# Execute process
exec gosu docker "$@"
# gosu magic

#eval "/usr/sbin/sshd"

eval "bash"

exec "$@"
