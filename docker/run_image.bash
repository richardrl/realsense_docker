# note: if you have an existing container, this will simply sync into that container and not create
# a new container

IMAGE=richardrl/realsense_docker_source:latest
XAUTH=/tmp/.docker.xauth
if [ ! -f $XAUTH ]
then
    xauth_list=$(xauth nlist :0 | sed -e 's/^..../ffff/')
    if [ ! -z "$xauth_list" ]
    then
        echo $xauth_list | xauth -f $XAUTH nmerge -
    else
        touch $XAUTH
    fi
    chmod a+r $XAUTH
fi

docker run -it \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --env="XAUTHORITY=$XAUTH" \
    --volume="$XAUTH:$XAUTH" \
    --volume="$PWD/../:/home/docker/realsense_docker" \
    --volume="/data/pulkitag/models/rli14/visual-pushing-grasping/:/home/docker/vpg" \
    --volume="/data/pulkitag/models/rli14/librealsense/:/home/docker/librealsense" \
    --privileged \
    --runtime=nvidia \
    --net=host \
    -e WANDB_API_KEY \
    -e UR5_IP \
    -e uid=$(id -u)\
    ${IMAGE} \
    bash