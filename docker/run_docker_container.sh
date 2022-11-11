#!/usr/bin/env bash

# Container name
CONT_NAME=VSLAM

# The root folder 
PROJECT_ROOT=${PWD%/*}

# X11 port forwarding
# http://wiki.ros.org/docker/Tutorials/GUI
XSOCK=/tmp/.X11-unix
XAUTH=/tmp/.docker.xauth
touch $XAUTH
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -

# Create and run the container
docker run \
-it \
--rm \
-e "DISPLAY=$DISPLAY" \
-e "QT_X11_NO_MITSHM=1" \
--name=$CONT_NAME \
--entrypoint /bin/bash \
--mount type=bind,source=$PROJECT_ROOT,target=/home/user/${PROJECT_ROOT##*/} \
-v /home/bryan/icl_nuim_lr_kt2:/home/user/${PROJECT_ROOT##*/}/datasets:ro \
-u $(id -u ${USER}):$(id -g ${USER}) \
--volume=$XSOCK:$XSOCK:rw \
--volume=$XAUTH:$XAUTH:rw \
--env="XAUTHORITY=${XAUTH}" \
--env="DISPLAY" \
-w /home/user/${PROJECT_ROOT##*/} \
ubuntu/2204:vslamtutorial