#!/usr/bin/env bash

# Container name
CONT_NAME=VSLAM

# The root folder 
PROJECT_ROOT=${PWD%/*}

# Create and run the container
docker run \
-it \
--rm \
-e "DISPLAY=$DISPLAY" \
-e "QT_X11_NO_MITSHM=1" \
--name=$CONT_NAME \
--entrypoint /bin/bash \
--mount type=bind,source=$PROJECT_ROOT,target=/home/user/${PROJECT_ROOT##*/} \
-u $(id -u ${USER}):$(id -g ${USER}) \
-w /home/user/VisualSLAMTutorial \
ubuntu/2204:vslamtutorial