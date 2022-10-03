#!/usr/bin/env bash

docker image build \
    --build-arg USER_UID=$(id -u ${USER}) \
    --build-arg USER_GID=$(id -g ${USER}) \
    -t ubuntu/2204:vslamtutorial \
    -f ubuntu2204.Dockerfile \
    .