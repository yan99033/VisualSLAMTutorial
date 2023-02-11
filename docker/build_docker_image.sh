#!/usr/bin/env bash

docker image build \
    -t ubuntu/2204:vslamtutorial \
    -f ubuntu2204.Dockerfile \
    .