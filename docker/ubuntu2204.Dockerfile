from osrf/ros:humble-desktop-full

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y \
  bash-completion \
  build-essential \
  cmake \
  git \
  vim \
  python3-pip \
  libopencv-dev \
  ninja-build 

RUN pip3 install cmake-format clang-format PyYAML
