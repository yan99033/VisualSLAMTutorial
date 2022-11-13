from ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y \
  bash-completion \
  build-essential \
  cmake \
  git \
  vim \
  python3-pip \
  libopencv-dev \
  ninja-build \
  sudo

RUN pip3 install cmake-format clang-format PyYAML

# Dependencies for Pangolin viewer
RUN apt-get update && apt-get install -y \
  libgl1-mesa-dev \
  libwayland-dev \
  libxkbcommon-dev \
  wayland-protocols \
  libegl1-mesa-dev \
  libc++-dev \
  libglew-dev \
  libeigen3-dev \
  g++ 

# Pangolin viewer
WORKDIR /
RUN git clone --recursive https://github.com/stevenlovegrove/Pangolin.git
WORKDIR /Pangolin
RUN git switch --detach v0.8
RUN cmake -B build
RUN cmake --build build
WORKDIR /Pangolin/build
RUN make install
WORKDIR /
RUN rm -rf Pangolin

# Sophus
WORKDIR /
RUN git clone https://github.com/strasdat/Sophus.git
WORKDIR /Sophus
RUN git switch --detach v22.10
RUN mkdir build 
WORKDIR /Sophus/build
RUN cmake -DSOPHUS_USE_BASIC_LOGGING=ON .. && make && make install
WORKDIR /
RUN rm -rf Sophus

ENV LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib

# Create a non-root user
ARG USER=user
ARG USER_UID=1000
ARG USER_GID=1000

RUN groupadd --gid $USER_GID $USER \
    && useradd --home-dir /home/$USER --create-home --uid $USER_UID \
        --gid $USER_GID --shell /bin/sh --skel /dev/null $USER

USER $USER