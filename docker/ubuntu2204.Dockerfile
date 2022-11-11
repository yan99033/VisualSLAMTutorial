from ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y \
  bash-completion \
  build-essential \
  cmake \
  git \
  vim \
  python3-pip \
  libopencv-dev

RUN pip3 install cmake-format clang-format PyYAML

# Create a non-root user
ARG USER=user
ARG USER_UID=1000
ARG USER_GID=1000

RUN groupadd --gid $USER_GID $USER \
    && useradd --home-dir /home/$USER --create-home --uid $USER_UID \
        --gid $USER_GID --shell /bin/sh --skel /dev/null $USER

USER $USER