# From this tutorial https://github.com/process1183/roomba-rpi/blob/master/docs/ros2_rpizw_build.md
# Create containter to cross-compile ROS2
# Copy /opt/ros2_humble/install folder to board after compilation

FROM ubuntu:focal-20230412

RUN chown ${USER}:${USER} /opt/

# Install tools
RUN apt update
RUN apt upgrade

ARG DEBIAN_FRONTEND=noninteractive

RUN apt install -y build-essential git gawk texinfo bison file wget rsync nano \
	gcc-arm-linux-gnueabihf g++-arm-linux-gnueabihf \
	wget tar python3-pip git cmake qemu-user-static python3-numpy

RUN pip3 install rosinstall_generator colcon-common-extensions vcstool lark-parser

# Setup environment
RUN mkdir -p /opt/rootfs
RUN mkdir -p /opt/ros2_humble/src

COPY ./res/toolchain.cmake /opt/
COPY ./res/build_ros.bash /opt/ros2_humble 
COPY ./res/rootfs.tar.gz /opt/rootfs

# Unpack rootfs
WORKDIR /opt/rootfs
RUN tar -xzf rootfs.tar.gz

# Download source code
WORKDIR /opt/ros2_humble
RUN wget https://raw.githubusercontent.com/ros2/ros2/humble/ros2.repos
RUN vcs import src < ros2.repos
