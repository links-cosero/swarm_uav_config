FROM ubuntu:focal

RUN apt update && apt upgrade
ARG DEBIAN_FRONTEND=noninteractive

RUN apt install -y \
    symlinks \
    bison \
    cmake \
    curl \
    libasio-dev \
    libbullet-dev \
    libcunit1-dev \
    libcurl4-openssl-dev \
    libeigen3-dev \
    liblog4cxx-dev \
    libopencv-dev \
    libtinyxml2-dev \
    python3-dev \
    python3-netifaces \
    python3-numpy \
    python3-setuptools \
    python3-yaml \
    python3-pip \
    python3-catkin-pkg
    
RUN pip3 install empy lark

WORKDIR /
RUN tar -cf - /usr /lib | gzip > rootfs_armhf.tar.gz
RUN mkdir -p /opt/ros2_humble
#COPY ros2_humble_install_armhf.tar.gz /opt/ros2_humble/
WORKDIR /opt/ros2_humble
#RUN tar -xzf ros2_humble_install_armhf.tar.gz
