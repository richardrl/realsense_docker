FROM nvidia/cuda:10.0-cudnn7-devel-ubuntu18.04

ENV NVIDIA_VISIBLE_DEVICES ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

ENV LDFLAGS=-L/usr/lib/x86_64-linux-gnu/

# disables need for -y
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get -y update && apt-get -y upgrade && apt-get -y dist-upgrade

# for add-apt-repository
RUN apt install -y software-properties-common

RUN apt update

# https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md
RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE ||  apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE

RUN add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u

# https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md
RUN apt-get install -y librealsense2-dkms
RUN apt-get install -y librealsense2-utils

RUN apt-get install -y udev

RUN apt-get install -y git

RUN git clone https://github.com/IntelRealSense/librealsense.git

WORKDIR librealsense

RUN ./scripts/setup_udev_rules.sh

#RUN ./scripts/patch-realsense-ubuntu-lts.sh