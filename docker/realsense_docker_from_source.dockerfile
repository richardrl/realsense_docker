# this is the best script because building from source
# lets you build realsense_multicam.cpp
#FROM python:3.6-slim

FROM nvidia/cuda:10.0-cudnn7-devel-ubuntu18.04

ENV NVIDIA_VISIBLE_DEVICES ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

ENV LDFLAGS=-L/usr/lib/x86_64-linux-gnu/

RUN apt-get update && apt-get install -y --no-install-recommends \
	python3-pip \
	python3-dev

RUN apt-get -y install git

# Install LibRealSense from source
RUN git clone https://github.com/IntelRealSense/librealsense.git

## Install the core packages required to build librealsense libs
RUN apt-get install -y libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev

### Distribution-specific packages for Ubuntu 18
RUN apt-get install -y libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev
### Run Intel Realsense permissions script
RUN pwd
WORKDIR ./librealsense
RUN pwd
RUN ls ./config
# Make sure that your RealSense cameras are disconnected at this point
# RUN ./scripts/setup_udev_rules.sh
#RUN cp ./config/99-realsense-libusb.rules /etc/udev/rules.d/
#RUN apt-get install -y udev
#RUN udevadm control --reload-rules
#RUN udevadm trigger
# Now starting the build
RUN mkdir build && cd build
## CMake with Python bindings
## see link: https://github.com/IntelRealSense/librealsense/tree/master/wrappers/python#building-from-source

RUN apt-get -y install cmake build-essential

WORKDIR build

RUN cmake ../ -DBUILD_PYTHON_BINDINGS:bool=true
### Recompile and install librealsense binaries
RUN make uninstall && make clean && make -j6 && make install
RUN export PYTHONPATH=$PYTHONPATH:/usr/local/lib/python3.6/pyrealsense2

WORKDIR /librealsense

#RUN ./scripts/setup_udev_rules.sh
RUN mkdir -p /etc/udev/rules.d/
RUN cp ./config/99-realsense-libusb.rules /etc/udev/rules.d/

ENV DEBIAN_FRONTEND=noninteractive

RUN pip3 install --upgrade setuptools

RUN pip3 install --upgrade pip

# andy zeng vpg stuff
RUN pip3 install --upgrade numpy scipy matplotlib

RUN pip3 install opencv-contrib-python

RUN apt-get install -y python3-tk

RUN pip3 install torch

# rtde stuff

#RUN add-apt-repository ppa:sdurobotics/ur-rtde
#
#RUN apt-get update
#
#RUN apt install librtde librtde-dev

RUN pip3 install --user ur_rtde


#RUN apt install software-properties-common
#
#RUN add-apt-repository ppa:deadsnakes/ppa
#
#RUN apt install python3.9

RUN apt-get install -y gdb


# install opencv and contrib packages
# https://docs.opencv.org/4.x/d7/d9f/tutorial_linux_install.html
WORKDIR /home/docker/opencv
RUN apt update && apt install -y cmake g++ wget unzip
RUN wget -O opencv.zip https://github.com/opencv/opencv/archive/master.zip
RUN wget -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/master.zip
RUN unzip opencv.zip
RUN unzip opencv_contrib.zip
RUN mkdir -p build

WORKDIR build

RUN cmake -DOPENCV_EXTRA_MODULES_PATH=../opencv_contrib-master/modules ../opencv-master -DCUDA_ARCH_BIN=10.0
RUN cmake --build .

RUN pip3 install open3d


# use gosu to switch from root to host user ID so we don't run into permissions issues
RUN set -eux; \
	apt-get update; \
	apt-get install -y gosu; \
	rm -rf /var/lib/apt/lists/*; \
# verify that the binary works
	gosu nobody true
# end gosu magic


# this is needed for the headers...
#RUN apt-get install -y libopencv-core-dev
COPY ./entrypoint.sh /

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]