# https://github.com/IntelRealSense/librealsense/tree/development/scripts/Docker

FROM librealsense/librealsense

RUN apt-get -y update

RUN apt-get -y install dirmngr

RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE

RUN add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u

#RUN apt-get install -y librealsense2-dkms

RUN apt-get install -y librealsense2-utils

