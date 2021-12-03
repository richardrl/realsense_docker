realsense_docker.dockerfile

- Confirmed working with realsense-viewer GUI
- Check if the host system can even detect it: https://github.com/IntelRealSense/librealsense/issues/3519
  - lsusb | grep Intel
  - lsusb -D /dev/bus/usb/002/026 | grep bcdUSB
- You must run setup_udev_rules.sh on the host machine for this to work
- You actually need 60-librealsense2-udev-rules.rules in /lib/udev/rules.d for some reason..


realsense_docker_from_source.dockerfile
- You need to build from source to get the #include/realsense/rs.hpp working
- OR figure out how to include that directory