# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /root/realsense_docker/realsense

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /root/realsense_docker/realsense

# Include any dependencies generated for this target.
include CMakeFiles/realsense_multicam_headless.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/realsense_multicam_headless.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/realsense_multicam_headless.dir/flags.make

CMakeFiles/realsense_multicam_headless.dir/realsense_multicam_headless.cpp.o: CMakeFiles/realsense_multicam_headless.dir/flags.make
CMakeFiles/realsense_multicam_headless.dir/realsense_multicam_headless.cpp.o: realsense_multicam_headless.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/root/realsense_docker/realsense/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/realsense_multicam_headless.dir/realsense_multicam_headless.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/realsense_multicam_headless.dir/realsense_multicam_headless.cpp.o -c /root/realsense_docker/realsense/realsense_multicam_headless.cpp

CMakeFiles/realsense_multicam_headless.dir/realsense_multicam_headless.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/realsense_multicam_headless.dir/realsense_multicam_headless.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /root/realsense_docker/realsense/realsense_multicam_headless.cpp > CMakeFiles/realsense_multicam_headless.dir/realsense_multicam_headless.cpp.i

CMakeFiles/realsense_multicam_headless.dir/realsense_multicam_headless.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/realsense_multicam_headless.dir/realsense_multicam_headless.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /root/realsense_docker/realsense/realsense_multicam_headless.cpp -o CMakeFiles/realsense_multicam_headless.dir/realsense_multicam_headless.cpp.s

CMakeFiles/realsense_multicam_headless.dir/realsense_multicam_headless.cpp.o.requires:

.PHONY : CMakeFiles/realsense_multicam_headless.dir/realsense_multicam_headless.cpp.o.requires

CMakeFiles/realsense_multicam_headless.dir/realsense_multicam_headless.cpp.o.provides: CMakeFiles/realsense_multicam_headless.dir/realsense_multicam_headless.cpp.o.requires
	$(MAKE) -f CMakeFiles/realsense_multicam_headless.dir/build.make CMakeFiles/realsense_multicam_headless.dir/realsense_multicam_headless.cpp.o.provides.build
.PHONY : CMakeFiles/realsense_multicam_headless.dir/realsense_multicam_headless.cpp.o.provides

CMakeFiles/realsense_multicam_headless.dir/realsense_multicam_headless.cpp.o.provides.build: CMakeFiles/realsense_multicam_headless.dir/realsense_multicam_headless.cpp.o


# Object files for target realsense_multicam_headless
realsense_multicam_headless_OBJECTS = \
"CMakeFiles/realsense_multicam_headless.dir/realsense_multicam_headless.cpp.o"

# External object files for target realsense_multicam_headless
realsense_multicam_headless_EXTERNAL_OBJECTS =

realsense_multicam_headless: CMakeFiles/realsense_multicam_headless.dir/realsense_multicam_headless.cpp.o
realsense_multicam_headless: CMakeFiles/realsense_multicam_headless.dir/build.make
realsense_multicam_headless: /usr/lib/x86_64-linux-gnu/libGL.so
realsense_multicam_headless: /usr/lib/x86_64-linux-gnu/libGLU.so
realsense_multicam_headless: /usr/lib/x86_64-linux-gnu/libglfw.so
realsense_multicam_headless: /root/build/lib/libopencv_gapi.so.4.5.4
realsense_multicam_headless: /root/build/lib/libopencv_stitching.so.4.5.4
realsense_multicam_headless: /root/build/lib/libopencv_aruco.so.4.5.4
realsense_multicam_headless: /root/build/lib/libopencv_barcode.so.4.5.4
realsense_multicam_headless: /root/build/lib/libopencv_bgsegm.so.4.5.4
realsense_multicam_headless: /root/build/lib/libopencv_bioinspired.so.4.5.4
realsense_multicam_headless: /root/build/lib/libopencv_ccalib.so.4.5.4
realsense_multicam_headless: /root/build/lib/libopencv_dnn_objdetect.so.4.5.4
realsense_multicam_headless: /root/build/lib/libopencv_dnn_superres.so.4.5.4
realsense_multicam_headless: /root/build/lib/libopencv_dpm.so.4.5.4
realsense_multicam_headless: /root/build/lib/libopencv_face.so.4.5.4
realsense_multicam_headless: /root/build/lib/libopencv_freetype.so.4.5.4
realsense_multicam_headless: /root/build/lib/libopencv_fuzzy.so.4.5.4
realsense_multicam_headless: /root/build/lib/libopencv_hfs.so.4.5.4
realsense_multicam_headless: /root/build/lib/libopencv_img_hash.so.4.5.4
realsense_multicam_headless: /root/build/lib/libopencv_intensity_transform.so.4.5.4
realsense_multicam_headless: /root/build/lib/libopencv_line_descriptor.so.4.5.4
realsense_multicam_headless: /root/build/lib/libopencv_mcc.so.4.5.4
realsense_multicam_headless: /root/build/lib/libopencv_quality.so.4.5.4
realsense_multicam_headless: /root/build/lib/libopencv_rapid.so.4.5.4
realsense_multicam_headless: /root/build/lib/libopencv_reg.so.4.5.4
realsense_multicam_headless: /root/build/lib/libopencv_rgbd.so.4.5.4
realsense_multicam_headless: /root/build/lib/libopencv_saliency.so.4.5.4
realsense_multicam_headless: /root/build/lib/libopencv_stereo.so.4.5.4
realsense_multicam_headless: /root/build/lib/libopencv_structured_light.so.4.5.4
realsense_multicam_headless: /root/build/lib/libopencv_superres.so.4.5.4
realsense_multicam_headless: /root/build/lib/libopencv_surface_matching.so.4.5.4
realsense_multicam_headless: /root/build/lib/libopencv_tracking.so.4.5.4
realsense_multicam_headless: /root/build/lib/libopencv_videostab.so.4.5.4
realsense_multicam_headless: /root/build/lib/libopencv_wechat_qrcode.so.4.5.4
realsense_multicam_headless: /root/build/lib/libopencv_xfeatures2d.so.4.5.4
realsense_multicam_headless: /root/build/lib/libopencv_xobjdetect.so.4.5.4
realsense_multicam_headless: /root/build/lib/libopencv_xphoto.so.4.5.4
realsense_multicam_headless: /root/build/lib/libopencv_shape.so.4.5.4
realsense_multicam_headless: /root/build/lib/libopencv_highgui.so.4.5.4
realsense_multicam_headless: /root/build/lib/libopencv_datasets.so.4.5.4
realsense_multicam_headless: /root/build/lib/libopencv_plot.so.4.5.4
realsense_multicam_headless: /root/build/lib/libopencv_text.so.4.5.4
realsense_multicam_headless: /root/build/lib/libopencv_ml.so.4.5.4
realsense_multicam_headless: /root/build/lib/libopencv_phase_unwrapping.so.4.5.4
realsense_multicam_headless: /root/build/lib/libopencv_optflow.so.4.5.4
realsense_multicam_headless: /root/build/lib/libopencv_ximgproc.so.4.5.4
realsense_multicam_headless: /root/build/lib/libopencv_video.so.4.5.4
realsense_multicam_headless: /root/build/lib/libopencv_videoio.so.4.5.4
realsense_multicam_headless: /root/build/lib/libopencv_imgcodecs.so.4.5.4
realsense_multicam_headless: /root/build/lib/libopencv_objdetect.so.4.5.4
realsense_multicam_headless: /root/build/lib/libopencv_calib3d.so.4.5.4
realsense_multicam_headless: /root/build/lib/libopencv_dnn.so.4.5.4
realsense_multicam_headless: /root/build/lib/libopencv_features2d.so.4.5.4
realsense_multicam_headless: /root/build/lib/libopencv_flann.so.4.5.4
realsense_multicam_headless: /root/build/lib/libopencv_photo.so.4.5.4
realsense_multicam_headless: /root/build/lib/libopencv_imgproc.so.4.5.4
realsense_multicam_headless: /root/build/lib/libopencv_core.so.4.5.4
realsense_multicam_headless: CMakeFiles/realsense_multicam_headless.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/root/realsense_docker/realsense/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable realsense_multicam_headless"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/realsense_multicam_headless.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/realsense_multicam_headless.dir/build: realsense_multicam_headless

.PHONY : CMakeFiles/realsense_multicam_headless.dir/build

CMakeFiles/realsense_multicam_headless.dir/requires: CMakeFiles/realsense_multicam_headless.dir/realsense_multicam_headless.cpp.o.requires

.PHONY : CMakeFiles/realsense_multicam_headless.dir/requires

CMakeFiles/realsense_multicam_headless.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/realsense_multicam_headless.dir/cmake_clean.cmake
.PHONY : CMakeFiles/realsense_multicam_headless.dir/clean

CMakeFiles/realsense_multicam_headless.dir/depend:
	cd /root/realsense_docker/realsense && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /root/realsense_docker/realsense /root/realsense_docker/realsense /root/realsense_docker/realsense /root/realsense_docker/realsense /root/realsense_docker/realsense/CMakeFiles/realsense_multicam_headless.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/realsense_multicam_headless.dir/depend

