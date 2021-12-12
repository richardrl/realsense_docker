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
include CMakeFiles/realsense_multicam.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/realsense_multicam.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/realsense_multicam.dir/flags.make

CMakeFiles/realsense_multicam.dir/realsense_multicam.cpp.o: CMakeFiles/realsense_multicam.dir/flags.make
CMakeFiles/realsense_multicam.dir/realsense_multicam.cpp.o: realsense_multicam.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/root/realsense_docker/realsense/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/realsense_multicam.dir/realsense_multicam.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/realsense_multicam.dir/realsense_multicam.cpp.o -c /root/realsense_docker/realsense/realsense_multicam.cpp

CMakeFiles/realsense_multicam.dir/realsense_multicam.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/realsense_multicam.dir/realsense_multicam.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /root/realsense_docker/realsense/realsense_multicam.cpp > CMakeFiles/realsense_multicam.dir/realsense_multicam.cpp.i

CMakeFiles/realsense_multicam.dir/realsense_multicam.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/realsense_multicam.dir/realsense_multicam.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /root/realsense_docker/realsense/realsense_multicam.cpp -o CMakeFiles/realsense_multicam.dir/realsense_multicam.cpp.s

CMakeFiles/realsense_multicam.dir/realsense_multicam.cpp.o.requires:

.PHONY : CMakeFiles/realsense_multicam.dir/realsense_multicam.cpp.o.requires

CMakeFiles/realsense_multicam.dir/realsense_multicam.cpp.o.provides: CMakeFiles/realsense_multicam.dir/realsense_multicam.cpp.o.requires
	$(MAKE) -f CMakeFiles/realsense_multicam.dir/build.make CMakeFiles/realsense_multicam.dir/realsense_multicam.cpp.o.provides.build
.PHONY : CMakeFiles/realsense_multicam.dir/realsense_multicam.cpp.o.provides

CMakeFiles/realsense_multicam.dir/realsense_multicam.cpp.o.provides.build: CMakeFiles/realsense_multicam.dir/realsense_multicam.cpp.o


# Object files for target realsense_multicam
realsense_multicam_OBJECTS = \
"CMakeFiles/realsense_multicam.dir/realsense_multicam.cpp.o"

# External object files for target realsense_multicam
realsense_multicam_EXTERNAL_OBJECTS =

realsense_multicam: CMakeFiles/realsense_multicam.dir/realsense_multicam.cpp.o
realsense_multicam: CMakeFiles/realsense_multicam.dir/build.make
realsense_multicam: /usr/lib/x86_64-linux-gnu/libGL.so
realsense_multicam: /usr/lib/x86_64-linux-gnu/libGLU.so
realsense_multicam: /usr/lib/x86_64-linux-gnu/libglfw.so
realsense_multicam: CMakeFiles/realsense_multicam.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/root/realsense_docker/realsense/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable realsense_multicam"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/realsense_multicam.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/realsense_multicam.dir/build: realsense_multicam

.PHONY : CMakeFiles/realsense_multicam.dir/build

CMakeFiles/realsense_multicam.dir/requires: CMakeFiles/realsense_multicam.dir/realsense_multicam.cpp.o.requires

.PHONY : CMakeFiles/realsense_multicam.dir/requires

CMakeFiles/realsense_multicam.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/realsense_multicam.dir/cmake_clean.cmake
.PHONY : CMakeFiles/realsense_multicam.dir/clean

CMakeFiles/realsense_multicam.dir/depend:
	cd /root/realsense_docker/realsense && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /root/realsense_docker/realsense /root/realsense_docker/realsense /root/realsense_docker/realsense /root/realsense_docker/realsense /root/realsense_docker/realsense/CMakeFiles/realsense_multicam.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/realsense_multicam.dir/depend

