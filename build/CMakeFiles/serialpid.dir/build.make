# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

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
CMAKE_SOURCE_DIR = /home/tongliuzhu/test_ws/serial_pid

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/tongliuzhu/test_ws/serial_pid/build

# Include any dependencies generated for this target.
include CMakeFiles/serialpid.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/serialpid.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/serialpid.dir/flags.make

CMakeFiles/serialpid.dir/src/main.cpp.o: CMakeFiles/serialpid.dir/flags.make
CMakeFiles/serialpid.dir/src/main.cpp.o: ../src/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/tongliuzhu/test_ws/serial_pid/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/serialpid.dir/src/main.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/serialpid.dir/src/main.cpp.o -c /home/tongliuzhu/test_ws/serial_pid/src/main.cpp

CMakeFiles/serialpid.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/serialpid.dir/src/main.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/tongliuzhu/test_ws/serial_pid/src/main.cpp > CMakeFiles/serialpid.dir/src/main.cpp.i

CMakeFiles/serialpid.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/serialpid.dir/src/main.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/tongliuzhu/test_ws/serial_pid/src/main.cpp -o CMakeFiles/serialpid.dir/src/main.cpp.s

CMakeFiles/serialpid.dir/src/main.cpp.o.requires:

.PHONY : CMakeFiles/serialpid.dir/src/main.cpp.o.requires

CMakeFiles/serialpid.dir/src/main.cpp.o.provides: CMakeFiles/serialpid.dir/src/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/serialpid.dir/build.make CMakeFiles/serialpid.dir/src/main.cpp.o.provides.build
.PHONY : CMakeFiles/serialpid.dir/src/main.cpp.o.provides

CMakeFiles/serialpid.dir/src/main.cpp.o.provides.build: CMakeFiles/serialpid.dir/src/main.cpp.o


CMakeFiles/serialpid.dir/src/PID.cpp.o: CMakeFiles/serialpid.dir/flags.make
CMakeFiles/serialpid.dir/src/PID.cpp.o: ../src/PID.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/tongliuzhu/test_ws/serial_pid/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/serialpid.dir/src/PID.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/serialpid.dir/src/PID.cpp.o -c /home/tongliuzhu/test_ws/serial_pid/src/PID.cpp

CMakeFiles/serialpid.dir/src/PID.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/serialpid.dir/src/PID.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/tongliuzhu/test_ws/serial_pid/src/PID.cpp > CMakeFiles/serialpid.dir/src/PID.cpp.i

CMakeFiles/serialpid.dir/src/PID.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/serialpid.dir/src/PID.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/tongliuzhu/test_ws/serial_pid/src/PID.cpp -o CMakeFiles/serialpid.dir/src/PID.cpp.s

CMakeFiles/serialpid.dir/src/PID.cpp.o.requires:

.PHONY : CMakeFiles/serialpid.dir/src/PID.cpp.o.requires

CMakeFiles/serialpid.dir/src/PID.cpp.o.provides: CMakeFiles/serialpid.dir/src/PID.cpp.o.requires
	$(MAKE) -f CMakeFiles/serialpid.dir/build.make CMakeFiles/serialpid.dir/src/PID.cpp.o.provides.build
.PHONY : CMakeFiles/serialpid.dir/src/PID.cpp.o.provides

CMakeFiles/serialpid.dir/src/PID.cpp.o.provides.build: CMakeFiles/serialpid.dir/src/PID.cpp.o


# Object files for target serialpid
serialpid_OBJECTS = \
"CMakeFiles/serialpid.dir/src/main.cpp.o" \
"CMakeFiles/serialpid.dir/src/PID.cpp.o"

# External object files for target serialpid
serialpid_EXTERNAL_OBJECTS =

serialpid: CMakeFiles/serialpid.dir/src/main.cpp.o
serialpid: CMakeFiles/serialpid.dir/src/PID.cpp.o
serialpid: CMakeFiles/serialpid.dir/build.make
serialpid: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_stitching3.so.3.3.1
serialpid: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_superres3.so.3.3.1
serialpid: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_videostab3.so.3.3.1
serialpid: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_aruco3.so.3.3.1
serialpid: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_bgsegm3.so.3.3.1
serialpid: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_bioinspired3.so.3.3.1
serialpid: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_ccalib3.so.3.3.1
serialpid: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_cvv3.so.3.3.1
serialpid: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_dpm3.so.3.3.1
serialpid: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_face3.so.3.3.1
serialpid: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_fuzzy3.so.3.3.1
serialpid: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_hdf3.so.3.3.1
serialpid: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_img_hash3.so.3.3.1
serialpid: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_line_descriptor3.so.3.3.1
serialpid: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_optflow3.so.3.3.1
serialpid: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_reg3.so.3.3.1
serialpid: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_rgbd3.so.3.3.1
serialpid: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_saliency3.so.3.3.1
serialpid: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_stereo3.so.3.3.1
serialpid: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_structured_light3.so.3.3.1
serialpid: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_surface_matching3.so.3.3.1
serialpid: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_tracking3.so.3.3.1
serialpid: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_xfeatures2d3.so.3.3.1
serialpid: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_ximgproc3.so.3.3.1
serialpid: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_xobjdetect3.so.3.3.1
serialpid: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_xphoto3.so.3.3.1
serialpid: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_shape3.so.3.3.1
serialpid: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_photo3.so.3.3.1
serialpid: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_datasets3.so.3.3.1
serialpid: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_plot3.so.3.3.1
serialpid: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_text3.so.3.3.1
serialpid: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_dnn3.so.3.3.1
serialpid: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_ml3.so.3.3.1
serialpid: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_video3.so.3.3.1
serialpid: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_calib3d3.so.3.3.1
serialpid: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_features2d3.so.3.3.1
serialpid: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_highgui3.so.3.3.1
serialpid: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_videoio3.so.3.3.1
serialpid: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_viz3.so.3.3.1
serialpid: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_phase_unwrapping3.so.3.3.1
serialpid: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_flann3.so.3.3.1
serialpid: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_imgcodecs3.so.3.3.1
serialpid: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_objdetect3.so.3.3.1
serialpid: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_imgproc3.so.3.3.1
serialpid: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_core3.so.3.3.1
serialpid: CMakeFiles/serialpid.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/tongliuzhu/test_ws/serial_pid/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable serialpid"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/serialpid.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/serialpid.dir/build: serialpid

.PHONY : CMakeFiles/serialpid.dir/build

CMakeFiles/serialpid.dir/requires: CMakeFiles/serialpid.dir/src/main.cpp.o.requires
CMakeFiles/serialpid.dir/requires: CMakeFiles/serialpid.dir/src/PID.cpp.o.requires

.PHONY : CMakeFiles/serialpid.dir/requires

CMakeFiles/serialpid.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/serialpid.dir/cmake_clean.cmake
.PHONY : CMakeFiles/serialpid.dir/clean

CMakeFiles/serialpid.dir/depend:
	cd /home/tongliuzhu/test_ws/serial_pid/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/tongliuzhu/test_ws/serial_pid /home/tongliuzhu/test_ws/serial_pid /home/tongliuzhu/test_ws/serial_pid/build /home/tongliuzhu/test_ws/serial_pid/build /home/tongliuzhu/test_ws/serial_pid/build/CMakeFiles/serialpid.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/serialpid.dir/depend

