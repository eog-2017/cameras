# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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
CMAKE_SOURCE_DIR = /home/komal/Desktop/Camera_Testing/Kinect_1/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/komal/Desktop/Camera_Testing/Kinect_1/build-src-Desktop-Default

# Include any dependencies generated for this target.
include vision/CMakeFiles/kinect_node.dir/depend.make

# Include the progress variables for this target.
include vision/CMakeFiles/kinect_node.dir/progress.make

# Include the compile flags for this target's objects.
include vision/CMakeFiles/kinect_node.dir/flags.make

vision/CMakeFiles/kinect_node.dir/src/kinect.cpp.o: vision/CMakeFiles/kinect_node.dir/flags.make
vision/CMakeFiles/kinect_node.dir/src/kinect.cpp.o: /home/komal/Desktop/Camera_Testing/Kinect_1/src/vision/src/kinect.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/komal/Desktop/Camera_Testing/Kinect_1/build-src-Desktop-Default/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object vision/CMakeFiles/kinect_node.dir/src/kinect.cpp.o"
	cd /home/komal/Desktop/Camera_Testing/Kinect_1/build-src-Desktop-Default/vision && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/kinect_node.dir/src/kinect.cpp.o -c /home/komal/Desktop/Camera_Testing/Kinect_1/src/vision/src/kinect.cpp

vision/CMakeFiles/kinect_node.dir/src/kinect.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/kinect_node.dir/src/kinect.cpp.i"
	cd /home/komal/Desktop/Camera_Testing/Kinect_1/build-src-Desktop-Default/vision && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/komal/Desktop/Camera_Testing/Kinect_1/src/vision/src/kinect.cpp > CMakeFiles/kinect_node.dir/src/kinect.cpp.i

vision/CMakeFiles/kinect_node.dir/src/kinect.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/kinect_node.dir/src/kinect.cpp.s"
	cd /home/komal/Desktop/Camera_Testing/Kinect_1/build-src-Desktop-Default/vision && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/komal/Desktop/Camera_Testing/Kinect_1/src/vision/src/kinect.cpp -o CMakeFiles/kinect_node.dir/src/kinect.cpp.s

vision/CMakeFiles/kinect_node.dir/src/kinect.cpp.o.requires:
.PHONY : vision/CMakeFiles/kinect_node.dir/src/kinect.cpp.o.requires

vision/CMakeFiles/kinect_node.dir/src/kinect.cpp.o.provides: vision/CMakeFiles/kinect_node.dir/src/kinect.cpp.o.requires
	$(MAKE) -f vision/CMakeFiles/kinect_node.dir/build.make vision/CMakeFiles/kinect_node.dir/src/kinect.cpp.o.provides.build
.PHONY : vision/CMakeFiles/kinect_node.dir/src/kinect.cpp.o.provides

vision/CMakeFiles/kinect_node.dir/src/kinect.cpp.o.provides.build: vision/CMakeFiles/kinect_node.dir/src/kinect.cpp.o

# Object files for target kinect_node
kinect_node_OBJECTS = \
"CMakeFiles/kinect_node.dir/src/kinect.cpp.o"

# External object files for target kinect_node
kinect_node_EXTERNAL_OBJECTS =

devel/lib/vision/kinect_node: vision/CMakeFiles/kinect_node.dir/src/kinect.cpp.o
devel/lib/vision/kinect_node: vision/CMakeFiles/kinect_node.dir/build.make
devel/lib/vision/kinect_node: /opt/ros/indigo/lib/libroscpp.so
devel/lib/vision/kinect_node: /usr/lib/x86_64-linux-gnu/libboost_signals.so
devel/lib/vision/kinect_node: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
devel/lib/vision/kinect_node: /opt/ros/indigo/lib/libxmlrpcpp.so
devel/lib/vision/kinect_node: /opt/ros/indigo/lib/libcv_bridge.so
devel/lib/vision/kinect_node: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.2.4.8
devel/lib/vision/kinect_node: /usr/lib/x86_64-linux-gnu/libopencv_video.so.2.4.8
devel/lib/vision/kinect_node: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.2.4.8
devel/lib/vision/kinect_node: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.2.4.8
devel/lib/vision/kinect_node: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.2.4.8
devel/lib/vision/kinect_node: /usr/lib/x86_64-linux-gnu/libopencv_ocl.so.2.4.8
devel/lib/vision/kinect_node: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.2.4.8
devel/lib/vision/kinect_node: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.2.4.8
devel/lib/vision/kinect_node: /usr/lib/x86_64-linux-gnu/libopencv_legacy.so.2.4.8
devel/lib/vision/kinect_node: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.2.4.8
devel/lib/vision/kinect_node: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.2.4.8
devel/lib/vision/kinect_node: /usr/lib/x86_64-linux-gnu/libopencv_gpu.so.2.4.8
devel/lib/vision/kinect_node: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.2.4.8
devel/lib/vision/kinect_node: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.2.4.8
devel/lib/vision/kinect_node: /usr/lib/x86_64-linux-gnu/libopencv_core.so.2.4.8
devel/lib/vision/kinect_node: /usr/lib/x86_64-linux-gnu/libopencv_contrib.so.2.4.8
devel/lib/vision/kinect_node: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.2.4.8
devel/lib/vision/kinect_node: /opt/ros/indigo/lib/librosconsole.so
devel/lib/vision/kinect_node: /opt/ros/indigo/lib/librosconsole_log4cxx.so
devel/lib/vision/kinect_node: /opt/ros/indigo/lib/librosconsole_backend_interface.so
devel/lib/vision/kinect_node: /usr/lib/liblog4cxx.so
devel/lib/vision/kinect_node: /usr/lib/x86_64-linux-gnu/libboost_regex.so
devel/lib/vision/kinect_node: /opt/ros/indigo/lib/libroscpp_serialization.so
devel/lib/vision/kinect_node: /opt/ros/indigo/lib/librostime.so
devel/lib/vision/kinect_node: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
devel/lib/vision/kinect_node: /opt/ros/indigo/lib/libcpp_common.so
devel/lib/vision/kinect_node: /usr/lib/x86_64-linux-gnu/libboost_system.so
devel/lib/vision/kinect_node: /usr/lib/x86_64-linux-gnu/libboost_thread.so
devel/lib/vision/kinect_node: /usr/lib/x86_64-linux-gnu/libpthread.so
devel/lib/vision/kinect_node: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so
devel/lib/vision/kinect_node: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.2.4.8
devel/lib/vision/kinect_node: /usr/lib/x86_64-linux-gnu/libopencv_video.so.2.4.8
devel/lib/vision/kinect_node: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.2.4.8
devel/lib/vision/kinect_node: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.2.4.8
devel/lib/vision/kinect_node: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.2.4.8
devel/lib/vision/kinect_node: /usr/lib/x86_64-linux-gnu/libopencv_ocl.so.2.4.8
devel/lib/vision/kinect_node: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.2.4.8
devel/lib/vision/kinect_node: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.2.4.8
devel/lib/vision/kinect_node: /usr/lib/x86_64-linux-gnu/libopencv_legacy.so.2.4.8
devel/lib/vision/kinect_node: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.2.4.8
devel/lib/vision/kinect_node: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.2.4.8
devel/lib/vision/kinect_node: /usr/lib/x86_64-linux-gnu/libopencv_gpu.so.2.4.8
devel/lib/vision/kinect_node: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.2.4.8
devel/lib/vision/kinect_node: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.2.4.8
devel/lib/vision/kinect_node: /usr/lib/x86_64-linux-gnu/libopencv_core.so.2.4.8
devel/lib/vision/kinect_node: /usr/lib/x86_64-linux-gnu/libopencv_contrib.so.2.4.8
devel/lib/vision/kinect_node: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.2.4.8
devel/lib/vision/kinect_node: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.2.4.8
devel/lib/vision/kinect_node: /usr/lib/x86_64-linux-gnu/libopencv_legacy.so.2.4.8
devel/lib/vision/kinect_node: /usr/lib/x86_64-linux-gnu/libopencv_video.so.2.4.8
devel/lib/vision/kinect_node: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.2.4.8
devel/lib/vision/kinect_node: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.2.4.8
devel/lib/vision/kinect_node: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.2.4.8
devel/lib/vision/kinect_node: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.2.4.8
devel/lib/vision/kinect_node: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.2.4.8
devel/lib/vision/kinect_node: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.2.4.8
devel/lib/vision/kinect_node: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.2.4.8
devel/lib/vision/kinect_node: /usr/lib/x86_64-linux-gnu/libopencv_core.so.2.4.8
devel/lib/vision/kinect_node: vision/CMakeFiles/kinect_node.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable ../devel/lib/vision/kinect_node"
	cd /home/komal/Desktop/Camera_Testing/Kinect_1/build-src-Desktop-Default/vision && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/kinect_node.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
vision/CMakeFiles/kinect_node.dir/build: devel/lib/vision/kinect_node
.PHONY : vision/CMakeFiles/kinect_node.dir/build

vision/CMakeFiles/kinect_node.dir/requires: vision/CMakeFiles/kinect_node.dir/src/kinect.cpp.o.requires
.PHONY : vision/CMakeFiles/kinect_node.dir/requires

vision/CMakeFiles/kinect_node.dir/clean:
	cd /home/komal/Desktop/Camera_Testing/Kinect_1/build-src-Desktop-Default/vision && $(CMAKE_COMMAND) -P CMakeFiles/kinect_node.dir/cmake_clean.cmake
.PHONY : vision/CMakeFiles/kinect_node.dir/clean

vision/CMakeFiles/kinect_node.dir/depend:
	cd /home/komal/Desktop/Camera_Testing/Kinect_1/build-src-Desktop-Default && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/komal/Desktop/Camera_Testing/Kinect_1/src /home/komal/Desktop/Camera_Testing/Kinect_1/src/vision /home/komal/Desktop/Camera_Testing/Kinect_1/build-src-Desktop-Default /home/komal/Desktop/Camera_Testing/Kinect_1/build-src-Desktop-Default/vision /home/komal/Desktop/Camera_Testing/Kinect_1/build-src-Desktop-Default/vision/CMakeFiles/kinect_node.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : vision/CMakeFiles/kinect_node.dir/depend

