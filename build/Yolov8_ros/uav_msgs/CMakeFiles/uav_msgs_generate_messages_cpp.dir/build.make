# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

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
CMAKE_SOURCE_DIR = /home/ros20/catkin_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ros20/catkin_ws/build

# Utility rule file for uav_msgs_generate_messages_cpp.

# Include the progress variables for this target.
include Yolov8_ros/uav_msgs/CMakeFiles/uav_msgs_generate_messages_cpp.dir/progress.make

Yolov8_ros/uav_msgs/CMakeFiles/uav_msgs_generate_messages_cpp: /home/ros20/catkin_ws/devel/include/uav_msgs/AngleRateThrottle.h
Yolov8_ros/uav_msgs/CMakeFiles/uav_msgs_generate_messages_cpp: /home/ros20/catkin_ws/devel/include/uav_msgs/DesiredStates.h
Yolov8_ros/uav_msgs/CMakeFiles/uav_msgs_generate_messages_cpp: /home/ros20/catkin_ws/devel/include/uav_msgs/Takeoff.h


/home/ros20/catkin_ws/devel/include/uav_msgs/AngleRateThrottle.h: /opt/ros/noetic/lib/gencpp/gen_cpp.py
/home/ros20/catkin_ws/devel/include/uav_msgs/AngleRateThrottle.h: /home/ros20/catkin_ws/src/Yolov8_ros/uav_msgs/msg/AngleRateThrottle.msg
/home/ros20/catkin_ws/devel/include/uav_msgs/AngleRateThrottle.h: /opt/ros/noetic/share/gencpp/msg.h.template
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/ros20/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Generating C++ code from uav_msgs/AngleRateThrottle.msg"
	cd /home/ros20/catkin_ws/src/Yolov8_ros/uav_msgs && /home/ros20/catkin_ws/build/catkin_generated/env_cached.sh /home/ros20/miniconda3/envs/airsim/bin/python3 /opt/ros/noetic/share/gencpp/cmake/../../../lib/gencpp/gen_cpp.py /home/ros20/catkin_ws/src/Yolov8_ros/uav_msgs/msg/AngleRateThrottle.msg -Iuav_msgs:/home/ros20/catkin_ws/src/Yolov8_ros/uav_msgs/msg -Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg -Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg -p uav_msgs -o /home/ros20/catkin_ws/devel/include/uav_msgs -e /opt/ros/noetic/share/gencpp/cmake/..

/home/ros20/catkin_ws/devel/include/uav_msgs/DesiredStates.h: /opt/ros/noetic/lib/gencpp/gen_cpp.py
/home/ros20/catkin_ws/devel/include/uav_msgs/DesiredStates.h: /home/ros20/catkin_ws/src/Yolov8_ros/uav_msgs/msg/DesiredStates.msg
/home/ros20/catkin_ws/devel/include/uav_msgs/DesiredStates.h: /opt/ros/noetic/share/geometry_msgs/msg/Vector3.msg
/home/ros20/catkin_ws/devel/include/uav_msgs/DesiredStates.h: /opt/ros/noetic/share/std_msgs/msg/Header.msg
/home/ros20/catkin_ws/devel/include/uav_msgs/DesiredStates.h: /opt/ros/noetic/share/gencpp/msg.h.template
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/ros20/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Generating C++ code from uav_msgs/DesiredStates.msg"
	cd /home/ros20/catkin_ws/src/Yolov8_ros/uav_msgs && /home/ros20/catkin_ws/build/catkin_generated/env_cached.sh /home/ros20/miniconda3/envs/airsim/bin/python3 /opt/ros/noetic/share/gencpp/cmake/../../../lib/gencpp/gen_cpp.py /home/ros20/catkin_ws/src/Yolov8_ros/uav_msgs/msg/DesiredStates.msg -Iuav_msgs:/home/ros20/catkin_ws/src/Yolov8_ros/uav_msgs/msg -Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg -Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg -p uav_msgs -o /home/ros20/catkin_ws/devel/include/uav_msgs -e /opt/ros/noetic/share/gencpp/cmake/..

/home/ros20/catkin_ws/devel/include/uav_msgs/Takeoff.h: /opt/ros/noetic/lib/gencpp/gen_cpp.py
/home/ros20/catkin_ws/devel/include/uav_msgs/Takeoff.h: /home/ros20/catkin_ws/src/Yolov8_ros/uav_msgs/srv/Takeoff.srv
/home/ros20/catkin_ws/devel/include/uav_msgs/Takeoff.h: /opt/ros/noetic/share/gencpp/msg.h.template
/home/ros20/catkin_ws/devel/include/uav_msgs/Takeoff.h: /opt/ros/noetic/share/gencpp/srv.h.template
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/ros20/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Generating C++ code from uav_msgs/Takeoff.srv"
	cd /home/ros20/catkin_ws/src/Yolov8_ros/uav_msgs && /home/ros20/catkin_ws/build/catkin_generated/env_cached.sh /home/ros20/miniconda3/envs/airsim/bin/python3 /opt/ros/noetic/share/gencpp/cmake/../../../lib/gencpp/gen_cpp.py /home/ros20/catkin_ws/src/Yolov8_ros/uav_msgs/srv/Takeoff.srv -Iuav_msgs:/home/ros20/catkin_ws/src/Yolov8_ros/uav_msgs/msg -Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg -Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg -p uav_msgs -o /home/ros20/catkin_ws/devel/include/uav_msgs -e /opt/ros/noetic/share/gencpp/cmake/..

uav_msgs_generate_messages_cpp: Yolov8_ros/uav_msgs/CMakeFiles/uav_msgs_generate_messages_cpp
uav_msgs_generate_messages_cpp: /home/ros20/catkin_ws/devel/include/uav_msgs/AngleRateThrottle.h
uav_msgs_generate_messages_cpp: /home/ros20/catkin_ws/devel/include/uav_msgs/DesiredStates.h
uav_msgs_generate_messages_cpp: /home/ros20/catkin_ws/devel/include/uav_msgs/Takeoff.h
uav_msgs_generate_messages_cpp: Yolov8_ros/uav_msgs/CMakeFiles/uav_msgs_generate_messages_cpp.dir/build.make

.PHONY : uav_msgs_generate_messages_cpp

# Rule to build all files generated by this target.
Yolov8_ros/uav_msgs/CMakeFiles/uav_msgs_generate_messages_cpp.dir/build: uav_msgs_generate_messages_cpp

.PHONY : Yolov8_ros/uav_msgs/CMakeFiles/uav_msgs_generate_messages_cpp.dir/build

Yolov8_ros/uav_msgs/CMakeFiles/uav_msgs_generate_messages_cpp.dir/clean:
	cd /home/ros20/catkin_ws/build/Yolov8_ros/uav_msgs && $(CMAKE_COMMAND) -P CMakeFiles/uav_msgs_generate_messages_cpp.dir/cmake_clean.cmake
.PHONY : Yolov8_ros/uav_msgs/CMakeFiles/uav_msgs_generate_messages_cpp.dir/clean

Yolov8_ros/uav_msgs/CMakeFiles/uav_msgs_generate_messages_cpp.dir/depend:
	cd /home/ros20/catkin_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ros20/catkin_ws/src /home/ros20/catkin_ws/src/Yolov8_ros/uav_msgs /home/ros20/catkin_ws/build /home/ros20/catkin_ws/build/Yolov8_ros/uav_msgs /home/ros20/catkin_ws/build/Yolov8_ros/uav_msgs/CMakeFiles/uav_msgs_generate_messages_cpp.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : Yolov8_ros/uav_msgs/CMakeFiles/uav_msgs_generate_messages_cpp.dir/depend

