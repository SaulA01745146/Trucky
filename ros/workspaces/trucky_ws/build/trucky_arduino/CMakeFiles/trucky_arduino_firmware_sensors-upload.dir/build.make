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
CMAKE_SOURCE_DIR = /home/jetson/TruckyAVProject/ros/workspaces/trucky_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/jetson/TruckyAVProject/ros/workspaces/trucky_ws/build

# Utility rule file for trucky_arduino_firmware_sensors-upload.

# Include the progress variables for this target.
include trucky_arduino/CMakeFiles/trucky_arduino_firmware_sensors-upload.dir/progress.make

trucky_arduino/CMakeFiles/trucky_arduino_firmware_sensors-upload:
	cd /home/jetson/TruckyAVProject/ros/workspaces/trucky_ws/build/trucky_arduino/firmware && /usr/bin/cmake --build /home/jetson/TruckyAVProject/ros/workspaces/trucky_ws/build/trucky_arduino/firmware -- sensors-upload

trucky_arduino_firmware_sensors-upload: trucky_arduino/CMakeFiles/trucky_arduino_firmware_sensors-upload
trucky_arduino_firmware_sensors-upload: trucky_arduino/CMakeFiles/trucky_arduino_firmware_sensors-upload.dir/build.make

.PHONY : trucky_arduino_firmware_sensors-upload

# Rule to build all files generated by this target.
trucky_arduino/CMakeFiles/trucky_arduino_firmware_sensors-upload.dir/build: trucky_arduino_firmware_sensors-upload

.PHONY : trucky_arduino/CMakeFiles/trucky_arduino_firmware_sensors-upload.dir/build

trucky_arduino/CMakeFiles/trucky_arduino_firmware_sensors-upload.dir/clean:
	cd /home/jetson/TruckyAVProject/ros/workspaces/trucky_ws/build/trucky_arduino && $(CMAKE_COMMAND) -P CMakeFiles/trucky_arduino_firmware_sensors-upload.dir/cmake_clean.cmake
.PHONY : trucky_arduino/CMakeFiles/trucky_arduino_firmware_sensors-upload.dir/clean

trucky_arduino/CMakeFiles/trucky_arduino_firmware_sensors-upload.dir/depend:
	cd /home/jetson/TruckyAVProject/ros/workspaces/trucky_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/jetson/TruckyAVProject/ros/workspaces/trucky_ws/src /home/jetson/TruckyAVProject/ros/workspaces/trucky_ws/src/trucky_arduino /home/jetson/TruckyAVProject/ros/workspaces/trucky_ws/build /home/jetson/TruckyAVProject/ros/workspaces/trucky_ws/build/trucky_arduino /home/jetson/TruckyAVProject/ros/workspaces/trucky_ws/build/trucky_arduino/CMakeFiles/trucky_arduino_firmware_sensors-upload.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : trucky_arduino/CMakeFiles/trucky_arduino_firmware_sensors-upload.dir/depend
