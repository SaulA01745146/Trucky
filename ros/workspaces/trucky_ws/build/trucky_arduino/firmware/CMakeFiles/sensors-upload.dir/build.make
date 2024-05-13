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
CMAKE_SOURCE_DIR = /home/jetson/TruckyAVProject/ros/workspaces/trucky_ws/src/trucky_arduino/firmware

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/jetson/TruckyAVProject/ros/workspaces/trucky_ws/build/trucky_arduino/firmware

# Utility rule file for sensors-upload.

# Include the progress variables for this target.
include CMakeFiles/sensors-upload.dir/progress.make

CMakeFiles/sensors-upload: /home/jetson/TruckyAVProject/ros/workspaces/trucky_ws/devel/share/trucky_arduino/sensors.elf
	/usr/share/arduino/hardware/tools/avrdude -C/usr/share/arduino/hardware/tools/avrdude.conf -patmega328p -carduino -b115200 -P/dev/ttyUSB0 -D -V -Uflash:w:/home/jetson/TruckyAVProject/ros/workspaces/trucky_ws/devel/share/trucky_arduino/sensors.hex -Ueeprom:w:/home/jetson/TruckyAVProject/ros/workspaces/trucky_ws/devel/share/trucky_arduino/sensors.eep:i

sensors-upload: CMakeFiles/sensors-upload
sensors-upload: CMakeFiles/sensors-upload.dir/build.make

.PHONY : sensors-upload

# Rule to build all files generated by this target.
CMakeFiles/sensors-upload.dir/build: sensors-upload

.PHONY : CMakeFiles/sensors-upload.dir/build

CMakeFiles/sensors-upload.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/sensors-upload.dir/cmake_clean.cmake
.PHONY : CMakeFiles/sensors-upload.dir/clean

CMakeFiles/sensors-upload.dir/depend:
	cd /home/jetson/TruckyAVProject/ros/workspaces/trucky_ws/build/trucky_arduino/firmware && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/jetson/TruckyAVProject/ros/workspaces/trucky_ws/src/trucky_arduino/firmware /home/jetson/TruckyAVProject/ros/workspaces/trucky_ws/src/trucky_arduino/firmware /home/jetson/TruckyAVProject/ros/workspaces/trucky_ws/build/trucky_arduino/firmware /home/jetson/TruckyAVProject/ros/workspaces/trucky_ws/build/trucky_arduino/firmware /home/jetson/TruckyAVProject/ros/workspaces/trucky_ws/build/trucky_arduino/firmware/CMakeFiles/sensors-upload.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/sensors-upload.dir/depend
