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
CMAKE_SOURCE_DIR = /home/zzz/dm_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/zzz/dm_ws/build

# Include any dependencies generated for this target.
include dm_motor_controller/ruckig_build/CMakeFiles/example-5_velocity.dir/depend.make

# Include the progress variables for this target.
include dm_motor_controller/ruckig_build/CMakeFiles/example-5_velocity.dir/progress.make

# Include the compile flags for this target's objects.
include dm_motor_controller/ruckig_build/CMakeFiles/example-5_velocity.dir/flags.make

dm_motor_controller/ruckig_build/CMakeFiles/example-5_velocity.dir/examples/5_velocity.cpp.o: dm_motor_controller/ruckig_build/CMakeFiles/example-5_velocity.dir/flags.make
dm_motor_controller/ruckig_build/CMakeFiles/example-5_velocity.dir/examples/5_velocity.cpp.o: /home/zzz/dm_ws/ruckig/examples/5_velocity.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zzz/dm_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object dm_motor_controller/ruckig_build/CMakeFiles/example-5_velocity.dir/examples/5_velocity.cpp.o"
	cd /home/zzz/dm_ws/build/dm_motor_controller/ruckig_build && /usr/lib/ccache/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/example-5_velocity.dir/examples/5_velocity.cpp.o -c /home/zzz/dm_ws/ruckig/examples/5_velocity.cpp

dm_motor_controller/ruckig_build/CMakeFiles/example-5_velocity.dir/examples/5_velocity.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/example-5_velocity.dir/examples/5_velocity.cpp.i"
	cd /home/zzz/dm_ws/build/dm_motor_controller/ruckig_build && /usr/lib/ccache/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zzz/dm_ws/ruckig/examples/5_velocity.cpp > CMakeFiles/example-5_velocity.dir/examples/5_velocity.cpp.i

dm_motor_controller/ruckig_build/CMakeFiles/example-5_velocity.dir/examples/5_velocity.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/example-5_velocity.dir/examples/5_velocity.cpp.s"
	cd /home/zzz/dm_ws/build/dm_motor_controller/ruckig_build && /usr/lib/ccache/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zzz/dm_ws/ruckig/examples/5_velocity.cpp -o CMakeFiles/example-5_velocity.dir/examples/5_velocity.cpp.s

# Object files for target example-5_velocity
example__5_velocity_OBJECTS = \
"CMakeFiles/example-5_velocity.dir/examples/5_velocity.cpp.o"

# External object files for target example-5_velocity
example__5_velocity_EXTERNAL_OBJECTS =

/home/zzz/dm_ws/devel/lib/dm_motor_controller/example-5_velocity: dm_motor_controller/ruckig_build/CMakeFiles/example-5_velocity.dir/examples/5_velocity.cpp.o
/home/zzz/dm_ws/devel/lib/dm_motor_controller/example-5_velocity: dm_motor_controller/ruckig_build/CMakeFiles/example-5_velocity.dir/build.make
/home/zzz/dm_ws/devel/lib/dm_motor_controller/example-5_velocity: /home/zzz/dm_ws/devel/lib/libruckig.so
/home/zzz/dm_ws/devel/lib/dm_motor_controller/example-5_velocity: dm_motor_controller/ruckig_build/CMakeFiles/example-5_velocity.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/zzz/dm_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/zzz/dm_ws/devel/lib/dm_motor_controller/example-5_velocity"
	cd /home/zzz/dm_ws/build/dm_motor_controller/ruckig_build && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/example-5_velocity.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
dm_motor_controller/ruckig_build/CMakeFiles/example-5_velocity.dir/build: /home/zzz/dm_ws/devel/lib/dm_motor_controller/example-5_velocity

.PHONY : dm_motor_controller/ruckig_build/CMakeFiles/example-5_velocity.dir/build

dm_motor_controller/ruckig_build/CMakeFiles/example-5_velocity.dir/clean:
	cd /home/zzz/dm_ws/build/dm_motor_controller/ruckig_build && $(CMAKE_COMMAND) -P CMakeFiles/example-5_velocity.dir/cmake_clean.cmake
.PHONY : dm_motor_controller/ruckig_build/CMakeFiles/example-5_velocity.dir/clean

dm_motor_controller/ruckig_build/CMakeFiles/example-5_velocity.dir/depend:
	cd /home/zzz/dm_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/zzz/dm_ws/src /home/zzz/dm_ws/ruckig /home/zzz/dm_ws/build /home/zzz/dm_ws/build/dm_motor_controller/ruckig_build /home/zzz/dm_ws/build/dm_motor_controller/ruckig_build/CMakeFiles/example-5_velocity.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : dm_motor_controller/ruckig_build/CMakeFiles/example-5_velocity.dir/depend

