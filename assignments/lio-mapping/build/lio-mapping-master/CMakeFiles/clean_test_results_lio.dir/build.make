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
CMAKE_SOURCE_DIR = /workspace/assignments/lio-mapping/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /workspace/assignments/lio-mapping/build

# Utility rule file for clean_test_results_lio.

# Include the progress variables for this target.
include lio-mapping-master/CMakeFiles/clean_test_results_lio.dir/progress.make

lio-mapping-master/CMakeFiles/clean_test_results_lio:
	cd /workspace/assignments/lio-mapping/build/lio-mapping-master && /usr/bin/python2 /opt/ros/melodic/share/catkin/cmake/test/remove_test_results.py /workspace/assignments/lio-mapping/build/test_results/lio

clean_test_results_lio: lio-mapping-master/CMakeFiles/clean_test_results_lio
clean_test_results_lio: lio-mapping-master/CMakeFiles/clean_test_results_lio.dir/build.make

.PHONY : clean_test_results_lio

# Rule to build all files generated by this target.
lio-mapping-master/CMakeFiles/clean_test_results_lio.dir/build: clean_test_results_lio

.PHONY : lio-mapping-master/CMakeFiles/clean_test_results_lio.dir/build

lio-mapping-master/CMakeFiles/clean_test_results_lio.dir/clean:
	cd /workspace/assignments/lio-mapping/build/lio-mapping-master && $(CMAKE_COMMAND) -P CMakeFiles/clean_test_results_lio.dir/cmake_clean.cmake
.PHONY : lio-mapping-master/CMakeFiles/clean_test_results_lio.dir/clean

lio-mapping-master/CMakeFiles/clean_test_results_lio.dir/depend:
	cd /workspace/assignments/lio-mapping/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /workspace/assignments/lio-mapping/src /workspace/assignments/lio-mapping/src/lio-mapping-master /workspace/assignments/lio-mapping/build /workspace/assignments/lio-mapping/build/lio-mapping-master /workspace/assignments/lio-mapping/build/lio-mapping-master/CMakeFiles/clean_test_results_lio.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : lio-mapping-master/CMakeFiles/clean_test_results_lio.dir/depend

