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
CMAKE_SOURCE_DIR = /home/cc/ee106b/sp23/class/ee106b-aba/Desktop/projects/proj2/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/cc/ee106b/sp23/class/ee106b-aba/Desktop/projects/proj2/build

# Utility rule file for _run_tests_stdr_server.

# Include the progress variables for this target.
include stdr_server/test/CMakeFiles/_run_tests_stdr_server.dir/progress.make

_run_tests_stdr_server: stdr_server/test/CMakeFiles/_run_tests_stdr_server.dir/build.make

.PHONY : _run_tests_stdr_server

# Rule to build all files generated by this target.
stdr_server/test/CMakeFiles/_run_tests_stdr_server.dir/build: _run_tests_stdr_server

.PHONY : stdr_server/test/CMakeFiles/_run_tests_stdr_server.dir/build

stdr_server/test/CMakeFiles/_run_tests_stdr_server.dir/clean:
	cd /home/cc/ee106b/sp23/class/ee106b-aba/Desktop/projects/proj2/build/stdr_server/test && $(CMAKE_COMMAND) -P CMakeFiles/_run_tests_stdr_server.dir/cmake_clean.cmake
.PHONY : stdr_server/test/CMakeFiles/_run_tests_stdr_server.dir/clean

stdr_server/test/CMakeFiles/_run_tests_stdr_server.dir/depend:
	cd /home/cc/ee106b/sp23/class/ee106b-aba/Desktop/projects/proj2/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/cc/ee106b/sp23/class/ee106b-aba/Desktop/projects/proj2/src /home/cc/ee106b/sp23/class/ee106b-aba/Desktop/projects/proj2/src/stdr_server/test /home/cc/ee106b/sp23/class/ee106b-aba/Desktop/projects/proj2/build /home/cc/ee106b/sp23/class/ee106b-aba/Desktop/projects/proj2/build/stdr_server/test /home/cc/ee106b/sp23/class/ee106b-aba/Desktop/projects/proj2/build/stdr_server/test/CMakeFiles/_run_tests_stdr_server.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : stdr_server/test/CMakeFiles/_run_tests_stdr_server.dir/depend

