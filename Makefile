# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

# Default target executed when no arguments are given to make.
default_target: all

.PHONY : default_target

# Allow only one "make -f Makefile2" at a time, but pass parallelism.
.NOTPARALLEL:


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
CMAKE_SOURCE_DIR = /home/leif/Documents/Swarm/SwarmFinalProject

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/leif/Documents/Swarm/SwarmFinalProject

#=============================================================================
# Targets provided globally by CMake.

# Special rule for the target rebuild_cache
rebuild_cache:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Running CMake to regenerate build system..."
	/usr/bin/cmake -H$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR)
.PHONY : rebuild_cache

# Special rule for the target rebuild_cache
rebuild_cache/fast: rebuild_cache

.PHONY : rebuild_cache/fast

# Special rule for the target edit_cache
edit_cache:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "No interactive CMake dialog available..."
	/usr/bin/cmake -E echo No\ interactive\ CMake\ dialog\ available.
.PHONY : edit_cache

# Special rule for the target edit_cache
edit_cache/fast: edit_cache

.PHONY : edit_cache/fast

# The main all target
all: cmake_check_build_system
	$(CMAKE_COMMAND) -E cmake_progress_start /home/leif/Documents/Swarm/SwarmFinalProject/CMakeFiles /home/leif/Documents/Swarm/SwarmFinalProject/CMakeFiles/progress.marks
	$(MAKE) -f CMakeFiles/Makefile2 all
	$(CMAKE_COMMAND) -E cmake_progress_start /home/leif/Documents/Swarm/SwarmFinalProject/CMakeFiles 0
.PHONY : all

# The main clean target
clean:
	$(MAKE) -f CMakeFiles/Makefile2 clean
.PHONY : clean

# The main clean target
clean/fast: clean

.PHONY : clean/fast

# Prepare targets for installation.
preinstall: all
	$(MAKE) -f CMakeFiles/Makefile2 preinstall
.PHONY : preinstall

# Prepare targets for installation.
preinstall/fast:
	$(MAKE) -f CMakeFiles/Makefile2 preinstall
.PHONY : preinstall/fast

# clear depends
depend:
	$(CMAKE_COMMAND) -H$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR) --check-build-system CMakeFiles/Makefile.cmake 1
.PHONY : depend

#=============================================================================
# Target rules for targets named mpga

# Build rule for target.
mpga: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 mpga
.PHONY : mpga

# fast build rule for target.
mpga/fast:
	$(MAKE) -f loop_functions/CMakeFiles/mpga.dir/build.make loop_functions/CMakeFiles/mpga.dir/build
.PHONY : mpga/fast

#=============================================================================
# Target rules for targets named experiment1_loop_functions

# Build rule for target.
experiment1_loop_functions: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 experiment1_loop_functions
.PHONY : experiment1_loop_functions

# fast build rule for target.
experiment1_loop_functions/fast:
	$(MAKE) -f loop_functions/CMakeFiles/experiment1_loop_functions.dir/build.make loop_functions/CMakeFiles/experiment1_loop_functions.dir/build
.PHONY : experiment1_loop_functions/fast

#=============================================================================
# Target rules for targets named mpga_autogen

# Build rule for target.
mpga_autogen: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 mpga_autogen
.PHONY : mpga_autogen

# fast build rule for target.
mpga_autogen/fast:
	$(MAKE) -f loop_functions/CMakeFiles/mpga_autogen.dir/build.make loop_functions/CMakeFiles/mpga_autogen.dir/build
.PHONY : mpga_autogen/fast

#=============================================================================
# Target rules for targets named experiment1_loop_functions_autogen

# Build rule for target.
experiment1_loop_functions_autogen: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 experiment1_loop_functions_autogen
.PHONY : experiment1_loop_functions_autogen

# fast build rule for target.
experiment1_loop_functions_autogen/fast:
	$(MAKE) -f loop_functions/CMakeFiles/experiment1_loop_functions_autogen.dir/build.make loop_functions/CMakeFiles/experiment1_loop_functions_autogen.dir/build
.PHONY : experiment1_loop_functions_autogen/fast

#=============================================================================
# Target rules for targets named experiment_1

# Build rule for target.
experiment_1: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 experiment_1
.PHONY : experiment_1

# fast build rule for target.
experiment_1/fast:
	$(MAKE) -f embedding/CMakeFiles/experiment_1.dir/build.make embedding/CMakeFiles/experiment_1.dir/build
.PHONY : experiment_1/fast

#=============================================================================
# Target rules for targets named experiment_1_autogen

# Build rule for target.
experiment_1_autogen: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 experiment_1_autogen
.PHONY : experiment_1_autogen

# fast build rule for target.
experiment_1_autogen/fast:
	$(MAKE) -f embedding/CMakeFiles/experiment_1_autogen.dir/build.make embedding/CMakeFiles/experiment_1_autogen.dir/build
.PHONY : experiment_1_autogen/fast

# Help Target
help:
	@echo "The following are some of the valid targets for this Makefile:"
	@echo "... all (the default if no target is provided)"
	@echo "... clean"
	@echo "... depend"
	@echo "... rebuild_cache"
	@echo "... edit_cache"
	@echo "... mpga"
	@echo "... experiment1_loop_functions"
	@echo "... mpga_autogen"
	@echo "... experiment1_loop_functions_autogen"
	@echo "... experiment_1"
	@echo "... experiment_1_autogen"
.PHONY : help



#=============================================================================
# Special targets to cleanup operation of make.

# Special rule to run CMake to check the build system integrity.
# No rule that depends on this can have commands that come from listfiles
# because they might be regenerated.
cmake_check_build_system:
	$(CMAKE_COMMAND) -H$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR) --check-build-system CMakeFiles/Makefile.cmake 0
.PHONY : cmake_check_build_system
