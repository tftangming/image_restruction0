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
CMAKE_SOURCE_DIR = /home/tangming/ImageBasedModellingEduV1.0

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/tangming/ImageBasedModellingEduV1.0/cmake-build-debug

# Include any dependencies generated for this target.
include examples/task7/CMakeFiles/task7_mesh_clean.dir/depend.make

# Include the progress variables for this target.
include examples/task7/CMakeFiles/task7_mesh_clean.dir/progress.make

# Include the compile flags for this target's objects.
include examples/task7/CMakeFiles/task7_mesh_clean.dir/flags.make

examples/task7/CMakeFiles/task7_mesh_clean.dir/class7_meshclean.cc.o: examples/task7/CMakeFiles/task7_mesh_clean.dir/flags.make
examples/task7/CMakeFiles/task7_mesh_clean.dir/class7_meshclean.cc.o: ../examples/task7/class7_meshclean.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/tangming/ImageBasedModellingEduV1.0/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object examples/task7/CMakeFiles/task7_mesh_clean.dir/class7_meshclean.cc.o"
	cd /home/tangming/ImageBasedModellingEduV1.0/cmake-build-debug/examples/task7 && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/task7_mesh_clean.dir/class7_meshclean.cc.o -c /home/tangming/ImageBasedModellingEduV1.0/examples/task7/class7_meshclean.cc

examples/task7/CMakeFiles/task7_mesh_clean.dir/class7_meshclean.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/task7_mesh_clean.dir/class7_meshclean.cc.i"
	cd /home/tangming/ImageBasedModellingEduV1.0/cmake-build-debug/examples/task7 && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/tangming/ImageBasedModellingEduV1.0/examples/task7/class7_meshclean.cc > CMakeFiles/task7_mesh_clean.dir/class7_meshclean.cc.i

examples/task7/CMakeFiles/task7_mesh_clean.dir/class7_meshclean.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/task7_mesh_clean.dir/class7_meshclean.cc.s"
	cd /home/tangming/ImageBasedModellingEduV1.0/cmake-build-debug/examples/task7 && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/tangming/ImageBasedModellingEduV1.0/examples/task7/class7_meshclean.cc -o CMakeFiles/task7_mesh_clean.dir/class7_meshclean.cc.s

examples/task7/CMakeFiles/task7_mesh_clean.dir/class7_meshclean.cc.o.requires:

.PHONY : examples/task7/CMakeFiles/task7_mesh_clean.dir/class7_meshclean.cc.o.requires

examples/task7/CMakeFiles/task7_mesh_clean.dir/class7_meshclean.cc.o.provides: examples/task7/CMakeFiles/task7_mesh_clean.dir/class7_meshclean.cc.o.requires
	$(MAKE) -f examples/task7/CMakeFiles/task7_mesh_clean.dir/build.make examples/task7/CMakeFiles/task7_mesh_clean.dir/class7_meshclean.cc.o.provides.build
.PHONY : examples/task7/CMakeFiles/task7_mesh_clean.dir/class7_meshclean.cc.o.provides

examples/task7/CMakeFiles/task7_mesh_clean.dir/class7_meshclean.cc.o.provides.build: examples/task7/CMakeFiles/task7_mesh_clean.dir/class7_meshclean.cc.o


# Object files for target task7_mesh_clean
task7_mesh_clean_OBJECTS = \
"CMakeFiles/task7_mesh_clean.dir/class7_meshclean.cc.o"

# External object files for target task7_mesh_clean
task7_mesh_clean_EXTERNAL_OBJECTS =

examples/task7/task7_mesh_clean: examples/task7/CMakeFiles/task7_mesh_clean.dir/class7_meshclean.cc.o
examples/task7/task7_mesh_clean: examples/task7/CMakeFiles/task7_mesh_clean.dir/build.make
examples/task7/task7_mesh_clean: util/libutil.a
examples/task7/task7_mesh_clean: core/libcore.a
examples/task7/task7_mesh_clean: surface/libsurface.a
examples/task7/task7_mesh_clean: util/libutil.a
examples/task7/task7_mesh_clean: /usr/lib/x86_64-linux-gnu/libpng.so
examples/task7/task7_mesh_clean: /usr/lib/x86_64-linux-gnu/libz.so
examples/task7/task7_mesh_clean: /usr/lib/x86_64-linux-gnu/libjpeg.so
examples/task7/task7_mesh_clean: /usr/lib/x86_64-linux-gnu/libtiff.so
examples/task7/task7_mesh_clean: examples/task7/CMakeFiles/task7_mesh_clean.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/tangming/ImageBasedModellingEduV1.0/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable task7_mesh_clean"
	cd /home/tangming/ImageBasedModellingEduV1.0/cmake-build-debug/examples/task7 && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/task7_mesh_clean.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
examples/task7/CMakeFiles/task7_mesh_clean.dir/build: examples/task7/task7_mesh_clean

.PHONY : examples/task7/CMakeFiles/task7_mesh_clean.dir/build

examples/task7/CMakeFiles/task7_mesh_clean.dir/requires: examples/task7/CMakeFiles/task7_mesh_clean.dir/class7_meshclean.cc.o.requires

.PHONY : examples/task7/CMakeFiles/task7_mesh_clean.dir/requires

examples/task7/CMakeFiles/task7_mesh_clean.dir/clean:
	cd /home/tangming/ImageBasedModellingEduV1.0/cmake-build-debug/examples/task7 && $(CMAKE_COMMAND) -P CMakeFiles/task7_mesh_clean.dir/cmake_clean.cmake
.PHONY : examples/task7/CMakeFiles/task7_mesh_clean.dir/clean

examples/task7/CMakeFiles/task7_mesh_clean.dir/depend:
	cd /home/tangming/ImageBasedModellingEduV1.0/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/tangming/ImageBasedModellingEduV1.0 /home/tangming/ImageBasedModellingEduV1.0/examples/task7 /home/tangming/ImageBasedModellingEduV1.0/cmake-build-debug /home/tangming/ImageBasedModellingEduV1.0/cmake-build-debug/examples/task7 /home/tangming/ImageBasedModellingEduV1.0/cmake-build-debug/examples/task7/CMakeFiles/task7_mesh_clean.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : examples/task7/CMakeFiles/task7_mesh_clean.dir/depend

