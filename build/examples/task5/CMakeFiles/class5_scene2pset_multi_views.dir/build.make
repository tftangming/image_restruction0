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
CMAKE_BINARY_DIR = /home/tangming/ImageBasedModellingEduV1.0/build

# Include any dependencies generated for this target.
include examples/task5/CMakeFiles/class5_scene2pset_multi_views.dir/depend.make

# Include the progress variables for this target.
include examples/task5/CMakeFiles/class5_scene2pset_multi_views.dir/progress.make

# Include the compile flags for this target's objects.
include examples/task5/CMakeFiles/class5_scene2pset_multi_views.dir/flags.make

examples/task5/CMakeFiles/class5_scene2pset_multi_views.dir/class5_scene2pset_multi_views.cc.o: examples/task5/CMakeFiles/class5_scene2pset_multi_views.dir/flags.make
examples/task5/CMakeFiles/class5_scene2pset_multi_views.dir/class5_scene2pset_multi_views.cc.o: ../examples/task5/class5_scene2pset_multi_views.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/tangming/ImageBasedModellingEduV1.0/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object examples/task5/CMakeFiles/class5_scene2pset_multi_views.dir/class5_scene2pset_multi_views.cc.o"
	cd /home/tangming/ImageBasedModellingEduV1.0/build/examples/task5 && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/class5_scene2pset_multi_views.dir/class5_scene2pset_multi_views.cc.o -c /home/tangming/ImageBasedModellingEduV1.0/examples/task5/class5_scene2pset_multi_views.cc

examples/task5/CMakeFiles/class5_scene2pset_multi_views.dir/class5_scene2pset_multi_views.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/class5_scene2pset_multi_views.dir/class5_scene2pset_multi_views.cc.i"
	cd /home/tangming/ImageBasedModellingEduV1.0/build/examples/task5 && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/tangming/ImageBasedModellingEduV1.0/examples/task5/class5_scene2pset_multi_views.cc > CMakeFiles/class5_scene2pset_multi_views.dir/class5_scene2pset_multi_views.cc.i

examples/task5/CMakeFiles/class5_scene2pset_multi_views.dir/class5_scene2pset_multi_views.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/class5_scene2pset_multi_views.dir/class5_scene2pset_multi_views.cc.s"
	cd /home/tangming/ImageBasedModellingEduV1.0/build/examples/task5 && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/tangming/ImageBasedModellingEduV1.0/examples/task5/class5_scene2pset_multi_views.cc -o CMakeFiles/class5_scene2pset_multi_views.dir/class5_scene2pset_multi_views.cc.s

examples/task5/CMakeFiles/class5_scene2pset_multi_views.dir/class5_scene2pset_multi_views.cc.o.requires:

.PHONY : examples/task5/CMakeFiles/class5_scene2pset_multi_views.dir/class5_scene2pset_multi_views.cc.o.requires

examples/task5/CMakeFiles/class5_scene2pset_multi_views.dir/class5_scene2pset_multi_views.cc.o.provides: examples/task5/CMakeFiles/class5_scene2pset_multi_views.dir/class5_scene2pset_multi_views.cc.o.requires
	$(MAKE) -f examples/task5/CMakeFiles/class5_scene2pset_multi_views.dir/build.make examples/task5/CMakeFiles/class5_scene2pset_multi_views.dir/class5_scene2pset_multi_views.cc.o.provides.build
.PHONY : examples/task5/CMakeFiles/class5_scene2pset_multi_views.dir/class5_scene2pset_multi_views.cc.o.provides

examples/task5/CMakeFiles/class5_scene2pset_multi_views.dir/class5_scene2pset_multi_views.cc.o.provides.build: examples/task5/CMakeFiles/class5_scene2pset_multi_views.dir/class5_scene2pset_multi_views.cc.o


# Object files for target class5_scene2pset_multi_views
class5_scene2pset_multi_views_OBJECTS = \
"CMakeFiles/class5_scene2pset_multi_views.dir/class5_scene2pset_multi_views.cc.o"

# External object files for target class5_scene2pset_multi_views
class5_scene2pset_multi_views_EXTERNAL_OBJECTS =

examples/task5/class5_scene2pset_multi_views: examples/task5/CMakeFiles/class5_scene2pset_multi_views.dir/class5_scene2pset_multi_views.cc.o
examples/task5/class5_scene2pset_multi_views: examples/task5/CMakeFiles/class5_scene2pset_multi_views.dir/build.make
examples/task5/class5_scene2pset_multi_views: mvs/libmvs.a
examples/task5/class5_scene2pset_multi_views: util/libutil.a
examples/task5/class5_scene2pset_multi_views: core/libcore.a
examples/task5/class5_scene2pset_multi_views: util/libutil.a
examples/task5/class5_scene2pset_multi_views: /usr/lib/x86_64-linux-gnu/libpng.so
examples/task5/class5_scene2pset_multi_views: /usr/lib/x86_64-linux-gnu/libz.so
examples/task5/class5_scene2pset_multi_views: /usr/lib/x86_64-linux-gnu/libjpeg.so
examples/task5/class5_scene2pset_multi_views: /usr/lib/x86_64-linux-gnu/libtiff.so
examples/task5/class5_scene2pset_multi_views: examples/task5/CMakeFiles/class5_scene2pset_multi_views.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/tangming/ImageBasedModellingEduV1.0/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable class5_scene2pset_multi_views"
	cd /home/tangming/ImageBasedModellingEduV1.0/build/examples/task5 && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/class5_scene2pset_multi_views.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
examples/task5/CMakeFiles/class5_scene2pset_multi_views.dir/build: examples/task5/class5_scene2pset_multi_views

.PHONY : examples/task5/CMakeFiles/class5_scene2pset_multi_views.dir/build

examples/task5/CMakeFiles/class5_scene2pset_multi_views.dir/requires: examples/task5/CMakeFiles/class5_scene2pset_multi_views.dir/class5_scene2pset_multi_views.cc.o.requires

.PHONY : examples/task5/CMakeFiles/class5_scene2pset_multi_views.dir/requires

examples/task5/CMakeFiles/class5_scene2pset_multi_views.dir/clean:
	cd /home/tangming/ImageBasedModellingEduV1.0/build/examples/task5 && $(CMAKE_COMMAND) -P CMakeFiles/class5_scene2pset_multi_views.dir/cmake_clean.cmake
.PHONY : examples/task5/CMakeFiles/class5_scene2pset_multi_views.dir/clean

examples/task5/CMakeFiles/class5_scene2pset_multi_views.dir/depend:
	cd /home/tangming/ImageBasedModellingEduV1.0/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/tangming/ImageBasedModellingEduV1.0 /home/tangming/ImageBasedModellingEduV1.0/examples/task5 /home/tangming/ImageBasedModellingEduV1.0/build /home/tangming/ImageBasedModellingEduV1.0/build/examples/task5 /home/tangming/ImageBasedModellingEduV1.0/build/examples/task5/CMakeFiles/class5_scene2pset_multi_views.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : examples/task5/CMakeFiles/class5_scene2pset_multi_views.dir/depend

