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
CMAKE_SOURCE_DIR = /home/boston/Documents/spirit_dep/spirit

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/boston/Documents/spirit_dep/spirit/build

# Include any dependencies generated for this target.
include examples/bvp_example/CMakeFiles/bvp_example.dir/depend.make

# Include the progress variables for this target.
include examples/bvp_example/CMakeFiles/bvp_example.dir/progress.make

# Include the compile flags for this target's objects.
include examples/bvp_example/CMakeFiles/bvp_example.dir/flags.make

examples/bvp_example/CMakeFiles/bvp_example.dir/main.cpp.o: examples/bvp_example/CMakeFiles/bvp_example.dir/flags.make
examples/bvp_example/CMakeFiles/bvp_example.dir/main.cpp.o: ../examples/bvp_example/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/boston/Documents/spirit_dep/spirit/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object examples/bvp_example/CMakeFiles/bvp_example.dir/main.cpp.o"
	cd /home/boston/Documents/spirit_dep/spirit/build/examples/bvp_example && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/bvp_example.dir/main.cpp.o -c /home/boston/Documents/spirit_dep/spirit/examples/bvp_example/main.cpp

examples/bvp_example/CMakeFiles/bvp_example.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/bvp_example.dir/main.cpp.i"
	cd /home/boston/Documents/spirit_dep/spirit/build/examples/bvp_example && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/boston/Documents/spirit_dep/spirit/examples/bvp_example/main.cpp > CMakeFiles/bvp_example.dir/main.cpp.i

examples/bvp_example/CMakeFiles/bvp_example.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/bvp_example.dir/main.cpp.s"
	cd /home/boston/Documents/spirit_dep/spirit/build/examples/bvp_example && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/boston/Documents/spirit_dep/spirit/examples/bvp_example/main.cpp -o CMakeFiles/bvp_example.dir/main.cpp.s

examples/bvp_example/CMakeFiles/bvp_example.dir/main.cpp.o.requires:

.PHONY : examples/bvp_example/CMakeFiles/bvp_example.dir/main.cpp.o.requires

examples/bvp_example/CMakeFiles/bvp_example.dir/main.cpp.o.provides: examples/bvp_example/CMakeFiles/bvp_example.dir/main.cpp.o.requires
	$(MAKE) -f examples/bvp_example/CMakeFiles/bvp_example.dir/build.make examples/bvp_example/CMakeFiles/bvp_example.dir/main.cpp.o.provides.build
.PHONY : examples/bvp_example/CMakeFiles/bvp_example.dir/main.cpp.o.provides

examples/bvp_example/CMakeFiles/bvp_example.dir/main.cpp.o.provides.build: examples/bvp_example/CMakeFiles/bvp_example.dir/main.cpp.o


# Object files for target bvp_example
bvp_example_OBJECTS = \
"CMakeFiles/bvp_example.dir/main.cpp.o"

# External object files for target bvp_example
bvp_example_EXTERNAL_OBJECTS =

examples/bvp_example/bvp_example: examples/bvp_example/CMakeFiles/bvp_example.dir/main.cpp.o
examples/bvp_example/bvp_example: examples/bvp_example/CMakeFiles/bvp_example.dir/build.make
examples/bvp_example/bvp_example: /usr/lib/x86_64-linux-gnu/libGLU.so
examples/bvp_example/bvp_example: /usr/lib/x86_64-linux-gnu/libGL.so
examples/bvp_example/bvp_example: /usr/lib/x86_64-linux-gnu/libGLEW.so
examples/bvp_example/bvp_example: /usr/lib/x86_64-linux-gnu/libglut.so
examples/bvp_example/bvp_example: /usr/lib/x86_64-linux-gnu/libpng.so
examples/bvp_example/bvp_example: /usr/lib/x86_64-linux-gnu/libz.so
examples/bvp_example/bvp_example: /usr/lib/x86_64-linux-gnu/libjpeg.so
examples/bvp_example/bvp_example: /usr/lib/x86_64-linux-gnu/libtiff.so
examples/bvp_example/bvp_example: /home/boston/Documents/spirit_dep/Pangolin/build/libpangolin.so
examples/bvp_example/bvp_example: /usr/lib/x86_64-linux-gnu/libassimp.so
examples/bvp_example/bvp_example: /usr/lib/x86_64-linux-gnu/libIL.so
examples/bvp_example/bvp_example: /usr/lib/x86_64-linux-gnu/libILU.so
examples/bvp_example/bvp_example: /usr/lib/x86_64-linux-gnu/libILUT.so
examples/bvp_example/bvp_example: /home/boston/Documents/spirit_dep/SceneGraph/build/libscenegraph.so
examples/bvp_example/bvp_example: /usr/local/lib/libBulletDynamics.so
examples/bvp_example/bvp_example: /usr/local/lib/libBulletCollision.so
examples/bvp_example/bvp_example: /usr/local/lib/libLinearMath.so
examples/bvp_example/bvp_example: /usr/local/lib/libBulletSoftBody.so
examples/bvp_example/bvp_example: /usr/local/lib/libceres.so
examples/bvp_example/bvp_example: /usr/lib/x86_64-linux-gnu/libglog.so
examples/bvp_example/bvp_example: /usr/lib/x86_64-linux-gnu/libcholmod.so
examples/bvp_example/bvp_example: /usr/lib/x86_64-linux-gnu/libccolamd.so
examples/bvp_example/bvp_example: /usr/lib/x86_64-linux-gnu/libcamd.so
examples/bvp_example/bvp_example: /usr/lib/x86_64-linux-gnu/libcolamd.so
examples/bvp_example/bvp_example: /usr/lib/x86_64-linux-gnu/libamd.so
examples/bvp_example/bvp_example: /usr/lib/x86_64-linux-gnu/libsuitesparseconfig.so
examples/bvp_example/bvp_example: /usr/lib/liblapack.so
examples/bvp_example/bvp_example: /usr/lib/libblas.so
examples/bvp_example/bvp_example: /usr/lib/x86_64-linux-gnu/libcxsparse.so
examples/bvp_example/bvp_example: /usr/lib/x86_64-linux-gnu/libprotobuf.so
examples/bvp_example/bvp_example: /usr/local/lib/libosg.so
examples/bvp_example/bvp_example: /usr/local/lib/libosgViewer.so
examples/bvp_example/bvp_example: /usr/local/lib/libosgUtil.so
examples/bvp_example/bvp_example: /usr/local/lib/libosgDB.so
examples/bvp_example/bvp_example: /usr/local/lib/libosgGA.so
examples/bvp_example/bvp_example: /usr/local/lib/libOpenThreads.so
examples/bvp_example/bvp_example: libspirit.so
examples/bvp_example/bvp_example: examples/bvp_example/CMakeFiles/bvp_example.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/boston/Documents/spirit_dep/spirit/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable bvp_example"
	cd /home/boston/Documents/spirit_dep/spirit/build/examples/bvp_example && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/bvp_example.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
examples/bvp_example/CMakeFiles/bvp_example.dir/build: examples/bvp_example/bvp_example

.PHONY : examples/bvp_example/CMakeFiles/bvp_example.dir/build

examples/bvp_example/CMakeFiles/bvp_example.dir/requires: examples/bvp_example/CMakeFiles/bvp_example.dir/main.cpp.o.requires

.PHONY : examples/bvp_example/CMakeFiles/bvp_example.dir/requires

examples/bvp_example/CMakeFiles/bvp_example.dir/clean:
	cd /home/boston/Documents/spirit_dep/spirit/build/examples/bvp_example && $(CMAKE_COMMAND) -P CMakeFiles/bvp_example.dir/cmake_clean.cmake
.PHONY : examples/bvp_example/CMakeFiles/bvp_example.dir/clean

examples/bvp_example/CMakeFiles/bvp_example.dir/depend:
	cd /home/boston/Documents/spirit_dep/spirit/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/boston/Documents/spirit_dep/spirit /home/boston/Documents/spirit_dep/spirit/examples/bvp_example /home/boston/Documents/spirit_dep/spirit/build /home/boston/Documents/spirit_dep/spirit/build/examples/bvp_example /home/boston/Documents/spirit_dep/spirit/build/examples/bvp_example/CMakeFiles/bvp_example.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : examples/bvp_example/CMakeFiles/bvp_example.dir/depend

