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
include examples/cost_surf/CMakeFiles/cost_surf.dir/depend.make

# Include the progress variables for this target.
include examples/cost_surf/CMakeFiles/cost_surf.dir/progress.make

# Include the compile flags for this target's objects.
include examples/cost_surf/CMakeFiles/cost_surf.dir/flags.make

examples/cost_surf/CMakeFiles/cost_surf.dir/main.cpp.o: examples/cost_surf/CMakeFiles/cost_surf.dir/flags.make
examples/cost_surf/CMakeFiles/cost_surf.dir/main.cpp.o: ../examples/cost_surf/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/boston/Documents/spirit_dep/spirit/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object examples/cost_surf/CMakeFiles/cost_surf.dir/main.cpp.o"
	cd /home/boston/Documents/spirit_dep/spirit/build/examples/cost_surf && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/cost_surf.dir/main.cpp.o -c /home/boston/Documents/spirit_dep/spirit/examples/cost_surf/main.cpp

examples/cost_surf/CMakeFiles/cost_surf.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/cost_surf.dir/main.cpp.i"
	cd /home/boston/Documents/spirit_dep/spirit/build/examples/cost_surf && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/boston/Documents/spirit_dep/spirit/examples/cost_surf/main.cpp > CMakeFiles/cost_surf.dir/main.cpp.i

examples/cost_surf/CMakeFiles/cost_surf.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/cost_surf.dir/main.cpp.s"
	cd /home/boston/Documents/spirit_dep/spirit/build/examples/cost_surf && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/boston/Documents/spirit_dep/spirit/examples/cost_surf/main.cpp -o CMakeFiles/cost_surf.dir/main.cpp.s

examples/cost_surf/CMakeFiles/cost_surf.dir/main.cpp.o.requires:

.PHONY : examples/cost_surf/CMakeFiles/cost_surf.dir/main.cpp.o.requires

examples/cost_surf/CMakeFiles/cost_surf.dir/main.cpp.o.provides: examples/cost_surf/CMakeFiles/cost_surf.dir/main.cpp.o.requires
	$(MAKE) -f examples/cost_surf/CMakeFiles/cost_surf.dir/build.make examples/cost_surf/CMakeFiles/cost_surf.dir/main.cpp.o.provides.build
.PHONY : examples/cost_surf/CMakeFiles/cost_surf.dir/main.cpp.o.provides

examples/cost_surf/CMakeFiles/cost_surf.dir/main.cpp.o.provides.build: examples/cost_surf/CMakeFiles/cost_surf.dir/main.cpp.o


# Object files for target cost_surf
cost_surf_OBJECTS = \
"CMakeFiles/cost_surf.dir/main.cpp.o"

# External object files for target cost_surf
cost_surf_EXTERNAL_OBJECTS =

examples/cost_surf/cost_surf: examples/cost_surf/CMakeFiles/cost_surf.dir/main.cpp.o
examples/cost_surf/cost_surf: examples/cost_surf/CMakeFiles/cost_surf.dir/build.make
examples/cost_surf/cost_surf: /usr/lib/x86_64-linux-gnu/libtinyxml2.so
examples/cost_surf/cost_surf: /usr/lib/x86_64-linux-gnu/libdc1394.so
examples/cost_surf/cost_surf: /usr/lib/x86_64-linux-gnu/libraw1394.so
examples/cost_surf/cost_surf: /usr/lib/x86_64-linux-gnu/libopencv_core.so.2.4.9
examples/cost_surf/cost_surf: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.2.4.9
examples/cost_surf/cost_surf: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.2.4.9
examples/cost_surf/cost_surf: /usr/lib/x86_64-linux-gnu/libprotobuf.so
examples/cost_surf/cost_surf: /usr/lib/x86_64-linux-gnu/libglog.so
examples/cost_surf/cost_surf: /home/boston/Documents/spirit_dep/HAL/build/HAL/libhal.a
examples/cost_surf/cost_surf: /usr/lib/x86_64-linux-gnu/libGLU.so
examples/cost_surf/cost_surf: /usr/lib/x86_64-linux-gnu/libGL.so
examples/cost_surf/cost_surf: /usr/lib/x86_64-linux-gnu/libGLEW.so
examples/cost_surf/cost_surf: /usr/lib/x86_64-linux-gnu/libglut.so
examples/cost_surf/cost_surf: /usr/lib/x86_64-linux-gnu/libpng.so
examples/cost_surf/cost_surf: /usr/lib/x86_64-linux-gnu/libz.so
examples/cost_surf/cost_surf: /usr/lib/x86_64-linux-gnu/libjpeg.so
examples/cost_surf/cost_surf: /usr/lib/x86_64-linux-gnu/libtiff.so
examples/cost_surf/cost_surf: /home/boston/Documents/spirit_dep/Pangolin/build/libpangolin.so
examples/cost_surf/cost_surf: /usr/lib/x86_64-linux-gnu/libassimp.so
examples/cost_surf/cost_surf: /usr/lib/x86_64-linux-gnu/libIL.so
examples/cost_surf/cost_surf: /usr/lib/x86_64-linux-gnu/libILU.so
examples/cost_surf/cost_surf: /usr/lib/x86_64-linux-gnu/libILUT.so
examples/cost_surf/cost_surf: /home/boston/Documents/spirit_dep/SceneGraph/build/libscenegraph.so
examples/cost_surf/cost_surf: /usr/local/lib/libBulletDynamics.so
examples/cost_surf/cost_surf: /usr/local/lib/libBulletCollision.so
examples/cost_surf/cost_surf: /usr/local/lib/libLinearMath.so
examples/cost_surf/cost_surf: /usr/local/lib/libBulletSoftBody.so
examples/cost_surf/cost_surf: /usr/local/lib/libceres.so
examples/cost_surf/cost_surf: /usr/lib/x86_64-linux-gnu/libglog.so
examples/cost_surf/cost_surf: /usr/lib/x86_64-linux-gnu/libcholmod.so
examples/cost_surf/cost_surf: /usr/lib/x86_64-linux-gnu/libccolamd.so
examples/cost_surf/cost_surf: /usr/lib/x86_64-linux-gnu/libcamd.so
examples/cost_surf/cost_surf: /usr/lib/x86_64-linux-gnu/libcolamd.so
examples/cost_surf/cost_surf: /usr/lib/x86_64-linux-gnu/libamd.so
examples/cost_surf/cost_surf: /usr/lib/x86_64-linux-gnu/libsuitesparseconfig.so
examples/cost_surf/cost_surf: /usr/lib/liblapack.so
examples/cost_surf/cost_surf: /usr/lib/libblas.so
examples/cost_surf/cost_surf: /usr/lib/x86_64-linux-gnu/libcxsparse.so
examples/cost_surf/cost_surf: /usr/lib/x86_64-linux-gnu/libprotobuf.so
examples/cost_surf/cost_surf: /usr/local/lib/libosg.so
examples/cost_surf/cost_surf: /usr/local/lib/libosgViewer.so
examples/cost_surf/cost_surf: /usr/local/lib/libosgUtil.so
examples/cost_surf/cost_surf: /usr/local/lib/libosgDB.so
examples/cost_surf/cost_surf: /usr/local/lib/libosgGA.so
examples/cost_surf/cost_surf: /usr/local/lib/libOpenThreads.so
examples/cost_surf/cost_surf: libspirit.so
examples/cost_surf/cost_surf: /usr/lib/x86_64-linux-gnu/libglog.so
examples/cost_surf/cost_surf: /home/boston/Documents/spirit_dep/HAL/build/HAL/libhal.a
examples/cost_surf/cost_surf: /usr/lib/x86_64-linux-gnu/libGLU.so
examples/cost_surf/cost_surf: /usr/lib/x86_64-linux-gnu/libGL.so
examples/cost_surf/cost_surf: /usr/lib/x86_64-linux-gnu/libGLEW.so
examples/cost_surf/cost_surf: /usr/lib/x86_64-linux-gnu/libglut.so
examples/cost_surf/cost_surf: /usr/lib/x86_64-linux-gnu/libpng.so
examples/cost_surf/cost_surf: /usr/lib/x86_64-linux-gnu/libz.so
examples/cost_surf/cost_surf: /usr/lib/x86_64-linux-gnu/libjpeg.so
examples/cost_surf/cost_surf: /usr/lib/x86_64-linux-gnu/libtiff.so
examples/cost_surf/cost_surf: /home/boston/Documents/spirit_dep/Pangolin/build/libpangolin.so
examples/cost_surf/cost_surf: /usr/lib/x86_64-linux-gnu/libassimp.so
examples/cost_surf/cost_surf: /usr/lib/x86_64-linux-gnu/libIL.so
examples/cost_surf/cost_surf: /usr/lib/x86_64-linux-gnu/libILU.so
examples/cost_surf/cost_surf: /usr/lib/x86_64-linux-gnu/libILUT.so
examples/cost_surf/cost_surf: /home/boston/Documents/spirit_dep/SceneGraph/build/libscenegraph.so
examples/cost_surf/cost_surf: /usr/local/lib/libBulletDynamics.so
examples/cost_surf/cost_surf: /usr/local/lib/libBulletCollision.so
examples/cost_surf/cost_surf: /usr/local/lib/libLinearMath.so
examples/cost_surf/cost_surf: /usr/local/lib/libBulletSoftBody.so
examples/cost_surf/cost_surf: /usr/local/lib/libceres.so
examples/cost_surf/cost_surf: /usr/lib/x86_64-linux-gnu/libcholmod.so
examples/cost_surf/cost_surf: /usr/lib/x86_64-linux-gnu/libccolamd.so
examples/cost_surf/cost_surf: /usr/lib/x86_64-linux-gnu/libcamd.so
examples/cost_surf/cost_surf: /usr/lib/x86_64-linux-gnu/libcolamd.so
examples/cost_surf/cost_surf: /usr/lib/x86_64-linux-gnu/libamd.so
examples/cost_surf/cost_surf: /usr/lib/x86_64-linux-gnu/libsuitesparseconfig.so
examples/cost_surf/cost_surf: /usr/lib/liblapack.so
examples/cost_surf/cost_surf: /usr/lib/libblas.so
examples/cost_surf/cost_surf: /usr/lib/x86_64-linux-gnu/libcxsparse.so
examples/cost_surf/cost_surf: /usr/local/lib/libosg.so
examples/cost_surf/cost_surf: /usr/local/lib/libosgViewer.so
examples/cost_surf/cost_surf: /usr/local/lib/libosgUtil.so
examples/cost_surf/cost_surf: /usr/local/lib/libosgDB.so
examples/cost_surf/cost_surf: /usr/local/lib/libosgGA.so
examples/cost_surf/cost_surf: /usr/local/lib/libOpenThreads.so
examples/cost_surf/cost_surf: libspirit.so
examples/cost_surf/cost_surf: examples/cost_surf/CMakeFiles/cost_surf.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/boston/Documents/spirit_dep/spirit/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable cost_surf"
	cd /home/boston/Documents/spirit_dep/spirit/build/examples/cost_surf && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/cost_surf.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
examples/cost_surf/CMakeFiles/cost_surf.dir/build: examples/cost_surf/cost_surf

.PHONY : examples/cost_surf/CMakeFiles/cost_surf.dir/build

examples/cost_surf/CMakeFiles/cost_surf.dir/requires: examples/cost_surf/CMakeFiles/cost_surf.dir/main.cpp.o.requires

.PHONY : examples/cost_surf/CMakeFiles/cost_surf.dir/requires

examples/cost_surf/CMakeFiles/cost_surf.dir/clean:
	cd /home/boston/Documents/spirit_dep/spirit/build/examples/cost_surf && $(CMAKE_COMMAND) -P CMakeFiles/cost_surf.dir/cmake_clean.cmake
.PHONY : examples/cost_surf/CMakeFiles/cost_surf.dir/clean

examples/cost_surf/CMakeFiles/cost_surf.dir/depend:
	cd /home/boston/Documents/spirit_dep/spirit/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/boston/Documents/spirit_dep/spirit /home/boston/Documents/spirit_dep/spirit/examples/cost_surf /home/boston/Documents/spirit_dep/spirit/build /home/boston/Documents/spirit_dep/spirit/build/examples/cost_surf /home/boston/Documents/spirit_dep/spirit/build/examples/cost_surf/CMakeFiles/cost_surf.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : examples/cost_surf/CMakeFiles/cost_surf.dir/depend
