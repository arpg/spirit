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
include examples/gamepad_drive/CMakeFiles/gamepad_drive.dir/depend.make

# Include the progress variables for this target.
include examples/gamepad_drive/CMakeFiles/gamepad_drive.dir/progress.make

# Include the compile flags for this target's objects.
include examples/gamepad_drive/CMakeFiles/gamepad_drive.dir/flags.make

examples/gamepad_drive/CMakeFiles/gamepad_drive.dir/main.cpp.o: examples/gamepad_drive/CMakeFiles/gamepad_drive.dir/flags.make
examples/gamepad_drive/CMakeFiles/gamepad_drive.dir/main.cpp.o: ../examples/gamepad_drive/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/boston/Documents/spirit_dep/spirit/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object examples/gamepad_drive/CMakeFiles/gamepad_drive.dir/main.cpp.o"
	cd /home/boston/Documents/spirit_dep/spirit/build/examples/gamepad_drive && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/gamepad_drive.dir/main.cpp.o -c /home/boston/Documents/spirit_dep/spirit/examples/gamepad_drive/main.cpp

examples/gamepad_drive/CMakeFiles/gamepad_drive.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/gamepad_drive.dir/main.cpp.i"
	cd /home/boston/Documents/spirit_dep/spirit/build/examples/gamepad_drive && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/boston/Documents/spirit_dep/spirit/examples/gamepad_drive/main.cpp > CMakeFiles/gamepad_drive.dir/main.cpp.i

examples/gamepad_drive/CMakeFiles/gamepad_drive.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/gamepad_drive.dir/main.cpp.s"
	cd /home/boston/Documents/spirit_dep/spirit/build/examples/gamepad_drive && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/boston/Documents/spirit_dep/spirit/examples/gamepad_drive/main.cpp -o CMakeFiles/gamepad_drive.dir/main.cpp.s

examples/gamepad_drive/CMakeFiles/gamepad_drive.dir/main.cpp.o.requires:

.PHONY : examples/gamepad_drive/CMakeFiles/gamepad_drive.dir/main.cpp.o.requires

examples/gamepad_drive/CMakeFiles/gamepad_drive.dir/main.cpp.o.provides: examples/gamepad_drive/CMakeFiles/gamepad_drive.dir/main.cpp.o.requires
	$(MAKE) -f examples/gamepad_drive/CMakeFiles/gamepad_drive.dir/build.make examples/gamepad_drive/CMakeFiles/gamepad_drive.dir/main.cpp.o.provides.build
.PHONY : examples/gamepad_drive/CMakeFiles/gamepad_drive.dir/main.cpp.o.provides

examples/gamepad_drive/CMakeFiles/gamepad_drive.dir/main.cpp.o.provides.build: examples/gamepad_drive/CMakeFiles/gamepad_drive.dir/main.cpp.o


# Object files for target gamepad_drive
gamepad_drive_OBJECTS = \
"CMakeFiles/gamepad_drive.dir/main.cpp.o"

# External object files for target gamepad_drive
gamepad_drive_EXTERNAL_OBJECTS =

examples/gamepad_drive/gamepad_drive: examples/gamepad_drive/CMakeFiles/gamepad_drive.dir/main.cpp.o
examples/gamepad_drive/gamepad_drive: examples/gamepad_drive/CMakeFiles/gamepad_drive.dir/build.make
examples/gamepad_drive/gamepad_drive: /usr/lib/x86_64-linux-gnu/libtinyxml2.so
examples/gamepad_drive/gamepad_drive: /usr/lib/x86_64-linux-gnu/libdc1394.so
examples/gamepad_drive/gamepad_drive: /usr/lib/x86_64-linux-gnu/libraw1394.so
examples/gamepad_drive/gamepad_drive: /usr/lib/x86_64-linux-gnu/libopencv_core.so.2.4.9
examples/gamepad_drive/gamepad_drive: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.2.4.9
examples/gamepad_drive/gamepad_drive: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.2.4.9
examples/gamepad_drive/gamepad_drive: /usr/lib/x86_64-linux-gnu/libprotobuf.so
examples/gamepad_drive/gamepad_drive: /usr/lib/x86_64-linux-gnu/libglog.so
examples/gamepad_drive/gamepad_drive: /home/boston/Documents/spirit_dep/HAL/build/HAL/libhal.a
examples/gamepad_drive/gamepad_drive: /usr/lib/x86_64-linux-gnu/libGLU.so
examples/gamepad_drive/gamepad_drive: /usr/lib/x86_64-linux-gnu/libGL.so
examples/gamepad_drive/gamepad_drive: /usr/lib/x86_64-linux-gnu/libGLEW.so
examples/gamepad_drive/gamepad_drive: /usr/lib/x86_64-linux-gnu/libglut.so
examples/gamepad_drive/gamepad_drive: /usr/lib/x86_64-linux-gnu/libpng.so
examples/gamepad_drive/gamepad_drive: /usr/lib/x86_64-linux-gnu/libz.so
examples/gamepad_drive/gamepad_drive: /usr/lib/x86_64-linux-gnu/libjpeg.so
examples/gamepad_drive/gamepad_drive: /usr/lib/x86_64-linux-gnu/libtiff.so
examples/gamepad_drive/gamepad_drive: /home/boston/Documents/spirit_dep/Pangolin/build/libpangolin.so
examples/gamepad_drive/gamepad_drive: /usr/lib/x86_64-linux-gnu/libassimp.so
examples/gamepad_drive/gamepad_drive: /usr/lib/x86_64-linux-gnu/libIL.so
examples/gamepad_drive/gamepad_drive: /usr/lib/x86_64-linux-gnu/libILU.so
examples/gamepad_drive/gamepad_drive: /usr/lib/x86_64-linux-gnu/libILUT.so
examples/gamepad_drive/gamepad_drive: /home/boston/Documents/spirit_dep/SceneGraph/build/libscenegraph.so
examples/gamepad_drive/gamepad_drive: /usr/local/lib/libBulletDynamics.so
examples/gamepad_drive/gamepad_drive: /usr/local/lib/libBulletCollision.so
examples/gamepad_drive/gamepad_drive: /usr/local/lib/libLinearMath.so
examples/gamepad_drive/gamepad_drive: /usr/local/lib/libBulletSoftBody.so
examples/gamepad_drive/gamepad_drive: /usr/local/lib/libceres.so
examples/gamepad_drive/gamepad_drive: /usr/lib/x86_64-linux-gnu/libglog.so
examples/gamepad_drive/gamepad_drive: /usr/lib/x86_64-linux-gnu/libcholmod.so
examples/gamepad_drive/gamepad_drive: /usr/lib/x86_64-linux-gnu/libccolamd.so
examples/gamepad_drive/gamepad_drive: /usr/lib/x86_64-linux-gnu/libcamd.so
examples/gamepad_drive/gamepad_drive: /usr/lib/x86_64-linux-gnu/libcolamd.so
examples/gamepad_drive/gamepad_drive: /usr/lib/x86_64-linux-gnu/libamd.so
examples/gamepad_drive/gamepad_drive: /usr/lib/x86_64-linux-gnu/libsuitesparseconfig.so
examples/gamepad_drive/gamepad_drive: /usr/lib/liblapack.so
examples/gamepad_drive/gamepad_drive: /usr/lib/libblas.so
examples/gamepad_drive/gamepad_drive: /usr/lib/x86_64-linux-gnu/libcxsparse.so
examples/gamepad_drive/gamepad_drive: /usr/lib/x86_64-linux-gnu/libprotobuf.so
examples/gamepad_drive/gamepad_drive: /usr/local/lib/libosg.so
examples/gamepad_drive/gamepad_drive: /usr/local/lib/libosgViewer.so
examples/gamepad_drive/gamepad_drive: /usr/local/lib/libosgUtil.so
examples/gamepad_drive/gamepad_drive: /usr/local/lib/libosgDB.so
examples/gamepad_drive/gamepad_drive: /usr/local/lib/libosgGA.so
examples/gamepad_drive/gamepad_drive: /usr/local/lib/libOpenThreads.so
examples/gamepad_drive/gamepad_drive: libspirit.so
examples/gamepad_drive/gamepad_drive: /usr/lib/x86_64-linux-gnu/libglog.so
examples/gamepad_drive/gamepad_drive: /home/boston/Documents/spirit_dep/HAL/build/HAL/libhal.a
examples/gamepad_drive/gamepad_drive: /usr/lib/x86_64-linux-gnu/libGLU.so
examples/gamepad_drive/gamepad_drive: /usr/lib/x86_64-linux-gnu/libGL.so
examples/gamepad_drive/gamepad_drive: /usr/lib/x86_64-linux-gnu/libGLEW.so
examples/gamepad_drive/gamepad_drive: /usr/lib/x86_64-linux-gnu/libglut.so
examples/gamepad_drive/gamepad_drive: /usr/lib/x86_64-linux-gnu/libpng.so
examples/gamepad_drive/gamepad_drive: /usr/lib/x86_64-linux-gnu/libz.so
examples/gamepad_drive/gamepad_drive: /usr/lib/x86_64-linux-gnu/libjpeg.so
examples/gamepad_drive/gamepad_drive: /usr/lib/x86_64-linux-gnu/libtiff.so
examples/gamepad_drive/gamepad_drive: /home/boston/Documents/spirit_dep/Pangolin/build/libpangolin.so
examples/gamepad_drive/gamepad_drive: /usr/lib/x86_64-linux-gnu/libassimp.so
examples/gamepad_drive/gamepad_drive: /usr/lib/x86_64-linux-gnu/libIL.so
examples/gamepad_drive/gamepad_drive: /usr/lib/x86_64-linux-gnu/libILU.so
examples/gamepad_drive/gamepad_drive: /usr/lib/x86_64-linux-gnu/libILUT.so
examples/gamepad_drive/gamepad_drive: /home/boston/Documents/spirit_dep/SceneGraph/build/libscenegraph.so
examples/gamepad_drive/gamepad_drive: /usr/local/lib/libBulletDynamics.so
examples/gamepad_drive/gamepad_drive: /usr/local/lib/libBulletCollision.so
examples/gamepad_drive/gamepad_drive: /usr/local/lib/libLinearMath.so
examples/gamepad_drive/gamepad_drive: /usr/local/lib/libBulletSoftBody.so
examples/gamepad_drive/gamepad_drive: /usr/local/lib/libceres.so
examples/gamepad_drive/gamepad_drive: /usr/lib/x86_64-linux-gnu/libcholmod.so
examples/gamepad_drive/gamepad_drive: /usr/lib/x86_64-linux-gnu/libccolamd.so
examples/gamepad_drive/gamepad_drive: /usr/lib/x86_64-linux-gnu/libcamd.so
examples/gamepad_drive/gamepad_drive: /usr/lib/x86_64-linux-gnu/libcolamd.so
examples/gamepad_drive/gamepad_drive: /usr/lib/x86_64-linux-gnu/libamd.so
examples/gamepad_drive/gamepad_drive: /usr/lib/x86_64-linux-gnu/libsuitesparseconfig.so
examples/gamepad_drive/gamepad_drive: /usr/lib/liblapack.so
examples/gamepad_drive/gamepad_drive: /usr/lib/libblas.so
examples/gamepad_drive/gamepad_drive: /usr/lib/x86_64-linux-gnu/libcxsparse.so
examples/gamepad_drive/gamepad_drive: /usr/local/lib/libosg.so
examples/gamepad_drive/gamepad_drive: /usr/local/lib/libosgViewer.so
examples/gamepad_drive/gamepad_drive: /usr/local/lib/libosgUtil.so
examples/gamepad_drive/gamepad_drive: /usr/local/lib/libosgDB.so
examples/gamepad_drive/gamepad_drive: /usr/local/lib/libosgGA.so
examples/gamepad_drive/gamepad_drive: /usr/local/lib/libOpenThreads.so
examples/gamepad_drive/gamepad_drive: libspirit.so
examples/gamepad_drive/gamepad_drive: examples/gamepad_drive/CMakeFiles/gamepad_drive.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/boston/Documents/spirit_dep/spirit/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable gamepad_drive"
	cd /home/boston/Documents/spirit_dep/spirit/build/examples/gamepad_drive && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/gamepad_drive.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
examples/gamepad_drive/CMakeFiles/gamepad_drive.dir/build: examples/gamepad_drive/gamepad_drive

.PHONY : examples/gamepad_drive/CMakeFiles/gamepad_drive.dir/build

examples/gamepad_drive/CMakeFiles/gamepad_drive.dir/requires: examples/gamepad_drive/CMakeFiles/gamepad_drive.dir/main.cpp.o.requires

.PHONY : examples/gamepad_drive/CMakeFiles/gamepad_drive.dir/requires

examples/gamepad_drive/CMakeFiles/gamepad_drive.dir/clean:
	cd /home/boston/Documents/spirit_dep/spirit/build/examples/gamepad_drive && $(CMAKE_COMMAND) -P CMakeFiles/gamepad_drive.dir/cmake_clean.cmake
.PHONY : examples/gamepad_drive/CMakeFiles/gamepad_drive.dir/clean

examples/gamepad_drive/CMakeFiles/gamepad_drive.dir/depend:
	cd /home/boston/Documents/spirit_dep/spirit/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/boston/Documents/spirit_dep/spirit /home/boston/Documents/spirit_dep/spirit/examples/gamepad_drive /home/boston/Documents/spirit_dep/spirit/build /home/boston/Documents/spirit_dep/spirit/build/examples/gamepad_drive /home/boston/Documents/spirit_dep/spirit/build/examples/gamepad_drive/CMakeFiles/gamepad_drive.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : examples/gamepad_drive/CMakeFiles/gamepad_drive.dir/depend

