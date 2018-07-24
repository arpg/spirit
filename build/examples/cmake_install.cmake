# Install script for directory: /home/boston/Documents/spirit_dep/spirit/examples

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "RelWithDebInfo")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/boston/Documents/spirit_dep/spirit/build/examples/car_robot/cmake_install.cmake")
  include("/home/boston/Documents/spirit_dep/spirit/build/examples/CarCalib/cmake_install.cmake")
  include("/home/boston/Documents/spirit_dep/spirit/build/examples/mpc_car/cmake_install.cmake")
  include("/home/boston/Documents/spirit_dep/spirit/build/examples/sim_car/cmake_install.cmake")
  include("/home/boston/Documents/spirit_dep/spirit/build/examples/shooting/cmake_install.cmake")
  include("/home/boston/Documents/spirit_dep/spirit/build/examples/bvp_example/cmake_install.cmake")
  include("/home/boston/Documents/spirit_dep/spirit/build/examples/gamepad_drive/cmake_install.cmake")
  include("/home/boston/Documents/spirit_dep/spirit/build/examples/rk4/cmake_install.cmake")
  include("/home/boston/Documents/spirit_dep/spirit/build/examples/rk4_car/cmake_install.cmake")
  include("/home/boston/Documents/spirit_dep/spirit/build/examples/ode_car_mpc/cmake_install.cmake")
  include("/home/boston/Documents/spirit_dep/spirit/build/examples/rk4_spirit_car/cmake_install.cmake")
  include("/home/boston/Documents/spirit_dep/spirit/build/examples/cost_surf/cmake_install.cmake")
  include("/home/boston/Documents/spirit_dep/spirit/build/examples/rk4_3D_car/cmake_install.cmake")

endif()

