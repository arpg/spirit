cmake_minimum_required(VERSION 2.8.10)
project(SPIRIT)

if( NOT CMAKE_BUILD_TYPE )
    set(CMAKE_BUILD_TYPE Release)
endif()

# Include local cmake modules directory.
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/")
include(def_library)
include(export_package)

# Default flags.
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall") #-DBOOST_NO_CXX11_NUMERIC_LIMITS")

IF(${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++")
ENDIF()

option(BUILD_APPLICATIONS "Build Applications" ON)

find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)
find_package(Pangolin 0.1 REQUIRED)
find_package(SceneGraph 0.1 REQUIRED)
find_package(CarPlanner REQUIRED)
find_package(Assimp REQUIRED)
find_package(TINYXML2 REQUIRED)


set( _INCLUDE_DIRS
        ${CMAKE_BINARY_DIR}
        ${EIGEN3_INCLUDE_DIR}
        ${Sophus_INCLUDE_DIR}
        ${Pangolin_INCLUDE_DIRS}
        ${SceneGraph_INCLUDE_DIRS}
        ${CarPlanner_INCLUDE_DIRS}
        ${TINYXML2_INCLUDE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        )
set( _LIBRARIES
        ${EIGEN3_LIBRARIES}
        ${Sophus_LIBRARIES}
        ${Pangolin_LIBRARIES}
        ${SceneGraph_LIBRARIES}
        ${CarPlanner_LIBRARIES}
        ${TINYXML2_LIBRARY}
        assimp
        )

include_directories( ${_INCLUDE_DIRS} )

set( HDRS
        include/spirit/Gui.h
        )
set( SRCS
        src/Gui.cpp
        )

if(BUILD_APPLICATIONS)
  add_subdirectory( applications )
endif()

set( LIBRARY_NAME ${PROJECT_NAME} )

def_library(${LIBRARY_NAME}
  SOURCES ${HDRS} ${SRCS}
  LINK_LIBS ${_LIBRARIES}
  PACKAGE ${PROJECT_NAME}
  )

export_package( ${PROJECT_NAME}
  TARGETS ${LIBRARY_NAME}
  LIBRARIES ${_LIBRARIES}
  INCLUDE_DIRS ${_INCLUDE_DIRS}
  )
