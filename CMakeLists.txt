cmake_minimum_required(VERSION 2.8.10)
project(Spirit)
set( VERSION 0.1 )

if( NOT CMAKE_BUILD_TYPE )
    set(CMAKE_BUILD_TYPE Debug)
endif()

if(POLICY CMP0042)
    cmake_policy(SET CMP0042 NEW) # MACOSX_RPATH
endif()


# Include local cmake modules directory.
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/")
include(install_package)

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
find_package(GLog REQUIRED)
find_package(GFlags REQUIRED)

set( _INCLUDE_DIRS
        ${CMAKE_BINARY_DIR}
        ${EIGEN3_INCLUDE_DIR}
        ${Sophus_INCLUDE_DIR}
        ${Pangolin_INCLUDE_DIRS}
        ${SceneGraph_INCLUDE_DIRS}
        ${CarPlanner_INCLUDE_DIRS}
        ${ASSIMP_INCLUDE_DIRS}
        ${GFLags_INCLUDE_DIRS}
        ${GLog_INCLUDE_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        )

set( _LIBRARIES
        ${EIGEN3_LIBRARIES}
        ${Sophus_LIBRARIES}
        ${Pangolin_LIBRARIES}
        ${SceneGraph_LIBRARIES}
        ${CarPlanner_LIBRARIES}
        ${ASSIMP_LIBRARIES}
        ${GFlags_LIBRARIES}
        ${GLog_LIBRARIES}
        )

include_directories( ${_INCLUDE_DIRS} )

set( HDRS
        include/spirit/Gui.h
        include/spirit/objects/SpiritAxes.h
        include/spirit/objects/SpiritWaypoints.h
        include/spirit/objects/CommonObj.h
        include/spirit/objects/SpiritStaticTerrain.h
        include/spirit/objects/SpiritCars.h
        include/spirit/objects/GLCar.h
        )
set( SRCS
        src/Gui.cpp
        src/SpiritAxes.cpp
        src/SpiritWaypoints.cpp
        src/SpiritStaticTerrain.cpp
        src/SpiritCars.cpp
        )

set( LIBRARY_NAME ${PROJECT_NAME} )
string( TOLOWER ${LIBRARY_NAME} LIBRARY_NAME )

add_library( ${LIBRARY_NAME} SHARED ${SRCS} ${HDRS} )
target_link_libraries( ${LIBRARY_NAME} ${_LIBRARIES} )

install_package(
  PKG_NAME ${PROJECT_NAME}
  LIB_NAME ${LIBRARY_NAME}
  VERSION ${VERSION}
  DESCRIPTION "CarPlanner GUI"
  #INSTALL_INCLUDE_DIR true
  DESTINATION ${CMAKE_INSTALL_PREFIX}
  INCLUDE_DIRS ${_INCLUDE_DIRS}
  LINK_LIBS ${_LIBRARIES}
)

if(BUILD_APPLICATIONS)
  set(Spirit_LIBRARIES ${_LIBRARIES} spirit )
  set(Spirit_INCLUDE_DIRS ${_INCLUDE_DIRS} )
  set(Spirit_DIR ${CMAKE_CURRENT_BINARY_DIR} )
  add_subdirectory( applications )
endif()

########################################
include( ${CMAKE_MODULE_PATH}/cmake_uninstall.cmake.in)

add_custom_target(uninstall
    COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)


