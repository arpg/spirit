cmake_minimum_required(VERSION 2.8.10)
project(spirit)
set( VERSION 0.1 ) 

if( NOT CMAKE_BUILD_TYPE )
    set(CMAKE_BUILD_TYPE RelWithDebInfo)
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

find_package( Pangolin 0.1 REQUIRED )
find_package( SceneGraph 0.1 REQUIRED )


set( _INCLUDE_DIRS
	${CMAKE_BINARY_DIR}
	${Pangolin_INCLUDE_DIRS}
	${SceneGraph_INCLUDE_DIRS}
	${CMAKE_CURRENT_SOURCE_DIR}/include
	)
set( _LIBRARIES
	${Pangolin_LIBRARIES}
	${SceneGraph_LIBRARIES}
	)


include_directories( ${_INCLUDE_DIRS} )

set( HDRS
	include/spirit/World.h
	include/spirit/Controller.h
	include/spirit/Gui.h
	include/spirit/Physics.h
	include/spirit/Planner.h
	include/spirit/Controllers/spCommonController.h
	include/spirit/Controllers/spMPC.h
	include/spirit/Gui/spCommonGui.h
	include/spirit/Gui/spPangolinScenegraphGui.h
	include/spirit/Physics/spBulletPhysicsInterface.h
	include/spirit/Physics/spCommonPhysics.h
	include/spirit/Planners/spAStar.h
	include/spirit/Planners/spCommonPlanner.h
	include/spirit/GeneralTools.h
	include/spirit/Settings.h
	)
set( SRCS
	src/World.cpp
	src/Controller.cpp
	src/Gui.cpp
	src/Physics.cpp
	src/Planner.cpp
	src/spMPC.cpp
	src/spPangolinScenegraphGui.cpp
	src/spBulletPhysicsInterface.cpp
	src/spAStar.cpp
	src/GeneralTools.cpp
	src/Settings.cpp
	)

option(BUILD_Examples "Build Examples" ON)

if(BUILD_Examples)
	add_subdirectory( examples )
endif()

set( LIBRARY_NAME ${PROJECT_NAME} )
string( TOLOWER ${LIBRARY_NAME} LIBRARY_NAME )

add_library( ${LIBRARY_NAME} ${SRCS} ${HDRS} )
target_link_libraries( ${LIBRARY_NAME} ${_LIBRARIES} )

install_package(
  PKG_NAME ${PROJECT_NAME}
  LIB_NAME ${LIBRARY_NAME}
  VERSION ${VERSION}
	DESCRIPTION "Vehicle Controller and GUI"
  #INSTALL_INCLUDE_DIR true
  DESTINATION ${CMAKE_INSTALL_PREFIX}
  INCLUDE_DIRS ${_INCLUDE_DIRS}
  LINK_LIBS ${_LIBRARIES}
)

