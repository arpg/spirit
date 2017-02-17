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
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall")

IF(${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++")
ENDIF()

find_package( Pangolin 0.1 REQUIRED )
find_package( Bullet REQUIRED )
find_package( SceneGraph 0.1 REQUIRED )
find_package( Eigen3 REQUIRED )
find_package( Sophus REQUIRED)
find_package( CeresSolver REQUIRED)
#find_package( tbb REQUIRED)


set( _INCLUDE_DIRS
	${CMAKE_BINARY_DIR}
	${Pangolin_INCLUDE_DIRS}
	${SceneGraph_INCLUDE_DIRS}
	${BULLET_INCLUDE_DIR}
	${Sophus_INCLUDE_DIRS}
	${CeresSolver_INCLUDE_DIRS}
#	${TBB_INCLUDE_DIRS}
	${CMAKE_CURRENT_SOURCE_DIR}/include
	)
set( _LIBRARIES
	${Pangolin_LIBRARIES}
	${SceneGraph_LIBRARIES}
	${BULLET_LIBRARIES}
	${CeresSolver_LIBRARIES}
#	${TBB_LIBRARY_DIRS}
	)

include_directories( ${_INCLUDE_DIRS} )

set( HDRS
	include/spirit/spirit.h
	include/spirit/Gui.h
#	include/spirit/Physics.h
	include/spirit/Gui/spCommonGui.h
	include/spirit/Gui/spPangolinScenegraphGui.h
#	include/spirit/Physics/spPhysicsWorld.h
#	include/spirit/Physics/spBulletWorld.h
	include/spirit/spGeneralTools.h
	include/spirit/spSettings.h
	include/spirit/Objects/spBox.h
	include/spirit/Objects/spCommonObject.h
	include/spirit/Objects.h
	include/spirit/Objects/spMesh.h
	include/spirit/Objects/spVehicle.h
	include/spirit/Objects/spAWSDCar.h
	include/spirit/Objects/spWheel.h
	include/spirit/Objects/spWaypoint.h
	include/spirit/Types/spTypes.h
	include/spirit/Controllers/spControllerInterface.h
	include/spirit/Planners/spBezierPlanner.h
	include/spirit/Objects/spLineStrip.h
	include/spirit/CarSimFunctor.h
	include/spirit/Types/ctpl_stl.h
	include/spirit/Controllers/spPID.h
	include/spirit/Planners/spTrajectory.h
	include/spirit/VehicleCeresCostFunc.h
	)

set( SRCS
	src/spirit.cpp
	src/Gui.cpp
#	src/Physics.cpp
	src/spPangolinScenegraphGui.cpp
	src/spGeneralTools.cpp
	src/spSettings.cpp
#	src/spBulletWorld.cpp
	src/spBox.cpp
	src/spWaypoint.cpp
	src/spCommonObject.cpp
	src/Objects.cpp
	src/spMesh.cpp
	src/spVehicle.cpp
	src/spAWSDCar.cpp
	src/spWheel.cpp
	src/spLineStrip.cpp
	src/spBezierPlanner.cpp
	src/spPID.cpp
	src/spTrajectory.cpp
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

