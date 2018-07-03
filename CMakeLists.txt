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

option( BUILD_SHARED_LIBS "Build shared libraries." ON )

# Default flags.
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall")

IF(${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++")
ENDIF()

find_package( Pangolin REQUIRED )
find_package( Bullet REQUIRED )
find_package( SceneGraph 0.1 REQUIRED )
find_package( Eigen3 REQUIRED )
find_package( CeresSolver REQUIRED)
#find_package( Matlab	REQUIRED ENG_LIBRARY MX_LIBRARY MAT_LIBRARY)
#find_package( tbb REQUIRED)

set( _INCLUDE_DIRS
	${CMAKE_BINARY_DIR}
	${Pangolin_INCLUDE_DIRS}
	${SceneGraph_INCLUDE_DIRS}
	${BULLET_INCLUDE_DIR}
	${CeresSolver_INCLUDE_DIRS}
#	${Matlab_INCLUDE_DIRS}
#	${TBB_INCLUDE_DIRS}
	${CMAKE_CURRENT_SOURCE_DIR}/include
	)

#message("Ceres Libs: ${CeresSolver_LIBRARIES}")

set( _LIBRARIES
	${Pangolin_LIBRARIES}
	${SceneGraph_LIBRARIES}
	${BULLET_LIBRARIES}
	${CeresSolver_LIBRARIES}
#	${Matlab_LIBRARIES}
#	${TBB_LIBRARY_DIRS}
	)

set( HDRS
	include/spirit/spirit.h
	include/spirit/Gui/spCommonGui.h
	include/spirit/Gui/spPangolinScenegraphGui.h
	include/spirit/Objects.h
	include/spirit/Gui.h
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
	include/spirit/Objects/spLineStrip.h
	include/spirit/CarSimFunctor.h
	include/spirit/CarSimFunctorRK4.h
	include/spirit/CarFunc.h
	include/spirit/Controllers/spPID.h
	include/spirit/Planners/spTrajectory.h
	include/spirit//Planners/VehicleCeresCostFunc.h
	include/spirit/Planners/spLocalPlanner.h
	include/spirit/Controllers/spMPC.h
	include/spirit/Controllers/MPCCostFunc.h
	include/spirit/Controllers/MPCManRegCostFunc.h
	include/spirit/Controllers/Maneuver.h
	include/spirit/Controllers/CircleMan.h
	include/spirit/Calibration/CalibCostFunc.h
	include/spirit/ParamLimitLossFunc.h
	include/spirit/Calibration/PriorityQueue.h
	include/spirit/Calibration/CandidateWindow.h
	include/spirit/Calibration/CalibCarSimFunctor.h
	include/spirit/Calibration/EntropyTable.h
	include/spirit/Planners/spCirclePlanner.h
	include/spirit/RK4.h
	include/spirit/CarODE.h
	include/spirit/LogBarrierLossFunc.h
	)

set( SRCS
	src/spirit.cpp
	src/Gui.cpp
	src/spLocalPlanner.cpp
	src/spPangolinScenegraphGui.cpp
	src/spGeneralTools.cpp
	src/spSettings.cpp
	src/spBox.cpp
	src/spWaypoint.cpp
	src/spCommonObject.cpp
	src/Objects.cpp
	src/spMesh.cpp
	src/spVehicle.cpp
	src/spAWSDCar.cpp
	src/spWheel.cpp
	src/spLineStrip.cpp
	src/spPID.cpp
	src/spTrajectory.cpp
	src/spMPC.cpp
	src/spCirclePlanner.cpp
	src/RK4.cpp
	)

option(BUILD_Examples "Build Examples" ON)

if(BUILD_Examples)
	add_subdirectory( examples )
endif()

include_directories( ${_INCLUDE_DIRS} )
set( LIBRARY_NAME ${PROJECT_NAME} )
string( TOLOWER ${LIBRARY_NAME} LIBRARY_NAME )

add_library( ${LIBRARY_NAME} ${SRCS} ${HDRS} )
target_link_libraries( ${LIBRARY_NAME} ${_LIBRARIES})

install_package(
  PKG_NAME ${PROJECT_NAME}
  LIB_NAME ${LIBRARY_NAME}
  VERSION ${VERSION}
	DESCRIPTION "Vehicle Controller and GUI"
#	INSTALL_INCLUDE_DIR true
  DESTINATION ${CMAKE_INSTALL_PREFIX}
  INCLUDE_DIRS ${_INCLUDE_DIRS}
  LINK_LIBS ${_LIBRARIES}
)

