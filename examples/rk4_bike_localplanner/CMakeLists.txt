#cmake_minimum_required(VERSION 2.8)

get_filename_component(ProjectId ${CMAKE_CURRENT_SOURCE_DIR} NAME)
string(REPLACE " " "_" ProjectId ${ProjectId})

#project(rk4_bike_localplanner)
#add_executable(${PROJECT_NAME} "main.cpp")

# Fine OSG
INCLUDE_DIRECTORIES(${OPENTHREADS_INCLUDE_DIR} ${OSG_INCLUDE_DIR})

# Find SPIRIT
find_package( spirit REQUIRED)
include_directories(${spirit_INCLUDE_DIRS})
add_executable( ${ProjectId} main.cpp)
target_link_libraries(${ProjectId}
	${spirit_LIBRARIES}
	pthread)
