get_filename_component(ProjectId ${CMAKE_CURRENT_SOURCE_DIR} NAME)
string(REPLACE " " "_" ProjectId ${ProjectId})

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -O3")

find_package( Eigen3 REQUIRED )

find_package(spirit REQUIRED)
include_directories(${spirit_INCLUDE_DIRS})

add_executable( ${ProjectId} main.cpp)
target_link_libraries(${ProjectId}
  ${spirit_LIBRARIES}
  glog
  dl
  )
