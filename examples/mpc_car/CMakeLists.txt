get_filename_component(ProjectId ${CMAKE_CURRENT_SOURCE_DIR} NAME)
string(REPLACE " " "_" ProjectId ${ProjectId})

# Find HAL
find_package(HAL REQUIRED)
include_directories(${HAL_INCLUDE_DIRS})

# Find Pangolin
#find_package(Pangolin 0.1 REQUIRED)
#include_directories(${Pangolin_INCLUDE_DIRS})

# Find Scenegraph
#find_package(SceneGraph 0.1 REQUIRED)
#include_directories(${SceneGraph_INCLUDE_DIRS})

find_package(spirit REQUIRED)
include_directories(${spirit_INCLUDE_DIRS})

add_executable( ${ProjectId} main.cpp)
target_link_libraries(${ProjectId}
                      ${HAL_LIBRARIES}
                      ${spirit_LIBRARIES}
#                      ${Pangolin_LIBRARIES}
#                      ${SceneGraph_LIBRARIES}
glog
dl
                      )
