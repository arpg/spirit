get_filename_component(ProjectId ${CMAKE_CURRENT_SOURCE_DIR} NAME)
string(REPLACE " " "_" ProjectId ${ProjectId})

# Find Pangolin
find_package(Pangolin 0.1 REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

# Find Scenegraph
find_package(SceneGraph 0.1 REQUIRED)
include_directories(${SceneGraph_INCLUDE_DIRS})

# Find GLOG
find_package(GLOG REQUIRED)
include_directories(${GLOG_INCLUDE_DIRS})

add_executable( ${ProjectId} main.cpp)
target_link_libraries(${ProjectId}
                            ${Pangolin_LIBRARIES}
                            ${SceneGraph_LIBRARIES}
                            ${GLOG_LIBRARIES}
                            assimp
                            )
