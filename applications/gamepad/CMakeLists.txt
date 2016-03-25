get_filename_component(ProjectId ${CMAKE_CURRENT_SOURCE_DIR} NAME)
string(REPLACE " " "_" ProjectId ${ProjectId})

# Find Pangolin
find_package(HAL REQUIRED)
include_directories(${HAL_INCLUDE_DIRS})
add_executable( ${ProjectId} main.cpp)
target_link_libraries(${ProjectId}
											${HAL_LIBRARIES}
											spirit
											)
