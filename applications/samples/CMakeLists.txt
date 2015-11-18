get_filename_component(ProjectId ${CMAKE_CURRENT_SOURCE_DIR} NAME)
string(REPLACE " " "_" ProjectId ${ProjectId})

include_directories( ${CMAKE_BINARY_DIR} ${Spirit_INCLUDE_DIRS} )

add_executable( ${ProjectId} main.cpp)
target_link_libraries(${ProjectId} ${Spirit_LIBRARIES} )
