  get_filename_component(ProjectId ${CMAKE_CURRENT_SOURCE_DIR} NAME)
  string(REPLACE " " "_" ProjectId ${ProjectId})

  find_package(spirit REQUIRED)
  include_directories(${spirit_INCLUDE_DIRS})

  add_executable( ${ProjectId} main.cpp)
  target_link_libraries(${ProjectId}
                        ${spirit_LIBRARIES}
#glog
#pthread
dl
                        )
