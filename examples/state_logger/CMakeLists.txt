get_filename_component(ProjectId ${CMAKE_CURRENT_SOURCE_DIR} NAME)
string(REPLACE " " "_" ProjectId ${ProjectId})

# Find HAL
find_package(HAL REQUIRED)
include_directories(${HAL_INCLUDE_DIRS})

# Find SPIRIT
find_package( spirit REQUIRED)
include_directories(${spirit_INCLUDE_DIRS})

# Find Ceres Solver
find_package( CeresSolver REQUIRED)
include_directories(${CeresSolver_INCLUDE_DIRS})

add_executable( state_logger state_logger.cpp)
target_link_libraries(state_logger
  ${HAL_LIBRARIES}
  ${spirit_LIBRARIES}
  dl
)

add_executable( calibrate_model calibrate_model.cpp)
target_link_libraries(calibrate_model
  ${HAL_LIBRARIES}
  ${spirit_LIBRARIES}
  ${CeresSolver_LIBRARIES}
  dl
)
