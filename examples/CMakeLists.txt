find_package( Spirit QUIET )
if( Spirit_FOUND )
      add_subdirectory( car_robot )
      add_subdirectory( CarCalib )
endif()
add_subdirectory( gamepad_drive )

