find_package( Spirit QUIET )
if( Spirit_FOUND )
      add_subdirectory( car_robot )
endif()
add_subdirectory( gamepad_drive )

