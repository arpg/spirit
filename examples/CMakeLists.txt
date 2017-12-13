find_package( spirit QUIET )
if( spirit_FOUND )
      add_subdirectory( car_robot )
      add_subdirectory( CarCalib )
      add_subdirectory( mpc_car )
endif()
add_subdirectory( gamepad_drive )

