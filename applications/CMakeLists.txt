find_package( Spirit QUIET )
if( Spirit_FOUND )
      add_subdirectory( ninja_gui )
      add_subdirectory( temp )
endif()


