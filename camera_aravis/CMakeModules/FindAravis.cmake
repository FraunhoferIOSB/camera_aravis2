
find_package(PkgConfig QUIET)

if( PKG_CONFIG_FOUND )
  pkg_check_modules( Aravis aravis-0.6 )
endif()

if( NOT Aravis_FOUND )
  message("Aravis (aravis-0.6) could not be found by pkg-config. Trying to manually find Aravis.")
  find_path(Aravis_INCLUDE_DIRS arv.h
    PATHS
    "$ENV{ARAVIS_INCLUDE_PATH}"
    /usr/local/include/aravis-0.6
    /usr/include/aravis-0.6
  )
  find_library(Aravis_LIBRARIES aravis-0.6
    PATHS
    "$ENV{ARAVIS_LIBRARY}"
    /usr/local/lib
    /usr/lib
    /usr/lib/x86_64-linux-gnu
  )
  include(FindPackageHandleStandardArgs)
  find_package_handle_standard_args( Aravis DEFAULT_MSG
    Aravis_INCLUDE_DIRS
    Aravis_LIBRARIES
  )
endif()
