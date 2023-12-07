# Set Aravis version to finde
set(ARAVIS_VERSION 0.8)

find_path(Aravis_INCLUDE_DIRS arv.h
  PATHS
  "$ENV{ARAVIS_INCLUDE_PATH}"
  /usr/local/include/aravis-${ARAVIS_VERSION}
  /usr/include/aravis-${ARAVIS_VERSION}
)

find_library(Aravis_LIBRARIES aravis-${ARAVIS_VERSION}
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
