cmake_minimum_required(VERSION 3.8)
project(camera_aravis)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to Release as none was specified.")
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build." FORCE)
  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
    "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

# Default to C11
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 11)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

find_package(std_msgs REQUIRED)
find_package(camera_aravis_msgs REQUIRED)

find_package(Aravis REQUIRED)
find_package(GLIB2 REQUIRED)

include_directories(include
  ${Aravis_INCLUDE_DIRS}
  ${GLIB2_INCLUDE_DIRS}
)

set(EXECUTABLE_NAME camera_aravis)
set(LIBRARY_NAME ${EXECUTABLE_NAME}_core)

set(DEPENDENCIES
  rclcpp
  rclcpp_components
  std_msgs
)

set(LIBRARIES
  ${Aravis_LIBRARIES}
  glib-2.0
  gmodule-2.0
  gobject-2.0
)

add_library(${LIBRARY_NAME} SHARED
  src/camera_aravis.cpp
)
target_link_libraries(${LIBRARY_NAME} SYSTEM ${LIBRARIES})
ament_target_dependencies(${LIBRARY_NAME} ${DEPENDENCIES})

add_executable(${EXECUTABLE_NAME} src/main.cpp)
target_link_libraries(${EXECUTABLE_NAME} ${LIBRARY_NAME})

rclcpp_components_register_nodes(${LIBRARY_NAME} "camera_aravis::CameraAravis")

install(TARGETS ${LIBRARY_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${EXECUTABLE_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include/)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
