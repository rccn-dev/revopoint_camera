cmake_minimum_required(VERSION 3.8)
project(revopoint_camera)

# colcon does not pass a build type by default, which leaves CMAKE_BUILD_TYPE
# empty and compiles the whole per-pixel image pipeline at -O0. Default to
# Release; an explicit -DCMAKE_BUILD_TYPE=... still wins.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
  message(STATUS "revopoint_camera: no CMAKE_BUILD_TYPE set, defaulting to Release")
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
  add_compile_options(-O3)
endif()

# The vendor SDK's hot paths (cs::Pointcloud) are header-only templates compiled
# by *this* project, and they are annotated with `#pragma omp parallel for`.
# Without OpenMP those pragmas are silently ignored and the code runs serial.
find_package(OpenMP QUIET)
if(OpenMP_CXX_FOUND)
  message(STATUS "revopoint_camera: OpenMP enabled")
else()
  message(STATUS "revopoint_camera: OpenMP not found, SDK omp pragmas will be ignored")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_srvs REQUIRED)
find_package(diagnostic_msgs REQUIRED)

set(SDK_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/3DCameraSDK-v3.2.229.20251110)
set(SDK_INCLUDE ${SDK_ROOT}/inc/3dcamera)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
  set(SDK_ARCH_DIR aarch64)
else()
  set(SDK_ARCH_DIR x64)
endif()

set(SDK_LIB ${SDK_ROOT}/lib/3dcamera/linux/${SDK_ARCH_DIR}/lib3DCamera.so)
set(SDK_LIB_DIR ${SDK_ROOT}/lib/3dcamera/linux/${SDK_ARCH_DIR})

add_executable(revopoint_camera_node src/revo_3dcamera_node.cpp)

target_include_directories(revopoint_camera_node PRIVATE
  ${SDK_INCLUDE}
)

target_link_libraries(revopoint_camera_node
  ${SDK_LIB}
)

if(OpenMP_CXX_FOUND)
  target_link_libraries(revopoint_camera_node OpenMP::OpenMP_CXX)
endif()

ament_target_dependencies(revopoint_camera_node
  rclcpp
  sensor_msgs
  camera_info_manager
  geometry_msgs
  tf2
  tf2_ros
  std_srvs
  diagnostic_msgs
)

set_target_properties(revopoint_camera_node PROPERTIES
  BUILD_RPATH "${SDK_LIB_DIR}"
  # The executable installs to lib/<pkg>/, so reaching share/<pkg>/ takes two
  # levels up, not one. With a single "..", the installed node fails to load
  # lib3DCamera.so unless a system-wide copy happens to shadow it.
  INSTALL_RPATH "\$ORIGIN/../../share/${PROJECT_NAME}/3dcamera_sdk/lib/3dcamera/linux/${SDK_ARCH_DIR}"
)

install(TARGETS revopoint_camera_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch presets rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/revo
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY ${SDK_ROOT}/lib
  DESTINATION share/${PROJECT_NAME}/3dcamera_sdk
)

install(DIRECTORY ${SDK_ROOT}/inc
  DESTINATION share/${PROJECT_NAME}/3dcamera_sdk
)

ament_package()
