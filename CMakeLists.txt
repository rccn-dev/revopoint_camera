cmake_minimum_required(VERSION 3.8)
project(revopoint_camera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

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

ament_target_dependencies(revopoint_camera_node
  rclcpp
  sensor_msgs
  camera_info_manager
  geometry_msgs
  tf2
  tf2_ros
)

set_target_properties(revopoint_camera_node PROPERTIES
  BUILD_RPATH "${SDK_LIB_DIR}"
  INSTALL_RPATH "\$ORIGIN/../share/${PROJECT_NAME}/3dcamera_sdk/lib/3dcamera/linux/${SDK_ARCH_DIR}"
)

install(TARGETS revopoint_camera_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY ${SDK_ROOT}/lib
  DESTINATION share/${PROJECT_NAME}/3dcamera_sdk
)

install(DIRECTORY ${SDK_ROOT}/inc
  DESTINATION share/${PROJECT_NAME}/3dcamera_sdk
)

ament_package()
