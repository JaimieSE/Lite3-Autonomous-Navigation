cmake_minimum_required(VERSION 3.16)
project(transfer)

# BUILD/INSTALL ONLY. No bridge execution, launch files or runtime tests.
# Upstream source is referenced in place and never rewritten or regenerated.
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(transfer_interfaces REQUIRED)
find_package(Threads REQUIRED)

get_filename_component(VENDOR_ROOT
  "${CMAKE_CURRENT_SOURCE_DIR}/../../vendor/Lite3_ROS" ABSOLUTE)
set(VENDOR_TRANSFER "${VENDOR_ROOT}/src/transfer")

# Refuse silently changed inputs to this particular recovery candidate.
foreach(entry
    "src/Jetson2Motion.cpp|169383cc460a702bcf2374a5a5321889f57111eb98033e5330c93591c69c9edc"
    "include/protocol.hpp|df60817a96e7a813cb7e5dfc7a60258a2207363c41fcea48651fee7c5042fd9c")
  string(REPLACE "|" ";" parts "${entry}")
  list(GET parts 0 source)
  list(GET parts 1 expected)
  file(SHA256 "${VENDOR_TRANSFER}/${source}" actual)
  if(NOT actual STREQUAL expected)
    message(FATAL_ERROR "Pinned vendor hash mismatch: ${source}")
  endif()
endforeach()

add_executable(jetson2motion "${VENDOR_TRANSFER}/src/Jetson2Motion.cpp")
target_compile_options(jetson2motion PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(jetson2motion rclcpp std_msgs sensor_msgs nav_msgs
  geometry_msgs tf2_msgs tf2 tf2_ros tf2_geometry_msgs transfer_interfaces)
target_link_libraries(jetson2motion Threads::Threads)

# Match the historical motion-only scope. Do NOT install the full vendor launch,
# jetson2app or sensor_checker; they are outside this recovery candidate.
install(TARGETS jetson2motion DESTINATION lib/${PROJECT_NAME})
# README links are source-checkout relative; install only standalone records.
install(FILES deployment_review.yaml
  "${VENDOR_ROOT}/LICENSE" "${VENDOR_ROOT}/SOURCE_MANIFEST.json"
  DESTINATION share/${PROJECT_NAME}/recovery)
ament_package()
