cmake_minimum_required(VERSION 3.5)
project(pandar_pointcloud)

# Remove the catkin dependency and add ament_cmake
find_package(ament_cmake REQUIRED)

set(PCL_110_INCLUDE_DIRS  "/usr/include/pcl-1.10") 
set(EIGEN3_INCLUDE_DIRS   "/usr/include/eigen3") 

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

# You specify the individual packages that are required for building like this:
find_package(builtin_interfaces REQUIRED)

find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(pcl_conversions)
find_package(sensor_msgs)
find_package(pandar_msgs)
find_package(Boost REQUIRED COMPONENTS system filesystem thread)
find_package(PCL REQUIRED COMPONENTS common)
find_package( ament_cmake REQUIRED COMPONENTS )

ament_export_dependencies(pandar_msgs)
ament_export_include_directories(include)
ament_export_libraries(pandar_cloud)

# Target
add_library(pandar_cloud
  src/pandar_cloud.cpp
  src/lib/calibration.cpp
  src/lib/tcp_command_client.cpp
  src/lib/tcp_util.cpp
  src/lib/decoder/pandar40_decoder.cpp
  src/lib/decoder/pandar_qt_decoder.cpp
)
target_link_libraries(pandar_cloud
  ${rclcpp_LIBRARIES}
)

## add node
add_executable(pandar_cloud_node
  src/cloud_node.cpp
)

target_link_libraries(pandar_cloud_node
  pandar_cloud
  ${rclcpp_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${pandar_msgs_LIBRARIES}
)

include_directories(include 
  ${PCL_110_INCLUDE_DIRS}
  ${pandar_pointcloud_INCLUDE_DIRS}
  ${pandar_msgs_INCLUDE_DIRS}
  ${pcl_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
)
# Install
## executables and libraries
install(
  TARGETS
    pandar_cloud_node
    pandar_cloud
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# PCL
set(BLACKLISTED_PCL_DEFINITIONS " -march=native -msse4.2 -mfpmath=sse ")
foreach(DEFINITION ${PCL_DEFINITIONS})
  list (FIND BLACKLISTED_PCL_DEFINITIONS "${DEFINITION}" DEFINITIONS_INDEX)
  if (${DEFINITIONS_INDEX} GREATER -1)
    continue()
  endif()
  set(TARGET_COMPILE_FLAGS "${TARGET_COMPILE_FLAGS} ${DEFINITION}")
endforeach()

## files
install(
  FILES
  DESTINATION share/${PROJECT_NAME}
)

## project namespaced headers
install(
  DIRECTORY
    include/${PROJECT_NAME}
)

## directories
install(
  DIRECTORY
    launch
    config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

