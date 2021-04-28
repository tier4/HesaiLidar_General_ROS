cmake_minimum_required(VERSION 3.5)
project(pandar_driver)

# Remove the catkin dependency and add ament_cmake
find_package(ament_cmake REQUIRED)

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
find_package(pandar_msgs)

find_package(
  ament_cmake REQUIRED COMPONENTS
)

include_directories(include 
  ${pandar_driver_INCLUDE_DIRS}
  ${pandar_msgs_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
)

ament_export_dependencies(pandar_msgs)
ament_export_include_directories(include)
ament_export_libraries(pandar_input)


# Target
add_library(pandar_input
  src/lib/socket_input.cpp
  src/lib/pcap_input.cpp
)

##ament_target_dependencies(pandar_input
target_link_libraries(pandar_input
  pcap
  ${rclcpp_LIBRARIES}
)

## add node
add_executable(pandar_driver_node
  src/driver/node.cpp
  src/driver/pandar_driver.cpp
)
##ament_target_dependencies(pandar_driver_node
target_link_libraries(pandar_driver_node
  pandar_input
  ${rclcpp_LIBRARIES}
)

# Install
## executables and libraries
install(
  TARGETS
    pandar_driver_node
    pandar_input
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

## files

## project namespaced headers
install(
  DIRECTORY
    include
  DESTINATION include/${PROJECT_NAME}
)

ament_export_include_directories(include)

## directories
install(
  DIRECTORY
    launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
