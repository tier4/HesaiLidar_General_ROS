cmake_minimum_required(VERSION 3.5)

project(pandar_msgs)

# Add flags to support compiling on windows, since ROS2 supports it
if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wextra")
endif()

# Remove the catkin dependency and add ament_cmake
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp_components REQUIRED)

set(msg_files
  "msg/PandarPacket.msg"
  "msg/PandarScan.msg"
)

# Generate the messages.
# Instead of calling generate_messages, use rosidl_generate_interfaces.
rosidl_generate_interfaces(${PROJECT_NAME}
    ${msg_files}
)

#rosidl_target_interfaces(vision_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Declare dependencies.
# Instead of catkin_package(CATKIN_DEPENDS ...), use ament_export_dependencies.
ament_export_dependencies(rosidl_default_runtime)

# Instead of catkin_package
ament_package()
