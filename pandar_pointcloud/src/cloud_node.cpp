#include <rclcpp/rclcpp.hpp>
#include "pandar_pointcloud/pandar_cloud.hpp"

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("pandar_cloud_node");

  pandar_pointcloud::PandarCloud conv(node);

  while (rclcpp::ok()) {
      rclcpp::spin_some(node);
  }
  return 0;
}
