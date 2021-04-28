#include <rclcpp/rclcpp.hpp>
#include <pandar_driver/pandar_driver.h>

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("pandar_driver_node");

   pandar_driver::PandarDriver driver(node);

  while (rclcpp::ok()) {
    driver.poll(node);
    rclcpp::spin_some(node);
  }

  return 0;
}
