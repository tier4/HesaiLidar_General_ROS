#pragma once
#include <rclcpp/rclcpp.hpp>
#include <pandar_msgs/msg/pandar_scan.hpp>

namespace pandar_driver
{
class Input;
class PandarDriver
{
public:
  PandarDriver(std::shared_ptr<rclcpp::Node> node);
  ~PandarDriver(){};
  bool poll(std::shared_ptr<rclcpp::Node> node);

private:
  std::string device_ip_;
  int lidar_port_;
  int gps_port_;
  double scan_phase_;
  size_t azimuth_index_;

  std::string model_;
  std::string frame_id_;
  std::string pcap_path_;

  std::shared_ptr<Input> input_;

  std::function<bool(size_t)> is_valid_packet_;

  rclcpp::Publisher<pandar_msgs::msg::PandarScan>::SharedPtr pandar_packet_pub_;

};
}  // namespace pandar_driver
