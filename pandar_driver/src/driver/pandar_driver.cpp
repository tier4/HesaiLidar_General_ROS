#include <pandar_driver/pandar_driver.h>
#include <pandar_driver/input.h>
#include <pandar_driver/pcap_input.h>
#include <pandar_driver/socket_input.h>
#include <pandar_msgs/msg/pandar_packet.hpp>
#include <pandar_msgs/msg/pandar_scan.hpp>
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"

using namespace pandar_driver;

PandarDriver::PandarDriver(std::shared_ptr<rclcpp::Node> node)
{

  node->declare_parameter("pcap");
  node->declare_parameter("device_ip");
  node->declare_parameter("lidar_port");
  node->declare_parameter("gps_port");
  node->declare_parameter("scan_phase");
  node->declare_parameter("model");
  node->declare_parameter("frame_id");

  rclcpp::Parameter  param_pcap_path_  = node->get_parameter("pcap");
  rclcpp::Parameter  param_device_ip_  = node->get_parameter("device_ip");
  rclcpp::Parameter  param_lidar_port_ = node->get_parameter("lidar_port");
  rclcpp::Parameter  param_gps_port_   = node->get_parameter("gps_port");
  rclcpp::Parameter  param_scan_phase_ = node->get_parameter("scan_phase");
  rclcpp::Parameter  param_model_      = node->get_parameter("model");
  rclcpp::Parameter  param_frame_id_   = node->get_parameter("frame_id");

  pcap_path_   = param_pcap_path_.as_string();
  device_ip_   = param_device_ip_.as_string();
  lidar_port_  = param_lidar_port_.as_int();
  gps_port_    = param_gps_port_.as_int();
  scan_phase_  = param_scan_phase_.as_double();
  model_       = param_model_.as_string();
  frame_id_    = param_frame_id_.as_string();

  if ( device_ip_.empty() ) {
    device_ip_ = "192.168.1.201";
  }

  if ( lidar_port_ == 0 ) {
    lidar_port_ = 2368;
  }

  if ( gps_port_ == 0 ) {
    gps_port_ = 11010;
  }

  if ( device_ip_.empty() ) {
    device_ip_ = "192.168.1.201";
  }

  if ( model_.empty() ) {
    model_ = "Pandar40P";
  }

  RCLCPP_INFO(node->get_logger(), "       pcap = '%s'", pcap_path_.c_str() );
  RCLCPP_INFO(node->get_logger(), "  device_ip = '%s'", device_ip_.c_str() );
  RCLCPP_INFO(node->get_logger(), " lidar_port = '%d'", lidar_port_);
  RCLCPP_INFO(node->get_logger(), "   gps_port = '%d'", gps_port_ );
  RCLCPP_INFO(node->get_logger(), " scan_phase = '%f'", scan_phase_ );
  RCLCPP_INFO(node->get_logger(), "      model = '%s'", model_.c_str() );
  RCLCPP_INFO(node->get_logger(), "   frame_id = '%s'", frame_id_.c_str() );

  if (!pcap_path_.empty()) {
    RCLCPP_INFO(node->get_logger(), " Data will be loaded from pcap file ");
    input_.reset(new PcapInput(lidar_port_, gps_port_, pcap_path_, model_));
  }
  else {
    RCLCPP_INFO(node->get_logger(), " Data will be received from udp packet ");
    input_.reset(new SocketInput(lidar_port_, gps_port_, node ));
  }

  if (model_ == "Pandar40P" || model_ == "Pandar40M") {
    azimuth_index_ = 2;  // 2 + 124 * [0-9]
    is_valid_packet_ = [](size_t packet_size) { return (packet_size == 1262 || packet_size == 1266); };
  }
  else if (model_ == "PandarQT") {
    azimuth_index_ = 12;  // 12 + 258 * [0-3]
    is_valid_packet_ = [](size_t packet_size) { return (packet_size == 1072); };
  }
  else if (model_ == "Pandar64") {
    azimuth_index_ = 8;  // 8 + 192 * [0-5]
    is_valid_packet_ = [](size_t packet_size) { return (packet_size == 1194 || packet_size == 1198); };
  }
  else if (model_ == "Pandar128") {
    azimuth_index_ = 12;  // 12 + 386 * [0-1]
    is_valid_packet_ = [](size_t packet_size) { return (packet_size == 812); };
  }
  else {
      RCLCPP_ERROR(node->get_logger(), "Invalid model name");
  }
}

bool PandarDriver::poll(std::shared_ptr<rclcpp::Node> node)
{
  int scan_phase = static_cast<int>(scan_phase_ * 100.0);

// ToDo:
//  pandar_packet_pub_ = node->create_publisher<pandar_msgs::msg::PandarScan_>("pandar_packet", rclcpp::QoS(10));

  pandar_msgs::msg::PandarScan * scan(new pandar_msgs::msg::PandarScan);
  for (int prev_phase = 0;;) {  // finish scan
    while (true) {              // until receive lidar packet
      pandar_msgs::msg::PandarPacket packet;
      int packet_type = input_->getPacket(&packet);
      if (packet_type == 0 && is_valid_packet_(packet.size)) {
        scan->packets.push_back(packet);
        break;
      }
      else if (packet_type == -1) {
        return false;
      }
    }

    int current_phase = 0;
    { 
      const auto& data = scan->packets.back().data;
      current_phase = (data[azimuth_index_] & 0xff) | ((data[azimuth_index_ + 1] & 0xff) << 8);
      current_phase = (static_cast<int>(current_phase) + 36000 - scan_phase) % 36000;
    }
    if (current_phase >= prev_phase || scan->packets.size() < 2) {
      prev_phase = current_phase;
    }
    else {
      // has scanned !
      break;
    }
  }

  scan->header.stamp = scan->packets.front().stamp;
  scan->header.frame_id = frame_id_;

//  pandar_packet_pub_->publish(scan);

  RCLCPP_INFO(node->get_logger(), " >>> Exit driver.poll()" );

  return true;
}
