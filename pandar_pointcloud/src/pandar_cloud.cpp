#include <rclcpp/rclcpp.hpp>
#include "pandar_pointcloud/pandar_cloud.hpp"
#include "pandar_msgs/msg/pandar_scan.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include "pandar_pointcloud/calibration.hpp"
#include "pandar_pointcloud/decoder/pandar40_decoder.hpp"
#include "pandar_pointcloud/decoder/pandar_qt_decoder.hpp"

#include <chrono>
#include <thread>

namespace
{
const uint16_t TCP_COMMAND_PORT = 9347;
const size_t TCP_RETRY_NUM = 5;
const double TCP_RETRY_WAIT_SEC = 0.1;
}  // namespace

using namespace pandar_pointcloud;

PandarCloud::PandarCloud(std::shared_ptr<rclcpp::Node> node)
{
  scan_phase_ = 0.0;

  node->declare_parameter("device_ip");
  node->declare_parameter("scan_phase");
  node->declare_parameter("model");
  node->declare_parameter("calibration");

  rclcpp::Parameter  param_device_ip_   = node->get_parameter("device_ip");
  rclcpp::Parameter  param_scan_phase_  = node->get_parameter("scan_phase");
  rclcpp::Parameter  param_model_       = node->get_parameter("model");
  rclcpp::Parameter  param_calibration_ = node->get_parameter("calibration");

  device_ip_   = param_device_ip_.as_string();
  scan_phase_  = param_scan_phase_.as_double();
  model_       = param_model_.as_string();
  calibration_path_ = param_calibration_.as_string();

  if ( device_ip_.empty() ) {
    device_ip_ = "192.168.1.201";
  }

  RCLCPP_INFO(node->get_logger(), "        device_ip = '%s'", device_ip_.c_str() );
  RCLCPP_INFO(node->get_logger(), "       scan_phase = '%f'", scan_phase_ );
  RCLCPP_INFO(node->get_logger(), "            model = '%s'", model_.c_str() );
  RCLCPP_INFO(node->get_logger(), " calibration path = '%s'", calibration_path_.c_str() );

  tcp_client_ = std::make_shared<TcpCommandClient>(device_ip_, TCP_COMMAND_PORT);
  if (!setupCalibration()) {
    RCLCPP_ERROR(node->get_logger(),"Unable to load calibration data");
    return;
  }

  if (model_ == "Pandar40P" || model_ == "Pandar40M") {
    decoder_ = std::make_shared<pandar40::Pandar40Decoder>(calibration_, scan_phase_,
                                                           pandar40::Pandar40Decoder::ReturnMode::DUAL);
  }
  else if (model_ == "PandarQT") {
    decoder_ = std::make_shared<pandar_qt::PandarQTDecoder>(calibration_, scan_phase_);
  }
  else {
    // TODO : Add other models
    RCLCPP_ERROR(node->get_logger(),"Invalid model name");
    return;
  }
  pandar_packet_sub_ =
      node->create_subscription<pandar_msgs::msg::PandarScan>("pandar_packets", rclcpp::QoS(10), std::bind(&PandarCloud::onProcessScan, this, std::placeholders::_1));
  pandar_points_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>("pandar_points", rclcpp::QoS(10));
  pandar_points_ex_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>("pandar_points_ex", rclcpp::QoS(10));
}

PandarCloud::~PandarCloud()
{
}

bool PandarCloud::setupCalibration()
{
  if (!calibration_path_.empty() && calibration_.loadFile(calibration_path_) == 0) {
    return true;
  }
  else if (tcp_client_) {
    std::string content("");
    for (size_t i = 0; i < TCP_RETRY_NUM; ++i) {
      auto ret = tcp_client_->getLidarCalibration(content);
      if (ret == TcpCommandClient::PTC_ErrCode::PTC_ERROR_NO_ERROR) {
        break;
      }
        rclcpp::sleep_for(std::chrono::milliseconds(10));
    }
    if (!content.empty()) {
      calibration_.loadContent(content);
      if (!calibration_path_.empty()) {
        calibration_.saveFile(calibration_path_);
      }
      return true;
    }
  }
  return false;
}

void PandarCloud::onProcessScan(const pandar_msgs::msg::PandarScan::SharedPtr scan_msg)
{
  PointcloudXYZIRADT pointcloud;
  pandar_msgs::msg::PandarPacket pkt;

  for (auto& packet : scan_msg->packets) {
    decoder_->unpack(packet);
    if (decoder_->hasScanned()) {
      pointcloud = decoder_->getPointcloud();
      if (pointcloud->points.size() > 0) {
        pointcloud->header.stamp = pcl_conversions::toPCL(rclcpp::Time(pointcloud->points[0].time_stamp));
        pointcloud->header.frame_id = scan_msg->header.frame_id;
        pointcloud->height = 1;
// ToDo:
//       pandar_points_ex_pub_->publish(pointcloud);
//       if (pandar_points_pub_->get_subscription_count() > 0) {
//          pandar_points_pub_->publish(convertPointcloud(pointcloud));
//        }
      }
    }
  }
}

pcl::PointCloud<PointXYZIR>::Ptr
PandarCloud::convertPointcloud(const pcl::PointCloud<PointXYZIRADT>::ConstPtr& input_pointcloud)
{
  pcl::PointCloud<PointXYZIR>::Ptr output_pointcloud(new pcl::PointCloud<PointXYZIR>);
  output_pointcloud->reserve(input_pointcloud->points.size());
  PointXYZIR point;
  for (const auto& p : input_pointcloud->points) {
    point.x = p.x;
    point.y = p.y;
    point.z = p.z;
    point.intensity = p.intensity;
    point.ring = p.ring;
    output_pointcloud->points.push_back(point);
  }

  output_pointcloud->header = input_pointcloud->header;
  output_pointcloud->height = 1;
  output_pointcloud->width = output_pointcloud->points.size();
  return output_pointcloud;
}
// namespace pandar_pointcloud
