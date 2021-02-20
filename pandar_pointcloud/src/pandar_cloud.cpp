#include "pandar_pointcloud/pandar_cloud.hpp"
#include <pandar_msgs/PandarScan.h>
#include "pandar_pointcloud/calibration.hpp"
#include "pandar_pointcloud/decoder/pandar40_decoder.hpp"
#include "pandar_pointcloud/decoder/pandar_qt_decoder.hpp"

#include <chrono>
#include <thread>

namespace
{
const uint16_t TCP_COMMAND_PORT = 9347;
const size_t TCP_RETRY_NUM = 5;
const size_t TCP_RETRY_WAIT = 100;
}  // namespace

namespace pandar_pointcloud
{
PandarCloud::PandarCloud(ros::NodeHandle node, ros::NodeHandle private_nh)
{
  private_nh.getParam("scan_phase", scan_phase_);
  private_nh.getParam("calibration", calibration_path_);
  private_nh.getParam("model", model_);

  pandar_packet_sub_ = node.subscribe(
    "pandar_packets", 10, &PandarCloud::onProcessScan, this,
    ros::TransportHints().tcpNoDelay(true));
  pandar_points_pub_ = node.advertise<sensor_msgs::PointCloud2>("pandar_points", 10);
  pandar_points_ex_pub_ = node.advertise<sensor_msgs::PointCloud2>("pandar_points_ex", 10);

  if (setupCalibration() != 0) {
    ROS_ERROR("Unable to load calibration data");
    return;
  }

  if (model_ == "Pandar40P" || model_ == "Pandar40M") {
    decoder_ = std::make_shared<pandar40::Pandar40Decoder>(
      calibration_, scan_phase_, pandar40::Pandar40Decoder::ReturnMode::DUAL);
  } else if (model_ == "PandarQT") {
    decoder_ = std::make_shared<pandar_qt::PandarQTDecoder>(calibration_, scan_phase_);
  } else {
    // TODO : Add other models
    ROS_ERROR("Invalid model name");
    return;
  }
}

PandarCloud::~PandarCloud() {}

int PandarCloud::setupCalibration()
{
  if (tcp_client_) {
    std::string content("");
    for (size_t i = 0; i < TCP_RETRY_NUM; ++i) {
      auto ret = tcp_client_->getLidarCalibration(content);
      if (ret == TcpCommandClient::PTC_ErrCode::PTC_ERROR_NO_ERROR) {
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(TCP_RETRY_WAIT));
    }
    if (!content.empty()) {
      calibration_.loadContent(content);
      return 0;
    }
  }
  if (!calibration_path_.empty() && calibration_.loadFile(calibration_path_) == 0) {
    return 0;
  } else {
    return -1;
  }
}

void PandarCloud::onProcessScan(const pandar_msgs::PandarScan::ConstPtr & scan_msg)
{
  PointcloudXYZIRADT pointcloud;
  pandar_msgs::PandarPacket pkt;

  for (auto & packet : scan_msg->packets) {
    decoder_->unpack(packet);
    if (decoder_->hasScanned()) {
      pointcloud = decoder_->getPointcloud();
      pointcloud->header.stamp =
        pcl_conversions::toPCL(ros::Time(pointcloud->points[0].time_stamp));
      pointcloud->header.frame_id = scan_msg->header.frame_id;
      pointcloud->height = 1;

      pandar_points_ex_pub_.publish(pointcloud);
      if (pandar_points_pub_.getNumSubscribers() > 0) {
        pandar_points_pub_.publish(convertPointcloud(pointcloud));
      }
    }
  }
}

pcl::PointCloud<PointXYZIR>::Ptr PandarCloud::convertPointcloud(
  const pcl::PointCloud<PointXYZIRADT>::ConstPtr & input_pointcloud)
{
  pcl::PointCloud<PointXYZIR>::Ptr output_pointcloud(new pcl::PointCloud<PointXYZIR>);
  output_pointcloud->reserve(input_pointcloud->points.size());
  PointXYZIR point;
  for (const auto & p : input_pointcloud->points) {
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
}  // namespace pandar_pointcloud