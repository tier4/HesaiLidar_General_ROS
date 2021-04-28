#include "pandar_pointcloud/decoder/pandar40_decoder.hpp"
#include "pandar_pointcloud/decoder/pandar40.hpp"

namespace
{
static inline double deg2rad(double degrees)
{
  return degrees * M_PI / 180.0;
}
}  // namespace

namespace pandar_pointcloud
{
namespace pandar40
{
Pandar40Decoder::Pandar40Decoder(Calibration& calibration, float scan_phase, ReturnMode return_mode)
{
  firing_order_ = { 7,  19, 14, 26, 6,  18, 4,  32, 36, 0, 10, 22, 17, 29, 9,  21, 5,  33, 37, 1,
                    13, 25, 20, 30, 12, 8,  24, 34, 38, 2, 16, 28, 23, 31, 15, 11, 27, 35, 39, 3 };

  firing_offset_ = { 42.22, 28.47, 16.04, 3.62,  45.49, 31.74, 47.46, 54.67, 20.62, 33.71, 40.91, 8.19,  20.62, 27.16,
                     50.73, 8.19,  14.74, 36.98, 45.49, 52.7,  23.89, 31.74, 38.95, 11.47, 18.65, 25.19, 48.76, 6.23,
                     12.77, 35.01, 21.92, 9.5,   43.52, 29.77, 17.35, 4.92,  42.22, 28.47, 16.04, 3.62 };

  for (int block = 0; block < (int)BLOCKS_PER_PACKET; ++block) {
    block_offset_single_[block] = 55.56f * (BLOCKS_PER_PACKET - block - 1) + 28.58f;
    block_offset_dual_[block] = 55.56f * ((BLOCKS_PER_PACKET - block - 1) / 2) + 28.58f;
  }

  // TODO: add calibration data validation
  // if(calibration.elev_angle_map.size() != num_lasers_){
  //   // calibration data is not valid!
  // }
  for (size_t laser = 0; laser < LASER_COUNT; ++laser) {
    elev_angle_[laser] = calibration.elev_angle_map[laser];
    azimuth_offset_[laser] = calibration.azimuth_offset_map[laser];
  }

  scan_phase_ = static_cast<uint16_t>(scan_phase * 100.0f);
  return_mode_ = return_mode;

  last_phase_ = 0;
  has_scanned_ = false;

  scan_pc_.reset(new pcl::PointCloud<PointXYZIRADT>);
  overflow_pc_.reset(new pcl::PointCloud<PointXYZIRADT>);
}

bool Pandar40Decoder::hasScanned()
{
  return has_scanned_;
}

PointcloudXYZIRADT Pandar40Decoder::getPointcloud()
{
  return scan_pc_;
}

void Pandar40Decoder::unpack(const pandar_msgs::msg::PandarPacket& raw_packet)
{
  if (!parsePacket(raw_packet)) {
    return;
  }

  if (has_scanned_) {
    scan_pc_ = overflow_pc_;
    overflow_pc_.reset(new pcl::PointCloud<PointXYZIRADT>);
    has_scanned_ = false;
  }

  bool dual_return = (packet_.echo == DUAL_ECHO);
  auto step = dual_return ? 2 : 1;

  for (int block_id = 0; block_id < (int)BLOCKS_PER_PACKET; block_id += step) {
    auto block_pc = dual_return ? convert_dual(block_id) : convert(block_id);
    int current_phase = (static_cast<int>(packet_.blocks[block_id].azimuth) - scan_phase_ + 36000) % 36000;
    if (current_phase > last_phase_ && !has_scanned_) {
      *scan_pc_ += *block_pc;
    }
    else {
      *overflow_pc_ += *block_pc;
      has_scanned_ = true;
    }
    last_phase_ = current_phase;
  }
  return;
}

PointcloudXYZIRADT Pandar40Decoder::convert(int block_id)
{
  PointcloudXYZIRADT block_pc(new pcl::PointCloud<PointXYZIRADT>);

  // double unix_second = raw_packet.header.stamp.toSec() // system-time (packet receive time)
  double unix_second = static_cast<double>(timegm(&packet_.t));  // sensor-time (ppt/gps)

  for (auto unit_id : firing_order_) {
    PointXYZIRADT point;
    const auto& block = packet_.blocks[block_id];
    const auto& unit = block.units[unit_id];
    // skip invalid points
    if (unit.distance <= 0.1 || unit.distance > 200.0) {
      continue;
    }
    double xyDistance = unit.distance * cosf(deg2rad(elev_angle_[unit_id]));

    point.x = static_cast<float>(
        xyDistance * sinf(deg2rad(azimuth_offset_[unit_id] + (static_cast<double>(block.azimuth)) / 100.0)));
    point.y = static_cast<float>(
        xyDistance * cosf(deg2rad(azimuth_offset_[unit_id] + (static_cast<double>(block.azimuth)) / 100.0)));
    point.z = static_cast<float>(unit.distance * sinf(deg2rad(elev_angle_[unit_id])));

    point.intensity = unit.intensity;
    point.distance = unit.distance;
    point.ring = unit_id;
    point.azimuth = block.azimuth + round(azimuth_offset_[unit_id] * 100.0f);

    point.time_stamp = unix_second + (static_cast<double>(packet_.usec)) / 1000000.0;
    point.time_stamp -= (static_cast<double>(block_offset_single_[block_id] + firing_offset_[unit_id]) / 1000000.0f);

    block_pc->push_back(point);
  }
  return block_pc;
}

PointcloudXYZIRADT Pandar40Decoder::convert_dual(int block_id)
{
  //   Under the Dual Return mode, the measurements from each round of firing are stored in two adjacent blocks:
  //   The odd number block is the last return, and the even number block is the strongest return
  //   If the last and strongest returns coincide, the second strongest return will be placed in the even number block
  //   The Azimuth changes every two blocks
  PointcloudXYZIRADT block_pc(new pcl::PointCloud<PointXYZIRADT>);
  double unix_second = static_cast<double>(timegm(&packet_.t));

  auto head = block_id + ((return_mode_ == ReturnMode::STRONGEST) ? 1 : 0);
  auto tail = block_id + ((return_mode_ == ReturnMode::LAST) ? 1 : 2);

  for (auto unit_id : firing_order_) {
    for (int i = head; i < tail; ++i) {
      PointXYZIRADT point;
      const auto& block = packet_.blocks[i];
      const auto& unit = block.units[unit_id];
      // skip invalid points
      if (unit.distance <= 0.1 || unit.distance > 200.0) {
        continue;
      }
      double xyDistance = unit.distance * cosf(deg2rad(elev_angle_[unit_id]));

      point.x = static_cast<float>(
          xyDistance * sinf(deg2rad(azimuth_offset_[unit_id] + (static_cast<double>(block.azimuth)) / 100.0)));
      point.y = static_cast<float>(
          xyDistance * cosf(deg2rad(azimuth_offset_[unit_id] + (static_cast<double>(block.azimuth)) / 100.0)));
      point.z = static_cast<float>(unit.distance * sinf(deg2rad(elev_angle_[unit_id])));

      point.intensity = unit.intensity;
      point.distance = unit.distance;

      point.ring = unit_id;
      point.azimuth = block.azimuth;

      point.time_stamp = unix_second + (static_cast<double>(packet_.usec)) / 1000000.0;
      point.time_stamp -= (static_cast<double>(block_offset_dual_[i] + firing_offset_[unit_id]) / 1000000.0f);

      block_pc->push_back(point);
    }
  }
  return block_pc;
}

bool Pandar40Decoder::parsePacket(const pandar_msgs::msg::PandarPacket& raw_packet)
{
  if (raw_packet.size != PACKET_SIZE && raw_packet.size != PACKET_SIZE + SEQ_NUM_SIZE) {
    // packet size mismatch !
    return false;
  }

  // auto buf = raw_packet.data;
  const uint8_t* buf = &raw_packet.data[0];

  int index = 0;
  for (int i = 0; i < (int)BLOCKS_PER_PACKET; i++) {
    Block& block = packet_.blocks[i];

    block.sob = (buf[index] & 0xff) | ((buf[index + 1] & 0xff) << 8);
    block.azimuth = (buf[index + 2] & 0xff) | ((buf[index + 3] & 0xff) << 8);
    index += SOB_ANGLE_SIZE;

    for (int j = 0; j < (int)LASER_COUNT; j++) {
      Unit& unit = block.units[j];
      uint32_t range = (buf[index] & 0xff) | ((buf[index + 1] & 0xff) << 8);

      unit.distance = (static_cast<double>(range)) * LASER_RETURN_TO_DISTANCE_RATE;
      unit.intensity = (buf[index + 2] & 0xff);

      if ((unit.distance == 0x010101  &&  unit.intensity == 0x0101) ||
           unit.distance > (200 * 1000 / 2 /* 200m -> 2mm */)) {
        unit.distance = 0;
        unit.intensity = 0;
      }

      index += RAW_MEASURE_SIZE;
    }
  }

  index += RESERVE_SIZE;

  index += REVOLUTION_SIZE;

  packet_.usec = (buf[index] & 0xff) | (buf[index + 1] & 0xff) << 8 | ((buf[index + 2] & 0xff) << 16) |
                 ((buf[index + 3] & 0xff) << 24);
  packet_.usec %= 1000000;

  index += TIMESTAMP_SIZE;
  packet_.echo = buf[index] & 0xff;

  index += FACTORY_INFO_SIZE + ECHO_SIZE;

  packet_.t.tm_year = (buf[index + 0] & 0xff) + 100;

  // in case of time error
  if (packet_.t.tm_year >= 200) {
    packet_.t.tm_year -= 100;
  }

  packet_.t.tm_mon = (buf[index + 1] & 0xff) - 1;
  packet_.t.tm_mday = buf[index + 2] & 0xff;
  packet_.t.tm_hour = buf[index + 3] & 0xff;
  packet_.t.tm_min = buf[index + 4] & 0xff;
  packet_.t.tm_sec = buf[index + 5] & 0xff;
  packet_.t.tm_isdst = 0;

  return true;
}

}  // namespace pandar40
}  // namespace pandar_pointcloud