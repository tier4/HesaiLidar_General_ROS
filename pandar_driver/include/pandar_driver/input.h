#pragma once
#include <pandar_msgs/msg/pandar_packet.hpp>

namespace pandar_driver
{
class Input
{
public:
  virtual ~Input(){};
  virtual int getPacket(pandar_msgs::msg::PandarPacket* pkt) = 0;
};
}  // namespace pandar_driver
