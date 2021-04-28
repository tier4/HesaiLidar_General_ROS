/******************************************************************************
 * Copyright 2019 The Hesai Technology Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#pragma once

#include <netinet/in.h>
#include <string>
#include "pandar_driver/input.h"

namespace pandar_driver
{
class SocketInput : public Input
{
public:
  SocketInput(uint16_t port, uint16_t gpsPort, std::shared_ptr<rclcpp::Node> node );
  ~SocketInput();
  int getPacket(pandar_msgs::msg::PandarPacket* pkt) override;

private:
  int socketForLidar;
  int socketForGPS;
  int socketNumber;
};

}  // namespace pandar_driver
