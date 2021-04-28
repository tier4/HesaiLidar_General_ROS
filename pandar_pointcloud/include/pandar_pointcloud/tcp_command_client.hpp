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

#include <string>

namespace pandar_pointcloud
{
class TcpCommandClient
{
public:
  TcpCommandClient(const std::string& ip, const uint16_t port);

  enum PTC_ErrCode : uint8_t
  {
    PTC_ERROR_NO_ERROR = 0,
    PTC_ERROR_BAD_PARAMETER,
    PTC_ERROR_CONNECT_SERVER_FAILED,
    PTC_ERROR_TRANSFER_FAILED,
    PTC_ERROR_NO_MEMORY,
  };

  PTC_ErrCode setCalibration(const std::string& content);
  PTC_ErrCode getCalibration(std::string& content);
  PTC_ErrCode getLidarCalibration(std::string& content);
  PTC_ErrCode resetCalibration();

private:
  enum PTC_COMMAND : uint8_t
  {
    PTC_COMMAND_GET_CALIBRATION = 0,
    PTC_COMMAND_SET_CALIBRATION,
    PTC_COMMAND_HEARTBEAT,
    PTC_COMMAND_RESET_CALIBRATION,
    PTC_COMMAND_TEST,
    PTC_COMMAND_GET_LIDAR_CALIBRATION,
  };

  struct TcpCommandHeader
  {
    uint8_t cmd;
    uint8_t ret_code;
    uint32_t len;
  };

  struct TC_Command
  {
    TcpCommandHeader header;
    uint8_t* data;
    uint8_t* ret_data;
    uint32_t ret_size;
  };

  pthread_mutex_t lock_;
  pthread_t tid_;

  std::string ip_;
  uint16_t port_;

  int parseHeader(unsigned char* buffer, TcpCommandHeader* header);
  int readCommand(int connfd, TC_Command* cmd);
  int buildHeader(char* buffer, TC_Command* cmd);
  PTC_ErrCode sendCmd(TC_Command* cmd);
};

}  // namespace pandar_pointcloud