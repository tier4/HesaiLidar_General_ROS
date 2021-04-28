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

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <linux/sockios.h>
#include <net/if.h>
#include <netinet/in.h>
#include <pthread.h>
#include <setjmp.h>
#include <signal.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/ipc.h>
#include <sys/msg.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <syslog.h>
#include <unistd.h>

#include <string>

#include "pandar_pointcloud/tcp_command_client.hpp"
#include "pandar_pointcloud/tcp_util.hpp"

namespace pandar_pointcloud
{
TcpCommandClient::TcpCommandClient(const std::string& ip, const unsigned short port)
{
  // if (!ip) {
  //   printf("Bad Parameter\n");
  // }

  ip_ = ip;
  port_ = port;

  pthread_mutex_init(&lock_, NULL);
}

TcpCommandClient::PTC_ErrCode TcpCommandClient::sendCmd(TC_Command* cmd)
{
  if (!cmd) {
    return PTC_ERROR_BAD_PARAMETER;
  }

  if (cmd->header.len != 0 && cmd->data == NULL) {
    return PTC_ERROR_BAD_PARAMETER;
  }

  pthread_mutex_lock(&lock_);
  int fd = tcp_open(ip_.c_str(), port_);
  if (fd < 0) {
    pthread_mutex_unlock(&lock_);
    return PTC_ERROR_CONNECT_SERVER_FAILED;
  }

  unsigned char buffer[128];
  int size = buildHeader((char*)buffer, cmd);

  int ret = write(fd, buffer, size);
  if (ret != size) {
    close(fd);
    pthread_mutex_unlock(&lock_);
    return PTC_ERROR_TRANSFER_FAILED;
  }

  if (cmd->header.len > 0 && cmd->data) {
    ret = write(fd, cmd->data, cmd->header.len);
    if (ret != (int)cmd->header.len) {
      close(fd);
      pthread_mutex_unlock(&lock_);
      return PTC_ERROR_TRANSFER_FAILED;
    }
  }

  TC_Command feedBack;
  ret = readCommand(fd, &feedBack);
  if (ret != 0) {
    close(fd);
    pthread_mutex_unlock(&lock_);
    return PTC_ERROR_TRANSFER_FAILED;
  }

  cmd->ret_data = feedBack.data;
  cmd->ret_size = feedBack.header.len;
  cmd->header.ret_code = feedBack.header.ret_code;

  close(fd);
  pthread_mutex_unlock(&lock_);
  return PTC_ERROR_NO_ERROR;
}

TcpCommandClient::PTC_ErrCode TcpCommandClient::setCalibration(const std::string& content)
{
  TC_Command cmd;
  memset(&cmd, 0, sizeof(TC_Command));
  cmd.header.cmd = PTC_COMMAND_SET_CALIBRATION;
  cmd.header.len = content.size();
  cmd.data = (unsigned char*)strdup(content.c_str());

  PTC_ErrCode errorCode = sendCmd(&cmd);
  if (errorCode != PTC_ERROR_NO_ERROR) {
    free(cmd.data);
    return errorCode;
  }
  free(cmd.data);

  if (cmd.ret_data) {
    // useless data;
    free(cmd.ret_data);
  }

  return (PTC_ErrCode)cmd.header.ret_code;
}

TcpCommandClient::PTC_ErrCode TcpCommandClient::getCalibration(std::string& content)
{
  TC_Command cmd;
  memset(&cmd, 0, sizeof(TC_Command));
  cmd.header.cmd = PTC_COMMAND_GET_CALIBRATION;
  cmd.header.len = 0;
  cmd.data = nullptr;

  PTC_ErrCode errorCode = sendCmd(&cmd);
  if (errorCode != PTC_ERROR_NO_ERROR) {
    return errorCode;
  }

  char* ret_str = (char*)malloc(cmd.ret_size + 1);
  memcpy(ret_str, cmd.ret_data, cmd.ret_size);
  ret_str[cmd.ret_size] = '\0';

  free(cmd.ret_data);

  // *buffer = ret_str;
  // *len = cmd.ret_size + 1;
  content = std::string(ret_str);
  free(ret_str);

  return (PTC_ErrCode)cmd.header.ret_code;
}

TcpCommandClient::PTC_ErrCode TcpCommandClient::getLidarCalibration(std::string& content)
{
  TC_Command cmd;
  memset(&cmd, 0, sizeof(TC_Command));
  cmd.header.cmd = PTC_COMMAND_GET_LIDAR_CALIBRATION;
  cmd.header.len = 0;
  cmd.data = nullptr;

  PTC_ErrCode errorCode = sendCmd(&cmd);
  if (errorCode != PTC_ERROR_NO_ERROR) {
    return errorCode;
  }

  char* ret_str = (char*)malloc(cmd.ret_size + 1);
  memcpy(ret_str, cmd.ret_data, cmd.ret_size);
  ret_str[cmd.ret_size] = '\0';
  free(cmd.ret_data);

  content = std::string(ret_str);
  free(ret_str);

  return (PTC_ErrCode)cmd.header.ret_code;
}

TcpCommandClient::PTC_ErrCode TcpCommandClient::resetCalibration()
{
  TC_Command cmd;
  memset(&cmd, 0, sizeof(TC_Command));
  cmd.header.cmd = PTC_COMMAND_RESET_CALIBRATION;
  cmd.header.len = 0;
  cmd.data = nullptr;

  PTC_ErrCode errorCode = sendCmd(&cmd);
  if (errorCode != PTC_ERROR_NO_ERROR) {
    return errorCode;
  }

  if (cmd.ret_data) {
    // useless data;
    free(cmd.ret_data);
  }

  return (PTC_ErrCode)cmd.header.ret_code;
}

int TcpCommandClient::parseHeader(unsigned char* buffer, TcpCommandHeader* header)
{
  int index = 0;
  header->cmd = buffer[index++];
  header->ret_code = buffer[index++];
  header->len = ((buffer[index] & 0xff) << 24) | ((buffer[index + 1] & 0xff) << 16) |
                ((buffer[index + 2] & 0xff) << 8) | ((buffer[index + 3] & 0xff) << 0);
  return 0;
}

int TcpCommandClient::readCommand(int connfd, TC_Command* cmd)
{
  int ret = 0;
  if (!cmd) {
    return -1;
  }
  memset(cmd, 0, sizeof(TC_Command));
  unsigned char buffer[1500];
  ret = sys_readn(connfd, buffer, 2);
  if (ret <= 0 || buffer[0] != 0x47 || buffer[1] != 0x74) {
    printf("Server Read failed\n");
    return -1;
  }

  ret = sys_readn(connfd, buffer + 2, 6);
  if (ret != 6) {
    printf("Server Read failed\n");
    return -1;
  }

  parseHeader(buffer + 2, &cmd->header);

  if (cmd->header.len > 0) {
    cmd->data = (unsigned char*)malloc(cmd->header.len);
    if (!cmd->data) {
      printf("malloc data error\n");
      return -1;
    }
  }

  ret = sys_readn(connfd, cmd->data, cmd->header.len);
  if (ret != (int)cmd->header.len) {
    free(cmd->data);
    printf("Server Read failed\n");
    return -1;
  }

  return 0;
}

int TcpCommandClient::buildHeader(char* buffer, TC_Command* cmd)
{
  if (!buffer) {
    return -1;
  }
  int index = 0;
  buffer[index++] = 0x47;
  buffer[index++] = 0x74;
  buffer[index++] = cmd->header.cmd;
  buffer[index++] = cmd->header.ret_code;  // color or mono
  buffer[index++] = (cmd->header.len >> 24) & 0xff;
  buffer[index++] = (cmd->header.len >> 16) & 0xff;
  buffer[index++] = (cmd->header.len >> 8) & 0xff;
  buffer[index++] = (cmd->header.len >> 0) & 0xff;

  return index;
}

}  // namespace pandar_pointcloud