// Copyright 2022 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include "rclcpp/rclcpp.hpp"

#include "jw_interface_msgs/msg/command_stamped.hpp"
#include "jw_interface_msgs/msg/status_stamped.hpp"

#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>

#include <cstdio>
#include <cstdlib>
#include <deque>
#include <string>
#include <thread>
#include <vector>

class JwInterfaceSerial : public rclcpp::Node
{
  enum class ControlMode { ManualJwStick, ManualJoyStick, Auto };
  // Header, SourceAddress, DestinationAddress, FrameNO, CoMmanD, BitC???, Data[], CHECK_SUM
  const unsigned char HEADER_HEX = 0x01;
  const unsigned char SA_HEX = 0x08;
  const unsigned char DA_HEX = 0x03;
  const unsigned char FNO_HEX = 0x00;
  const unsigned char CSUM_HEX = 0x00;

  const std::vector<unsigned char> serial_cmd_academic_mode = {
    HEADER_HEX, SA_HEX, DA_HEX, FNO_HEX, 0x68, 0x08, 0x01, 0x00,
    0x00,       0x00,   0x01,   0x00,    0x00, 0x00, 0x00};
  const std::vector<unsigned char> serial_cmd_normal_mode = {
    HEADER_HEX, SA_HEX, DA_HEX, FNO_HEX, 0x68, 0x08, 0x00, 0x00,
    0x00,       0x00,   0x00,   0x00,    0x00, 0x00, 0x00};
  const std::vector<unsigned char> serial_cmd_stop = {
    HEADER_HEX, SA_HEX, DA_HEX, FNO_HEX, 0x69, 0x10, 0x02, 0x00, 0x00, 0x00, 0x00,    0x00,
    0x00,       0x00,   0x00,   0x00,    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, CSUM_HEX};

public:
  JwInterfaceSerial();
  ~JwInterfaceSerial();

  void run();

private:
  void callbackCommand(const jw_interface_msgs::msg::CommandStamped::ConstSharedPtr msg);

  bool openSerial(const std::string port);
  void closeSerial();

  void readSerial();
  void writeSerial();

  unsigned char calcCheckSum(const std::vector<unsigned char> & cmd_array);
  std::array<u_char, 2> toHexString(const int val);

  rclcpp::Subscription<jw_interface_msgs::msg::CommandStamped>::SharedPtr command_sub_;

  rclcpp::Publisher<jw_interface_msgs::msg::StatusStamped>::SharedPtr status_pub_;

  rclcpp::TimerBase::SharedPtr timer_;

  // rosparam
  std::string serial_port_;
  double vehicle_cmd_timeout_;

  // variable
  int fd;
  struct termios tio, old_tio;
  std::vector<unsigned char> mode_cmd, speed_cmd;

  ControlMode control_mode;
  jw_interface_msgs::msg::StatusStamped status_msg_;
};
