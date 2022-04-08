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

#include "jw_interface_serial/jw_interface_serial_core.hpp"

JwInterfaceSerial::JwInterfaceSerial()
: Node("jw_interface_serial"),
  serial_port_("/dev/ttyUSB0"),
  mode_cmd(serial_cmd_academic_mode),
  speed_cmd(serial_cmd_stop),
  control_mode(ControlMode::ManualJwStick),
  status_msg_()
{
  using std::placeholders::_1;

  // rosparam
  serial_port_ = this->declare_parameter<std::string>("serial_port", serial_port_);
  vehicle_cmd_timeout_ = this->declare_parameter<double>("vehicle_cmd_timeout", 0.2);  // [sec]

  // Subscriber
  // command_sub_ = nh_.subscribe<jw_interface_msgs::msg::CommandStamped>("/jw/command", 1,
  // &JwInterfaceSerial::callbackCommand, this);
  command_sub_ = create_subscription<jw_interface_msgs::msg::CommandStamped>(
    "/jw/command", 1, std::bind(&JwInterfaceSerial::callbackCommand, this, _1));

  // Publisher
  status_pub_ = create_publisher<jw_interface_msgs::msg::StatusStamped>(
    "/jw/status", rclcpp::QoS{10}.transient_local());

  openSerial(serial_port_);

  std::thread serial_read_thread(&JwInterfaceSerial::readSerial, this);
  serial_read_thread.detach();

  // Write Serial Timer
  auto timer_callback = std::bind(&JwInterfaceSerial::writeSerial, this);
  auto period =
    std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / 50.0));
  timer_ = std::make_shared<rclcpp::GenericTimer<decltype(timer_callback)>>(
    this->get_clock(), period, std::move(timer_callback),
    this->get_node_base_interface()->get_context());
  this->get_node_timers_interface()->add_timer(timer_, nullptr);
}

JwInterfaceSerial::~JwInterfaceSerial() { closeSerial(); }

void JwInterfaceSerial::callbackCommand(
  const jw_interface_msgs::msg::CommandStamped::ConstSharedPtr msg)
{
  if (
    vehicle_cmd_timeout_ <
    (rclcpp::Time(this->now()) - rclcpp::Time(msg->header.stamp)).seconds()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "vehicle_cmd msg timeout");
    return;
  }

  // TODO(Yamato Ando)
  // u_char mode_char = msg->command.mode.mode;

  std::array<u_char, 2> js_front_back_char = toHexString(msg->command.js_ad.front_back_ratio);
  std::array<u_char, 2> js_left_right_char = toHexString(msg->command.js_ad.left_right_ratio);

  // TODO(Yamato Ando)
  // std::array<u_char, 2> motor_left_rpm_char = toHexString(msg->command.motor_rpm.left_rpm);
  // std::array<u_char, 2> motor_right_rpm_char = toHexString(msg->command.motor_rpm.right_rpm);

  speed_cmd = {
    HEADER_HEX,
    SA_HEX,
    DA_HEX,
    0x00,
    0x69,
    0x10,
    0x01,
    0x00,
    js_front_back_char[0],
    js_front_back_char[1],
    js_left_right_char[0],
    js_left_right_char[1],
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00};

  // TODO(Yamato Ando)
  // speed_cmd = {HEADER_HEX, SA_HEX, DA_HEX, 0x00, 0x69, 0x10, mode_char, 0x00,
  // js_front_back_char[0], js_front_back_char[1], js_left_right_char[0], js_left_right_char[1],
  // motor_left_rpm_char[0], motor_left_rpm_char[1], motor_right_rpm_char[0],
  // motor_right_rpm_char[1], 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

  // int trans_ratio = 0, angular_ratio = 0;
  // convertSpeedToStickRatio(msg->twist_cmd.twist.linear.x, -msg->twist_cmd.twist.angular.z,
  // &trans_ratio, &angular_ratio);
}

bool JwInterfaceSerial::openSerial(const std::string port)
{
  fd = open(port.c_str(), O_RDWR | O_NONBLOCK);

  if (fd < 0) {
    std::cout << "Serial port cannot open." << std::endl;
    return false;
  }

  tcgetattr(fd, &old_tio);
  tio = old_tio;
  tio.c_iflag = 0;
  tio.c_oflag = 0;
  tio.c_cflag |= (tio.c_cflag & ~CSIZE) | CS8 | CREAD | PARENB;
  tio.c_lflag = 0;
  cfsetspeed(&tio, B38400);
  tcsetattr(fd, TCSANOW, &tio);
  ioctl(fd, TCSETS, &tio);

  std::cout << "Serial port open." << std::endl;
  return true;
}

void JwInterfaceSerial::closeSerial()
{
  speed_cmd = serial_cmd_stop;
  writeSerial();

  tcsetattr(fd, TCSANOW, &old_tio);
  close(fd);
  std::cout << "Serial port close." << std::endl;
}

void JwInterfaceSerial::readSerial()
{
  const int buf_size = 64;
  std::deque<u_char> keep_buf;

  auto received_time = std::chrono::system_clock::now();
  while (1) {
    unsigned char read_buf[buf_size] = {0};
    int read_len = read(fd, read_buf, buf_size);

    // get read_buf and push it to keep_buf
    if (read_len > 0) {
      received_time = std::chrono::system_clock::now();
      for (int i = 0; i < read_len; ++i) {
        keep_buf.push_back(read_buf[i]);
      }
    } else {
      const auto now_time = std::chrono::system_clock::now();
      const auto diff_time =
        std::chrono::duration_cast<std::chrono::microseconds>(now_time - received_time).count() /
        1000.0;
      if (diff_time > 100.0) {
        std::cout << "reopen serial" << std::endl;
        closeSerial();
        std::this_thread::sleep_for(std::chrono::microseconds(1000));
        openSerial(serial_port_);
        received_time = std::chrono::system_clock::now();
        mode_cmd = serial_cmd_academic_mode;
        status_msg_.header.stamp = this->now();
        status_msg_.status.motor_rpm.left_rpm = 0.0;
        status_msg_.status.motor_rpm.right_rpm = 0.0;
        status_pub_->publish(status_msg_);
      }

      std::this_thread::sleep_for(std::chrono::microseconds(10000));
      continue;
    }

    if (
      keep_buf.size() > 30 && (keep_buf[0] != 0x01 || keep_buf[1] != 0x03 || keep_buf[2] != 0x08)) {
      while (keep_buf[0] != 0x01 || keep_buf[1] != 0x03 || keep_buf[2] != 0x08) {
        // something error in read_buf
        keep_buf.pop_front();
        if (keep_buf.size() < 3) {
          break;
        }
      }
    }

    int read_size = 0;
    if (
      keep_buf.size() >= 23 && keep_buf[0] == 0x01 && keep_buf[1] == 0x03 && keep_buf[2] == 0x08 &&
      keep_buf[4] == 0xe9) {
      // control mode
      read_size = 23;
    } else if (
      keep_buf.size() >= 15 && keep_buf[0] == 0x01 && keep_buf[1] == 0x03 && keep_buf[2] == 0x08 &&
      keep_buf[4] == 0xe8) {
      // academic mode
      read_size = 15;
    } else {
      if (keep_buf[4] != 0xe9 && keep_buf[4] != 0xe8) {
        keep_buf.pop_front();
      }
      continue;
    }

    // read check sum
    unsigned char read_check_sum = 0x00;
    for (int i = 1; i < read_size - 2; ++i) {
      read_check_sum += keep_buf[i];
    }

    if (read_check_sum != keep_buf[read_size - 1]) {
    } else if (keep_buf[4] == 0xe8) {
      if (keep_buf[6] == 0x06) {
        std::cout << "Academic mode OK." << std::endl;
        control_mode = ControlMode::ManualJoyStick;
        speed_cmd = serial_cmd_stop;
      } else if (keep_buf[6] == 0x15) {
        control_mode = ControlMode::ManualJwStick;
        std::cout << "Academic mode NG..." << std::endl;
      }
    } else if (keep_buf[4] == 0xe9) {
      int timestamp_str_int = keep_buf[7];
      timestamp_str_int <<= 8;
      timestamp_str_int = (timestamp_str_int | keep_buf[6]);
      int r_motor_rpm_str_int = keep_buf[9];
      r_motor_rpm_str_int <<= 8;
      r_motor_rpm_str_int = (r_motor_rpm_str_int | keep_buf[8]);
      if (r_motor_rpm_str_int > 32767) {
        r_motor_rpm_str_int -= 65535;
      }
      int l_motor_rpm_str_int = keep_buf[11];
      l_motor_rpm_str_int <<= 8;
      l_motor_rpm_str_int = (l_motor_rpm_str_int | keep_buf[10]);
      if (l_motor_rpm_str_int > 32767) {
        l_motor_rpm_str_int -= 65535;
      }
      int batt_ampere_str_int = keep_buf[13];
      batt_ampere_str_int <<= 8;
      batt_ampere_str_int = (batt_ampere_str_int | keep_buf[12]);
      int batt_percent_str_int = keep_buf[14];
      int speed_setting_str_int = keep_buf[15];
      int fb_js_ad_str_int = keep_buf[16];
      fb_js_ad_str_int <<= 8;
      fb_js_ad_str_int = (fb_js_ad_str_int | keep_buf[17]);
      int lr_js_ad_str_int = keep_buf[18];
      lr_js_ad_str_int <<= 8;
      lr_js_ad_str_int = (lr_js_ad_str_int | keep_buf[19]);

      // double trans_vel = 0, angular_vel = 0;

      status_msg_.header.stamp = this->now();
      status_msg_.header.frame_id = "base_link";
      status_msg_.status.time_stamp.time_stamp = timestamp_str_int;
      status_msg_.status.motor_rpm.left_rpm = l_motor_rpm_str_int;
      status_msg_.status.motor_rpm.right_rpm = r_motor_rpm_str_int;
      status_msg_.status.battery.amperage = batt_ampere_str_int;
      status_msg_.status.battery.charge = batt_percent_str_int;
      status_msg_.status.speed_setting.speed_setting = speed_setting_str_int;
      status_msg_.status.js_ad.front_back_ratio = fb_js_ad_str_int;
      status_msg_.status.js_ad.left_right_ratio = lr_js_ad_str_int;

      // convertRpmToSpeed(r_motor_rpm_str_int, l_motor_rpm_str_int, &trans_vel, &angular_vel);

      status_pub_->publish(status_msg_);
    }

    // erase keep_buf
    keep_buf.erase(keep_buf.begin(), keep_buf.begin() + read_size);

    std::this_thread::sleep_for(std::chrono::microseconds(10000));
  }
}

void JwInterfaceSerial::writeSerial()
{
  std::vector<unsigned char> command;

  if (mode_cmd.empty()) {
    command = speed_cmd;
  } else {
    command = mode_cmd;
    mode_cmd.clear();
  }

  if (command.size() != 15 && command.size() != 23) {
    std::cout << "Serial write command error!!!" << std::endl;
    return;
  }

  static unsigned char frame_no = 0x00;
  command[3] = frame_no;
  if (++frame_no > 128) {
    frame_no = 0x00;
  }

  command[command.size() - 1] = calcCheckSum(command);

  // set write buf
  unsigned char buf[command.size()] = {0};
  for (std::size_t i = 0; i < command.size(); ++i) {
    buf[i] = command[i];
  }

  // serial write
  write(fd, buf, sizeof(buf));
}

unsigned char JwInterfaceSerial::calcCheckSum(const std::vector<unsigned char> & cmd_array)
{
  unsigned char check_sum = 0x00;
  for (size_t i = 1; i < cmd_array.size(); ++i) {
    check_sum += cmd_array[i];
  }
  return check_sum;
}

std::array<u_char, 2> JwInterfaceSerial::toHexString(const int val)
{
  std::array<u_char, 2> hex_char;

  char val_char[8];

  std::snprintf(&val_char[0], sizeof(&val_char[0]), "%8x", static_cast<int>(val));

  for (int i = 0; i < 8; ++i) {
    if (val_char[i] == ' ' || val_char[i] == '\0') {
      val_char[i] = '0';
    }
  }

  std::string lowerbyte_string(val_char + 6, val_char + 8);
  std::string higherbyte_string(val_char + 4, val_char + 6);
  hex_char[0] = std::stoi(lowerbyte_string, nullptr, 16);
  hex_char[1] = std::stoi(higherbyte_string, nullptr, 16);

  return hex_char;
}
