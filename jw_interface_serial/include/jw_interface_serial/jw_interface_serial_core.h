#pragma once

#include <cstdio>
#include <cstdlib>
#include <thread>
#include <deque>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <termios.h>


#include <ros/ros.h>

#include <jw_interface_msgs/CommandStamped.h>
#include <jw_interface_msgs/StatusStamped.h>

class JwInterfaceSerial {

  enum class ControlMode {ManualJwStick, ManualJoyStick, Auto};
  // Header, SourceAdress, DestinationAdress, FrameNO, CoMmanD, BitC???, Data[], CHECK_SUM
  const unsigned char HEADER_HEX = 0x01;
  const unsigned char SA_HEX = 0x08;
  const unsigned char DA_HEX = 0x03;
  const unsigned char FNO_HEX = 0x00;
  const unsigned char CSUM_HEX = 0x00;

  const std::vector<unsigned char> serial_cmd_academic_mode = {HEADER_HEX, SA_HEX, DA_HEX, FNO_HEX, 0x68, 0x08, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00};
  const std::vector<unsigned char> serial_cmd_normal_mode = {HEADER_HEX, SA_HEX, DA_HEX, FNO_HEX, 0x68, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  const std::vector<unsigned char> serial_cmd_stop = {HEADER_HEX, SA_HEX, DA_HEX, FNO_HEX, 0x69, 0x10, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, CSUM_HEX};

 public:
  JwInterfaceSerial(const ros::NodeHandle &nh, const ros::NodeHandle &private_nh);
  ~JwInterfaceSerial();

  void run();

 private:
  void callbackCommand(const jw_interface_msgs::CommandStamped::ConstPtr &msg);

  bool openSerial(const std::string port);
  bool closeSerial();

  void readSerial();
  void writeSerial();

  unsigned char calcCheckSum(const std::vector<unsigned char> &cmd_array);
  std::array<u_char, 2> toHexString(const int val);

  // void convertSpeedToStickRatio(const double trans_vel, const double angular_vel, int* const trans_ratio, int* const angular_ratio);
  // void convertRpmToSpeed(const int right_motor_rpm, const int left_motor_rpm, double* const trans_vel, double* const angular_vel);

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  ros::Subscriber command_sub_;

  ros::Publisher status_pub_;

  //rosparam
  std::string serial_port_;
  double tire_circumference_;
  double gear_ratio_;
  double tread_;
  double vehicle_cmd_timeout_;

  // variable
  int fd;
  struct termios tio, oldtio;
  std::vector<unsigned char> mode_cmd, speed_cmd;

  ControlMode control_mode;
};
