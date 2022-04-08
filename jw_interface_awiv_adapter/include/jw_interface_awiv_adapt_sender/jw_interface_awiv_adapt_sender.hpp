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

#ifndef JW_INTERFACE_AWIV_ADAPT_SENDER__JW_INTERFACE_AWIV_ADAPT_SENDER_HPP_
#define JW_INTERFACE_AWIV_ADAPT_SENDER__JW_INTERFACE_AWIV_ADAPT_SENDER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <jw_interface_msgs/msg/command_stamped.hpp>
// #include <autoware_auto_vehicle_msgs/msg/turn_indicators_command.hpp>
#include "vehicle_info_util/vehicle_info_util.hpp"

#include <autoware_auto_vehicle_msgs/msg/engage.hpp>
#include <autoware_auto_vehicle_msgs/msg/gear_command.hpp>
#include <tier4_debug_msgs/msg/bool_stamped.hpp>
#include <tier4_vehicle_msgs/msg/vehicle_emergency_stamped.hpp>

class JwInterfaceAWIVAdaptSender : public rclcpp::Node
{
public:
  explicit JwInterfaceAWIVAdaptSender(const rclcpp::NodeOptions & node_options);

private:
  void callbackAckermannControlCmd(
    const autoware_auto_control_msgs::msg::AckermannControlCommand::ConstSharedPtr msg_ptr);
  // void callbackTurnSignalCmd(
  //  const autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand::ConstSharedPtr msg_ptr);
  void callbackEngage(const autoware_auto_vehicle_msgs::msg::Engage::ConstSharedPtr msg_ptr);
  void callbackGearCmd(const autoware_auto_vehicle_msgs::msg::GearCommand::ConstSharedPtr msg_ptr);
  void callbackEmergencyCmd(
    const tier4_vehicle_msgs::msg::VehicleEmergencyStamped::ConstSharedPtr msg_ptr);
  void onTimer();
  void publishCommand();
  void publishZeroCommand();
  void convertSpeedToStickRatio(
    const double trans_vel, const double angular_vel, int * const trans_ratio,
    int * const angular_ratio);

  // ros subscriber
  rclcpp::Subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>::SharedPtr
    control_cmd_sub_;
  // rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand>::SharedPtr
  // turn_signal_cmd_sub_;
  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::Engage>::SharedPtr engage_cmd_sub_;
  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::GearCommand>::SharedPtr gear_cmd_sub_;
  rclcpp::Subscription<tier4_vehicle_msgs::msg::VehicleEmergencyStamped>::SharedPtr
    emergency_cmd_sub_;

  // ros publisher
  rclcpp::Publisher<jw_interface_msgs::msg::CommandStamped>::SharedPtr jw_command_pub_;
  rclcpp::Publisher<tier4_debug_msgs::msg::BoolStamped>::SharedPtr jw_engage_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr jw_debug_twist_pub_;

  jw_interface_msgs::msg::CommandStamped jw_command_stamped_msg_;
  rclcpp::TimerBase::SharedPtr cmd_timer_;

  vehicle_info_util::VehicleInfo vehicle_info_;
  double wheel_base_;
  double wheel_tread_;
  double wheel_radius_;
  double steering_offset_deg_;
  int angular_ratio_correction_cycle_;
  int angular_ratio_correction_coefficient_;
  autoware_auto_vehicle_msgs::msg::GearCommand::SharedPtr gear_cmd_ptr_;
  bool is_emergency_{false};

  double loop_rate_;
  double vehicle_cmd_timeout_sec_;
  rclcpp::Time prev_vehicle_cmd_stamp_{0, 0, RCL_ROS_TIME};
};

#endif  // JW_INTERFACE_AWIV_ADAPT_SENDER__JW_INTERFACE_AWIV_ADAPT_SENDER_HPP_
