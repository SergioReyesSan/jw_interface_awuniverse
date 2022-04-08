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

#ifndef JW_INTERFACE_AWIV_ADAPT_RECEIVER__JW_INTERFACE_AWIV_ADAPT_RECEIVER_HPP_
#define JW_INTERFACE_AWIV_ADAPT_RECEIVER__JW_INTERFACE_AWIV_ADAPT_RECEIVER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_vehicle_msgs/msg/control_mode_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/gear_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/steering_report.hpp>
// #include <autoware_auto_vehicle_msgs/msg/turn_indicators_report.hpp>
#include "vehicle_info_util/vehicle_info_util.hpp"

#include <autoware_auto_vehicle_msgs/msg/velocity_report.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <jw_interface_msgs/msg/status_stamped.hpp>
#include <tier4_debug_msgs/msg/bool_stamped.hpp>
#include <tier4_debug_msgs/msg/float32_stamped.hpp>
#include <tier4_vehicle_msgs/msg/battery_status.hpp>

class JwInterfaceAWIVAdaptReceiver : public rclcpp::Node
{
public:
  explicit JwInterfaceAWIVAdaptReceiver(const rclcpp::NodeOptions & node_options);

private:
  void callbackVehicleStatus(
    const jw_interface_msgs::msg::StatusStamped::ConstSharedPtr vehicle_status_msg_ptr);

  autoware_auto_vehicle_msgs::msg::VelocityReport convertRpmToTwist(
    const jw_interface_msgs::msg::MotorRPMStatus & motor_rpm_msg);

  // ros subscriber
  rclcpp::Subscription<jw_interface_msgs::msg::StatusStamped>::SharedPtr vehicle_status_sub_;

  // ros publisher
  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::ControlModeReport>::SharedPtr
    control_mode_status_pub_;
  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::GearReport>::SharedPtr gear_status_pub_;
  // rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport>::SharedPtr
  // turn_indicators_status_pub_;
  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::VelocityReport>::SharedPtr
    velocity_status_pub_;
  rclcpp::Publisher<tier4_debug_msgs::msg::Float32Stamped>::SharedPtr velocity_kmph_pub_;
  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::SteeringReport>::SharedPtr
    steering_status_pub_;
  rclcpp::Publisher<tier4_debug_msgs::msg::Float32Stamped>::SharedPtr steer_wheel_deg_pub_;
  rclcpp::Publisher<tier4_vehicle_msgs::msg::BatteryStatus>::SharedPtr battery_charge_status_pub_;

  vehicle_info_util::VehicleInfo vehicle_info_;
  double wheel_base_;
  double wheel_tread_;
  double wheel_radius_;
};

#endif  // JW_INTERFACE_AWIV_ADAPT_RECEIVER__JW_INTERFACE_AWIV_ADAPT_RECEIVER_HPP_
