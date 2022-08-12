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

#include "jw_interface_awiv_adapt_sender/jw_interface_awiv_adapt_sender.hpp"

#include <memory>

JwInterfaceAWIVAdaptSender::JwInterfaceAWIVAdaptSender(const rclcpp::NodeOptions & node_options)
: Node("jw_interface_awiv_adapt_sender", node_options),
  vehicle_info_(vehicle_info_util::VehicleInfoUtil(*this).getVehicleInfo())
{
  using std::placeholders::_1;

  wheel_base_ = vehicle_info_.wheel_base_m;
  wheel_tread_ = vehicle_info_.wheel_tread_m;
  wheel_radius_ = vehicle_info_.wheel_radius_m;
  steering_offset_deg_ = this->declare_parameter<double>("steering_offset_deg", 0.0);
  angular_ratio_correction_cycle_ =
    this->declare_parameter<int>("angular_ratio_correction_cycle", 0);
  angular_ratio_correction_coefficient_ =
    this->declare_parameter<int>("angular_ratio_correction_coefficient", 0);
  loop_rate_ = declare_parameter<double>("loop_rate", 50.0);
  vehicle_cmd_timeout_sec_ = declare_parameter<double>("vehicle_cmd_timeout_sec", 1.0);

  // subscriber
  control_cmd_sub_ = create_subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>(
    "/control/command/control_cmd", 1,
    std::bind(&JwInterfaceAWIVAdaptSender::callbackAckermannControlCmd, this, _1));
  // turn_signal_cmd_sub_ = create_subscription(
  //  "/control/turn_signal_cmd", 1,
  //  std::bind(&JwInterfaceAWIVAdaptSender::callbackTurnSignalCmd, this, _1));
  engage_cmd_sub_ = create_subscription<autoware_auto_vehicle_msgs::msg::Engage>(
    "/vehicle/engage", 1, std::bind(&JwInterfaceAWIVAdaptSender::callbackEngage, this, _1));
  emergency_cmd_sub_ = create_subscription<tier4_vehicle_msgs::msg::VehicleEmergencyStamped>(
    "/control/command/emergency_cmd", 1,
    std::bind(&JwInterfaceAWIVAdaptSender::callbackEmergencyCmd, this, _1));
  gear_cmd_sub_ = create_subscription<autoware_auto_vehicle_msgs::msg::GearCommand>(
    "/control/command/gear_cmd", 1,
    std::bind(&JwInterfaceAWIVAdaptSender::callbackGearCmd, this, _1));

  // publisher
  jw_command_pub_ = create_publisher<jw_interface_msgs::msg::CommandStamped>(
    "/jw/command", rclcpp::QoS{10}.transient_local());
  jw_debug_twist_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
    "/jw/debug/command/twist", rclcpp::QoS{10}.transient_local());
  jw_engage_pub_ = create_publisher<tier4_debug_msgs::msg::BoolStamped>(
    "/jw/engage", rclcpp::QoS{10}.transient_local());

  // Timer
  const auto period_ns = rclcpp::Rate(loop_rate_).period();
  cmd_timer_ = rclcpp::create_timer(
    this, this->get_clock(), period_ns, std::bind(&JwInterfaceAWIVAdaptSender::onTimer, this));
}

void JwInterfaceAWIVAdaptSender::onTimer()
{
  const auto time_diff = this->now() - prev_vehicle_cmd_stamp_;
  if (time_diff.seconds() > vehicle_cmd_timeout_sec_) {
    publishZeroCommand();
    RCLCPP_WARN_STREAM_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000 /* ms */, "vehicle_cmd is not subscribed");
  } else {
    publishCommand();
  }
}

void JwInterfaceAWIVAdaptSender::publishZeroCommand()
{
  jw_command_stamped_msg_.header.stamp = this->now();
  jw_command_stamped_msg_.command.js_ad.front_back_ratio = 0.0;
  jw_command_stamped_msg_.command.js_ad.left_right_ratio = 0.0;
  jw_command_pub_->publish(jw_command_stamped_msg_);
}

void JwInterfaceAWIVAdaptSender::publishCommand()
{
  jw_command_pub_->publish(jw_command_stamped_msg_);
}

void JwInterfaceAWIVAdaptSender::callbackAckermannControlCmd(
  const autoware_auto_control_msgs::msg::AckermannControlCommand::ConstSharedPtr msg_ptr)
{
  if (!gear_cmd_ptr_) {
    RCLCPP_WARN_STREAM(this->get_logger(), "gear command is not subscribed");
    return;
  }

  double steering_offset_rad = steering_offset_deg_ * M_PI / 180.0;
  jw_command_stamped_msg_.header.stamp = msg_ptr->stamp;

  jw_command_stamped_msg_.command.mode.mode = jw_interface_msgs::msg::ModeCommand::JS_AD_CONTROL;

  geometry_msgs::msg::Twist twist_cmd;
  twist_cmd.linear.x = msg_ptr->longitudinal.speed;
  twist_cmd.angular.z = msg_ptr->longitudinal.speed *
                        std::tan(msg_ptr->lateral.steering_tire_angle + steering_offset_rad) /
                        wheel_base_;

  int trans_ratio{0};
  int angular_ratio{0};
  if (
    !is_emergency_ &&
    gear_cmd_ptr_->command != autoware_auto_vehicle_msgs::msg::GearCommand::PARK) {
    convertSpeedToStickRatio(
      twist_cmd.linear.x, -twist_cmd.angular.z, &trans_ratio, &angular_ratio);
  }

  jw_command_stamped_msg_.command.js_ad.front_back_ratio = trans_ratio;
  jw_command_stamped_msg_.command.js_ad.left_right_ratio = angular_ratio;

  geometry_msgs::msg::TwistStamped debug_twist_msg;
  debug_twist_msg.header.stamp = msg_ptr->stamp;
  debug_twist_msg.twist = twist_cmd;
  jw_debug_twist_pub_->publish(debug_twist_msg);

  prev_vehicle_cmd_stamp_ = msg_ptr->stamp;
}

void JwInterfaceAWIVAdaptSender::callbackEngage(
  const autoware_auto_vehicle_msgs::msg::Engage::ConstSharedPtr msg_ptr)
{
  tier4_debug_msgs::msg::BoolStamped engage_msg{};
  engage_msg.stamp = msg_ptr->stamp;
  engage_msg.data = msg_ptr->engage;
  jw_engage_pub_->publish(engage_msg);
}

void JwInterfaceAWIVAdaptSender::callbackGearCmd(
  const autoware_auto_vehicle_msgs::msg::GearCommand::ConstSharedPtr msg_ptr)
{
  gear_cmd_ptr_ = std::make_shared<autoware_auto_vehicle_msgs::msg::GearCommand>(*msg_ptr);
}

void JwInterfaceAWIVAdaptSender::callbackEmergencyCmd(
  const tier4_vehicle_msgs::msg::VehicleEmergencyStamped::ConstSharedPtr msg_ptr)
{
  is_emergency_ = msg_ptr->emergency;
}

void JwInterfaceAWIVAdaptSender::convertSpeedToStickRatio(
  const double trans_vel, const double angular_vel, int * const trans_ratio,
  int * const angular_ratio)
{
  const double gear_ratio = 12.64;
  const double tire_circumference = 2.0 * M_PI * wheel_radius_;

  if (trans_vel >= 0) {
    const double trans_min_ratio = 3.0;
    const double trans_max_ratio = 100.0;
    const double trans_min_rpm = 20.0;
    const double trans_max_rpm = 1680.0;
    const double trans_min_vel = trans_min_rpm / 60.0 * tire_circumference / gear_ratio / 2.0;
    const double trans_max_vel = trans_max_rpm / 60.0 * tire_circumference / gear_ratio / 2.0;
    *trans_ratio = (trans_vel > trans_min_vel)
                     ? ((trans_vel - trans_min_vel) / (trans_max_vel - trans_min_vel) *
                          (trans_max_ratio - trans_min_ratio) +
                        trans_min_ratio)
                     : 0;
  } else {
    const double trans_negative_min_ratio = 5.0;
    const double trans_negative_max_ratio = 100.0;
    const double trans_negative_min_rpm = 15.0;
    const double trans_negative_max_rpm = 815.0;
    const double trans_negative_min_vel =
      trans_negative_min_rpm / 60.0 * tire_circumference / gear_ratio / 2.0;
    const double trans_negative_max_vel =
      trans_negative_max_rpm / 60.0 * tire_circumference / gear_ratio / 2.0;
    *trans_ratio = (std::fabs(trans_vel) > trans_negative_min_vel)
                     ? ((std::fabs(trans_vel) - trans_negative_min_vel) /
                          (trans_negative_max_vel - trans_negative_min_vel) *
                          (trans_negative_max_ratio - trans_negative_min_ratio) +
                        trans_negative_min_ratio)
                     : 0;
    *trans_ratio *= -1;
  }

  const double angular_min_ratio = 14;
  const double angular_max_ratio = 100.0;
  const double angular_min_rpm = 20;
  const double angular_max_rpm = 320.0;
  const double angular_min_vel =
    angular_min_rpm / 60.0 * tire_circumference / gear_ratio / wheel_tread_;
  const double angular_max_vel =
    angular_max_rpm / 60.0 * tire_circumference / gear_ratio / wheel_tread_;
  *angular_ratio =
    (std::fabs(angular_vel) > angular_min_vel)
      ? ((std::fabs(angular_vel) - angular_min_vel) / (angular_max_vel - angular_min_vel) *
           (angular_max_ratio - angular_min_ratio) +
         angular_min_ratio)
      : 0;
  if (angular_vel < 0) {
    *angular_ratio *= -1;
  }

  static int cycle_count = 0;
  if (
    angular_ratio_correction_cycle_ > 0 && cycle_count++ >= angular_ratio_correction_cycle_ &&
    *angular_ratio == 0 && *trans_ratio != 0) {
    cycle_count = 0;
    *angular_ratio += angular_ratio_correction_coefficient_;
  }

  // limit
  if (*trans_ratio > 100) {
    *trans_ratio = 100;
  } else if (*trans_ratio < -100) {
    *trans_ratio = -100;
  }

  if (*angular_ratio > 100) {
    *angular_ratio = 100;
  } else if (*angular_ratio < -100) {
    *angular_ratio = -100;
  }
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(JwInterfaceAWIVAdaptSender)
