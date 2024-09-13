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

#include "jw_interface_awiv_adapt_receiver/jw_interface_awiv_adapt_receiver.hpp"

#include "std_msgs/msg/header.hpp"

JwInterfaceAWIVAdaptReceiver::JwInterfaceAWIVAdaptReceiver(const rclcpp::NodeOptions & node_options)
: Node("jw_interface_awiv_adapt_receiver", node_options),
  vehicle_info_(autoware::vehicle_info_utils::VehicleInfoUtils(*this).getVehicleInfo())
{
  using std::placeholders::_1;

  wheel_base_ = vehicle_info_.wheel_base_m;
  wheel_tread_ = vehicle_info_.wheel_tread_m;
  wheel_radius_ = vehicle_info_.wheel_radius_m;
  speed_scale_factor_ = declare_parameter<double>("speed_scale_factor", 1.0);

  // subscriber
  vehicle_status_sub_ = create_subscription<jw_interface_msgs::msg::StatusStamped>(
    "/jw/status", 1, std::bind(&JwInterfaceAWIVAdaptReceiver::callbackVehicleStatus, this, _1));

  // publisher
  control_mode_status_pub_ = create_publisher<autoware_vehicle_msgs::msg::ControlModeReport>(
    "/vehicle/status/control_mode", rclcpp::QoS{10}.transient_local());
  gear_status_pub_ = create_publisher<autoware_vehicle_msgs::msg::GearReport>(
    "/vehicle/status/gear_status", rclcpp::QoS{10}.transient_local());
  // turn_signal_pub_ = create_publisher<autoware_vehicle_msgs::msg::TurnIndicatorsStatus>(
  // "/vehicle/status/turn_indicators_status", rclcpp::QoS{10}.transient_local());
  velocity_status_pub_ = create_publisher<autoware_vehicle_msgs::msg::VelocityReport>(
    "/vehicle/status/velocity_status", rclcpp::QoS{10}.transient_local());
  velocity_kmph_pub_ = create_publisher<tier4_debug_msgs::msg::Float32Stamped>(
    "/vehicle/status/velocity_kmph", rclcpp::QoS{10}.transient_local());
  steering_status_pub_ = create_publisher<autoware_vehicle_msgs::msg::SteeringReport>(
    "/vehicle/status/steering_status", rclcpp::QoS{10}.transient_local());
  steer_wheel_deg_pub_ = create_publisher<tier4_debug_msgs::msg::Float32Stamped>(
    "/vehicle/status/steering_wheel_deg", rclcpp::QoS{10}.transient_local());
  battery_charge_status_pub_ = create_publisher<tier4_vehicle_msgs::msg::BatteryStatus>(
    "/vehicle/status/battery_charge", rclcpp::QoS{10}.transient_local());
}

void JwInterfaceAWIVAdaptReceiver::callbackVehicleStatus(
  const jw_interface_msgs::msg::StatusStamped::ConstSharedPtr jw_status_msg_ptr)
{
  autoware_vehicle_msgs::msg::ControlModeReport control_mode_status_msg;
  control_mode_status_msg.stamp = jw_status_msg_ptr->header.stamp;
  // TODO(someone): fix handling control mode
  control_mode_status_msg.mode = autoware_vehicle_msgs::msg::ControlModeReport::AUTONOMOUS;
  control_mode_status_pub_->publish(control_mode_status_msg);

  auto velocity_status_msg = convertRpmToTwist(jw_status_msg_ptr->status.motor_rpm);
  velocity_status_msg.header = jw_status_msg_ptr->header;
  velocity_status_pub_->publish(velocity_status_msg);

  autoware_vehicle_msgs::msg::GearReport gear_report_msg;
  gear_report_msg.stamp = jw_status_msg_ptr->header.stamp;
  if (velocity_status_msg.longitudinal_velocity >= 0.0) {
    gear_report_msg.report = autoware_vehicle_msgs::msg::GearReport::DRIVE;
  } else {
    gear_report_msg.report = autoware_vehicle_msgs::msg::GearReport::REVERSE;
  }
  gear_status_pub_->publish(gear_report_msg);

  // turn_signal_pub_->publish(turn_signal_msg);

  tier4_debug_msgs::msg::Float32Stamped velocity_kmph_msg;
  velocity_kmph_msg.data = velocity_status_msg.longitudinal_velocity * 3.6;
  velocity_kmph_msg.stamp = this->now();
  velocity_kmph_pub_->publish(velocity_kmph_msg);

  autoware_vehicle_msgs::msg::SteeringReport steering_status_msg;
  steering_status_msg.stamp = jw_status_msg_ptr->header.stamp;
  steering_status_msg.steering_tire_angle = velocity_status_msg.longitudinal_velocity != 0.0
                                              ? std::atan(
                                                  velocity_status_msg.heading_rate * wheel_base_ /
                                                  velocity_status_msg.longitudinal_velocity)
                                              : 0.0;
  steering_status_pub_->publish(steering_status_msg);

  tier4_debug_msgs::msg::Float32Stamped steer_wheel_deg_msg;
  steer_wheel_deg_msg.data = steering_status_msg.steering_tire_angle * 180.0 / M_PI;
  steer_wheel_deg_msg.stamp = this->now();
  steer_wheel_deg_pub_->publish(steer_wheel_deg_msg);

  tier4_vehicle_msgs::msg::BatteryStatus battery_charge_msg;
  battery_charge_msg.energy_level = jw_status_msg_ptr->status.battery.charge;
  battery_charge_msg.stamp = this->now();
  battery_charge_status_pub_->publish(battery_charge_msg);
}

autoware_vehicle_msgs::msg::VelocityReport JwInterfaceAWIVAdaptReceiver::convertRpmToTwist(
  const jw_interface_msgs::msg::MotorRPMStatus & motor_rpm_msg)
{
  autoware_vehicle_msgs::msg::VelocityReport twist;

  const double gear_ratio = 12.64;
  const double tire_circumference = 2.0 * M_PI * wheel_radius_;

  const double right_velocity = motor_rpm_msg.right_rpm / 60.0 * tire_circumference / gear_ratio;
  const double left_velocity = motor_rpm_msg.left_rpm / 60.0 * tire_circumference / gear_ratio;
  twist.longitudinal_velocity = (right_velocity + left_velocity) / 2.0 * speed_scale_factor_;
  twist.heading_rate = (right_velocity - left_velocity) / wheel_tread_ * speed_scale_factor_;

  // std::cout << "v:" << twist.linear.x << " w:" << twist.angular.z << std::endl;

  return twist;
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(JwInterfaceAWIVAdaptReceiver)
