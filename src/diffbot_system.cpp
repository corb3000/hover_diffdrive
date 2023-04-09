// Copyright 2021 ros2_control Development Team
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

#include "hover_diffdrive/diffbot_system.hpp"
#include "hover_diffdrive/hover_comms.h"
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include <iostream>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
// #include "std_msgs/msg/string.hpp"
#include <rclcpp_components/register_node_macro.hpp>



namespace  hover_diffdrive
{
HardwarePub::HardwarePub() : Node("hardware_publisher")
{
  status_pub_   = this->create_publisher<robot_interfaces::msg::MotorStatus>("hover_diffdrive/status", 10);
  debug_pub_   = this->create_publisher<robot_interfaces::msg::MotorDebug>("hover_diffdrive/debug", 10);
}

void HardwarePub::publishData(double v, double t)
{
  auto message = robot_interfaces::msg::MotorStatus();
  message.voltage = v;
  message.temp = t;
  status_pub_->publish(message);
}
void HardwarePub::publishDebugData(int16_t sl, int16_t sr, int16_t cl, int16_t cr, int16_t clx, int16_t crx)
{
  auto message = robot_interfaces::msg::MotorDebug();
  message.speedr = sr;
  message.speedl = sl;
  message.wheelr = cr;
  message.wheell = cl;
  message.wheelrx = crx;
  message.wheellx = clx;

  debug_pub_->publish(message);
}

hardware_interface::CallbackReturn HoverDiffDrive::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  base_x_ = 0.0;
  base_y_ = 0.0;
  base_theta_ = 0.0;
  hw_pub_ = std::make_shared<HardwarePub>();  //fire up the publisher node

  hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // DiffBotSystem has exactly two states and one command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("HoverDiffDrive"),
        "1Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }
    
    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("HoverDiffDrive"),
        "2Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("HoverDiffDrive"),
        "3Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("HoverDiffDrive"),
        "4Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("HoverDiffDrive"),
        "5Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> HoverDiffDrive::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> HoverDiffDrive::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn HoverDiffDrive::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  
  hover_comms.setup();


  // set some default values
  for (auto i = 0u; i < hw_positions_.size(); i++)
  {
    if (std::isnan(hw_positions_[i]))
    {
      hw_positions_[i] = 0;
      hw_velocities_[i] = 0;
      hw_commands_[i] = 0;
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("HoverDiffDrive"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn HoverDiffDrive::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{


  RCLCPP_INFO(rclcpp::get_logger("HoverDiffDrive"), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type HoverDiffDrive::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  double radius = 0.02;  // radius of the wheels
  double dist_w = 0.1;   // distance between the wheels
  SerialFeedback read_msg = hover_comms.readValues();
  hw_positions_[0] =  M_PI * 2 * (read_msg.wheelR_cnt + (read_msg.wheelR_multR * ENCODER_MAX)) / TICKS_PER_ROTATION ; // 0 is Right
  hw_velocities_[0] = read_msg.speedR_meas * 0.10472;  // convert rpm to Rad/s
  hw_positions_[1] =  M_PI * 2 * (read_msg.wheelL_cnt + (read_msg.wheelL_multL * ENCODER_MAX))/TICKS_PER_ROTATION ;// 1 is Left
  hw_velocities_[1] = read_msg.speedL_meas * 0.10472;  // convert rpm to Rad/s

  // for (uint i = 0; i < hw_commands_.size(); i++)
  // {
    
  //   // Simulate DiffBot wheels's movement as a first-order system
  //   // Update the joint status: this is a revolute joint without any limit.
  //   // Simply integrates
  //   hw_positions_[i] = hw_positions_[1] + period.seconds() * hw_commands_[i];
  //   hw_velocities_[i] = hw_commands_[i];

  // }

  // Update the free-flyer, i.e. the base notation using the classical
  // wheel differentiable kinematics
  double base_dx = 0.5 * radius * (hw_commands_[0] + hw_commands_[1]) * cos(base_theta_);
  double base_dy = 0.5 * radius * (hw_commands_[0] + hw_commands_[1]) * sin(base_theta_);
  double base_dtheta = radius * (hw_commands_[0] - hw_commands_[1]) / dist_w;
  base_x_ += base_dx * period.seconds();
  base_y_ += base_dy * period.seconds();
  base_theta_ += base_dtheta * period.seconds();

  double v = read_msg.batVoltage/100.0;
  double t = read_msg.boardTemp/10.0;
  hw_pub_->publishData(v, t);  //publish to topic
  hw_pub_->publishDebugData(read_msg.speedR_meas, read_msg.speedL_meas, read_msg.wheelR_cnt, read_msg.wheelL_cnt, read_msg.wheelR_multR, read_msg.wheelL_multL);  //publish debug to topic

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type  hover_diffdrive::HoverDiffDrive::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  double vel[2] = {hw_commands_[1], hw_commands_[0]};
  hover_comms.setMotorValues(vel);


  return hardware_interface::return_type::OK;
}

}  // namespace  hover_diffdrive

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
   hover_diffdrive::HoverDiffDrive, hardware_interface::SystemInterface)
