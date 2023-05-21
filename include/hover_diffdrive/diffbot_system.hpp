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

#ifndef  HOVER_DIFFDRIVE__DIFFBOT_SYSTEM_HPP_
#define  HOVER_DIFFDRIVE__DIFFBOT_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp" 
#include "robot_interfaces/msg/motor_status.hpp"
#include "robot_interfaces/msg/motor_debug.hpp"

#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "hover_diffdrive/visibility_control.h"
#include "hover_comms.h"

 


namespace  hover_diffdrive
{

class HardwarePub : public rclcpp::Node  //the node definition for the publisher to talk to micro-ROS agent
{
  public:
    HardwarePub();
    void publishData(double, double);
    void publishDebugData(int16_t, int16_t, int16_t, int16_t, int16_t, int16_t);

  private:
    rclcpp::Publisher<robot_interfaces::msg::MotorStatus>::SharedPtr status_pub_;
    rclcpp::Publisher<robot_interfaces::msg::MotorDebug>::SharedPtr debug_pub_;
};


class HoverDiffDrive : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(HoverDiffDrive);

   HOVER_DIFFDRIVE_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

   HOVER_DIFFDRIVE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

   HOVER_DIFFDRIVE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

   HOVER_DIFFDRIVE_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

   HOVER_DIFFDRIVE_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

   HOVER_DIFFDRIVE_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

   HOVER_DIFFDRIVE_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  std::shared_ptr<HardwarePub> hw_pub_;    //make the publisher node a member

private:

  // Store the command for the simulated robot
  std::vector<double> hw_commands_;
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;

  HoverComms hover_comms;
};

}  // namespace  hover_diffdrive

#endif  //  HOVER_DIFFDRIVE__DIFFBOT_SYSTEM_HPP_
