// Copyright (c) 2025, Jay
// Copyright (c) 2025, Stogl Robotics Consulting UG (haftungsbeschränkt) (template)
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

#ifndef AVONE__AVONE_SYSTEM_HARDWARE_HPP_
#define AVONE__AVONE_SYSTEM_HARDWARE_HPP_

#include <string>
#include <vector>

#include "avone/visibility_control.h"
#include "avone/avone_can_interface.hpp"


#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace avone
{
class AvoneSystemHardware : public hardware_interface::SystemInterface
{
public:
  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // CAN interface
  std::unique_ptr<AvoneCanInterface> can_iface_;

  // Joint indices (still useful if you ever use vectors!)
  size_t lsteer_idx_, rsteer_idx_, rlmotor_idx_, rrmotor_idx_;

  // Dummy for testing 
  double dummy_state_{0.0}, dummy_cmd_{0.0};
  


  // --- Command interface storage ---
  double lsteer_cmd_{0.0};
  double rsteer_cmd_{0.0};
  double rlmotor_cmd_{0.0};
  double rrmotor_cmd_{0.0};

  // --- State interface storage ---
  double lsteer_position_{0.0};
  double rsteer_position_{0.0};
  double rlmotor_position_{0.0};
  double rlmotor_velocity_{0.0};
  double rrmotor_position_{0.0};
  double rrmotor_velocity_{0.0};

  // Add FLWheel/FRWheel if needed:
  double flwheel_position_{0.0};
  double frwheel_position_{0.0};

  // Hardware parameters
  double max_steer_angle_, min_steer_angle_;
  int max_rpm_, can_baudrate_, read_timeout_ms_;
  
  uint32_t l_motor_can_id_;
  uint32_t r_motor_can_id_;
  uint32_t steer_can_id_;

  uint32_t cmd_l_motor_can_id_;
  uint32_t cmd_r_motor_can_id_;
  uint32_t cmd_steer_can_id_;


  std::string can_interface_;


  rclcpp::Time last_read_time_;

};

}  // namespace avone

#endif  // AVONE__AVONE_SYSTEM_HARDWARE_HPP_
