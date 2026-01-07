#ifndef AVONE__AVONE_SYSTEM_HARDWARE_HPP_
#define AVONE__AVONE_SYSTEM_HARDWARE_HPP_

#include <string>
#include <vector>
#include <memory>
#include <cstdint>

#include "avone/visibility_control.h"
#include "avone/avone_can_interface.hpp"

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
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

  // Joint indices
  size_t lsteer_idx_{0}, rsteer_idx_{0}, rlmotor_idx_{0}, rrmotor_idx_{0};

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
  double flwheel_position_{0.0};
  double frwheel_position_{0.0};

  // Hardware parameters
  double max_steer_angle_{0.0}, min_steer_angle_{0.0};
  int    max_rpm_{0}, can_baudrate_{250000}, read_timeout_ms_{20};

  bool sim_mode_{true};  // option 2: configurable via ros2_control param
  std::string can_interface_{"can0"};

  uint32_t l_motor_can_id_{0};
  uint32_t r_motor_can_id_{0};
  uint32_t steer_can_id_{0};

  uint32_t cmd_l_motor_can_id_{0};
  uint32_t cmd_r_motor_can_id_{0};
  uint32_t cmd_steer_can_id_{0};

  // Last read time (optional diagnostics)
  rclcpp::Time last_read_time_{0,0,RCL_SYSTEM_TIME};
};

}  // namespace avone

#endif  // AVONE__AVONE_SYSTEM_HARDWARE_HPP_
