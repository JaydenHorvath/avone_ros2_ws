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

#include <limits>
#include <vector>
#include <cmath>  // For M_PI
#include "avone/avone_system_hardware.hpp"
#include "avone/avone_can_interface.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

bool sim_mode_ = true; // Set to true for simulation

namespace avone
{
hardware_interface::CallbackReturn AvoneSystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

   // 2. Assign indices for each joint (robust, auto-mapping)
  for (size_t i = 0; i < info.joints.size(); ++i) {
    if (info.joints[i].name == "LSteer") lsteer_idx_ = i;
    else if (info.joints[i].name == "RSteer") rsteer_idx_ = i;
    else if (info.joints[i].name == "RLMotor") rlmotor_idx_ = i;
    else if (info.joints[i].name == "RRMotor") rrmotor_idx_ = i;
  }



  // 4. Parse global hardware parameters (with fallback defaults)
  try {
    max_steer_angle_ = info.hardware_parameters.count("max_steer_angle") ?
        std::stod(info.hardware_parameters.at("max_steer_angle")) : 0.7;
    min_steer_angle_ = info.hardware_parameters.count("min_steer_angle") ?
        std::stod(info.hardware_parameters.at("min_steer_angle")) : -0.7;
    max_rpm_ = info.hardware_parameters.count("max_rpm") ?
        std::stoi(info.hardware_parameters.at("max_rpm")) : 15000;
    can_interface_ = info.hardware_parameters.count("can_interface") ?
        info.hardware_parameters.at("can_interface") : "can0";
    can_baudrate_ = info.hardware_parameters.count("can_baudrate") ?
        std::stoi(info.hardware_parameters.at("can_baudrate")) : 250000;
    read_timeout_ms_ = info.hardware_parameters.count("read_timeout_ms") ?
        std::stoi(info.hardware_parameters.at("read_timeout_ms")) : 20;

    l_motor_can_id_ = info.hardware_parameters.count("l_motor_can_id") ?
        std::stoi(info.hardware_parameters.at("l_motor_can_id"), nullptr, 16) : 0x101;
    r_motor_can_id_ = info.hardware_parameters.count("r_motor_can_id") ?
        std::stoi(info.hardware_parameters.at("r_motor_can_id"), nullptr, 16) : 0x102;
    steer_can_id_ = info.hardware_parameters.count("steer_can_id") ?
        std::stoi(info.hardware_parameters.at("steer_can_id"), nullptr, 16) : 0x200;

    cmd_l_motor_can_id_ = info.hardware_parameters.count("cmd_l_motor_can_id") ?
        std::stoi(info.hardware_parameters.at("cmd_l_motor_can_id"), nullptr, 16) : 0x27;
    cmd_r_motor_can_id_ = info.hardware_parameters.count("cmd_r_motor_can_id") ?
        std::stoi(info.hardware_parameters.at("cmd_r_motor_can_id"), nullptr, 16) : 0x28;
    cmd_steer_can_id_ = info.hardware_parameters.count("cmd_steer_can_id") ?
        std::stoi(info.hardware_parameters.at("cmd_steer_can_id"), nullptr, 16) : 0x20;
  }
  catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("AvoneSystemHardware"),
                 "Parameter parsing error: %s", e.what());
    return CallbackReturn::ERROR;
  }

  // 5. Open CAN interface (stub/mock example, replace with your real CAN class)
  can_iface_ = std::make_unique<AvoneCanInterface>(can_interface_, can_baudrate_);

  if (!can_iface_->open()) {
    RCLCPP_ERROR(rclcpp::get_logger("AvoneSystemHardware"),
                 "Failed to open CAN interface '%s'", can_interface_.c_str());
    return CallbackReturn::ERROR;
}


  // 6. (Optional) Initialize diagnostic/status stuff

  // 7. Log startup config (good for debugging)
  RCLCPP_INFO(rclcpp::get_logger("AvoneSystemHardware"),
              "Initialized AVONE hardware on %s, baud %d, steer_id=0x%X, Lmotor_id=0x%X, Rmotor_id=0x%X",
              can_interface_.c_str(), can_baudrate_, steer_can_id_, l_motor_can_id_, r_motor_can_id_);


  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AvoneSystemHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(rclcpp::get_logger("AvoneSystemHardware"), "Running on_configure...");

    // 1. Confirm the CAN interface is open
    if (!can_iface_ || !can_iface_->is_open()) {
        RCLCPP_ERROR(rclcpp::get_logger("AvoneSystemHardware"),
                     "CAN interface not open at configure!");
        return hardware_interface::CallbackReturn::ERROR;
    }

    // 2. Try a non-blocking CAN read to see if the bus responds (optional for real hardware)
    uint32_t test_id = 0;
    std::vector<uint8_t> test_data;
    bool got_frame = can_iface_->read_frame(test_id, test_data, /*timeout_ms=*/2);

    if (got_frame) {
        RCLCPP_INFO(rclcpp::get_logger("AvoneSystemHardware"),
                    "Received CAN frame during configure (ID=0x%X, len=%zu)",
                    test_id, test_data.size());
        // You might want to check the CAN ID matches expected devices
    } else {
        RCLCPP_WARN(rclcpp::get_logger("AvoneSystemHardware"),
                    "No CAN frame received during configure (bus might be quiet, that's OK for now)");
    }

    // 3. (Optional) Zero command/state buffers for safety
    lsteer_cmd_ = 0.0;
    rsteer_cmd_ = 0.0;
    rlmotor_cmd_ = 0.0;
    rrmotor_cmd_ = 0.0;

    lsteer_position_ = 0.0;
    rsteer_position_ = 0.0;
    rlmotor_position_ = 0.0;
    rlmotor_velocity_ = 0.0;
    rrmotor_position_ = 0.0;
    rrmotor_velocity_ = 0.0;

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> AvoneSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> out;
  // Steer positions
  out.emplace_back("LSteer", hardware_interface::HW_IF_POSITION, &lsteer_position_);
  out.emplace_back("RSteer", hardware_interface::HW_IF_POSITION, &rsteer_position_);
  // Motor positions and velocities
  out.emplace_back("RLMotor", hardware_interface::HW_IF_POSITION, &rlmotor_position_);
  out.emplace_back("RLMotor", hardware_interface::HW_IF_VELOCITY, &rlmotor_velocity_);
  out.emplace_back("RRMotor", hardware_interface::HW_IF_POSITION, &rrmotor_position_);
  out.emplace_back("RRMotor", hardware_interface::HW_IF_VELOCITY, &rrmotor_velocity_);
  // Passive/fixed wheels for URDF completeness (positions only, probably just 0.0 or fixed)
  out.emplace_back("FLWheel", hardware_interface::HW_IF_POSITION, &flwheel_position_);
  out.emplace_back("FRWheel", hardware_interface::HW_IF_POSITION, &frwheel_position_);
  return out;
}



std::vector<hardware_interface::CommandInterface> AvoneSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> out;
  // Only actuated joints
  out.emplace_back("LSteer", hardware_interface::HW_IF_POSITION, &lsteer_cmd_);
  out.emplace_back("RSteer", hardware_interface::HW_IF_POSITION, &rsteer_cmd_);
  out.emplace_back("RLMotor", hardware_interface::HW_IF_VELOCITY, &rlmotor_cmd_);
  out.emplace_back("RRMotor", hardware_interface::HW_IF_VELOCITY, &rrmotor_cmd_);
  return out;
}

hardware_interface::CallbackReturn AvoneSystemHardware::on_activate(const rclcpp_lifecycle::State&)
{
    RCLCPP_INFO(rclcpp::get_logger("AvoneSystemHardware"), "Hardware activating!");

    // Zero commands to be safe
    lsteer_cmd_ = lsteer_position_;
    rsteer_cmd_ = rsteer_position_;
    rlmotor_cmd_ = 0.0;
    rrmotor_cmd_ = 0.0;

    // DUMMY: Optionally send "enable" command to hardware
    if (!can_iface_->send_enable_command()) {
        RCLCPP_ERROR(rclcpp::get_logger("AvoneSystemHardware"), "Failed to send enable command to actuators!");
        return hardware_interface::CallbackReturn::ERROR;
    }

    // TURN ON TS??

    // (Optional) Check hardware status, error states, etc.

    return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn AvoneSystemHardware::on_deactivate(const rclcpp_lifecycle::State&)
{
  RCLCPP_INFO(rclcpp::get_logger("AvoneSystemHardware"), "Deactivating hardware!");

  // 1. Zero all commands to ensure no actuation
  lsteer_cmd_ = lsteer_position_;   // Hold last position or go to safe neutral
  rsteer_cmd_ = rsteer_position_;
  rlmotor_cmd_ = 0.0;               // Stop motors
  rrmotor_cmd_ = 0.0;

  // TURN OFF TS?

  // 2. Optionally, send a "disable" or "stop" command over CAN
  // (implement this in your CAN interface if you want it, for now just log)
  if (!can_iface_->send_disable_command()) {
        RCLCPP_ERROR(rclcpp::get_logger("AvoneSystemHardware"), "Failed to send disable command to actuators!");
        return hardware_interface::CallbackReturn::ERROR;
    }


  // 3. (Optional) Check or reset error states, hardware flags, etc.

  // 4. Return SUCCESS
  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::return_type AvoneSystemHardware::read(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  (void)period;

  double dt = period.seconds();
  if (sim_mode_) {
    // Simulate perfect actuators: set position/velocity = command
    rlmotor_velocity_ = rlmotor_cmd_;
    rrmotor_velocity_ = rrmotor_cmd_;
    rlmotor_position_ += rlmotor_velocity_ * dt;
    rrmotor_position_ += rrmotor_velocity_ * dt;
    // For steering, just set position to last commanded position
    lsteer_position_ = lsteer_cmd_;
    rsteer_position_ = rsteer_cmd_;
  }


  // Calculate delta time since last read call
  // double dt = 0.0;
  if (last_read_time_.nanoseconds() != 0) {
    dt = (time - last_read_time_).seconds();
  }
  last_read_time_ = time;

  // Read up to 2 CAN frames per cycle (simulate)
  for (int i = 0; i < 3; ++i) {
    uint32_t can_id = 0;
    std::vector<uint8_t> data;
    bool got_msg = can_iface_->read_frame(can_id, data);

    if (got_msg && data.size() == 8) {
      // --- Rear Left Motor ---
      if (can_id == l_motor_can_id_) {
        int16_t rpm_raw = static_cast<int16_t>((data[0] << 8) | data[1]);
        float rpm = static_cast<float>(rpm_raw);
        rlmotor_velocity_ = (rpm * 2.0f * M_PI) / 60.0f;  // Convert RPM to rad/s
        rlmotor_position_ += rlmotor_velocity_ * dt;       // Integrate position
      }
      // --- Rear Right Motor ---
      else if (can_id == r_motor_can_id_) {
        int16_t rpm_raw = static_cast<int16_t>((data[0] << 8) | data[1]);
        float rpm = static_cast<float>(rpm_raw);
        rrmotor_velocity_ = (rpm * 2.0f * M_PI) / 60.0f;  // rad/s
        rrmotor_position_ += rrmotor_velocity_ * dt;       // integrate
      }
      // --- Steering Controller ---
      else if (can_id == steer_can_id_) {
        // Assume steering angle encoded as int16_t in bytes 0-1, scaled between -0.7 and 0.7 rad
        int16_t raw_angle = static_cast<int16_t>((data[0] << 8) | data[1]);
        // Map raw int16_t [-32768, 32767] to steering angle range [-0.7, 0.7]
        float angle = (static_cast<float>(raw_angle) / 32767.0f) * 0.7f;

        lsteer_position_ = angle;
        rsteer_position_ = angle;
      }
    }
  }

  // RCLCPP_INFO(rclcpp::get_logger("AvoneSystemHardware"),
  //             "Right Rear Motor Position: %f, Velocity: %f, Steering Angle: %f",
  //             rrmotor_position_, rrmotor_velocity_, rsteer_position_);

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type AvoneSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // RCLCPP_INFO(rclcpp::get_logger("AvoneSystemHardware"), 
  //           "Write called: lsteer_cmd=%.3f, rsteer_cmd=%.3f, rlmotor_cmd=%.3f, rrmotor_cmd=%.3f",
  //           lsteer_cmd_, rsteer_cmd_, rlmotor_cmd_, rrmotor_cmd_);

  // Prepare CAN data buffers (8 bytes each)
  std::vector<uint8_t> steer_data(8, 0);
  std::vector<uint8_t> left_data(8, 0);
  std::vector<uint8_t> right_data(8, 0);

  // --- Pack steering command ---
  // Use average steer angle for single actuator
  float central_steer = 0.5f * (lsteer_cmd_ + rsteer_cmd_);
  int16_t steer_raw = static_cast<int16_t>(
    (central_steer / max_steer_angle_) * 32767.0);

  steer_data[0] = (steer_raw >> 8) & 0xFF;
  steer_data[1] = steer_raw & 0xFF;
  // Remaining bytes zeroed
   RCLCPP_INFO(rclcpp::get_logger("AvoneSystemHardware"), 
            "Write called: central_steer_cmd=%.3f, rlmotor_cmd=%.3f, rrmotor_cmd=%.3f",
            central_steer, rlmotor_cmd_, rrmotor_cmd_);

  // --- Pack left motor command ---
  int16_t left_rpm = static_cast<int16_t>(
    (rlmotor_cmd_ * 60.0) / (2.0 * M_PI));
  left_data[0] = (left_rpm >> 8) & 0xFF;
  left_data[1] = left_rpm & 0xFF;

  // --- Pack right motor command ---
  int16_t right_rpm = static_cast<int16_t>(
    (rrmotor_cmd_ * 60.0) / (2.0 * M_PI));
  right_data[0] = (right_rpm >> 8) & 0xFF;
  right_data[1] = right_rpm & 0xFF;

  // Send steering command CAN frame
  if (!can_iface_->send_frame(cmd_steer_can_id_, steer_data)) {
    RCLCPP_ERROR(rclcpp::get_logger("AvoneSystemHardware"), "Failed to send steering command");
    return hardware_interface::return_type::ERROR;
  }

  // Send left motor command CAN frame
  if (!can_iface_->send_frame(cmd_l_motor_can_id_, left_data)) {
    RCLCPP_ERROR(rclcpp::get_logger("AvoneSystemHardware"), "Failed to send left motor command");
    return hardware_interface::return_type::ERROR;
  }

  // Send right motor command CAN frame
  if (!can_iface_->send_frame(cmd_r_motor_can_id_, right_data)) {
    RCLCPP_ERROR(rclcpp::get_logger("AvoneSystemHardware"), "Failed to send right motor command");
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}


}  // namespace avone

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  avone::AvoneSystemHardware, hardware_interface::SystemInterface)
