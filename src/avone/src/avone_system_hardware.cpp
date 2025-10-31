// Copyright...
#include <limits>
#include <vector>
#include <cmath>
#include <algorithm>
#include <chrono>
#include <iomanip>
#include <iostream>

#include "avone/avone_system_hardware.hpp"
#include "avone/avone_can_interface.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

bool sim_mode_ = false; // Set to true to test without hardware

namespace avone
{

hardware_interface::CallbackReturn AvoneSystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  // Map joint indices (robust auto-mapping)
  for (size_t i = 0; i < info.joints.size(); ++i) {
    const auto & j = info.joints[i].name;
    if      (j == "LSteer")  lsteer_idx_  = i;
    else if (j == "RSteer")  rsteer_idx_  = i;
    else if (j == "RLMotor") rlmotor_idx_ = i;
    else if (j == "RRMotor") rrmotor_idx_ = i;
  }

  // Parse parameters
  try {
    auto & p = info.hardware_parameters;

    max_steer_angle_ = p.count("max_steer_angle") ? std::stod(p.at("max_steer_angle")) : 0.610865;
    min_steer_angle_ = p.count("min_steer_angle") ? std::stod(p.at("min_steer_angle")) : -0.610865;
    max_rpm_         = p.count("max_rpm")         ? std::stoi(p.at("max_rpm"))         : 300;

    can_interface_ = p.count("can_interface") ? p.at("can_interface") : "can0";
    can_baudrate_  = p.count("can_baudrate")  ? std::stoi(p.at("can_baudrate")) : 250000;
    read_timeout_ms_ = p.count("read_timeout_ms") ? std::stoi(p.at("read_timeout_ms")) : 20;

    l_motor_can_id_ = p.count("l_motor_can_id") ?
      std::stoul(p.at("l_motor_can_id"), nullptr, 0) : 0x0CF11E05;
    r_motor_can_id_ = p.count("r_motor_can_id") ?
      std::stoul(p.at("r_motor_can_id"), nullptr, 0) : 0x0CF11E06;
    steer_can_id_   = p.count("steer_can_id") ?
      std::stoul(p.at("steer_can_id"),   nullptr, 0) : 0x0D;

    cmd_l_motor_can_id_ = p.count("cmd_l_motor_can_id") ?
      std::stoul(p.at("cmd_l_motor_can_id"), nullptr, 0) : 0x0B;
    cmd_r_motor_can_id_ = p.count("cmd_r_motor_can_id") ?
      std::stoul(p.at("cmd_r_motor_can_id"), nullptr, 0) : 0x0C;
    cmd_steer_can_id_   = p.count("cmd_steer_can_id") ?
      std::stoul(p.at("cmd_steer_can_id"),   nullptr, 0) : 0x004;
  }
  catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("AvoneSystemHardware"),
                 "Parameter parsing error: %s", e.what());
    return CallbackReturn::ERROR;
  }

  // Open CAN
  can_iface_ = std::make_unique<AvoneCanInterface>(can_interface_, can_baudrate_);
  if (!sim_mode_) {
    if (!can_iface_->open()) {
      RCLCPP_ERROR(rclcpp::get_logger("AvoneSystemHardware"),
                   "Failed to open CAN interface '%s'", can_interface_.c_str());
      return CallbackReturn::ERROR;
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("AvoneSystemHardware"),
              "Initialized AVONE hardware on %s, baud %d, steer_id=0x%X, Lmotor_id=0x%X, Rmotor_id=0x%X",
              can_interface_.c_str(), can_baudrate_, steer_can_id_, l_motor_can_id_, r_motor_can_id_);

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AvoneSystemHardware::on_configure(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("AvoneSystemHardware"), "Running on_configure...");

  if (!sim_mode_) {
    if (!can_iface_ || !can_iface_->is_open()) {
      RCLCPP_ERROR(rclcpp::get_logger("AvoneSystemHardware"),
                  "CAN interface not open at configure!");
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  // Zero command/state
  lsteer_cmd_ = rsteer_cmd_ = 0.0;
  rlmotor_cmd_ = rrmotor_cmd_ = 0.0;

  lsteer_position_ = rsteer_position_ = 0.0;
  rlmotor_position_ = rrmotor_position_ = 0.0;
  rlmotor_velocity_ = rrmotor_velocity_ = 0.0;

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> AvoneSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> out;
  // Steer positions
  out.emplace_back("LSteer", hardware_interface::HW_IF_POSITION, &lsteer_position_);
  out.emplace_back("RSteer", hardware_interface::HW_IF_POSITION, &rsteer_position_);
  // Motor pos/vel
  out.emplace_back("RLMotor", hardware_interface::HW_IF_POSITION, &rlmotor_position_);
  out.emplace_back("RLMotor", hardware_interface::HW_IF_VELOCITY, &rlmotor_velocity_);
  out.emplace_back("RRMotor", hardware_interface::HW_IF_POSITION, &rrmotor_position_);
  out.emplace_back("RRMotor", hardware_interface::HW_IF_VELOCITY, &rrmotor_velocity_);
  // Passive wheels
  out.emplace_back("FLWheel", hardware_interface::HW_IF_POSITION, &flwheel_position_);
  out.emplace_back("FRWheel", hardware_interface::HW_IF_POSITION, &frwheel_position_);
  return out;
}

std::vector<hardware_interface::CommandInterface> AvoneSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> out;
  out.emplace_back("LSteer", hardware_interface::HW_IF_POSITION, &lsteer_cmd_);
  out.emplace_back("RSteer", hardware_interface::HW_IF_POSITION, &rsteer_cmd_);
  out.emplace_back("RLMotor", hardware_interface::HW_IF_VELOCITY, &rlmotor_cmd_);
  out.emplace_back("RRMotor", hardware_interface::HW_IF_VELOCITY, &rrmotor_cmd_);
  return out;
}

hardware_interface::CallbackReturn AvoneSystemHardware::on_activate(const rclcpp_lifecycle::State&)
{
  RCLCPP_INFO(rclcpp::get_logger("AvoneSystemHardware"), "Hardware activating!");

  lsteer_cmd_ = lsteer_position_;
  rsteer_cmd_ = rsteer_position_;
  rlmotor_cmd_ = 0.0;
  rrmotor_cmd_ = 0.0;

  if (!sim_mode_) {
    if (!can_iface_->send_enable_command()) {
      RCLCPP_ERROR(rclcpp::get_logger("AvoneSystemHardware"), "Failed to send enable command to actuators!");
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AvoneSystemHardware::on_deactivate(const rclcpp_lifecycle::State&)
{
  RCLCPP_INFO(rclcpp::get_logger("AvoneSystemHardware"), "Deactivating hardware!");

  lsteer_cmd_ = lsteer_position_;
  rsteer_cmd_ = rsteer_position_;
  rlmotor_cmd_ = 0.0;
  rrmotor_cmd_ = 0.0;

  if (!sim_mode_) {
    if (!can_iface_->send_disable_command()) {
      RCLCPP_ERROR(rclcpp::get_logger("AvoneSystemHardware"), "Failed to send disable command to actuators!");
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type AvoneSystemHardware::read(
  const rclcpp::Time &, const rclcpp::Duration & period)
{
  const double dt = period.seconds();
  const rclcpp::Time now = rclcpp::Clock(RCL_SYSTEM_TIME).now();

  static float last_rlmotor_velocity = 0.0f;
  static float last_rrmotor_velocity = 0.0f;
  static float last_steer_angle_rad  = 0.0f;
  static rclcpp::Time last_steer_time = now;

  // ---------------------------------------------------------------------
  // SIM MODE SHORT-CIRCUIT
  // ---------------------------------------------------------------------
  if (sim_mode_) {
    rlmotor_velocity_ = rlmotor_cmd_;
    rrmotor_velocity_ = rrmotor_cmd_;
    rlmotor_position_ += rlmotor_velocity_ * dt;
    rrmotor_position_ += rrmotor_velocity_ * dt;
    lsteer_position_   = lsteer_cmd_;
    rsteer_position_   = rsteer_cmd_;
    return hardware_interface::return_type::OK;
  }

  bool got_steer = false;

  // ---------------------------------------------------------------------
  // READ CAN FRAMES
  // ---------------------------------------------------------------------
  for (int i = 0; i < 20; ++i) {
    uint32_t can_id = 0;
    std::vector<uint8_t> data;
    if (!can_iface_->read_frame(can_id, data))
      break;
    // if (can_id == steer_can_id_ || can_id == 0x00D) {
    //   RCLCPP_INFO(rclcpp::get_logger("AvoneSystemHardware"),
    //               "STEER RX id=0x%03X data[0]=0x%02X (%3d)",
    //               can_id, data.size() ? data[0] : 0, data.size() ? data[0] : 0);
    // }

    // General debug print of all frames
    RCLCPP_DEBUG(rclcpp::get_logger("AvoneSystemHardware"),
      "CAN RX: id=0x%03X  len=%zu  data=%s",
      can_id, data.size(),
      [&](){
        std::ostringstream oss;
        for (auto b : data)
          oss << std::hex << std::uppercase << std::setw(2)
              << std::setfill('0') << (int)b << " ";
        return oss.str();
      }().c_str()
    );

    // --- LEFT MOTOR SPEED ---
    if (can_id == l_motor_can_id_ && data.size() >= 2) {
      int16_t raw = static_cast<int16_t>((data[1] << 8) | data[0]);
      float rpm = static_cast<float>(raw);
      last_rlmotor_velocity = (rpm * 2.0f * M_PI) / 60.0f;
    }

    // --- RIGHT MOTOR SPEED ---
    else if (can_id == r_motor_can_id_ && data.size() >= 2) {
      int16_t raw = static_cast<int16_t>((data[1] << 8) | data[0]);
      float rpm = static_cast<float>(raw);
      last_rrmotor_velocity = (rpm * 2.0f * M_PI) / 60.0f;
    }

    // --- STEERING ANGLE (ID 0x00D) ---
    else if (can_id == steer_can_id_ && data.size() >= 1) {
      uint8_t raw = data[0];                           // 0-255 range
      float angle_deg = (128 - static_cast<int>(raw)) * 1.0f;  // invert direction
      float angle_rad = angle_deg * static_cast<float>(M_PI / 180.0f);

      last_steer_angle_rad = angle_rad;
      last_steer_time = now;
      got_steer = true;

      RCLCPP_DEBUG(rclcpp::get_logger("AvoneSystemHardware"),
        "[STEER] raw=0x%02X (%3d) → %.2f° (%.3f rad)",
        raw, raw, angle_deg, angle_rad);
    }
  }

  // ---------------------------------------------------------------------
  // FALLBACK IF NO NEW STEERING FRAME
  // ---------------------------------------------------------------------
  if (!got_steer && (now - last_steer_time).seconds() > 0.25) {
    // open-loop fallback to commanded steer
    last_steer_angle_rad = 0.5f * static_cast<float>(lsteer_cmd_ + rsteer_cmd_);
  }

  // ---------------------------------------------------------------------
  // INTEGRATE WHEEL POSITIONS
  // ---------------------------------------------------------------------
  rlmotor_position_ += last_rlmotor_velocity * dt;
  rrmotor_position_ += last_rrmotor_velocity * dt;

  // Update state variables
  rlmotor_velocity_ = last_rlmotor_velocity;
  rrmotor_velocity_ = last_rrmotor_velocity;
  lsteer_position_  = last_steer_angle_rad;
  rsteer_position_  = last_steer_angle_rad;

  RCLCPP_DEBUG(rclcpp::get_logger("AvoneSystemHardware"),
    "[STATE] LSteer=%.3f rad, RSteer=%.3f rad, RLMotor=%.2f rad/s, RRMotor=%.2f rad/s",
    lsteer_position_, rsteer_position_, rlmotor_velocity_, rrmotor_velocity_);

  return hardware_interface::return_type::OK;
}


hardware_interface::return_type AvoneSystemHardware::write(
  const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
  if (!sim_mode_) {
    if (!can_iface_ || !can_iface_->is_open()) {
      RCLCPP_ERROR(rclcpp::get_logger("AvoneSystemHardware"), "CAN interface not open");
      return hardware_interface::return_type::ERROR;
    }
  }

  // ----- 1) Rear Left Motor -----
  {
    float rpm_f = static_cast<float>(rlmotor_cmd_ * (60.0 / (2.0 * M_PI))); // rad/s→rpm
    uint16_t rpm_raw = static_cast<uint16_t>(std::clamp(rpm_f, 0.0f, 20000.0f));
    uint8_t lsb = rpm_raw & 0xFF;
    uint8_t msb = (rpm_raw >> 8) & 0xFF;
    if (!sim_mode_) can_iface_->send_frame(cmd_l_motor_can_id_, { lsb, msb });
  }

  // ----- 2) Rear Right Motor -----
  {
    float rpm_f = static_cast<float>(rrmotor_cmd_ * (60.0 / (2.0 * M_PI)));
    uint16_t rpm_raw = static_cast<uint16_t>(std::clamp(rpm_f, 0.0f, 20000.0f));
    uint8_t lsb = rpm_raw & 0xFF;
    uint8_t msb = (rpm_raw >> 8) & 0xFF;
    if (!sim_mode_) can_iface_->send_frame(cmd_r_motor_can_id_, { lsb, msb });
  }

  // ----- 3) Steering -----
  float central_steer = 0.5f * static_cast<float>(lsteer_cmd_ + rsteer_cmd_);  // rad
  float steer_deg     = central_steer * (180.0f / static_cast<float>(M_PI));
  float raw_f         = (steer_deg + 90.0f) / 0.7f; // scale 0.7 deg/bit, offset -90
  uint8_t steer_raw   = static_cast<uint8_t>(std::clamp(raw_f, 0.0f, 255.0f));
  uint8_t act_steer_raw = static_cast<uint8_t>(-static_cast<int8_t>(steer_raw));
  if (!sim_mode_) can_iface_->send_frame(cmd_steer_can_id_, { act_steer_raw });

  return hardware_interface::return_type::OK;
}

}  // namespace avone

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(avone::AvoneSystemHardware, hardware_interface::SystemInterface)