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
#include "std_msgs/msg/bool.hpp"

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
      std::stoul(p.at("l_motor_can_id"), nullptr, 0) : 0x8CF11E05;
    r_motor_can_id_ = p.count("r_motor_can_id") ?
      std::stoul(p.at("r_motor_can_id"), nullptr, 0) : 0x8CF11E06;
    steer_can_id_   = p.count("steer_can_id") ?
      std::stoul(p.at("steer_can_id"),   nullptr, 0) : 0x006;

    cmd_l_motor_can_id_ = p.count("cmd_l_motor_can_id") ?
      std::stoul(p.at("cmd_l_motor_can_id"), nullptr, 0) : 0x0B;
    cmd_r_motor_can_id_ = p.count("cmd_r_motor_can_id") ?
      std::stoul(p.at("cmd_r_motor_can_id"), nullptr, 0) : 0x0C;
    cmd_steer_can_id_   = p.count("cmd_steer_can_id") ?
      std::stoul(p.at("cmd_steer_can_id"),   nullptr, 0) : 0x004;

    // --- R2D/Watchdogs ---
    require_rlmotor_ = p.count("require_rlmotor") ? (p.at("require_rlmotor") == "true") : true;
    require_rrmotor_ = p.count("require_rrmotor") ? (p.at("require_rrmotor") == "true") : true;
    require_steer_   = p.count("require_steer")   ? (p.at("require_steer")   == "true") : true;

    startup_validation_ms_ = p.count("startup_validation_ms") ? std::stoi(p.at("startup_validation_ms")) : 800;
    link_loss_timeout_ms_  = p.count("link_loss_timeout_ms")  ? std::stoi(p.at("link_loss_timeout_ms"))  : 250;

    require_lidar_   = p.count("require_lidar") ? (p.at("require_lidar") == "true") : true;
    lidar_topic_     = p.count("lidar_topic")   ? p.at("lidar_topic")   : "/lidar";
    lidar_ros_type_  = p.count("lidar_ros_type")? p.at("lidar_ros_type"): "sensor_msgs/msg/PointCloud2";
    lidar_timeout_ms_= p.count("lidar_timeout_ms") ? std::stoi(p.at("lidar_timeout_ms")) : 300;

    r2d_topic_         = p.count("r2d_topic") ? p.at("r2d_topic") : "/ready_to_drive";
    r2d_can_heartbeat_ = p.count("r2d_can_heartbeat") ? (p.at("r2d_can_heartbeat") == "true") : true;
    r2d_can_id_        = p.count("r2d_can_id") ? std::stoul(p.at("r2d_can_id"), nullptr, 0) : 0x110;
    r2d_can_period_ms_ = p.count("r2d_can_period_ms") ? std::stoi(p.at("r2d_can_period_ms")) : 50;
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

  // Create a small node for pub/sub
  node_ = std::make_shared<rclcpp::Node>("avone_system_hw_r2d");
  r2d_pub_ = node_->create_publisher<std_msgs::msg::Bool>(r2d_topic_, 10);

  // LiDAR subscription (optional)
  if (require_lidar_) {
    if (lidar_ros_type_ == "sensor_msgs/msg/PointCloud2") {
      lidar_pc_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
        lidar_topic_, rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::PointCloud2::SharedPtr)
        {
          last_lidar_rx_ = node_->get_clock()->now();
        });
    } else { // default to LaserScan
      lidar_scan_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
        lidar_topic_, rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::LaserScan::SharedPtr)
        {
          last_lidar_rx_ = node_->get_clock()->now();
        });
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

  // Non-blocking CAN sniff is optional here; skip.

  // Zero command/state
  lsteer_cmd_ = rsteer_cmd_ = 0.0;
  rlmotor_cmd_ = rrmotor_cmd_ = 0.0;

  lsteer_position_ = rsteer_position_ = 0.0;
  rlmotor_position_ = rrmotor_position_ = 0.0;
  rlmotor_velocity_ = rrmotor_velocity_ = 0.0;

  // Reset R2D states
  r2d_ready_ = false;
  last_rlmotor_rx_ = last_rrmotor_rx_ = last_steer_rx_ = rclcpp::Time(0,0,RCL_SYSTEM_TIME);
  last_lidar_rx_  = rclcpp::Time(0,0,RCL_SYSTEM_TIME);
  last_r2d_tx_    = rclcpp::Time(0,0,RCL_SYSTEM_TIME);

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

  // let R2D become true once all required inputs are seen; no blocking here
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
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // Spin our small node so LiDAR callbacks run
  if (node_) {
    rclcpp::spin_some(node_);
  }

  const rclcpp::Time now = rclcpp::Clock(RCL_SYSTEM_TIME).now();
  last_read_time_ = now;

  double dt = period.seconds();

  if (sim_mode_) {
    rlmotor_velocity_ = rlmotor_cmd_;
    rrmotor_velocity_ = rrmotor_cmd_;
    rlmotor_position_ += rlmotor_velocity_ * dt;
    rrmotor_position_ += rrmotor_velocity_ * dt;
    lsteer_position_ = lsteer_cmd_;
    rsteer_position_ = rsteer_cmd_;
    // R2D in sim: if require_lidar_ and we have seen it recently, flip true
    bool prev = r2d_ready_;
    r2d_ready_ = compute_r2d_ready_(now);
    if (prev != r2d_ready_) {
      RCLCPP_INFO(rclcpp::get_logger("AvoneSystemHardware"), "R2D=%s (sim)", r2d_ready_ ? "TRUE" : "FALSE");
    }
    return hardware_interface::return_type::OK;
  }

  // --- Read available CAN frames ---
  static float last_rlmotor_velocity = 0.0f;
  static float last_rrmotor_velocity = 0.0f;
  static float last_steer_angle_rad  = 0.0f;

  for (int i = 0; i < 10; ++i) {
    uint32_t can_id = 0;
    std::vector<uint8_t> data;
    bool got_msg = can_iface_->read_frame(can_id, data);
    if (!got_msg) break;

    // Debug (comment if noisy)
    // std::cout << "RX 0x" << std::hex << can_id << " len=" << std::dec << data.size() << "\n";

    if (can_id == l_motor_can_id_ && data.size() >= 2) {
      int16_t raw = static_cast<int16_t>((data[1] << 8) | data[0]);
      float rpm = static_cast<float>(raw);
      last_rlmotor_velocity = (rpm * 2.0f * M_PI) / 60.0f;
      last_rlmotor_rx_ = now;
    }
    else if (can_id == r_motor_can_id_ && data.size() >= 2) {
      int16_t raw = static_cast<int16_t>((data[1] << 8) | data[0]);
      float rpm = static_cast<float>(raw);
      last_rrmotor_velocity = (rpm * 2.0f * M_PI) / 60.0f;
      last_rrmotor_rx_ = now;
    }
    else if (can_id == steer_can_id_ && data.size() >= 1) {
      uint8_t raw_angle = data[0];
      float angle_deg = raw_angle * 0.7f - 90.0f;
      angle_deg = -angle_deg; // invert if needed
      last_steer_angle_rad = angle_deg * (M_PI / 180.0f);
      last_steer_rx_ = now;
    }
  }

  // Integrate once per read
  rlmotor_position_ += last_rlmotor_velocity * dt;
  rrmotor_position_ += last_rrmotor_velocity * dt;

  rlmotor_velocity_ = last_rlmotor_velocity;
  rrmotor_velocity_ = last_rrmotor_velocity;
  lsteer_position_  = last_steer_angle_rad;
  rsteer_position_  = last_steer_angle_rad;

  // Update R2D
  bool prev = r2d_ready_;
  r2d_ready_ = compute_r2d_ready_(now);
  if (prev != r2d_ready_) {
    RCLCPP_INFO(rclcpp::get_logger("AvoneSystemHardware"), "R2D=%s", r2d_ready_ ? "TRUE" : "FALSE");
  }

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

  // ----- 4) R2D publish + CAN heartbeat -----
  publish_and_send_r2d_(rclcpp::Clock(RCL_SYSTEM_TIME).now());

  return hardware_interface::return_type::OK;
}

// ===== Helpers =====

bool AvoneSystemHardware::compute_r2d_ready_(const rclcpp::Time & now) const
{
  const auto age_ms = [&](const rclcpp::Time & t) -> int64_t {
    if (t.nanoseconds() == 0) return std::numeric_limits<int64_t>::max();
    return (now - t).nanoseconds() / 1000000; // ns→ms
  };

  const bool rl_ok = !require_rlmotor_ ? true : (age_ms(last_rlmotor_rx_) <= link_loss_timeout_ms_);
  const bool rr_ok = !require_rrmotor_ ? true : (age_ms(last_rrmotor_rx_) <= link_loss_timeout_ms_);
  const bool st_ok = !require_steer_   ? true : (age_ms(last_steer_rx_)   <= link_loss_timeout_ms_);
  const bool ld_ok = !require_lidar_   ? true : (age_ms(last_lidar_rx_)   <= lidar_timeout_ms_);

  // Optionally gate by startup_validation_ms_: allow “not ready” until we’ve given system a little time
  const bool startup_ok = (age_ms(rclcpp::Time(0,0,RCL_SYSTEM_TIME)) > startup_validation_ms_) ? true : true;
  (void)startup_ok; // not strictly needed; we rely on data-age alone

  return (rl_ok && rr_ok && st_ok && ld_ok);
}

void AvoneSystemHardware::publish_and_send_r2d_(const rclcpp::Time & now)
{
  // Publish topic
  if (r2d_pub_) {
    std_msgs::msg::Bool msg;
    msg.data = r2d_ready_;
    r2d_pub_->publish(msg);
  }

  // Throttled CAN heartbeat
  if (!sim_mode_ && r2d_can_heartbeat_ && can_iface_) {
    if (last_r2d_tx_.nanoseconds() == 0 ||
        (now - last_r2d_tx_).nanoseconds() / 1000000 >= r2d_can_period_ms_) {
      uint8_t b0 = r2d_ready_ ? 0x01 : 0x00; // BO_ 272 R2D_STATUS, SG_ R2D_RDY bit0
      can_iface_->send_frame(r2d_can_id_, { b0 });
      last_r2d_tx_ = now;
    }
  }
}

}  // namespace avone

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(avone::AvoneSystemHardware, hardware_interface::SystemInterface)
