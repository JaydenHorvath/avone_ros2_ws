#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/u_int16.hpp> 
#include <std_msgs/msg/int32.hpp>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <net/if.h>
#include <cstring>
#include <unistd.h>
#include <sys/ioctl.h>
#include "av1.h"

class CanToRos2Node : public rclcpp::Node {
public:
    CanToRos2Node() : Node("can_to_ros2") {
        // Individual publishers for every field
        pub_auto_mode_ = create_publisher<std_msgs::msg::Bool>("/AV1/auto_mode", 10);
        pub_manual_mode_ = create_publisher<std_msgs::msg::Bool>("/AV1/manual_mode", 10);
        pub_rc_armed_ = create_publisher<std_msgs::msg::Bool>("/AV1/rc_armed", 10);
        pub_steer_actual_ = create_publisher<std_msgs::msg::Float64>("/AV1/steer_angle_actual", 10);
        pub_ros_steer_target_ = create_publisher<std_msgs::msg::Float64>("/AV1/ros_steer_angle_target", 10);
        pub_rc_steer_target_ = create_publisher<std_msgs::msg::Float64>("/AV1/rc_steer_angle_target", 10);
        pub_brkprs_r_ = create_publisher<std_msgs::msg::Float64>("/AV1/brake_pressure_rear", 10);
        pub_brkprs_f_ = create_publisher<std_msgs::msg::Float64>("/AV1/brake_pressure_front", 10);
        pub_rc_brk_request_ = create_publisher<std_msgs::msg::Float64>("/AV1/rc_brake_request", 10);
        pub_rc_throttle_request_ = create_publisher<std_msgs::msg::Float64>("/AV1/rc_throttle_request", 10);
        pub_ros_rpm_request_ = create_publisher<std_msgs::msg::Float64>("/AV1/ros_rpm_request", 10);
        pub_ros_lmotor_rpm_request_ = create_publisher<std_msgs::msg::Float64>("/AV1/ros_lmotor_rpm_request", 10);
        pub_ros_rmotor_rpm_request_ = create_publisher<std_msgs::msg::Float64>("/AV1/ros_rmotor_rpm_request", 10);

        pub_pwm_request_ = create_publisher<std_msgs::msg::Float64>("/AV1/pwm_request", 10);
        pub_estop_remote_ = create_publisher<std_msgs::msg::Bool>("/AV1/estop_remote", 10);
        pub_estop_car_ = create_publisher<std_msgs::msg::Bool>("/AV1/estop_car", 10);
        pub_hb_ben_ = create_publisher<std_msgs::msg::Bool>("/AV1/hb_ben", 10);
        pub_hb_sen_ = create_publisher<std_msgs::msg::Bool>("/AV1/hb_sen", 10);
        pub_hb_ren_ = create_publisher<std_msgs::msg::Bool>("/AV1/hb_ren", 10);
        pub_hb_lvd_ = create_publisher<std_msgs::msg::Bool>("/AV1/hb_lvd", 10);
        pub_hb_pre_ = create_publisher<std_msgs::msg::Bool>("/AV1/hb_pre", 10);
        pub_charge_ = create_publisher<std_msgs::msg::Bool>("/AV1/charge", 10);
        pub_charge_ack_ = create_publisher<std_msgs::msg::Bool>("/AV1/charge_ack", 10);
        pub_ts_state_ = create_publisher<std_msgs::msg::Bool>("/AV1/ts_state", 10);
        // In the constructor:

        // Left motor publishers
        pub_motor_rpm_left_ = create_publisher<std_msgs::msg::Float64>("/AV1Lmotor/rpm", 10);
        pub_motor_current_left_ = create_publisher<std_msgs::msg::Float64>("/AV1Lmotor/current", 10);
        pub_motor_voltage_left_ = create_publisher<std_msgs::msg::Float64>("/AV1Lmotor/voltage", 10);
        pub_motor_error_left_ = create_publisher<std_msgs::msg::UInt16>("/AV1Lmotor/error_code", 10);

        
        pub_motor_throttle_left_ = create_publisher<std_msgs::msg::UInt16>("/AV1Lmotor/throttle", 10);
        pub_motor_controller_temp_left_ = create_publisher<std_msgs::msg::Int32>("/AV1Lmotor/controller_temp", 10);
        pub_motor_temp_left_     = create_publisher<std_msgs::msg::Int32>("/AV1Lmotor/motor_temp", 10);


        // Right motor publishers  
        pub_motor_rpm_right_ = create_publisher<std_msgs::msg::Float64>("/AV1Rmotor/rpm", 10);
        pub_motor_current_right_ = create_publisher<std_msgs::msg::Float64>("/AV1Rmotor/current", 10);
        pub_motor_voltage_right_ = create_publisher<std_msgs::msg::Float64>("/AV1Rmotor/voltage", 10);
        pub_motor_error_right_ = create_publisher<std_msgs::msg::UInt16>("/AV1Rmotor/error_code", 10);
        pub_motor_throttle_right_ = create_publisher<std_msgs::msg::UInt16>("/AV1Rmotor/throttle", 10);
        
        pub_motor_throttle_right_ = create_publisher<std_msgs::msg::UInt16>("/AV1Rmotor/throttle", 10);
        pub_motor_controller_temp_right_ = create_publisher<std_msgs::msg::Int32>("/AV1Rmotor/controller_temp", 10);
        pub_motor_temp_right_     = create_publisher<std_msgs::msg::Int32>("/AV1Rmotor/motor_temp", 10);
        
        // CAN socket setup
        open_can_socket("can0");
        main_loop();
    }

private:
    int s_;
    void open_can_socket(const char *ifname) {
        s_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (s_ < 0) throw std::runtime_error("CAN socket open failed");
        struct ifreq ifr;
        std::strncpy(ifr.ifr_name, ifname, IFNAMSIZ);
        if (ioctl(s_, SIOCGIFINDEX, &ifr) < 0) throw std::runtime_error("ioctl failed");
        struct sockaddr_can addr = {};
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;
        if (bind(s_, (struct sockaddr *)&addr, sizeof(addr)) < 0) throw std::runtime_error("CAN bind failed");
    }

    void main_loop() {
        while (rclcpp::ok()) {
            struct can_frame frame = {};
            int nbytes = read(s_, &frame, sizeof(struct can_frame));
            if (nbytes < 0) continue;

            // Instead of switch...
            if ((frame.can_id & CAN_EFF_FLAG) && ((frame.can_id & CAN_EFF_MASK) == 0x0CF11E05)) {
                struct av1_kelly_motor_status1_left_t msg;
                if (av1_kelly_motor_status1_left_unpack(&msg, frame.data, frame.can_dlc) == 0) {
                    uint16_t rpm = ((uint16_t)msg.left_speed_rpm_msb << 8) | msg.left_speed_rpm_lsb;
                    uint16_t current_raw = ((uint16_t)msg.left_current_msb << 8) | msg.left_current_lsb;
                    uint16_t voltage_raw = ((uint16_t)msg.left_voltage_msb << 8) | msg.left_voltage_lsb;
                    uint16_t error = ((uint16_t)msg.left_error_code_msb << 8) | msg.left_error_code_lsb;

                    double current = current_raw * 0.1;
                    double voltage = voltage_raw * 0.1;

                    auto rpm_msg = std_msgs::msg::Float64();
                    rpm_msg.data = rpm;
                    pub_motor_rpm_left_->publish(rpm_msg);

                    auto current_msg = std_msgs::msg::Float64();
                    current_msg.data = current;
                    pub_motor_current_left_->publish(current_msg);

                    auto voltage_msg = std_msgs::msg::Float64();
                    voltage_msg.data = voltage;
                    pub_motor_voltage_left_->publish(voltage_msg);

                    auto error_msg = std_msgs::msg::UInt16();
                    error_msg.data = error;
                    pub_motor_error_left_->publish(error_msg);
                }
                continue; // Or break if you stay with a switch structure
            }

            if ((frame.can_id & CAN_EFF_FLAG) && ((frame.can_id & CAN_EFF_MASK) == 0x0CF11F05)) {
                struct av1_kelly_motor_status2_left_t msg;
                if (av1_kelly_motor_status2_left_unpack(&msg, frame.data, frame.can_dlc) == 0) {
                    // Throttle (raw 0–65535)
                    uint16_t throttle_raw = msg.left_throttle_signal;
                    auto throttle_msg = std_msgs::msg::UInt16();
                    throttle_msg.data = throttle_raw;
                    pub_motor_throttle_left_->publish(throttle_msg);

                    // Controller temp (signed, with offset) → Int32
                    auto ctrl_temp_msg = std_msgs::msg::Int32();
                    ctrl_temp_msg.data = static_cast<int32_t>(msg.left_controller_temp);
                    pub_motor_controller_temp_left_->publish(ctrl_temp_msg);

                    // Motor temp (signed, with offset) → Int32
                    auto motor_temp_msg = std_msgs::msg::Int32();
                    motor_temp_msg.data = static_cast<int32_t>(msg.left_motor_temp);
                    pub_motor_temp_left_->publish(motor_temp_msg);
                }
                continue;
            }



            if ((frame.can_id & CAN_EFF_FLAG) && ((frame.can_id & CAN_EFF_MASK) == 0x0CF11E06)) {
                struct av1_kelly_motor_status1_right_t msg;
                if (av1_kelly_motor_status1_right_unpack(&msg, frame.data, frame.can_dlc) == 0) {


                    // printf("Right motor - Raw CAN data: ");
                    // for (int i = 0; i < frame.can_dlc; i++) {
                    //     printf("%02X ", frame.data[i]);
                    // }
                    // printf("\n");
                    // printf("Right motor - MSB: 0x%02X (%d), LSB: 0x%02X (%d)\n", 
                    //     msg.right_speed_rpm_msb, msg.right_speed_rpm_msb,
                    //     msg.right_speed_rpm_lsb, msg.right_speed_rpm_lsb);

                    uint16_t rpm = ((uint16_t)msg.right_speed_rpm_msb << 8) | msg.right_speed_rpm_lsb;

                    // printf("Right motor - Calculated RPM: %d\n", rpm);

                    uint16_t current_raw = ((uint16_t)msg.right_current_msb << 8) | msg.right_current_lsb;
                    uint16_t voltage_raw = ((uint16_t)msg.right_voltage_msb << 8) | msg.right_voltage_lsb;
                    uint16_t error = ((uint16_t)msg.right_error_code_msb << 8) | msg.right_error_code_lsb;

                    double current = current_raw * 0.1;
                    double voltage = voltage_raw * 0.1;

                    auto rpm_msg = std_msgs::msg::Float64();
                    rpm_msg.data = rpm;
                    pub_motor_rpm_right_->publish(rpm_msg);

                    auto current_msg = std_msgs::msg::Float64();
                    current_msg.data = current;
                    pub_motor_current_right_->publish(current_msg);

                    auto voltage_msg = std_msgs::msg::Float64();
                    voltage_msg.data = voltage;
                    pub_motor_voltage_right_->publish(voltage_msg);

                    auto error_msg = std_msgs::msg::UInt16();
                    error_msg.data = error;
                    pub_motor_error_right_->publish(error_msg);
                }
                continue; // Or break if you stay with a switch structure
            }


            if ((frame.can_id & CAN_EFF_FLAG) && ((frame.can_id & CAN_EFF_MASK) == 0x0CF11F06)) {
                struct av1_kelly_motor_status2_right_t msg;

                if (av1_kelly_motor_status2_right_unpack(&msg, frame.data, frame.can_dlc) == 0) {
                    // Extract throttle signal directly
                    uint16_t throttle_raw = msg.right_throttle_signal;

                    auto throttle_msg = std_msgs::msg::UInt16();
                    throttle_msg.data = throttle_raw;
                    pub_motor_throttle_right_->publish(throttle_msg);

                    // You can also publish temps/status if you want later
                    // e.g. msg.left_controller_temp, msg.left_motor_temp, etc.
                }

                continue;
            }
// ...else, handle all the other messages as before!



            

            switch (frame.can_id) {
            case 0x0: { // AUTO_MODE
                struct av1_auto_mode_t msg;
                if (av1_auto_mode_unpack(&msg, frame.data, frame.can_dlc) == 0) {
                    std_msgs::msg::Bool out;
                    out.data = (msg.auto_mode != 0);
                    pub_auto_mode_->publish(out);
                }
                break;
            }
            case 0x1: { // MANUAL_MODE
                struct av1_manual_mode_t msg;
                if (av1_manual_mode_unpack(&msg, frame.data, frame.can_dlc) == 0) {
                    std_msgs::msg::Bool out;
                    out.data = (msg.manual_mode != 0);
                    pub_manual_mode_->publish(out);
                }
                break;
            }
            case 0x2: { // RC_ARMED
                struct av1_rc_armed_t msg;
                if (av1_rc_armed_unpack(&msg, frame.data, frame.can_dlc) == 0) {
                    std_msgs::msg::Bool out;
                    out.data = (msg.rc_armed != 0);
                    pub_rc_armed_->publish(out);
                }
                break;
            }

            case 0x3: { // ROS_RPM_REQUEST
                struct av1_ros_rpm_request_t msg;
                if (av1_ros_rpm_request_unpack(&msg, frame.data, frame.can_dlc) == 0) {
                    std_msgs::msg::Float64 out;
                    // raw value is already rpm in your DBC (scale=1, offset=0)
                    out.data = static_cast<double>(msg.ros_rpm_request);
                    pub_ros_rpm_request_->publish(out);

                    // RCLCPP_INFO(this->get_logger(), "ROS_RPM_REQUEST: %.1f rpm", out.data);
                }
                break;
            }

            


            case 0x4: { // RC_STEER_ANG_TARGET
                struct av1_ros_steer_ang_target_t msg;
                if (av1_ros_steer_ang_target_unpack(&msg, frame.data, frame.can_dlc) == 0) {
                    std_msgs::msg::Float64 out;
                    out.data = av1_ros_steer_ang_target_ros_steer_ang_target_decode(msg.ros_steer_ang_target);
                    pub_ros_steer_target_->publish(out);
                }
                break;
            }
            case 0x6: { // STEER_ANG_ACTUAL
                struct av1_steer_ang_actual_t msg;
                if (av1_steer_ang_actual_unpack(&msg, frame.data, frame.can_dlc) == 0) {
                    std_msgs::msg::Float64 out;
                    out.data = av1_steer_ang_actual_steer_ang_actual_decode(msg.steer_ang_actual);
                    pub_steer_actual_->publish(out);
                }
                break;
            }
            case 0x7: { // RC_STEER_ANG_TARGET
                struct av1_rc_steer_ang_target_t msg;
                if (av1_rc_steer_ang_target_unpack(&msg, frame.data, frame.can_dlc) == 0) {
                    std_msgs::msg::Float64 out;
                    out.data = av1_rc_steer_ang_target_rc_steer_ang_target_decode(msg.rc_steer_ang_target);
                    pub_rc_steer_target_->publish(out);
                }
                break;
            }
            case 0x8: { // BRKPRS_R
                struct av1_brkprs_r_t msg;
                if (av1_brkprs_r_unpack(&msg, frame.data, frame.can_dlc) == 0) {
                    std_msgs::msg::Float64 out;
                    out.data = av1_brkprs_r_brkprs_r_decode(msg.brkprs_r);
                    pub_brkprs_r_->publish(out);
                }
                break;
            }
            case 0x9: { // BRKPRS_F
                struct av1_brkprs_f_t msg;
                if (av1_brkprs_f_unpack(&msg, frame.data, frame.can_dlc) == 0) {
                    std_msgs::msg::Float64 out;
                    out.data = av1_brkprs_f_brkprs_f_decode(msg.brkprs_f);
                    pub_brkprs_f_->publish(out);
                }
                break;
            }
            case 0xA: { // RC_BRK_REQUEST
                struct av1_rc_brk_request_t msg;
                if (av1_rc_brk_request_unpack(&msg, frame.data, frame.can_dlc) == 0) {
                    std_msgs::msg::Float64 out;
                    out.data = av1_rc_brk_request_rc_brk_request_decode(msg.rc_brk_request);
                    pub_rc_brk_request_->publish(out);
                }
                break;
            }

            case 0x0B: { // 11 dec — ROS_RMOTOR_RPM_REQUEST
            struct av1_ros_rmotor_rpm_request_t msg;
            if (av1_ros_rmotor_rpm_request_unpack(&msg, frame.data, frame.can_dlc) == 0) {
                std_msgs::msg::Float64 out;
                // SG_ ROS_RMOTOR_RPM_REQUEST : 0|16@1+ (1,0)
                out.data = static_cast<double>(msg.ros_rmotor_rpm_request);
                pub_ros_rmotor_rpm_request_->publish(out);
                // RCLCPP_INFO(this->get_logger(), "RIGHT RPM_REQ: %.1f rpm", out.data);
            }
            break;
            }

            case 0x0C: { // 12 dec — ROS_LMOTOR_RPM_REQUEST
            struct av1_ros_lmotor_rpm_request_t msg;
            if (av1_ros_lmotor_rpm_request_unpack(&msg, frame.data, frame.can_dlc) == 0) {
                std_msgs::msg::Float64 out;
                // SG_ ROS_LMOTOR_RPM_REQUEST : 0|16@1+ (1,0)
                out.data = static_cast<double>(msg.ros_lmotor_rpm_request);
                pub_ros_lmotor_rpm_request_->publish(out);
                // RCLCPP_INFO(this->get_logger(), "LEFT RPM_REQ: %.1f rpm", out.data);
            }
            break;
            }
            case 0x12: { // RC_THROTTLE_REQUEST
                struct av1_rc_throttle_request_t msg;
                if (av1_rc_throttle_request_unpack(&msg, frame.data, frame.can_dlc) == 0) {
                    std_msgs::msg::Float64 out;
                    out.data = av1_rc_throttle_request_rc_throttle_request_decode(msg.rc_throttle_request);
                    pub_rc_throttle_request_->publish(out);
                }
                break;
            }
            case 0x13: { // PWM_REQUEST
                struct av1_pwm_request_t msg;
                if (av1_pwm_request_unpack(&msg, frame.data, frame.can_dlc) == 0) {
                    std_msgs::msg::Float64 out;
                    out.data = av1_pwm_request_pwm_request_decode(msg.pwm_request);
                    pub_pwm_request_->publish(out);
                }
                break;
            }
            case 0x14: { // ESTOP_REMOTE
                struct av1_estop_remote_t msg;
                if (av1_estop_remote_unpack(&msg, frame.data, frame.can_dlc) == 0) {
                    std_msgs::msg::Bool out;
                    out.data = (msg.estop_remote != 0);
                    pub_estop_remote_->publish(out);
                }
                break;
            }
            case 0x15: { // ESTOP_CAR
                struct av1_estop_car_t msg;
                if (av1_estop_car_unpack(&msg, frame.data, frame.can_dlc) == 0) {
                    std_msgs::msg::Bool out;
                    out.data = (msg.estop_car != 0);
                    pub_estop_car_->publish(out);
                }
                break;
            }
            case 0x16: { // HB_BEN
                struct av1_hb_ben_t msg;
                if (av1_hb_ben_unpack(&msg, frame.data, frame.can_dlc) == 0) {
                    std_msgs::msg::Bool out;
                    out.data = (msg.hb_ben != 0);
                    pub_hb_ben_->publish(out);
                }
                break;
            }
            case 0x17: { // HB_SEN
                struct av1_hb_sen_t msg;
                if (av1_hb_sen_unpack(&msg, frame.data, frame.can_dlc) == 0) {
                    std_msgs::msg::Bool out;
                    out.data = (msg.hb_sen != 0);
                    pub_hb_sen_->publish(out);
                }
                break;
            }
            case 0x18: { // HB_REN
                struct av1_hb_ren_t msg;
                if (av1_hb_ren_unpack(&msg, frame.data, frame.can_dlc) == 0) {
                    std_msgs::msg::Bool out;
                    out.data = (msg.hb_ren != 0);
                    pub_hb_ren_->publish(out);
                }
                break;
            }
            case 0x19: { // HB_LVD
                struct av1_hb_lvd_t msg;
                if (av1_hb_lvd_unpack(&msg, frame.data, frame.can_dlc) == 0) {
                    std_msgs::msg::Bool out;
                    out.data = (msg.hb_lvd != 0);
                    pub_hb_lvd_->publish(out);
                }
                break;
            }
            case 0x1A: { // HB_PRE
                struct av1_hb_pre_t msg;
                if (av1_hb_pre_unpack(&msg, frame.data, frame.can_dlc) == 0) {
                    std_msgs::msg::Bool out;
                    out.data = (msg.hb_pre != 0);
                    pub_hb_pre_->publish(out);
                }
                break;
            }
            case 0xFA: { // CHARGE
                struct av1_charge_t msg;
                if (av1_charge_unpack(&msg, frame.data, frame.can_dlc) == 0) {
                    std_msgs::msg::Bool out;
                    out.data = (msg.charge != 0);
                    pub_charge_->publish(out);
                }
                break;
            }
            case 0xFB: { // CHARGE_ACK
                struct av1_charge_ack_t msg;
                if (av1_charge_ack_unpack(&msg, frame.data, frame.can_dlc) == 0) {
                    std_msgs::msg::Bool out;
                    out.data = (msg.charge_ack != 0);
                    pub_charge_ack_->publish(out);
                }
                break;
            }
            case 0x214: { // TS_STATE
                struct av1_ts_state_t msg;
                if (av1_ts_state_unpack(&msg, frame.data, frame.can_dlc) == 0) {
                    std_msgs::msg::Bool out;
                    out.data = (msg.ts_state != 0);
                    pub_ts_state_->publish(out);
                }
                break;
            }
            // Add any other messages (multi-field, e.g., DCDC or Kelly) as needed here!

            // case 0x8CFF0105: { // 2364612101
            //     struct av1_kelly_motor_status_t msg;
            //     if (av1_kelly_motor_status_unpack(&msg, frame.data, frame.can_dlc) == 0) {
            //         // Combine LSB and MSB for 16-bit values
            //         uint16_t rpm = ((uint16_t)msg.speed_rpm_msb << 8) | msg.speed_rpm_lsb;
            //         uint16_t current_raw = ((uint16_t)msg.current_msb << 8) | msg.current_lsb;
            //         uint16_t voltage_raw = ((uint16_t)msg.voltage_msb << 8) | msg.voltage_lsb;
            //         uint16_t error = ((uint16_t)msg.error_code_msb << 8) | msg.error_code_lsb;

            //         // Scale current and voltage
            //         double current = current_raw * 0.1; // 0.1A/bit
            //         double voltage = voltage_raw * 0.1; // 0.1V/bit

            //         // Publish RPM
            //         auto rpm_msg = std_msgs::msg::Float64();
            //         rpm_msg.data = rpm;
            //         pub_motor_rpm_->publish(rpm_msg);

            //         // Publish Current
            //         auto current_msg = std_msgs::msg::Float64();
            //         current_msg.data = current;
            //         pub_motor_current_->publish(current_msg);

            //         // Publish Voltage
            //         auto voltage_msg = std_msgs::msg::Float64();
            //         voltage_msg.data = voltage;
            //         pub_motor_voltage_->publish(voltage_msg);

            //         // Publish Error code (as UInt16)
            //         auto error_msg = std_msgs::msg::UInt16();
            //         error_msg.data = error;
            //         pub_motor_error_->publish(error_msg);
            //     }
            //     break;
            // }





            default:
                break;
        }
        }
    }

    // Publishers
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_auto_mode_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_manual_mode_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_rc_armed_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_steer_actual_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_rc_steer_target_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_ros_steer_target_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_brkprs_r_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_brkprs_f_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_rc_brk_request_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_rc_throttle_request_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_pwm_request_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_ros_rpm_request_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_ros_lmotor_rpm_request_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_ros_rmotor_rpm_request_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_estop_remote_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_estop_car_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_hb_ben_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_hb_sen_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_hb_ren_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_hb_lvd_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_hb_pre_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_charge_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_charge_ack_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_ts_state_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_motor_rpm_left_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_motor_current_left_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_motor_voltage_left_;
    rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr pub_motor_error_left_;
    
    rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr pub_motor_throttle_left_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr  pub_motor_controller_temp_left_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr  pub_motor_temp_left_;

    
    
    // Right motor publishers
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_motor_rpm_right_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_motor_current_right_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_motor_voltage_right_;
    rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr pub_motor_error_right_;

    rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr pub_motor_throttle_right_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr  pub_motor_controller_temp_right_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr  pub_motor_temp_right_;
    

};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CanToRos2Node>();
    rclcpp::spin(node);
    return 0;
}


