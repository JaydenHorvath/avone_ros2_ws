/*
  AV.ONE SocketCAN Interface
  --------------------------
  Thin wrapper around Linux SocketCAN for sending and receiving raw CAN frames.

  Key points:
  - Uses PF_CAN/SOCK_RAW/CAN_RAW sockets (standard SocketCAN).
  - open() binds to a named interface like "can0" (real bus) or "vcan0" (virtual bus).
  - send_frame() transmits up to 8 bytes (classic CAN).
  - read_frame() polls with a timeout and returns the next received frame.

  Notes:
  - This class keeps one socket (sock_fd_) open for general RX/TX.
  - send_enable_command() and send_disable_command() currently open their own
    temporary sockets and send a single 1 byte command frame. 
*/


#include "avone/avone_can_interface.hpp"
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <unistd.h>
#include <cstring>
#include <poll.h>
#include <iostream>
#include <algorithm>
#include <rclcpp/rclcpp.hpp>

AvoneCanInterface::AvoneCanInterface(const std::string& interface_name, int baudrate)
    : iface_name_(interface_name), baudrate_(baudrate), sock_fd_(-1), is_open_(false)
{}

AvoneCanInterface::~AvoneCanInterface() {
    // Ensure we do not leak the socket if the object is destroyed
    close();
}

bool AvoneCanInterface::open() {
    // Create a raw CAN socket
    sock_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock_fd_ < 0) {
        perror("socket");
        return false;
    }

    // Translate interface name ("can0", "vcan0", etc.) into an interface index
    struct ifreq ifr;
    std::strncpy(ifr.ifr_name, iface_name_.c_str(), IFNAMSIZ);
    if (ioctl(sock_fd_, SIOCGIFINDEX, &ifr) < 0) {
        perror("ioctl");
        close();
        return false;
    }

    // Bind the socket to the CAN interface index so RX/TX uses that bus
    struct sockaddr_can addr = {};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(sock_fd_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("bind");
        close();
        return false;
    }
    is_open_ = true;
    return true;
}

bool AvoneCanInterface::is_open() const {
    return is_open_;
}

bool AvoneCanInterface::send_frame(uint32_t can_id, const std::vector<uint8_t>& data) {
   
    // Must have a bound socket before sending
    if (!is_open_) return false;

    // Populate a classic CAN frame (max 8 bytes)
    struct can_frame frame = {};
    frame.can_id = can_id;
    frame.can_dlc = std::min<size_t>(data.size(), 8);
    std::memcpy(frame.data, data.data(), frame.can_dlc);
    
    // Write the full frame struct to the socket
    int nbytes = ::write(sock_fd_, &frame, sizeof(frame));
    return nbytes == sizeof(frame);
}


bool AvoneCanInterface::read_frame(uint32_t& can_id, std::vector<uint8_t>& data, int timeout_ms) {
   
    // Must have a bound socket before receiving
    if (!is_open_) return false;

    // poll() lets us block for up to timeout_ms waiting for RX without busy looping
    struct pollfd pfd = {sock_fd_, POLLIN, 0};


    int poll_ret = poll(&pfd, 1, timeout_ms);

    // poll_ret > 0 means something happened; check if it is readable
    if (poll_ret > 0 && (pfd.revents & POLLIN)) {
        struct can_frame frame;
        int nbytes = ::read(sock_fd_, &frame, sizeof(frame));
        if (nbytes == sizeof(frame)) {
            can_id = frame.can_id;
            data.assign(frame.data, frame.data + frame.can_dlc);
            return true;
        }
    }

    // Timeout (poll_ret == 0) or error (poll_ret < 0) or short read
    return false;
}

bool AvoneCanInterface::send_enable_command()
{   

    // Sends a one-shot "enable" command frame. Good for monitoring purposes - not currently utilised



    int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("AvoneCanInterface"), "Failed to open CAN socket");
        return false;
    }

    struct ifreq ifr;
    std::strncpy(ifr.ifr_name, iface_name_.c_str(), IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';  // Make sure null-terminated
    if (ioctl(s, SIOCGIFINDEX, &ifr) < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("AvoneCanInterface"), "ioctl failed for %s", iface_name_.c_str());
        ::close(s);
        return false;
    }

    struct sockaddr_can addr = {};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("AvoneCanInterface"), "Failed to bind CAN socket");
        ::close(s);
        return false;
    }

    struct can_frame frame = {};
    frame.can_id = 0x101;  // "ENABLE" CAN ID
    frame.can_dlc = 1;
    frame.data[0] = 0xAB;   // Payload

    int nbytes = write(s, &frame, sizeof(frame));
    if (nbytes != sizeof(frame)) {
        RCLCPP_ERROR(rclcpp::get_logger("AvoneCanInterface"), "CAN write error");
        ::close(s);
        return false;
    }

    RCLCPP_INFO(rclcpp::get_logger("AvoneCanInterface"), "Sent CAN enable command on %s", iface_name_.c_str());
    ::close(s);
    return true;
}


bool AvoneCanInterface::send_disable_command()
{

    // Sends a one-shot "diable" command frame. Good for monitoring purposes - not currently utilised


    int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("AvoneCanInterface"), "Failed to open CAN socket");
        return false;
    }

    struct ifreq ifr;
    std::strncpy(ifr.ifr_name, iface_name_.c_str(), IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';  // Make sure null-terminated
    if (ioctl(s, SIOCGIFINDEX, &ifr) < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("AvoneCanInterface"), "ioctl failed for %s", iface_name_.c_str());
        ::close(s);
        return false;
    }

    struct sockaddr_can addr = {};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("AvoneCanInterface"), "Failed to bind CAN socket");
        ::close(s);
        return false;
    }

    struct can_frame frame = {};
    frame.can_id = 0x101;  // "DISABLE" CAN ID/
    frame.can_dlc = 1;
    frame.data[0] = 0xCD;  // Payload

    int nbytes = write(s, &frame, sizeof(frame));
    if (nbytes != sizeof(frame)) {
        RCLCPP_ERROR(rclcpp::get_logger("AvoneCanInterface"), "CAN write error");
        ::close(s);
        return false;
    }

    RCLCPP_INFO(rclcpp::get_logger("AvoneCanInterface"), "Sent CAN DISABLE command on %s", iface_name_.c_str());
    ::close(s);
    return true;
}



void AvoneCanInterface::close() {
    if (sock_fd_ >= 0) ::close(sock_fd_);
    sock_fd_ = -1;
    is_open_ = false;
}
