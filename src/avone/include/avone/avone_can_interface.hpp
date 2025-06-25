#pragma once

#include <string>
#include <vector>
#include <cstdint>

typedef int socket_t;

class AvoneCanInterface
{
public:
    AvoneCanInterface(const std::string& interface_name, int baudrate);
    ~AvoneCanInterface();

    bool open();
    bool is_open() const;
    bool send_enable_command();
    bool send_disable_command();
    bool send_frame(uint32_t can_id, const std::vector<uint8_t>& data);
    bool read_frame(uint32_t& can_id, std::vector<uint8_t>& data, int timeout_ms = 10);
    void close();

    

private:
    std::string iface_name_;
    int baudrate_;
    socket_t sock_fd_;
    bool is_open_;
};
