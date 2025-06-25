#include "avone/avone_can_interface.hpp"
#include <iostream>
#include <vector>
#include <iomanip>
#include <csignal>
#include <atomic>

std::atomic<bool> running(true);

void sigint_handler(int) {
    running = false;
}

int main() {
    std::signal(SIGINT, sigint_handler);

    AvoneCanInterface can("vcan0", 250000);
    if (!can.open()) {
        std::cerr << "Failed to open CAN interface vcan0" << std::endl;
        return 1;
    }
    std::cout << "Listening for CAN frames on vcan0. Press Ctrl+C to exit." << std::endl;
    while (running) {
        uint32_t id;
        std::vector<uint8_t> data;
        if (can.read_frame(id, data, 1000)) { // 1s timeout
            std::cout << "Received frame: ID=0x" << std::hex << id << " Data=[";
            for (auto b : data)
                std::cout << " 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(b);
            std::cout << " ]" << std::endl;
        }
    }
    can.close();
    std::cout << "Exited CAN read loop." << std::endl;
    return 0;
}
