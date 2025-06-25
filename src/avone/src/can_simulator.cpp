#include <iostream>
#include <cstring>
#include <unistd.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>

int main() {
    const char* can_iface = "vcan0"; // Change to your CAN interface, e.g., can0
    int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s < 0) {
        perror("Socket");
        return 1;
    }

    struct ifreq ifr;
    std::strncpy(ifr.ifr_name, can_iface, IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';

    if (ioctl(s, SIOCGIFINDEX, &ifr) < 0) {
        perror("ioctl");
        return 1;
    }

    struct sockaddr_can addr = {};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(s, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("bind");
        return 1;
    }

    const uint32_t LEFT_MOTOR_CAN_ID = 0x127;
    const uint32_t RIGHT_MOTOR_CAN_ID = 0x128;
    const uint32_t STEER_CAN_ID = 0x200;

    // Fixed values
    const int left_rpm = 4000;    // RPM
    const int right_rpm = 4000;   // RPM
    const float steer_angle = 0.5f; // radians (between -0.7 and 0.7)

    struct can_frame frame;

    while (true) {
        // Left motor RPM frame
        std::memset(&frame, 0, sizeof(frame));
        frame.can_id = LEFT_MOTOR_CAN_ID;
        frame.can_dlc = 8;
        frame.data[0] = 0x00; // reserved or throttle
        frame.data[1] = (left_rpm >> 8) & 0xFF;
        frame.data[2] = left_rpm & 0xFF;
        for (int i = 3; i < 8; ++i) frame.data[i] = 0x00;

        if (write(s, &frame, sizeof(frame)) != sizeof(frame)) {
            perror("Write left motor");
        }

        // Right motor RPM frame
        frame.can_id = RIGHT_MOTOR_CAN_ID;
        frame.data[0] = 0x00;
        frame.data[1] = (right_rpm >> 8) & 0xFF;
        frame.data[2] = right_rpm & 0xFF;
        for (int i = 3; i < 8; ++i) frame.data[i] = 0x00;

        if (write(s, &frame, sizeof(frame)) != sizeof(frame)) {
            perror("Write right motor");
        }

        // Steering angle frame
        int16_t steer_raw = static_cast<int16_t>((steer_angle / 0.7f) * 32767);
        frame.can_id = STEER_CAN_ID;
        frame.data[0] = (steer_raw >> 8) & 0xFF;
        frame.data[1] = steer_raw & 0xFF;
        for (int i = 2; i < 8; ++i) frame.data[i] = 0x00;

        if (write(s, &frame, sizeof(frame)) != sizeof(frame)) {
            perror("Write steering");
        }

        std::cout << "Sent Left RPM: " << left_rpm
                  << " | Right RPM: " << right_rpm
                  << " | Steering (rad): " << steer_angle << std::endl;

        usleep(20000); // 50Hz
    }

    close(s);
    return 0;
}
