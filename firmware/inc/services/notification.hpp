#pragma once

#include <cstdint>

namespace services {

class Notification {
public:
    static Notification* instance();

    void init();
    void reset();
    void update(bool ignore_maze = false);
    void send_maze();

    Notification(const Notification&) = delete;

private:
    Notification() {};

    // The service uses a mini-fsm to know when it's safe to send the next packet
    uint8_t state;
    uint32_t last_sent;

    uint8_t last_x = 0;
    uint8_t last_y = 0;
};

}
