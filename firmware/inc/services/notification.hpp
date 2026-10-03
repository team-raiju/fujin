#pragma once

#include <cstdint>
#include <utility>
#include <vector>

#include "utils/types.hpp"

namespace services {

class Notification {
public:
    static Notification* instance();

    void init();
    void reset();
    void update(bool ignore_maze = false);
    void send_maze();
    void send_target_movements(const std::vector<std::pair<Movement, uint8_t>>& movements);

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
