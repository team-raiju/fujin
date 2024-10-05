#include <cstdio>

#include "bsp/ble.hpp"
#include "bsp/timers.hpp"
#include "services/maze.hpp"
#include "services/notification.hpp"

/// @section Constants

static constexpr uint32_t min_interval_ms = 5;

enum State {
    SEND_MAZE = 0,
};

/// @section Service implementation

namespace services {

Notification* Notification::instance() {
    static Notification p;
    return &p;
}

void Notification::init() {
    reset();
}

void Notification::reset() {
    last_sent = 0;
    state = SEND_MAZE;
}

void Notification::update() {
    if (bsp::get_tick_ms() - last_sent < min_interval_ms) {
        return;
    }

    last_sent = bsp::get_tick_ms();

    switch (state) {
    case SEND_MAZE: {
        auto maze = services::Maze::instance();

        uint8_t data[] = {
            bsp::ble::header,
            bsp::ble::BlePacketType::MazeData,
            (uint8_t)((last_x << 4) | last_y),
            maze->map[last_x][last_y].walls,
            maze->map[last_x][last_y].visited,
            maze->map[last_x][last_y].distance,
            bsp::ble::header,
        };

        bsp::ble::transmit(data, sizeof(data));

        last_x += 1;
        if (last_x == services::Maze::CELLS_X) {
            last_x = 0;
            last_y += 1;
            if (last_y == services::Maze::CELLS_Y) {
                last_y = 0;
                state = SEND_MAZE; // Switch to next data to be sent
            }
        }
        break;
    }
    }
}

}
