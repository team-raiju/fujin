#include <cstdio>

#include "bsp/analog_sensors.hpp"
#include "bsp/ble.hpp"
#include "bsp/timers.hpp"
#include "services/maze.hpp"
#include "services/notification.hpp"

/// @section Constants

static constexpr uint32_t min_interval_ms = 5;

enum State {
    SEND_MAZE = 0,
    SEND_SENSORS = 1,
    SEND_BATTERY = 2,
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
                state = SEND_SENSORS; // Switch to next data to be sent
            }
        }
        break;
    }

    case SEND_SENSORS: {
        auto sensors = bsp::analog_sensors::ir_latest_reading();
        uint8_t data[] = {
            bsp::ble::header,
            bsp::ble::BlePacketType::SensorData,
            uint8_t((sensors[0] & 0x00F0) >> 4),
            uint8_t(sensors[0] & 0x000F),
            uint8_t((sensors[1] & 0x00F0) >> 4),
            uint8_t(sensors[1] & 0x000F),
            uint8_t((sensors[2] & 0x00F0) >> 4),
            uint8_t(sensors[2] & 0x000F),
            uint8_t((sensors[3] & 0x00F0) >> 4),
            uint8_t(sensors[3] & 0x000F),
            bsp::ble::header,
        };

        bsp::ble::transmit(data, sizeof(data));
        state = SEND_BATTERY;
        break;
    }

    case SEND_BATTERY: {
        auto bat = bsp::analog_sensors::battery_latest_reading();

        uint8_t data[] = {
            bsp::ble::header, bsp::ble::BlePacketType::BatteryData, uint8_t((bat & 0x00F0) >> 4), uint8_t(bat & 0x000F),
            bsp::ble::header,
        };

        bsp::ble::transmit(data, sizeof(data));
        state = SEND_MAZE;
        break;
    }
    }
}

}
