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

void Notification::update(bool ignore_maze) {
    if (bsp::get_tick_ms() - last_sent < min_interval_ms) {
        return;
    }

    last_sent = bsp::get_tick_ms();

    if (ignore_maze && state == SEND_MAZE) {
        state = SEND_SENSORS;
    }

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
            0,
            0,
            0,
            0,
        };

        // bsp::ble::transmit(data, sizeof(data));

        last_x += 1;
        if (last_x == services::Maze::CELLS_X) {
            last_x = 0;
            last_y += 1;
            if (last_y == services::Maze::CELLS_Y) {
                last_y = 0;
                // state = SEND_MAZE; // Switch to next data to be sent
            }
        }

        data[6] = (uint8_t)((last_x << 4) | last_y);
        data[7] = maze->map[last_x][last_y].walls;
        data[8] = maze->map[last_x][last_y].visited;
        data[9] = maze->map[last_x][last_y].distance;

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
        using namespace bsp::analog_sensors;
        auto sensors = ir_latest_reading();
        uint8_t data[] = {
            bsp::ble::header,
            bsp::ble::BlePacketType::SensorData,
            uint8_t((sensors[SensingDirection::LEFT] & 0xFF00) >> 8),
            uint8_t(sensors[SensingDirection::LEFT] & 0x00FF),
            uint8_t((sensors[SensingDirection::FRONT_LEFT] & 0xFF00) >> 8),
            uint8_t(sensors[SensingDirection::FRONT_LEFT] & 0x00FF),
            uint8_t((sensors[SensingDirection::FRONT_RIGHT] & 0xFF00) >> 8),
            uint8_t(sensors[SensingDirection::FRONT_RIGHT] & 0x00FF),
            uint8_t((sensors[SensingDirection::RIGHT] & 0xFF00) >> 8),
            uint8_t(sensors[SensingDirection::RIGHT] & 0x00FF),
        };

        bsp::ble::transmit(data, sizeof(data));
        state = SEND_BATTERY;
        break;
    }

    case SEND_BATTERY: {
        uint32_t bat = bsp::analog_sensors::battery_latest_reading_mv();

        uint8_t data[] = {
            bsp::ble::header,
            bsp::ble::BlePacketType::BatteryData,
            uint8_t((bat & 0xFF00) >> 8),
            uint8_t(bat & 0x00FF),
        };

        bsp::ble::transmit(data, sizeof(data));
        state = SEND_MAZE;
        break;
    }
    }
}

}
