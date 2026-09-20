#include <cstdio>
#include <algorithm>

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

void Notification::send_maze() {
    auto maze = services::Maze::instance();

    for (int y = (services::Maze::CELLS_Y - 1); y >= 0; y--) {
        for (int x = 0; x < services::Maze::CELLS_X; x++) {
            uint8_t data[] = {
                bsp::ble::header,
                bsp::ble::BlePacketType::MazeData,
                (uint8_t)((x << 4) | y),
                maze->map[x][y].walls,
                maze->map[x][y].visited(),
                maze->map[x][y].distance,
                0,
                0,
                0,
                0,
            };

            data[6] = (uint8_t)((x << 4) | y);
            data[7] = maze->map[x][y].walls;
            data[8] = maze->map[x][y].visited();
            data[9] = maze->map[x][y].distance;

            bsp::ble::transmit(data, sizeof(data));
            bsp::delay_ms(10);
        }
    }
}

void Notification::send_target_movements(const std::vector<std::pair<Movement, uint8_t>>& movements) {
    constexpr uint8_t entries_per_packet = 8;
    const uint8_t packet_count =
        static_cast<uint8_t>((movements.size() + entries_per_packet - 1) / entries_per_packet);

    if (packet_count == 0) {
        uint8_t data[bsp::ble::max_packet_size] = {bsp::ble::header,
                                                    bsp::ble::BlePacketType::TargetMovementSequence, 0, 0};
        bsp::ble::transmit(data, sizeof(data));
        return;
    }

    for (uint8_t packet_index = 0; packet_index < packet_count; packet_index++) {
        uint8_t data[bsp::ble::max_packet_size] = {
            bsp::ble::header,
            bsp::ble::BlePacketType::TargetMovementSequence,
            packet_index,
            packet_count,
        };
        const size_t first = packet_index * entries_per_packet;
        const size_t last = std::min(first + entries_per_packet, movements.size());
        for (size_t i = first; i < last; i++) {
            const size_t offset = 4 + ((i - first) * 2);
            data[offset] = static_cast<uint8_t>(movements[i].first);
            data[offset + 1] = movements[i].second;
        }
        bsp::ble::transmit(data, sizeof(data));
        bsp::delay_ms(5);
    }
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
            maze->map[last_x][last_y].visited(),
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
        data[8] = maze->map[last_x][last_y].visited();
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
        uint32_t bat = bsp::analog_sensors::battery_latest_reading_mv_real();

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
