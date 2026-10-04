#include <iostream>
#include <thread>
#include <variant>
#include <cstring>

#include "bsp/analog_sensors.hpp"
#include "bsp/ble.hpp"
#include "bsp/buttons.hpp"
#include "bsp/buzzer.hpp"
#include "bsp/core.hpp"
#include "bsp/debug.hpp"
#include "bsp/eeprom.hpp"
#include "bsp/encoders.hpp"
#include "bsp/fan.hpp"
#include "bsp/imu.hpp"
#include "bsp/leds.hpp"
#include "bsp/motors.hpp"
#include "bsp/timers.hpp"
#include "bsp/usb.hpp"

#include "fsm/event.hpp"
#include "fsm/fsm.hpp"
#include "utils/RingBuffer.hpp"
#include "utils/soft_timer.hpp"

#include "services/navigation.hpp"
#include "services/maze.hpp"
#include "services/config.hpp"
#include "amaterasu/amaterasu.hpp"

using bsp::eeprom::EepromResult;
using bsp::imu::ImuResult;

namespace bsp {

namespace timers {
void advance_sim_tick(void);
void reset_sim_time(void);
}

namespace buttons {
void button_1_pressed(PressType type = PressType::SHORT);
void button_2_pressed(PressType type = PressType::SHORT);
}

namespace ble {
void received();
}

/// @section Interface implementation

void init() {
    setlinebuf(stdout);
    analog_sensors::init();
    buttons::init();
    buzzer::init();
    encoders::init();
    fan::init();
    leds::init();
    motors::init();
    timers::init();
    usb::init();

    if (imu::init() != ImuResult::OK) {
        std::cerr << "Failed to initialize imu" << std::endl;
    }

    if (eeprom::init() != EepromResult::OK) {
        std::cerr << "Failed to initialize eeprom" << std::endl;
    }

    // Initialize Config calibration parameters on target-pc (Calibration Test Map)
    services::Config::ir_wall_dist_ref_right = 1391.0f;
    services::Config::ir_wall_dist_ref_front_left = 200.0f;
    services::Config::ir_wall_dist_ref_front_right = 200.0f;
    services::Config::ir_wall_dist_ref_left = 800.0f;
    services::Config::ir_wall_control_th_right = 1100.0f;
    services::Config::ir_wall_control_th_front_left = 500.0f;
    services::Config::ir_wall_control_th_front_right = 500.0f;
    services::Config::ir_wall_control_th_left = 650.0f;
    services::Config::ir_wall_detect_th_right = 1200.0f;
    services::Config::ir_wall_detect_th_front_left = 500.0f;
    services::Config::ir_wall_detect_th_front_right = 800.0f;
    services::Config::ir_wall_detect_th_left = 700.0f;
    services::Config::z_imu_bias = -0.4905f;
    services::Config::min_move_speed = 0.2f;

    // Initialize amaterasu simulation bridge
    amaterasu::init(8765, amaterasu::SimMode::LOCKSTEP);

    delay_ms(20);

    std::thread([] {
        auto dispatch_buttons = []() -> bool {
            bool handled = false;
            auto b1 = amaterasu::consume_button_press(0);
            if (b1 == amaterasu::ButtonPressType::SHORT_PRESS) {
                buttons::button_1_pressed(buttons::PressType::SHORT);
                handled = true;
            } else if (b1 == amaterasu::ButtonPressType::LONG_PRESS) {
                buttons::button_1_pressed(buttons::PressType::LONG);
                handled = true;
            }

            auto b2 = amaterasu::consume_button_press(1);
            if (b2 == amaterasu::ButtonPressType::SHORT_PRESS) {
                buttons::button_2_pressed(buttons::PressType::SHORT);
                handled = true;
            } else if (b2 == amaterasu::ButtonPressType::LONG_PRESS) {
                buttons::button_2_pressed(buttons::PressType::LONG);
                handled = true;
            }
            return handled;
        };

        static uint32_t last_synced_maze_version = 0;
        auto sync_maze_to_eeprom = []() {
            if (!amaterasu::has_maze()) return;
            uint32_t ver = amaterasu::get_maze_version();
            if (ver == last_synced_maze_version) return;

            last_synced_maze_version = ver;
            uint8_t data[4];
            for (uint8_t x = 0; x < 16; ++x) {
                for (uint8_t y = 0; y < 16; ++y) {
                    uint8_t walls = 0;
                    if (amaterasu::get_maze_cell(x, y, walls)) {
                        data[0] = walls;
                        data[1] = 0x0F; // All walls known (visited = true)
                        data[2] = 0;
                        data[3] = 0;
                        eeprom::write_u32(eeprom::param_addresses_t::ADDR_MAZE_START + 4 * (x * 16 + y), *(uint32_t*)data);
                        eeprom::write_u32(eeprom::param_addresses_t::ADDR_MAZE_BACKUP_START + 4 * (x * 16 + y), *(uint32_t*)data);
                    }
                }
            }
            std::printf("[core-pc] Synced simulator maze (version %u) to EEPROM\r\n", ver);
        };

        auto sync_bot_maze_to_telemetry = []() {
            auto maze = services::Maze::instance();
            if (!maze) return;
            std::vector<std::vector<bool>> h(17, std::vector<bool>(16, false));
            std::vector<std::vector<bool>> v(16, std::vector<bool>(17, false));
            for (int x = 0; x < 16; ++x) {
                for (int y = 0; y < 16; ++y) {
                    const auto& cell = maze->map[x][y];
                    if (cell.walls & Walls::N) h[15 - y][x] = true;
                    if (cell.walls & Walls::S) h[16 - y][x] = true;
                    if (cell.walls & Walls::W) v[15 - y][x] = true;
                    if (cell.walls & Walls::E) v[15 - y][x + 1] = true;
                }
            }
            amaterasu::set_telemetry_walls(h, v);
        };

        auto compute_maze_hash = []() -> uint64_t {
            auto maze = services::Maze::instance();
            if (!maze) return 0;
            uint64_t hash = 14695981039346656037ULL;
            for (int x = 0; x < 16; ++x) {
                for (int y = 0; y < 16; ++y) {
                    hash ^= maze->map[x][y].walls;
                    hash *= 1099511628211ULL;
                    hash ^= maze->map[x][y].known_walls;
                    hash *= 1099511628211ULL;
                }
            }
            return hash;
        };
        uint64_t last_synced_bot_maze_hash = 0;

        for (;;) {
            if (amaterasu::is_connected() && amaterasu::get_mode() == amaterasu::SimMode::LOCKSTEP) {
                sync_maze_to_eeprom();

                uint64_t cur_hash = compute_maze_hash();
                if (cur_hash != last_synced_bot_maze_hash) {
                    last_synced_bot_maze_hash = cur_hash;
                    sync_bot_maze_to_telemetry();
                }

                bool got_tick = amaterasu::wait_for_tick(20);
                bool btn_handled = dispatch_buttons();

                if (amaterasu::consume_reset_request()) {
                    auto nav = services::Navigation::instance();
                    if (nav) {
                        nav->reset(services::Navigation::SEARCH_SLOW);
                    }
                    bsp::imu::reset();
                    bsp::encoders::reset();
                    bsp::timers::reset_sim_time();
                    last_synced_bot_maze_hash = 0;
                    sync_bot_maze_to_telemetry();
                    amaterasu::set_telemetry_pose(0.090f, 0.090f, 0.0f);
                    amaterasu::set_telemetry_cell(0, 0);
                    amaterasu::broadcast_telemetry();
                    std::printf("[core-pc] Robot reset to start position (0, 0)\r\n");
                }

                if (got_tick) {
                    if (btn_handled) {
                        // Allow fsm.spin() in main thread a brief moment to process the state transition
                        std::this_thread::sleep_for(std::chrono::milliseconds(2));
                    }

                    // Advance timers and simulation clock
                    bsp::timers::advance_sim_tick();
                    soft_timer::tick();

                    // Update navigation telemetry if services are available
                    auto nav = services::Navigation::instance();
                    if (nav) {
                        auto pos = nav->get_robot_position_mm();
                        auto cell = nav->get_robot_cell_position();
                        amaterasu::set_telemetry_pose(pos.x / 1000.0f, pos.y / 1000.0f, bsp::imu::get_angle());
                        amaterasu::set_telemetry_cell(cell.x, cell.y);
                    }

                    // Publish actuators and state back to simulator
                    amaterasu::publish_step_ack();
                } else if (btn_handled) {
                    // Button was pressed while simulator is paused/idle
                    std::this_thread::sleep_for(std::chrono::milliseconds(2));
                    amaterasu::broadcast_telemetry();
                }
            } else {
                // Free-running mode when disconnected or in REALTIME mode
                delay_ms(1);
                dispatch_buttons();
                soft_timer::tick();
                if (amaterasu::is_connected()) {
                    sync_maze_to_eeprom();
                    auto nav = services::Navigation::instance();
                    if (nav) {
                        auto pos = nav->get_robot_position_mm();
                        auto cell = nav->get_robot_cell_position();
                        amaterasu::set_telemetry_pose(pos.x / 1000.0f, pos.y / 1000.0f, bsp::imu::get_angle());
                        amaterasu::set_telemetry_cell(cell.x, cell.y);
                    }
                    amaterasu::broadcast_telemetry();
                }
            }
        }
    }).detach();
}

void debug::print(const char* s) {
    std::cout << s << std::endl;
    amaterasu::add_log(s);
    if (std::strncmp(s, "state:", 6) == 0) {
        amaterasu::set_telemetry_state(std::string(s + 6));
        if (amaterasu::is_connected()) {
            amaterasu::broadcast_telemetry();
        }
    }
}

} // namespace bsp
