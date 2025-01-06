#include <cstdio>
#include <string>

#include "bsp/analog_sensors.hpp"
#include "bsp/encoders.hpp"
#include "bsp/imu.hpp"
#include "bsp/leds.hpp"
#include "bsp/motors.hpp"
#include "bsp/timers.hpp"
#include "services/config.hpp"
#include "services/navigation.hpp"
#include "utils/math.hpp"
#include "utils/movement_params.hpp"
#include "utils/types.hpp"

/// @section Constants

static constexpr float WHEEL_RADIUS_CM = (1.275);
static constexpr float WHEEL_PERIMETER_CM = (M_TWOPI * WHEEL_RADIUS_CM);

static constexpr float WHEEL_TO_ENCODER_RATIO = (1.0);
static constexpr float ENCODER_PPR = (1024.0);
static constexpr float PULSES_PER_WHEEL_ROTATION = (WHEEL_TO_ENCODER_RATIO * ENCODER_PPR);

static constexpr float WHEELS_DIST_CM = (7.0);
static constexpr float ENCODER_DIST_CM_PULSE = (WHEEL_PERIMETER_CM / PULSES_PER_WHEEL_ROTATION);
static constexpr float ENCODER_DIST_MM_PULSE = (ENCODER_DIST_CM_PULSE * 10.0);

static constexpr float MM_PER_US_TO_M_PER_S = 1000.0; // 1mm/us = 1000m/s

static constexpr float CONTROL_FREQUENCY_HZ = 1000.0;

static std::map<Movement, TurnParams> turn_params;
static std::map<Movement, ForwardParams> forward_params;

using bsp::leds::Color;

/// @section Service implementation

namespace services {

Navigation* Navigation::instance() {
    static Navigation p;
    return &p;
}

void Navigation::init() {
    reset(true);
    control = Control::instance();

    if (!is_initialized) {
        bsp::encoders::register_callback_encoder_left(
            [this](auto type) { encoder_left_counter += type == bsp::encoders::DirectionType::CCW ? 1 : -1; });

        bsp::encoders::register_callback_encoder_right(
            [this](auto type) { encoder_right_counter += type == bsp::encoders::DirectionType::CCW ? 1 : -1; });

        is_initialized = true;
    }
}

void Navigation::reset(bool search_mode) {
    bsp::imu::reset_angle();

    traveled_dist_cm = 0;
    encoder_left_counter = 0;
    encoder_right_counter = 0;
    current_position = {0, 0};
    current_direction = Direction::NORTH;

    is_finished = false;
    control->reset();

    target_travel_cm = HALF_CELL_SIZE_CM + ROBOT_DIST_FROM_CENTER_START_CM;
    current_movement = Movement::FORWARD;
    reference_time = bsp::get_tick_ms();
    bsp::encoders::reset_linear_velocity_m_s();

    if (search_mode) {
        turn_params = turn_params_search;
        forward_params = forward_params_search;
    } else {
        turn_params = turn_params_slow;
        forward_params = forward_params_slow;
    }
}

float Navigation::get_torricelli_distance(float final_speed, float initial_speed, float acceleration) {
    return (final_speed * final_speed - initial_speed * initial_speed) / (2 * acceleration);
}

void Navigation::update(void) {
    static uint32_t last_update_vel_time_us = 0;

    uint32_t current_time_us = bsp::get_tick_us();
    uint32_t delta_time_us = current_time_us - last_update_vel_time_us;

    bsp::imu::update();

    if (encoder_left_counter == 0 && encoder_right_counter == 0) {
        // 50ms without movement
        // min meaasured vel is ENCODER_DIST_MM_PULSE / 50 = 0.00156 m/s
        if (delta_time_us > 50000) {
            bsp::encoders::set_linear_velocity_m_s(0);
        } else {
            float velocity_m_s = (ENCODER_DIST_MM_PULSE / delta_time_us) * MM_PER_US_TO_M_PER_S;
            bsp::encoders::set_linear_velocity_m_s(velocity_m_s);
        }
        return;
    }

    float estimated_delta_l_mm = (encoder_left_counter * ENCODER_DIST_MM_PULSE);
    float estimated_delta_r_mm = (encoder_right_counter * ENCODER_DIST_MM_PULSE);
    float delta_x_mm = (estimated_delta_l_mm + estimated_delta_r_mm) / 2.0;
    traveled_dist_cm += delta_x_mm / 10.0;

    float velocity_right_m_s = (estimated_delta_r_mm / delta_time_us) * MM_PER_US_TO_M_PER_S;
    float velocity_left_m_s = (estimated_delta_l_mm / delta_time_us) * MM_PER_US_TO_M_PER_S;
    float velocity_m_s = (velocity_right_m_s + velocity_left_m_s) / 2.0;
    bsp::encoders::set_linear_velocity_m_s(velocity_m_s);
    last_update_vel_time_us = current_time_us;

    encoder_left_counter = 0;
    encoder_right_counter = 0;
}

bool Navigation::step() {
    using bsp::analog_sensors::ir_side_wall_error;
    using bsp::analog_sensors::SensingDirection;

    switch (current_movement) {
    case Movement::FORWARD:
    case Movement::FORWARD_BEFORE_TURN_45:
    case Movement::FORWARD_AFTER_DIAGONAL:
    case Movement::DIAGONAL:
    case Movement::STOP: {
        using bsp::analog_sensors::ir_reading;

        bool front_emergency = ir_reading(SensingDirection::FRONT_LEFT) > 2850 &&
                               ir_reading(SensingDirection::FRONT_RIGHT) > 2850 &&
                               ir_reading(SensingDirection::LEFT) > 2470 && ir_reading(SensingDirection::RIGHT) > 2850;

        float final_speed = forward_params[current_movement].end_speed;
        float max_speed = forward_params[current_movement].max_speed;
        float acceleration = forward_params[current_movement].acceleration;
        float deceleration = forward_params[current_movement].deceleration;

        float control_linear_speed = control->get_target_linear_speed();

        if (std::abs(traveled_dist_cm) <
            (target_travel_cm - (100 * get_torricelli_distance(final_speed, control_linear_speed, -deceleration)))) {
            if (control_linear_speed < max_speed) {
                control_linear_speed += acceleration / CONTROL_FREQUENCY_HZ;
                control_linear_speed = std::min(control_linear_speed, max_speed);
            }
        } else {
            if (control_linear_speed > final_speed) {
                control_linear_speed -= deceleration / CONTROL_FREQUENCY_HZ;
                control_linear_speed = std::max(control_linear_speed, Config::min_move_speed);
            }
        }

        control->set_target_linear_speed(control_linear_speed);
        control->set_target_angular_speed(0);

        if (current_movement == Movement::DIAGONAL) {
            control->set_wall_pid_enabled(false);
            control->set_diagonal_pid_enabled(true);
            front_emergency = false;
        } else if (current_movement == Movement::STOP) {
            control->set_wall_pid_enabled(false);
            control->set_diagonal_pid_enabled(false);
            front_emergency = false;
        } else {
            control->set_wall_pid_enabled(true);
            control->set_diagonal_pid_enabled(false);
        }

        if (std::abs(traveled_dist_cm) >= target_travel_cm || front_emergency) {
            bsp::leds::stripe_set(Color::Red);
            is_finished = true;
        }

        break;
    }

    case Movement::TURN_LEFT_45:
    case Movement::TURN_RIGHT_45:
    case Movement::TURN_LEFT_90:
    case Movement::TURN_RIGHT_90:
    case Movement::TURN_LEFT_135:
    case Movement::TURN_RIGHT_135:
    case Movement::TURN_RIGHT_180:
    case Movement::TURN_LEFT_180:
    case Movement::TURN_RIGHT_45_FROM_45:
    case Movement::TURN_LEFT_45_FROM_45:
    case Movement::TURN_RIGHT_90_FROM_45:
    case Movement::TURN_LEFT_90_FROM_45:
    case Movement::TURN_RIGHT_135_FROM_45:
    case Movement::TURN_LEFT_135_FROM_45: {

        float angular_acceleration = turn_params[current_movement].angular_accel;
        float angular_speed = turn_params[current_movement].max_angular_speed;
        uint16_t elapsed_t_start =
            (turn_params[current_movement].start / turn_params[current_movement].turn_linear_speed);
        uint16_t elapsed_t_accel = elapsed_t_start + turn_params[current_movement].t_accel;
        uint16_t elapsed_t_max_ang_vel = elapsed_t_accel + turn_params[current_movement].t_max_ang_vel;
        uint16_t elapsed_t_decel = elapsed_t_max_ang_vel + turn_params[current_movement].t_accel;
        uint16_t elapsed_t_end =
            elapsed_t_decel + (turn_params[current_movement].end / turn_params[current_movement].turn_linear_speed);
        int turn_sign = turn_params[current_movement].sign;

        uint32_t elapsed_time = bsp::get_tick_ms() - reference_time;

        if (elapsed_time <= elapsed_t_start) {
            control->set_target_angular_speed(0);
        } else if (elapsed_time <= elapsed_t_accel) {
            float ideal_rad_s =
                std::abs(control->get_target_angular_speed()) + (angular_acceleration / CONTROL_FREQUENCY_HZ);
            ideal_rad_s = std::min(ideal_rad_s, angular_speed);
            control->set_target_angular_speed(ideal_rad_s * turn_sign);
        } else if (elapsed_time <= elapsed_t_max_ang_vel) {
            float ideal_rad_s = angular_speed;
            control->set_target_angular_speed(ideal_rad_s * turn_sign);
        } else if (elapsed_time <= elapsed_t_decel) {
            float ideal_rad_s =
                std::abs(control->get_target_angular_speed()) - (angular_acceleration / CONTROL_FREQUENCY_HZ);
            ideal_rad_s = std::max(ideal_rad_s, 0.0f);
            control->set_target_angular_speed(ideal_rad_s * turn_sign);
        } else if (elapsed_time <= elapsed_t_end) {
            control->set_target_angular_speed(0);
        } else {
            is_finished = true;
        }

        control->set_wall_pid_enabled(false);
        control->set_diagonal_pid_enabled(false);

        break;
    }

    case Movement::TURN_AROUND: {
        float angular_acceleration = turn_params[current_movement].angular_accel;
        float angular_speed = turn_params[current_movement].max_angular_speed;
        uint16_t elapsed_t_start = 2000; // arbitrary large value
        uint16_t elapsed_t_accel = elapsed_t_start + turn_params[current_movement].t_accel;
        uint16_t elapsed_t_max_ang_vel = elapsed_t_accel + turn_params[current_movement].t_max_ang_vel;
        uint16_t elapsed_t_decel = elapsed_t_max_ang_vel + turn_params[current_movement].t_accel;
        uint16_t elapsed_t_end = elapsed_t_decel;
        int turn_sign = turn_params[current_movement].sign;

        uint32_t elapsed_time = bsp::get_tick_ms() - reference_time;

        if (elapsed_time <= elapsed_t_start || elapsed_time >= elapsed_t_end) {
            bool front_emergency =
                ir_reading(SensingDirection::FRONT_LEFT) > 2850 && ir_reading(SensingDirection::FRONT_RIGHT) > 2850 &&
                ir_reading(SensingDirection::LEFT) > 2470 && ir_reading(SensingDirection::RIGHT) > 2850;

            // Final speed is first 0 to stop and turn around, then max speed to exit the movement with speed
            float final_speed;
            if (elapsed_time >= elapsed_t_end) {
                final_speed = forward_params[current_movement].max_speed;
            } else {
                final_speed = 0;
            }

            float max_speed = forward_params[current_movement].max_speed;
            float acceleration = forward_params[current_movement].acceleration;
            float deceleration = forward_params[current_movement].deceleration;
            float control_linear_speed = control->get_target_linear_speed();

            if (std::abs(traveled_dist_cm) <
                (target_travel_cm -
                 (100 * get_torricelli_distance(final_speed, control_linear_speed, -deceleration)))) {
                if (control_linear_speed < max_speed) {
                    control_linear_speed += acceleration / CONTROL_FREQUENCY_HZ;
                    control_linear_speed = std::min(control_linear_speed, max_speed);
                }
            } else {
                if (control_linear_speed > final_speed) {
                    control_linear_speed -= deceleration / CONTROL_FREQUENCY_HZ;
                    control_linear_speed = std::max(control_linear_speed, Config::min_move_speed);
                }
            }

            if (std::abs(traveled_dist_cm) >= target_travel_cm || front_emergency) {
                control_linear_speed = final_speed;

                if (elapsed_time >= elapsed_t_end) {
                    is_finished = true;
                } else {
                    // We want to start the turn around, so we change ref time
                    reference_time = bsp::get_tick_ms() - elapsed_t_start;
                }
            }

            control->set_target_linear_speed(control_linear_speed);
            control->set_target_angular_speed(0);
            control->set_diagonal_pid_enabled(false);
        } else if (elapsed_time <= elapsed_t_accel) {
            control->set_target_linear_speed(0);
            float ideal_rad_s =
                std::abs(control->get_target_angular_speed()) + (angular_acceleration / CONTROL_FREQUENCY_HZ);
            ideal_rad_s = std::min(ideal_rad_s, angular_speed);
            control->set_target_angular_speed(ideal_rad_s * turn_sign);
        } else if (elapsed_time <= elapsed_t_max_ang_vel) {
            float ideal_rad_s = angular_speed;
            control->set_target_angular_speed(ideal_rad_s * turn_sign);
        } else if (elapsed_time <= elapsed_t_decel) {
            float ideal_rad_s =
                std::abs(control->get_target_angular_speed()) - (angular_acceleration / CONTROL_FREQUENCY_HZ);
            ideal_rad_s = std::max(ideal_rad_s, 0.0f);
            control->set_target_angular_speed(ideal_rad_s * turn_sign);
            traveled_dist_cm = 0;
        }
        
        control->set_wall_pid_enabled(false);
        control->set_diagonal_pid_enabled(false);

        break;
    }
    }

    if (is_finished) {
        update_position();
    }

    control->update();

    return is_finished;
}

Point Navigation::get_robot_position(void) {
    return current_position;
}

Direction Navigation::get_robot_direction(void) {
    return current_direction;
}

void Navigation::move(Direction dir) {
    bsp::imu::reset_angle();

    target_direction = dir;
    current_movement = get_movement(dir, current_direction);

    traveled_dist_cm = 0;
    reference_time = bsp::get_tick_ms();
    target_travel_cm = forward_params[current_movement].target_travel_cm;
    is_finished = false;
}

Movement Navigation::get_movement(Direction target_dir, Direction current_dir) {
    using enum Direction;

    if (target_dir == current_dir) {
        return FORWARD;
    } else if ((target_dir == NORTH && current_dir == WEST) || (target_dir == EAST && current_dir == NORTH) ||
               (target_dir == SOUTH && current_dir == EAST) || (target_dir == WEST && current_dir == SOUTH)) {
        return TURN_RIGHT_90;
    } else if ((target_dir == NORTH && current_dir == SOUTH) || (target_dir == EAST && current_dir == WEST) ||
               (target_dir == SOUTH && current_dir == NORTH) || (target_dir == WEST && current_dir == EAST)) {
        return TURN_AROUND;
    } else {
        return TURN_LEFT_90;
    }
}

void Navigation::update_position() {
    current_direction = target_direction;
    switch (current_direction) {
    case Direction::NORTH:
        current_position.y++;
        break;
    case Direction::EAST:
        current_position.x++;
        break;
    case Direction::SOUTH:
        current_position.y--;
        break;
    case Direction::WEST:
        current_position.x--;
        break;
    default:
        break;
    }
}

void Navigation::set_movement(Movement movement, uint8_t count) {
    bsp::imu::reset_angle();

    current_movement = movement;

    traveled_dist_cm = 0;
    reference_time = bsp::get_tick_ms();
    is_finished = false;

    if (movement == Movement::STOP) {
        bsp::leds::stripe_set(Color::Blue);
    }

    target_travel_cm = forward_params[movement].target_travel_cm * count;
}

std::vector<std::pair<Movement, uint8_t>>
Navigation::get_default_target_movements(std::vector<Direction> target_directions) {

    std::vector<std::pair<Movement, uint8_t>> default_target_movements = {};

    Direction robot_direction = Direction::NORTH;
    default_target_movements.push_back({Movement::FORWARD, 1});

    for (auto target_dir : target_directions) {
        Movement movement = get_movement(target_dir, robot_direction);
        default_target_movements.push_back({movement, 1});
        robot_direction = target_dir;
    }

    default_target_movements.push_back({Movement::STOP, 1});

    print_movement_sequence(default_target_movements, "Default");
    return default_target_movements;
}

std::vector<std::pair<Movement, uint8_t>>
Navigation::get_smooth_movements(std::vector<std::pair<Movement, uint8_t>> default_target_movements) {

    std::vector<std::pair<Movement, uint8_t>> smooth_movements = {};

    smooth_movements.push_back(default_target_movements[0]);
    uint8_t forward_count = 1;
    for (uint32_t i = 1; i < default_target_movements.size() - 1; i++) {
        Movement movement = default_target_movements[i].first;
        Movement next_movement = default_target_movements[i + 1].first;
        if (movement == Movement::TURN_LEFT_90 && next_movement == Movement::TURN_LEFT_90) {
            smooth_movements.push_back({Movement::TURN_LEFT_180, 1});
            i++;
        } else if (movement == Movement::TURN_RIGHT_90 && next_movement == Movement::TURN_RIGHT_90) {
            smooth_movements.push_back({Movement::TURN_RIGHT_180, 1});
            i++;
        } else if (movement == Movement::FORWARD && next_movement == Movement::FORWARD) {
            forward_count++;
        } else if (movement == Movement::FORWARD && next_movement != Movement::FORWARD) {
            smooth_movements.push_back({Movement::FORWARD, forward_count});
            forward_count = 1;
        } else {
            smooth_movements.push_back(default_target_movements[i]);
        }
    }

    smooth_movements.push_back({Movement::STOP, 1});
    print_movement_sequence(smooth_movements, "Smooth");

    return smooth_movements;
}

std::vector<std::pair<Movement, uint8_t>>
Navigation::get_diagonal_movements(std::vector<std::pair<Movement, uint8_t>> default_target_movements) {
    std::vector<std::pair<Movement, uint8_t>> temp_target_movements = {};
    std::vector<std::pair<Movement, uint8_t>> final_target_movements = {};

    uint8_t diagonal_count = 0;
    uint8_t forward_count = 1;
    temp_target_movements.push_back(default_target_movements[0]);

    for (uint32_t i = 1; i < default_target_movements.size() - 2; i++) {
        Movement movement = default_target_movements[i].first;
        Movement next_movement_1 = default_target_movements[i + 1].first;
        Movement next_movement_2 = default_target_movements[i + 2].first;

        if (movement == Movement::TURN_LEFT_90) {
            if (!diagonal_count) {
                if (next_movement_1 == Movement::TURN_RIGHT_90) {
                    temp_target_movements.push_back({Movement::TURN_LEFT_45, 1});
                    diagonal_count = 1;
                } else if (next_movement_1 == Movement::TURN_LEFT_90 && next_movement_2 == Movement::TURN_RIGHT_90) {
                    temp_target_movements.push_back({Movement::TURN_LEFT_135, 1});
                    diagonal_count = 1;
                    i++;
                } else if (next_movement_1 == Movement::TURN_LEFT_90) {
                    temp_target_movements.push_back({Movement::TURN_LEFT_180, 1});
                    i++;
                } else {
                    temp_target_movements.push_back(default_target_movements[i]);
                }
            } else {
                if (next_movement_1 == Movement::TURN_RIGHT_90) {
                    diagonal_count++;
                } else {
                    temp_target_movements.push_back({Movement::DIAGONAL, diagonal_count});
                    diagonal_count = 0;
                }
            }
        } else if (movement == Movement::TURN_RIGHT_90) {
            if (!diagonal_count) {
                if (next_movement_1 == Movement::TURN_LEFT_90) {
                    temp_target_movements.push_back({Movement::TURN_RIGHT_45, 1});
                    diagonal_count = 1;
                } else if (next_movement_1 == Movement::TURN_RIGHT_90 && next_movement_2 == Movement::TURN_LEFT_90) {
                    temp_target_movements.push_back({Movement::TURN_RIGHT_135, 1});
                    diagonal_count = 1;
                    i++;
                } else if (next_movement_1 == Movement::TURN_RIGHT_90) {
                    temp_target_movements.push_back({Movement::TURN_RIGHT_180, 1});
                    i++;
                } else {
                    temp_target_movements.push_back(default_target_movements[i]);
                }
            } else {
                if (next_movement_1 == Movement::TURN_LEFT_90) {
                    diagonal_count++;
                } else {
                    temp_target_movements.push_back({Movement::DIAGONAL, diagonal_count});
                    diagonal_count = 0;
                }
            }
        } else if (movement == Movement::FORWARD && next_movement_1 == Movement::FORWARD) {
            forward_count++;
        } else if (movement == Movement::FORWARD && next_movement_1 != Movement::FORWARD) {
            temp_target_movements.push_back({Movement::FORWARD, forward_count});
            forward_count = 1;
        } else {
            temp_target_movements.push_back(default_target_movements[i]);
        }
    }

    if (diagonal_count > 0) {
        temp_target_movements.push_back({Movement::DIAGONAL, diagonal_count});
    } else if (forward_count > 1) {
        temp_target_movements.push_back({Movement::FORWARD, forward_count});
    } else if (temp_target_movements.back().first != Movement::TURN_LEFT_180 &&
               temp_target_movements.back().first != Movement::TURN_RIGHT_180) {
        temp_target_movements.push_back(default_target_movements[default_target_movements.size() - 2]);
    }

    temp_target_movements.push_back({Movement::STOP, 1});

    // print_movement_sequence(temp_target_movements, "Temp: ");

    // Fix movements after and before diagonals
    final_target_movements.push_back(temp_target_movements[0]);
    for (uint32_t i = 1; i < temp_target_movements.size() - 1; i++) {
        Movement movement = temp_target_movements[i].first;
        uint8_t mov_count = temp_target_movements[i].second;
        Movement next_movement = temp_target_movements[i + 1].first;
        uint8_t next_movement_count = temp_target_movements[i + 1].second;
        Movement prev_movement = temp_target_movements[i - 1].first;

        if (movement == Movement::FORWARD && (next_movement == TURN_LEFT_45 || next_movement == TURN_RIGHT_45)) {

            if (mov_count > 1) {
                final_target_movements.push_back({Movement::FORWARD, (mov_count - 1)});
            }

            final_target_movements.push_back({Movement::FORWARD_BEFORE_TURN_45, 1});
        } else if (movement == Movement::DIAGONAL) {
            if (mov_count > 1) {
                final_target_movements.push_back({Movement::DIAGONAL, mov_count - 1});
            }

            bool diagonal_start_right = (prev_movement == TURN_RIGHT_45 || prev_movement == TURN_RIGHT_135);

            if (diagonal_start_right) {
                if (next_movement == FORWARD || next_movement == STOP) {
                    final_target_movements.push_back({Movement::TURN_LEFT_45_FROM_45, 1});
                    final_target_movements.push_back({Movement::FORWARD_AFTER_DIAGONAL, 1});

                    if (next_movement_count > 2) {
                        final_target_movements.push_back({Movement::FORWARD, (next_movement_count - 1)});
                    }
                } else if (next_movement == TURN_LEFT_45) {
                    final_target_movements.push_back({Movement::TURN_LEFT_90_FROM_45, 1});
                } else if (next_movement == TURN_LEFT_90) {
                    final_target_movements.push_back({Movement::TURN_LEFT_135_FROM_45, 1});
                }

            } else {
                if ((next_movement == FORWARD || next_movement == STOP) && !diagonal_start_right) {
                    final_target_movements.push_back({Movement::TURN_RIGHT_45_FROM_45, 1});
                    final_target_movements.push_back({Movement::FORWARD_AFTER_DIAGONAL, 1});
                    if (next_movement_count > 2) {
                        final_target_movements.push_back({Movement::FORWARD, (next_movement_count - 1)});
                    }
                } else if (next_movement == TURN_RIGHT_45) {
                    final_target_movements.push_back({Movement::TURN_RIGHT_90_FROM_45, 1});
                } else if (next_movement == TURN_RIGHT_90) {
                    final_target_movements.push_back({Movement::TURN_RIGHT_135_FROM_45, 1});
                }
            }

            i++;
        } else {
            final_target_movements.push_back({movement, mov_count});
        }
    }

    final_target_movements.push_back({Movement::STOP, 1});

    print_movement_sequence(final_target_movements, "Final: ");

    return final_target_movements;
}

void Navigation::print_movement_sequence(std::vector<std::pair<Movement, uint8_t>> movements, std::string name) {
    std::printf("%s: \r\n", name.c_str());
    bsp::delay_ms(2);
    for (auto movement : movements) {
        for (uint32_t i = 0; i < sizeof(movementInfoMap) / sizeof(movementInfoMap[0]); i++) {
            if (movementInfoMap[i].first == movement.first) {
                std::printf("%d - %s\r\n", movement.second, movementInfoMap[i].second);
                bsp::delay_ms(2);
                break;
            }
        }
    }
}

}
