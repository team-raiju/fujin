#include <cstdio>
#include <string>

#include "bsp/analog_sensors.hpp"
#include "bsp/buzzer.hpp"
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

static constexpr float CONTROL_FREQUENCY_HZ = 1000.0;
static constexpr bool fix_position_enabled = false;
static constexpr float dist_start_seeing_wall_break_cm = 5.5;

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
    control = Control::instance();
    reset(true);

    if (!is_initialized) {
        is_initialized = true;
    }
}

void Navigation::reset(bool search_mode) {
    bsp::imu::reset_angle();
    mini_fsm_state = 0;

    traveled_dist_cm = 0;
    encoder_left_counter = 0;
    encoder_right_counter = 0;
    current_cell = {0, 0};
    current_position_mm = {0, 0};
    current_angle_rad = 0;
    current_direction = Direction::NORTH;

    is_finished = false;
    control->reset();

    reference_time = bsp::get_tick_ms();
    bsp::encoders::reset();

    if (search_mode) {
        turn_params = turn_params_search;
        forward_params = forward_params_search;
    } else {
        turn_params = turn_params_slow;
        forward_params = forward_params_slow;
        // turn_params = turn_params_medium;
        // forward_params = forward_params_medium;
        // turn_params = turn_params_fast;
        // forward_params = forward_params_fast;
    }

    current_movement = Movement::START;
    target_travel_cm = forward_params[Movement::START].target_travel_cm;
    forward_end_speed = forward_params[Movement::START].max_speed;
}

float Navigation::get_torricelli_distance(float final_speed, float initial_speed, float acceleration) {
    return (final_speed * final_speed - initial_speed * initial_speed) / (2.0f * acceleration);
}

void Navigation::reset_wall_break() {
    wall_right_counter_on = 0;
    wall_left_counter_on = 0;

    wall_right_counter_off = 0;
    wall_left_counter_off = 0;

    wall_break_already_detected = false;
}

bool Navigation::wall_break_detected() {
    // We only return one wall break. This only resets when reset_wall_break is called
    if (wall_break_already_detected) {
        return false;
    }

    bool right_seeing = bsp::analog_sensors::ir_reading_wall(bsp::analog_sensors::SensingDirection::RIGHT);
    bool left_seeing = bsp::analog_sensors::ir_reading_wall(bsp::analog_sensors::SensingDirection::LEFT);

    if (right_seeing) {
        wall_right_counter_on += 1;
    } else {
        wall_right_counter_off += 1;
    }

    if (left_seeing) {
        wall_left_counter_on += 1;
    } else {
        wall_left_counter_off += 1;
    }

    if (right_seeing) {
        wall_right_counter_off = 0;
    }

    if (left_seeing) {
        wall_right_counter_off = 0;
    }

    // If more than 10 consecutive wall "off" are read, and we already measured sufficient
    // wall "on" states, we consider we are on a wall break state

    // TODO: change this counter depending on the speed of the robot
    // When the robot is slow, this value should be larger
    if (wall_right_counter_on > 100 && wall_right_counter_off > 10) {
        wall_break_already_detected = true;
        return true;
    }

    if (wall_left_counter_on > 100 && wall_left_counter_off > 10) {
        wall_break_already_detected = true;
        return true;
    }

    return false;
}

void Navigation::update(void) {
    bsp::imu::update();
    bsp::encoders::update_ticks();
    bsp::encoders::update_velocities();

    bsp::encoders::EncoderData left_encoder = bsp::encoders::get_data(bsp::encoders::EncoderSide::LEFT);
    bsp::encoders::EncoderData right_encoder = bsp::encoders::get_data(bsp::encoders::EncoderSide::RIGHT);
    float measured_angle_rad = bsp::imu::get_angle();

    if (left_encoder.ticks != 0 || right_encoder.ticks != 0) {
        float estimated_delta_l_mm = (left_encoder.ticks * bsp::encoders::get_encoder_dist_mm_pulse());
        float estimated_delta_r_mm = (right_encoder.ticks * bsp::encoders::get_encoder_dist_mm_pulse());

        float delta_x_mm = (estimated_delta_l_mm + estimated_delta_r_mm) / 2.0;
        traveled_dist_cm += delta_x_mm / 10.0;

        float delta_angle_rad = get_shortest_delta_angle(measured_angle_rad, current_angle_rad);

        float intermediate_angle_rad = current_angle_rad + (delta_angle_rad / 2.0);
        intermediate_angle_rad = limit_angle_minus_pi_pi(intermediate_angle_rad);

        Position rotated_delta;
        rotated_delta.x = delta_x_mm * std::cos(intermediate_angle_rad);
        rotated_delta.y = delta_x_mm * std::sin(intermediate_angle_rad);
        current_position_mm.x += rotated_delta.x;
        current_position_mm.y += rotated_delta.y;
    }

    current_angle_rad = measured_angle_rad;
    bsp::encoders::clear_ticks();
}

bool Navigation::step() {
    using bsp::analog_sensors::ir_side_wall_error;
    using bsp::analog_sensors::SensingDirection;

    switch (current_movement) {
    case Movement::START:
    case Movement::FORWARD:
    case Movement::DIAGONAL:
    case Movement::STOP: {
        using bsp::analog_sensors::ir_reading;

        bool front_emergency = ir_reading(SensingDirection::FRONT_LEFT) > 2850 &&
                               ir_reading(SensingDirection::FRONT_RIGHT) > 2850 &&
                               ir_reading(SensingDirection::LEFT) > 2470 && ir_reading(SensingDirection::RIGHT) > 2850;

        if (current_movement == Movement::STOP) {
            forward_end_speed = 0.0;
        }

        float max_speed = forward_params[current_movement].max_speed;
        float acceleration = forward_params[current_movement].acceleration;
        float deceleration = forward_params[current_movement].deceleration;
        float control_linear_speed = control->get_target_linear_speed();

        if (control_linear_speed < 1.0) {
            acceleration = std::min(acceleration, 5.0f);
        }

        if (control_linear_speed > 3.8) {
            acceleration *= 0.75;
        }

        if (control_linear_speed > 4.8) {
            acceleration *= 0.65;
        }

        if (fix_position_enabled && wall_break_detected()) {
            int cells_traveled = (traveled_dist_cm / CELL_SIZE_CM);
            float corrected_distance_cm = (cells_traveled * CELL_SIZE_CM) + dist_start_seeing_wall_break_cm;
            float distance_error_cm = traveled_dist_cm - corrected_distance_cm;
            if (std::abs(distance_error_cm) < 2.0) {
                traveled_dist_cm = corrected_distance_cm;
                // bsp::buzzer::start();
                // bsp::leds::stripe_set(Color::Blue);
            }
        }

        if (std::abs(traveled_dist_cm) <
            (target_travel_cm -
             (100.0f * get_torricelli_distance(forward_end_speed, control_linear_speed, -deceleration)))) {
            if (control_linear_speed < max_speed) {
                control_linear_speed += acceleration / CONTROL_FREQUENCY_HZ;
                control_linear_speed = std::min(control_linear_speed, max_speed);
            }
        } else {
            if (control_linear_speed > forward_end_speed) {
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
        } else if (current_movement == Movement::FORWARD || current_movement == Movement::START) {
            control->set_wall_pid_enabled(true);
            control->set_diagonal_pid_enabled(false);
        } else {
            control->set_wall_pid_enabled(false);
            control->set_diagonal_pid_enabled(false);
        }

        if (std::abs(traveled_dist_cm) >= target_travel_cm || front_emergency) {
            // bsp::leds::stripe_set(Color::Red);
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

        // If the robot is moving too slow, we use the medium turn params.
        // This can happen if the robot is turning right afer the start movement. TODO: generalize this function
        auto current_turn_params = turn_params[current_movement];
        if (bsp::encoders::get_linear_velocity_m_s() < current_turn_params.turn_linear_speed - 0.25) {
            current_turn_params = turn_params_medium[current_movement];
        }

        // Start turn parameters
        float angular_max_speed = current_turn_params.max_angular_speed;
        float angular_acceleration = current_turn_params.angular_accel;
        float angular_deceleration = current_turn_params.angular_accel;
        float control_angular_speed_abs = std::abs(control->get_target_angular_speed());

        float angle_until_acceleration = std::abs(current_turn_params.angle_accel_rad);
        float angle_untill_start_deceleration = std::abs(current_turn_params.angle_max_ang_speed_rad);
        float deceleration_angle = angle_until_acceleration;
        float final_angle_rad = angle_untill_start_deceleration + deceleration_angle;
        int turn_sign = current_turn_params.sign;

        float angular_end_speed = 0.0;
        float break_angle_margin = 0.0875;

        if (std::abs(bsp::imu::get_incremental_angle()) <
            ((final_angle_rad - break_angle_margin) -
             (get_torricelli_distance(angular_end_speed, control_angular_speed_abs, -angular_deceleration)))) {
            if (control_angular_speed_abs < angular_max_speed) {
                control_angular_speed_abs += angular_acceleration / CONTROL_FREQUENCY_HZ;
                control_angular_speed_abs = std::min(control_angular_speed_abs, angular_max_speed);
            }
        } else {
            if (control_angular_speed_abs > angular_end_speed) {
                control_angular_speed_abs -= angular_acceleration / CONTROL_FREQUENCY_HZ;
                control_angular_speed_abs = std::max(control_angular_speed_abs, 0.0175f);
            }
        }

        control->set_target_angular_speed(control_angular_speed_abs * turn_sign);
        control->set_wall_pid_enabled(false);
        control->set_diagonal_pid_enabled(false);

        if (std::abs(bsp::imu::get_incremental_angle()) >= final_angle_rad) {
            control->set_target_angular_speed(0);
            is_finished = true;
        }

        break;
    }

    case Movement::TURN_RIGHT_90_SEARCH_MODE:
    case Movement::TURN_LEFT_90_SEARCH_MODE:
    case Movement::TURN_AROUND: {
        // Mini FSM
        // 0. Move forward
        // 1. Turn Desired degrees
        // 2. Move forward

        if (mini_fsm_state == 0 || mini_fsm_state == 2) { // Move Forward

            bool front_emergency =
                ir_reading(SensingDirection::FRONT_LEFT) > 2850 && ir_reading(SensingDirection::FRONT_RIGHT) > 2850 &&
                ir_reading(SensingDirection::LEFT) > 2470 && ir_reading(SensingDirection::RIGHT) > 2850;

            float max_speed = forward_params[current_movement].max_speed;
            float acceleration = forward_params[current_movement].acceleration;
            float deceleration = forward_params[current_movement].deceleration;

            float final_speed = forward_params[current_movement].max_speed;
            if (current_movement == Movement::TURN_AROUND && mini_fsm_state == 0) {
                final_speed = 0.0f;
                target_travel_cm = 9.0 + 1.6;
            } else if (current_movement == Movement::TURN_AROUND) {
                target_travel_cm = 8.8;
            }

            if (current_movement == Movement::TURN_RIGHT_90_SEARCH_MODE && mini_fsm_state == 0) {
                target_travel_cm = 2.1; //1.3
            } else if (current_movement == Movement::TURN_RIGHT_90_SEARCH_MODE) {
                //target_travel_cm = std::abs(-90.0 - current_position_mm.y);
                // target_travel_cm = 1.6;
            }

            if (current_movement == Movement::TURN_LEFT_90_SEARCH_MODE && mini_fsm_state == 0) {
                target_travel_cm = 2.1; //1.9
            } else if (current_movement == Movement::TURN_LEFT_90_SEARCH_MODE) {
                //target_travel_cm = 90 - current_position_mm.y;
                // target_travel_cm = 1.8;
            }

            float control_linear_speed = control->get_target_linear_speed();

            if (std::abs(traveled_dist_cm) <
                (target_travel_cm -
                 (100.0f * get_torricelli_distance(final_speed, control_linear_speed, -deceleration)))) {
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

            if (std::abs(traveled_dist_cm) >= target_travel_cm || front_emergency) {

                if (mini_fsm_state == 0) {
                    if (current_movement == Movement::TURN_AROUND) {
                        static int counter = 0;
                        control->set_target_linear_speed(0.0);
                        if (counter++ > 200) {
                            counter = 0;
                            mini_fsm_state = 1;
                            traveled_dist_cm = 0;
                            reference_time = bsp::get_tick_ms();
                            bsp::imu::reset_angle();
                            control->reset();
                        }
                    } else {
                        mini_fsm_state = 1;
                        traveled_dist_cm = 0;
                        reference_time = bsp::get_tick_ms();
                        bsp::imu::reset_angle();
                    }
                } else {
                    is_finished = true;
                    mini_fsm_state = 0;
                }
            }
        } else if (mini_fsm_state == 1) { // Turn Left or Right
            auto current_turn_params = turn_params[current_movement];
            float angular_max_speed = current_turn_params.max_angular_speed;
            float angular_acceleration = current_turn_params.angular_accel;
            float angular_deceleration = current_turn_params.angular_accel;
            float control_angular_speed_abs = std::abs(control->get_target_angular_speed());

            float angle_until_acceleration = std::abs(current_turn_params.angle_accel_rad);
            float angle_untill_start_deceleration = std::abs(current_turn_params.angle_max_ang_speed_rad);
            float deceleration_angle = angle_until_acceleration;
            float final_angle_rad = angle_untill_start_deceleration + deceleration_angle;
            int turn_sign = current_turn_params.sign;

            float angular_end_speed = 0.0;
            float break_angle_margin = 0.00f;

            if (std::abs(bsp::imu::get_incremental_angle()) <
                ((final_angle_rad - break_angle_margin) -
                 (get_torricelli_distance(angular_end_speed, control_angular_speed_abs, -angular_deceleration)))) {
                if (control_angular_speed_abs < angular_max_speed) {
                    control_angular_speed_abs += angular_acceleration / CONTROL_FREQUENCY_HZ;
                    control_angular_speed_abs = std::min(control_angular_speed_abs, angular_max_speed);
                }
            } else {
                if (control_angular_speed_abs > angular_end_speed) {
                    control_angular_speed_abs -= angular_acceleration / CONTROL_FREQUENCY_HZ;
                    control_angular_speed_abs = std::max(control_angular_speed_abs, 0.200f);
                }
            }

            control->set_target_angular_speed(control_angular_speed_abs * turn_sign);

            if (std::abs(bsp::imu::get_incremental_angle()) >= (final_angle_rad - break_angle_margin)) {
                if (current_movement == Movement::TURN_AROUND) {
                    static int counter = 0;
                    control->reset();
                    control->set_target_angular_speed(0.0);
                    if (counter++ > 500) {
                        counter = 0;
                        mini_fsm_state = 2;
                        traveled_dist_cm = 0;
                        control->reset();
                    }
                } else {
                    control->set_target_angular_speed(0);
                    target_travel_cm = (90.0 - std::abs(current_position_mm.y)) / 10.0f;

                    mini_fsm_state = 2;
                    traveled_dist_cm = 0;
                }
            }

        } else { // Should not reach here
            is_finished = true;
            mini_fsm_state = 0;
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

Point Navigation::get_robot_cell_position(void) {
    return current_cell;
}

Position Navigation::get_robot_position(void) {
    return current_position_mm;
}

Direction Navigation::get_robot_direction(void) {
    return current_direction;
}

void Navigation::set_movement(Direction dir) {
    bsp::imu::reset_angle();
    mini_fsm_state = 0;

    target_direction = dir;
    current_movement = get_movement(dir, current_direction, true);

    traveled_dist_cm = 0;
    current_position_mm = {0, 0};
    current_angle_rad = 0;
    reset_wall_break();
    reference_time = bsp::get_tick_ms();
    is_finished = false;

    target_travel_cm = forward_params[current_movement].target_travel_cm;

    if (current_movement == Movement::STOP) {
        forward_end_speed = 0;
    } else {
        forward_end_speed = forward_params[Movement::FORWARD].max_speed;
    }
}

Movement Navigation::get_movement(Direction target_dir, Direction current_dir, bool search_mode) {
    using enum Direction;

    if (target_dir == current_dir) {
        return FORWARD;
    } else if ((target_dir == NORTH && current_dir == WEST) || (target_dir == EAST && current_dir == NORTH) ||
               (target_dir == SOUTH && current_dir == EAST) || (target_dir == WEST && current_dir == SOUTH)) {
        return search_mode ? TURN_RIGHT_90_SEARCH_MODE : TURN_RIGHT_90;
    } else if ((target_dir == NORTH && current_dir == SOUTH) || (target_dir == EAST && current_dir == WEST) ||
               (target_dir == SOUTH && current_dir == NORTH) || (target_dir == WEST && current_dir == EAST)) {
        return TURN_AROUND;
    } else {
        return search_mode ? TURN_LEFT_90_SEARCH_MODE : TURN_LEFT_90;
    }
}

void Navigation::update_position() {
    current_direction = target_direction;
    switch (current_direction) {
    case Direction::NORTH:
        current_cell.y++;
        break;
    case Direction::EAST:
        current_cell.x++;
        break;
    case Direction::SOUTH:
        current_cell.y--;
        break;
    case Direction::WEST:
        current_cell.x--;
        break;
    default:
        break;
    }
}

void Navigation::set_movement(Movement movement, Movement prev_movement, Movement next_movement, uint8_t count) {
    bsp::imu::reset_angle();
    mini_fsm_state = 0;

    current_movement = movement;

    traveled_dist_cm = 0;
    current_position_mm = {0, 0};
    current_angle_rad = 0;
    reset_wall_break();
    reference_time = bsp::get_tick_ms();
    is_finished = false;

    float complete_prev_move_travel = -turn_params[prev_movement].end;

    target_travel_cm = complete_prev_move_travel + (forward_params[movement].target_travel_cm * count) +
                       turn_params[next_movement].start;

    if (current_movement == Movement::STOP) {
        forward_end_speed = 0;
        bsp::leds::stripe_set(Color::Blue);
    } else if (next_movement == Movement::FORWARD || next_movement == Movement::DIAGONAL) {
        forward_end_speed = forward_params[next_movement].max_speed;
    } else if (next_movement == Movement::STOP) {
        forward_end_speed = forward_params[next_movement].max_speed;
    } else {
        forward_end_speed = turn_params[next_movement].turn_linear_speed;
    }
}

std::vector<std::pair<Movement, uint8_t>>
Navigation::get_default_target_movements(std::vector<Direction> target_directions) {

    std::vector<std::pair<Movement, uint8_t>> default_target_movements = {};

    Direction robot_direction = Direction::NORTH;
    default_target_movements.push_back({Movement::START, 1});

    for (auto target_dir : target_directions) {
        Movement movement = get_movement(target_dir, robot_direction, false);
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
    if (default_target_movements.empty()) {
        return {};
    }

    std::vector<Movement> flat_moves;
    for (const auto& move_pair : default_target_movements) {
        for (uint8_t i = 0; i < move_pair.second; ++i) {
            flat_moves.push_back(move_pair.first);
        }
    }

    // Initialize the FSM
    std::vector<std::pair<Movement, uint8_t>> output_movements;
    PathState state = PathState::Start;
    uint8_t run_length = 0;

    // Process the flat list of moves through the state machine
    for (const auto& move : flat_moves) {

        switch (state) {
        case PathState::Start:
            if (move == Movement::START) {
                output_movements.push_back({Movement::START, 1});
                state = PathState::Ortho_F;
                run_length = 0;
            } else if (move == Movement::STOP) {
                state = PathState::Stop;
            }
            break;

        case PathState::Ortho_F:
            if (move == Movement::FORWARD) {
                run_length++;
            } else {
                if (run_length > 0) {
                    output_movements.push_back({Movement::FORWARD, run_length});
                    run_length = 0;
                }
                if (move == Movement::TURN_RIGHT_90)
                    state = PathState::Ortho_R;
                else if (move == Movement::TURN_LEFT_90)
                    state = PathState::Ortho_L;
                else if (move == Movement::STOP)
                    state = PathState::Stop;
            }
            break;

        case PathState::Ortho_R:             // Previous move was TURN_RIGHT_90
            if (move == Movement::FORWARD) { // R-F -> Simple 90-degree turn
                output_movements.push_back({Movement::TURN_RIGHT_90, 1});
                run_length = 1;
                state = PathState::Ortho_F;
            } else if (move == Movement::TURN_RIGHT_90) { // R-R -> Potential 180 turn
                state = PathState::Ortho_RR;
            } else if (move == Movement::TURN_LEFT_90) { // R-L -> Enter Diagonal
                output_movements.push_back({Movement::TURN_RIGHT_45, 1});
                run_length = 0;
                state = PathState::Diag_RL;
            } else if (move == Movement::STOP) { // Path ends with a turn
                output_movements.push_back({Movement::TURN_RIGHT_90, 1});
                state = PathState::Stop;
            }
            break;

        case PathState::Ortho_L:             // Previous move was TURN_LEFT_90
            if (move == Movement::FORWARD) { // L-F -> Simple 90-degree turn
                output_movements.push_back({Movement::TURN_LEFT_90, 1});
                run_length = 1;
                state = PathState::Ortho_F;
            } else if (move == Movement::TURN_LEFT_90) { // L-L -> Potential 180 turn
                state = PathState::Ortho_LL;
            } else if (move == Movement::TURN_RIGHT_90) { // L-R -> Enter Diagonal
                output_movements.push_back({Movement::TURN_LEFT_45, 1});
                run_length = 0;
                state = PathState::Diag_LR;
            } else if (move == Movement::STOP) {
                output_movements.push_back({Movement::TURN_LEFT_90, 1});
                state = PathState::Stop;
            }
            break;

        case PathState::Ortho_RR:            // Previous moves were R-R
            if (move == Movement::FORWARD) { // R-R-F -> 180-degree turn
                output_movements.push_back({Movement::TURN_RIGHT_180, 1});
                run_length = 1;
                state = PathState::Ortho_F;
            } else if (move == Movement::TURN_LEFT_90) { // R-R-L -> Enter Diagonal 135
                output_movements.push_back({Movement::TURN_RIGHT_135, 1});
                run_length = 0;
                state = PathState::Diag_RL;
            } else if (move == Movement::STOP) {
                output_movements.push_back({Movement::TURN_RIGHT_180, 1});
                state = PathState::Stop;
            }
            break;

        case PathState::Ortho_LL:            // Previous moves were L-L
            if (move == Movement::FORWARD) { // L-L-F -> 180-degree turn
                output_movements.push_back({Movement::TURN_LEFT_180, 1});
                run_length = 1;
                state = PathState::Ortho_F;
            } else if (move == Movement::TURN_RIGHT_90) { // L-L-R -> Enter Diagonal 135
                output_movements.push_back({Movement::TURN_LEFT_135, 1});
                run_length = 0;
                state = PathState::Diag_LR;
            } else if (move == Movement::STOP) {
                output_movements.push_back({Movement::TURN_LEFT_180, 1});
                state = PathState::Stop;
            }
            break;

        case PathState::Diag_RL:             // On diagonal, last Turn was Left
            if (move == Movement::FORWARD) { // Exit diagonal path
                if (run_length > 0) {
                    output_movements.push_back({Movement::DIAGONAL, run_length});
                }
                output_movements.push_back({Movement::TURN_LEFT_45_FROM_45, 1});
                run_length = 1;
                state = PathState::Ortho_F;
            } else if (move == Movement::TURN_RIGHT_90) { // Turn right
                run_length++;
                state = PathState::Diag_LR;
            } else if (move == Movement::TURN_LEFT_90) { // Potential D-D turn
                state = PathState::Diag_LL;
            } else if (move == Movement::STOP) {
                if (run_length > 0) {
                    output_movements.push_back({Movement::DIAGONAL, run_length});
                }
                output_movements.push_back({Movement::TURN_LEFT_45_FROM_45, 1});
                state = PathState::Stop;
            }
            break;

        case PathState::Diag_LR:             // On diagonal, last turn was Right
            if (move == Movement::FORWARD) { // Exit diagonal path
                if (run_length > 0) {
                    output_movements.push_back({Movement::DIAGONAL, run_length});
                }
                output_movements.push_back({Movement::TURN_RIGHT_45_FROM_45, 1});
                run_length = 1;
                state = PathState::Ortho_F;
            } else if (move == Movement::TURN_LEFT_90) { // Turn left
                run_length++;
                state = PathState::Diag_RL;
            } else if (move == Movement::TURN_RIGHT_90) { // Potential D-D turn
                state = PathState::Diag_RR;
            } else if (move == Movement::STOP) {
                if (run_length > 0) {
                    output_movements.push_back({Movement::DIAGONAL, run_length});
                }
                output_movements.push_back({Movement::TURN_RIGHT_45_FROM_45, 1});
                state = PathState::Stop;
            }
            break;

        case PathState::Diag_LL:                   // On diagonal, saw L-L pattern
            if (move == Movement::TURN_RIGHT_90) { // L-L-R -> 90-degree D-D turn
                if (run_length > 0) {
                    output_movements.push_back({Movement::DIAGONAL, run_length});
                }
                output_movements.push_back({Movement::TURN_LEFT_90_FROM_45, 1});
                run_length = 0;
                state = PathState::Diag_LR;
            } else if (move == Movement::FORWARD) { // L-L-F -> 135-degree exit
                if (run_length > 0) {
                    output_movements.push_back({Movement::DIAGONAL, run_length});
                }
                output_movements.push_back({Movement::TURN_LEFT_135_FROM_45, 1});
                run_length = 1;
                state = PathState::Ortho_F;
            } else if (move == Movement::STOP) {
                if (run_length > 0) {
                    output_movements.push_back({Movement::DIAGONAL, run_length});
                }
                output_movements.push_back({Movement::TURN_LEFT_135_FROM_45, 1});
                state = PathState::Stop;
            }
            break;

        case PathState::Diag_RR:                  // On diagonal, saw R-R pattern
            if (move == Movement::TURN_LEFT_90) { // R-R-L -> 90-degree D-D turn
                if (run_length > 0) {
                    output_movements.push_back({Movement::DIAGONAL, run_length});
                }
                output_movements.push_back({Movement::TURN_RIGHT_90_FROM_45, 1});
                run_length = 0;
                state = PathState::Diag_RL;
            } else if (move == Movement::FORWARD) { // R-R-F -> 135-degree exit
                if (run_length > 0) {
                    output_movements.push_back({Movement::DIAGONAL, run_length});
                }
                output_movements.push_back({Movement::TURN_RIGHT_135_FROM_45, 1});
                run_length = 1;
                state = PathState::Ortho_F;
            } else if (move == Movement::STOP) {
                if (run_length > 0) {
                    output_movements.push_back({Movement::DIAGONAL, run_length});
                }
                output_movements.push_back({Movement::TURN_RIGHT_135_FROM_45, 1});
                state = PathState::Stop;
            }
            break;

        case PathState::Stop:
            break;
        }

        break;
    }

    output_movements.push_back({Movement::STOP, 1});
    print_movement_sequence(output_movements, "Diagonal");

    return output_movements;
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
