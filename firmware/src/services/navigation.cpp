#include <algorithm>
#include <cmath>
#include <cstdio>
#include <map>
#include <string>

#include "bsp/analog_sensors.hpp"
#include "bsp/encoders.hpp"
#include "bsp/imu.hpp"
#include "bsp/leds.hpp"
#include "bsp/timers.hpp"
#include "services/config.hpp"
#include "services/navigation.hpp"
#include "utils/math.hpp"
#include "utils/movement_params.hpp"
#include "utils/types.hpp"

/// @section Constants

static std::map<Movement, TurnParams> turn_params;
static std::map<Movement, ForwardParams> forward_params;
static GeneralParams general_params;

using bsp::leds::Color;

namespace {

constexpr float FRONT_EMERGENCY_DISTANCE_MM = 50.0f;
constexpr float WALL_BREAK_DEBUG_DISTANCE_MIN_MM = 90.0f;
constexpr float WALL_BREAK_DEBUG_DISTANCE_MAX_MM = 97.5f;
constexpr float SEARCH_WALL_BREAK_MIN_DISTANCE_MM = 35.0f;
constexpr uint32_t WALL_BREAK_CONFIRM_COUNT = 4;
constexpr float WALL_BREAK_MAX_CORRECTION_ERROR_MM = 40.0f;
constexpr float LINEAR_BRAKE_MARGIN_MM = 20.0f;
constexpr float LINEAR_ACCEL_MARGIN_MM = 20.0f;
constexpr float DIAGONAL_PID_START_DISTANCE_MM = 50.0f;
constexpr uint32_t STABILIZE_FORWARD_TIME_MS = 200;
constexpr uint32_t STABILIZE_TURN_TIME_MS = 400;
constexpr float MILLIMETERS_PER_METER = 1000.0f;

bool is_search_mode(services::Navigation::navigation_mode_t mode) {
    return mode == services::Navigation::SEARCH_FAST || mode == services::Navigation::SEARCH_MEDIUM ||
           mode == services::Navigation::SEARCH_SLOW;
}

GeneralParams make_custom_general_params() {
    return {
        services::Config::fan_speed,
        services::Config::angular_kp,
        services::Config::angular_ki,
        services::Config::angular_kd,
        services::Config::angular_acc_feed_forward_k,
        services::Config::angular_brake_feed_forward_k,
        services::Config::angular_vel_feed_forward_k,
        services::Config::linear_vel_acc_feed_forward_k,
        services::Config::linear_vel_brake_feed_forward_k,
        services::Config::linear_vel_feed_forward_k,
        services::Config::wall_kp,
        services::Config::wall_ki,
        services::Config::wall_kd,
        services::Config::linear_vel_kp,
        services::Config::linear_vel_ki,
        services::Config::linear_vel_kd,
        services::Config::diagonal_walls_kp,
        services::Config::diagonal_walls_ki,
        services::Config::diagonal_walls_kd,
        services::Config::start_wall_break_mm_left,
        services::Config::start_wall_break_mm_right,
        services::Config::enable_wall_break_correction,
        services::Config::max_linear_acc_jerk,
        services::Config::max_linear_brake_jerk,
        services::Config::coulomb_ff,
        services::Config::angular_coulomb_ff,
        services::Config::angular_static_ff,
        services::Config::angular_coulomb_ff_inplace,
        services::Config::angular_static_ff_inplace,
    };
}

} // namespace

/// @section Service implementation

namespace services {

Navigation* Navigation::instance() {
    static Navigation p;
    return &p;
}

void Navigation::init() {
    control = Control::instance();
    reset(SEARCH_SLOW);

    if (!is_initialized) {
        is_initialized = true;
    }
}

void Navigation::reset(navigation_mode_t mode) {

    reset_movement_variables(true);
    encoder_left_counter = 0;
    encoder_right_counter = 0;
    current_cell = {0, 0};
    complete_prev_move_travel = 0;
    waiting_for_fast_param = false;
    turn_end_correction_mm = 0.0f;

    current_direction = Direction::NORTH;

    bsp::encoders::reset();
    bsp::imu::reset();

    selected_mode = mode;
    configure_mode(mode);

    control->reset(general_params);

    current_movement = Movement::START;
    previous_movement = Movement::START;
    target_travel_mm = forward_params[Movement::START].target_travel_mm;
    forward_end_speed = forward_params[Movement::START].max_speed;
}

void Navigation::configure_mode(navigation_mode_t mode) {
    if (mode == CUSTOM) {
        turn_params = turn_params_custom;
        forward_params = forward_params_custom;
        general_params = make_custom_general_params();
    } else {
        turn_params = get_turn_params(mode);
        forward_params = get_forward_params(mode);
        general_params = get_general_params(mode);
    }
}

bool Navigation::is_front_emergency() const {
    if (current_movement != Movement::FORWARD && current_movement != Movement::START) {
        return false;
    }

    using bsp::analog_sensors::ir_distance_mm;
    using bsp::analog_sensors::SensingDirection;

    return ir_distance_mm(SensingDirection::FRONT_LEFT) < FRONT_EMERGENCY_DISTANCE_MM &&
           ir_distance_mm(SensingDirection::FRONT_RIGHT) < FRONT_EMERGENCY_DISTANCE_MM &&
           ir_distance_mm(SensingDirection::LEFT) < FRONT_EMERGENCY_DISTANCE_MM &&
           ir_distance_mm(SensingDirection::RIGHT) < FRONT_EMERGENCY_DISTANCE_MM;
}

void Navigation::reset_movement_variables(bool reset_linear_accel) {
    bsp::imu::reset_angle();
    mini_fsm_state = MiniFSMStates::FORWARD_1;
    current_angular_acceleration = 0.0f;
    if (reset_linear_accel) {
        current_linear_acceleration = 0.0f;
    }

    traveled_dist_mm = 0;
    current_position_mm = {0, 0};
    current_angle_rad = 0;
    reset_wall_break();
    reference_time = bsp::get_tick_ms();
    turn_tick_counter = 0;
    is_finished = false;
    is_braking = false;
    control->set_use_inplace_friction(false);

    if (!is_linear_movement(current_movement) || previous_movement != current_movement) {
        bsp::analog_sensors::ir_reset_all_wall_hysteresis();
    }
}

float Navigation::get_torricelli_distance(float final_speed, float initial_speed, float acceleration) {
    return (final_speed * final_speed - initial_speed * initial_speed) / (2.0f * acceleration);
}

float Navigation::get_s_curve_brake_distance(float initial_speed, float final_speed, float deceleration, float jerk) {
    if (initial_speed <= final_speed || deceleration <= 0.0f || jerk <= 0.0f) {
        return 0.0f;
    }

    float delta_v = initial_speed - final_speed;
    float v_threshold = (deceleration * deceleration) / jerk;

    if (delta_v >= v_threshold) {
        return ((initial_speed * initial_speed - final_speed * final_speed) / (2.0f * deceleration)) +
               (((initial_speed + final_speed) * deceleration) / (2.0f * jerk));
    } else {
        return (initial_speed + final_speed) * std::sqrt(delta_v / jerk);
    }
}

bool Navigation::start_accel_ramp_down(float current_speed, float current_accel, float max_speed, float jerk) {
    if (jerk <= 0.0f) {
        return false;
    }
    if (current_accel <= 0.0f) {
        return current_speed >= max_speed;
    }
    float pred_speed = current_speed + (current_accel * current_accel) / (2.0f * jerk);
    return pred_speed >= max_speed;
}

bool Navigation::start_brake_ramp_up(float current_speed, float current_accel, float final_speed, float jerk) {
    if (jerk <= 0.0f) {
        return false;
    }
    if (current_accel >= 0.0f) {
        return current_speed <= final_speed;
    }
    float pred_speed = current_speed - (current_accel * current_accel) / (2.0f * jerk);
    return pred_speed <= final_speed;
}

float Navigation::get_effective_max_acceleration(float current_speed, float base_max_accel) {
    if (current_speed > 7.0f) {
        return 17.0f;
    } else if (current_speed > 6.5f) {
        return 22.0f;
    } else if (current_speed > 6.0f) {
        return 25.0f;
    } else if (current_speed > 5.5f) {
        return 30.0f;
    } else if (current_speed > 5.0f) {
        return 35.0f;
    } else {
        return base_max_accel;
    }
}

void Navigation::reset_wall_break() {
    wall_right_was_confirmed = false;
    wall_left_was_confirmed = false;

    wall_break_last_dist = 0.0f;
    current_wall_break_detected = false;
}

Navigation::WallBreak Navigation::process_wall_break() {
    if (current_movement != Movement::FORWARD) {
        wall_left_was_confirmed = false;
        wall_right_was_confirmed = false;
        return WallBreak::NONE;
    }

    const bool right_is_confirmed = bsp::analog_sensors::ir_is_wall_confirmed(bsp::analog_sensors::SensingDirection::RIGHT);
    const bool left_is_confirmed = bsp::analog_sensors::ir_is_wall_confirmed(bsp::analog_sensors::SensingDirection::LEFT);

    const bool right_break = bsp::analog_sensors::ir_wall_break_condition(bsp::analog_sensors::SensingDirection::RIGHT);
    const bool left_break = bsp::analog_sensors::ir_wall_break_condition(bsp::analog_sensors::SensingDirection::LEFT);

    const bool falling_edge_right = wall_right_was_confirmed && right_break;
    const bool falling_edge_left  = wall_left_was_confirmed  && left_break;

    wall_right_was_confirmed = right_is_confirmed;
    wall_left_was_confirmed = left_is_confirmed;

    bool process = false;
    const float wall_break_distance = traveled_dist_mm - wall_break_last_dist;
    if (is_search_mode(selected_mode)) {
        bool valid_previous_move =
            (previous_movement == FORWARD || previous_movement == START || previous_movement == TURN_AROUND ||
             previous_movement == TURN_RIGHT_90_SEARCH_MODE || previous_movement == TURN_LEFT_90_SEARCH_MODE);

        if (valid_previous_move && wall_break_distance > SEARCH_WALL_BREAK_MIN_DISTANCE_MM &&
            !current_wall_break_detected) {
            process = true;
        }
    } else if (wall_break_distance > CELL_SIZE_MM) {
        process = true;
    } else if ((previous_movement == START) && (current_movement == FORWARD) && (wall_break_last_dist < 0.1)) {
        process = true;
    }

    if (!process) {
        return WallBreak::NONE;
    }

    if (falling_edge_right) {
        wall_break_last_dist = traveled_dist_mm;
        current_wall_break_detected = true;
        return WallBreak::RIGHT;
    }

    if (falling_edge_left) {
        wall_break_last_dist = traveled_dist_mm;
        current_wall_break_detected = true;
        return WallBreak::LEFT;
    }

    return WallBreak::NONE;
}
void Navigation::apply_wall_break_correction() {
    const WallBreak wall_break = process_wall_break();
    if (wall_break == WallBreak::NONE) {
        return;
    }

    using bsp::analog_sensors::ir_distance_mm;
    using bsp::analog_sensors::ir_reading_wall;
    using bsp::analog_sensors::SensingDirection;

    constexpr float DEG_TO_RAD = 3.14159265358979323846f / 180.0f;
    constexpr float MAX_LATERAL_OFFSET_MM = 10.0f;
    constexpr float MAX_LONGITUDINAL_CORRECTION = 10.0f;

    const float current_movement_traveled = traveled_dist_mm - complete_prev_move_travel;
    const int cells_traveled = static_cast<int>(current_movement_traveled / CELL_SIZE_MM);

    float base_offset_mm = 0.0f;
    float longitudinal_correction_mm = 0.0f;

    if (wall_break == WallBreak::LEFT) {
        base_offset_mm = general_params.start_wall_break_mm_left;

        // Left wall just broke: measure lateral displacement using opposite (Right) sensor
        if (Config::enable_lateral_correction_wall && ir_reading_wall(SensingDirection::RIGHT)) {
            const float cos_theta_r = std::cos(Config::right_sensor_angle_deg * DEG_TO_RAD);
            const float tan_theta_l = std::tan(Config::left_sensor_angle_deg * DEG_TO_RAD);

            const float measured_dist_r = ir_distance_mm(SensingDirection::RIGHT);
            const float delta_l_r = measured_dist_r - Config::ir_wall_dist_ref_right;

            // lateral_offset_mm > 0 means further from right wall => shifted toward the left wall
            float lateral_offset_mm = delta_l_r * cos_theta_r;
            lateral_offset_mm = std::clamp(lateral_offset_mm, -MAX_LATERAL_OFFSET_MM, MAX_LATERAL_OFFSET_MM);

            // Closer to left wall means the beam caught the break later along the track (+dx)
            longitudinal_correction_mm = lateral_offset_mm * tan_theta_l;
            longitudinal_correction_mm =
                std::clamp(longitudinal_correction_mm, -MAX_LONGITUDINAL_CORRECTION, MAX_LONGITUDINAL_CORRECTION);
        }
    } else if (wall_break == WallBreak::RIGHT) {
        base_offset_mm = general_params.start_wall_break_mm_right;

        // Right wall just broke: measure lateral displacement using opposite (Left) sensor
        if (Config::enable_lateral_correction_wall && ir_reading_wall(SensingDirection::LEFT)) {
            const float cos_theta_l = std::cos(Config::left_sensor_angle_deg * DEG_TO_RAD);
            const float tan_theta_r = std::tan(Config::right_sensor_angle_deg * DEG_TO_RAD);

            const float measured_dist_l = ir_distance_mm(SensingDirection::LEFT);
            const float delta_l_l = measured_dist_l - Config::ir_wall_dist_ref_left;

            // lateral_offset_mm > 0 means further from left wall => shifted toward the right wall
            float lateral_offset_mm = delta_l_l * cos_theta_l;
            lateral_offset_mm = std::clamp(lateral_offset_mm, -MAX_LATERAL_OFFSET_MM, MAX_LATERAL_OFFSET_MM);

            // Closer to right wall means the beam caught the break later along the track (+dx)
            longitudinal_correction_mm = lateral_offset_mm * tan_theta_r;
            longitudinal_correction_mm =
                std::clamp(longitudinal_correction_mm, -MAX_LONGITUDINAL_CORRECTION, MAX_LONGITUDINAL_CORRECTION);
        }
    }

    const float corrected_distance_mm =
        (cells_traveled * CELL_SIZE_MM) + base_offset_mm + longitudinal_correction_mm + complete_prev_move_travel;

    const float distance_error_mm = current_movement_traveled - corrected_distance_mm;

    if (std::abs(distance_error_mm) < WALL_BREAK_MAX_CORRECTION_ERROR_MM) {
        traveled_dist_mm = corrected_distance_mm;
        bsp::leds::stripe_set(Color::Red);
    } else {
        bsp::leds::stripe_set(Color::White);
    }
}

float Navigation::get_acceleration_ramp_distance_m(float current_speed, float acceleration, float brake_jerk) const {
    const float ramp_time = acceleration / brake_jerk;
    return (current_speed * ramp_time) +
           (acceleration * acceleration * acceleration) / (3.0f * brake_jerk * brake_jerk);
}

void Navigation::update(void) {
    bsp::imu::update();
    bsp::encoders::update_ticks();
    float target_linear_accel = control ? control->get_target_linear_acceleration() : 0.0f;
    bsp::encoders::update_velocities(target_linear_accel);

    bsp::encoders::EncoderData left_encoder = bsp::encoders::get_data(bsp::encoders::EncoderSide::LEFT);
    bsp::encoders::EncoderData right_encoder = bsp::encoders::get_data(bsp::encoders::EncoderSide::RIGHT);
    float measured_angle_rad = bsp::imu::get_angle();

    float estimated_delta_l_mm = (left_encoder.ticks * bsp::encoders::get_encoder_dist_mm_pulse());
    float estimated_delta_r_mm = (right_encoder.ticks * bsp::encoders::get_encoder_dist_mm_pulse());

    float delta_x_mm = (estimated_delta_l_mm + estimated_delta_r_mm) / 2.0;
    float encoder_diff_mm = (estimated_delta_r_mm - estimated_delta_l_mm);
    if (left_encoder.ticks == 0 && right_encoder.ticks == 0) {
        delta_x_mm = 0.0f;
        encoder_diff_mm = 0.0f;
    }

    float imu_delta_angle_rad = get_shortest_delta_angle(measured_angle_rad, current_angle_rad);

    float intermediate_angle_rad = current_angle_rad + (imu_delta_angle_rad / 2.0);
    intermediate_angle_rad = limit_angle_minus_pi_pi(intermediate_angle_rad);

    float expected_diff_mm = imu_delta_angle_rad * Config::WHEELS_DIST_MM;
    encoder_imu_diff = encoder_diff_mm - expected_diff_mm;

    traveled_dist_mm += delta_x_mm;
    bsp::analog_sensors::ir_update_wall_hysteresis(delta_x_mm);

    Position rotated_delta;
    rotated_delta.x = delta_x_mm * std::cos(intermediate_angle_rad);
    rotated_delta.y = delta_x_mm * std::sin(intermediate_angle_rad);
    current_position_mm.x += rotated_delta.x;
    current_position_mm.y += rotated_delta.y;

    current_angle_rad = measured_angle_rad;
    bsp::encoders::clear_ticks();
}

bool Navigation::is_linear_movement(Movement movement) const {
    return movement == Movement::START || movement == Movement::FORWARD || movement == Movement::DIAGONAL ||
           movement == Movement::STOP;
}

bool Navigation::is_turn_around_movement() const {
    return current_movement == Movement::TURN_AROUND || current_movement == Movement::TURN_AROUND_INPLACE;
}

bool Navigation::is_search_turn_movement() const {
    return current_movement == Movement::TURN_RIGHT_90_SEARCH_MODE ||
           current_movement == Movement::TURN_LEFT_90_SEARCH_MODE;
}

bool Navigation::is_turn_from_diagonal() const {
    return current_movement == Movement::TURN_RIGHT_90_FROM_45 || current_movement == Movement::TURN_LEFT_90_FROM_45;
}

bool Navigation::is_turn_movement(Movement movement) const {
    switch (movement) {
    case Movement::TURN_LEFT_45:
    case Movement::TURN_RIGHT_45:
    case Movement::TURN_LEFT_90:
    case Movement::TURN_RIGHT_90:
    case Movement::TURN_LEFT_135:
    case Movement::TURN_RIGHT_135:
    case Movement::TURN_RIGHT_180:
    case Movement::TURN_LEFT_180:
    case Movement::TURN_RIGHT_90_SEARCH_MODE:
    case Movement::TURN_LEFT_90_SEARCH_MODE:
    case Movement::TURN_RIGHT_45_FROM_45:
    case Movement::TURN_LEFT_45_FROM_45:
    case Movement::TURN_RIGHT_90_FROM_45:
    case Movement::TURN_LEFT_90_FROM_45:
    case Movement::TURN_RIGHT_135_FROM_45:
    case Movement::TURN_LEFT_135_FROM_45:
    case Movement::TURN_AROUND_INPLACE:
    case Movement::TURN_AROUND:
        return true;
    default:
        return false;
    }
}

float Navigation::get_required_brake_distance(float control_linear_speed, float deceleration,
                                              bool continuous_start_to_forward) {
    if (continuous_start_to_forward) {
        return 0.0f;
    }

    float brake_distance_mm = LINEAR_BRAKE_MARGIN_MM;

    if (current_linear_acceleration > 0.0f && general_params.max_linear_brake_jerk > 0.0f) {
        float brake_jerk = general_params.max_linear_brake_jerk;
        float delta_v = (current_linear_acceleration * current_linear_acceleration) / (2.0f * brake_jerk);
        float peak_speed = control_linear_speed + delta_v;
        float ramp_distance_m =
            get_acceleration_ramp_distance_m(control_linear_speed, current_linear_acceleration, brake_jerk);

        brake_distance_mm +=
            MILLIMETERS_PER_METER *
            (ramp_distance_m + get_s_curve_brake_distance(peak_speed, forward_end_speed, deceleration, brake_jerk));
    } else {
        brake_distance_mm +=
            MILLIMETERS_PER_METER * get_s_curve_brake_distance(control_linear_speed, forward_end_speed, deceleration,
                                                               general_params.max_linear_brake_jerk);
    }

    return brake_distance_mm;
}

void Navigation::update_linear_target_speed(float& control_linear_speed, float max_speed, float max_acceleration,
                                            float deceleration, bool continuous_start_to_forward) {
    const float accel_jerk = general_params.max_linear_acc_jerk;
    const float brake_jerk = general_params.max_linear_brake_jerk;
    const float required_brake_distance =
        get_required_brake_distance(control_linear_speed, deceleration, continuous_start_to_forward);
    const bool requires_turn_margin = (previous_movement != Movement::START) && (control_linear_speed >= 1.0f);

    const bool should_accelerate =
        !is_braking &&
        (continuous_start_to_forward || (std::abs(traveled_dist_mm) < (target_travel_mm - required_brake_distance)));

    if (should_accelerate) {
        if (requires_turn_margin && std::abs(traveled_dist_mm) <= LINEAR_ACCEL_MARGIN_MM) {
            return;
        }

        if (control_linear_speed >= max_speed) {
            if (current_linear_acceleration > 0.0f) {
                current_linear_acceleration -= accel_jerk / Config::CONTROL_FREQUENCY_HZ;
                current_linear_acceleration = std::max(current_linear_acceleration, 0.0f);
            } else {
                current_linear_acceleration = 0.0f;
            }
            control_linear_speed = max_speed;
            return;
        }

        if (start_accel_ramp_down(control_linear_speed, current_linear_acceleration, max_speed, accel_jerk)) {
            current_linear_acceleration -= accel_jerk / Config::CONTROL_FREQUENCY_HZ;
            current_linear_acceleration = std::max(current_linear_acceleration, 0.0f);
            control_linear_speed += current_linear_acceleration / Config::CONTROL_FREQUENCY_HZ;
            control_linear_speed = std::min(control_linear_speed, max_speed);
            return;
        }

        const float effective_max_acceleration = get_effective_max_acceleration(control_linear_speed, max_acceleration);
        if (current_linear_acceleration < effective_max_acceleration) {
            current_linear_acceleration += accel_jerk / Config::CONTROL_FREQUENCY_HZ;
            current_linear_acceleration = std::min(current_linear_acceleration, effective_max_acceleration);
        } else if (current_linear_acceleration > effective_max_acceleration) {
            current_linear_acceleration -= accel_jerk / Config::CONTROL_FREQUENCY_HZ;
            current_linear_acceleration = std::max(current_linear_acceleration, effective_max_acceleration);
        }

        control_linear_speed += current_linear_acceleration / Config::CONTROL_FREQUENCY_HZ;
        control_linear_speed = std::min(control_linear_speed, max_speed);
        return;
    }

    // else if !should_accelerate continues here...
    if (continuous_start_to_forward || std::abs(traveled_dist_mm) <= LINEAR_ACCEL_MARGIN_MM) {
        return;
    }

    is_braking = true;

    if (control_linear_speed > forward_end_speed) {
        if (start_brake_ramp_up(control_linear_speed, current_linear_acceleration, forward_end_speed, brake_jerk)) {
            current_linear_acceleration += brake_jerk / Config::CONTROL_FREQUENCY_HZ;
            current_linear_acceleration = std::min(current_linear_acceleration, 0.0f);
        } else {
            current_linear_acceleration -= brake_jerk / Config::CONTROL_FREQUENCY_HZ;
            current_linear_acceleration = std::max(current_linear_acceleration, -deceleration);
        }

        control_linear_speed += current_linear_acceleration / Config::CONTROL_FREQUENCY_HZ;
        control_linear_speed = std::max(control_linear_speed, forward_end_speed);
        if (forward_end_speed > 0.0f) {
            control_linear_speed = std::max(control_linear_speed, Config::min_move_speed);
        }
    } else {
        current_linear_acceleration = 0.0f;
        control_linear_speed = std::min(control_linear_speed, forward_end_speed);
        if (forward_end_speed > 0.0f) {
            control_linear_speed = std::max(control_linear_speed, Config::min_move_speed);
        }
    }
}

void Navigation::configure_linear_pid() {
    if (current_movement == Movement::DIAGONAL) {
        control->set_wall_pid_enabled(false);
        const bool before_diagonal_end =
            std::abs(traveled_dist_mm) < (target_travel_mm - (CELL_DIAGONAL_SIZE_MM / 2.0f));
        control->set_diagonal_pid_enabled(before_diagonal_end);
        return;
    }

    if (current_movement == Movement::STOP) {
        control->set_wall_pid_enabled(false);
        control->set_diagonal_pid_enabled(false);
        return;
    }

    if (current_movement == Movement::FORWARD || current_movement == Movement::START) {
        control->set_wall_pid_enabled(true);
        control->set_diagonal_pid_enabled(false);
        return;
    }

    control->set_wall_pid_enabled(false);
    control->set_diagonal_pid_enabled(false);
}

void Navigation::finish_linear_movement(float control_linear_speed) {
    const bool reached_target = std::abs(traveled_dist_mm) >= target_travel_mm;
    const bool finished_stop = current_movement == Movement::STOP && is_braking && control_linear_speed <= 0.0f;

    if (reached_target || finished_stop || is_front_emergency()) {
        is_finished = true;
    }
}

void Navigation::step_linear_movement() {
    control->set_use_inplace_friction(false);

    if (current_movement == Movement::STOP) {
        forward_end_speed = 0.0f;
    }

    const bool continuous_start_to_forward =
        current_movement == Movement::START && forward_end_speed > forward_params[Movement::START].max_speed;

    const float max_speed =
        continuous_start_to_forward ? forward_end_speed : forward_params[current_movement].max_speed;
    const float max_acceleration = forward_params[current_movement].acceleration;
    const float deceleration = forward_params[current_movement].deceleration;
    float control_linear_speed = control->get_target_linear_speed();

    if (general_params.enable_wall_break_correction) {
        apply_wall_break_correction();
    }

    update_linear_target_speed(control_linear_speed, max_speed, max_acceleration, deceleration,
                               continuous_start_to_forward);

    control->set_target_linear_speed(control_linear_speed);
    control->set_target_angular_speed(0);
    configure_linear_pid();
    finish_linear_movement(control_linear_speed);
}

void Navigation::update_turn_linear_speed(float& control_linear_speed, float max_speed, float acceleration,
                                          float deceleration, float final_speed) {
    const float braking_distance_mm =
        MILLIMETERS_PER_METER * get_torricelli_distance(final_speed, control_linear_speed, -deceleration);
    const bool before_braking_point =
        !is_braking && (std::abs(traveled_dist_mm) < (target_travel_mm - braking_distance_mm));

    if (before_braking_point) {
        if (control_linear_speed < max_speed) {
            control_linear_speed += acceleration / Config::CONTROL_FREQUENCY_HZ;
            control_linear_speed = std::min(control_linear_speed, max_speed);
        }
    } else if (control_linear_speed > final_speed) {
        is_braking = true;
        control_linear_speed -= deceleration / Config::CONTROL_FREQUENCY_HZ;
        control_linear_speed = std::max(control_linear_speed, final_speed);
        if (final_speed > 0.0f) {
            control_linear_speed = std::max(control_linear_speed, Config::min_move_speed);
        }
    }
}

void Navigation::transition_after_turn_forward() {
    is_braking = false;
    if (is_turn_around_movement()) {
        if (mini_fsm_state == MiniFSMStates::FORWARD_2) {
            is_finished = true;
            mini_fsm_state = MiniFSMStates::FORWARD_1;
        } else {
            control->set_target_linear_speed(0.0f);
            control->set_motor_control_disabled(true);
            reference_time = bsp::get_tick_ms();
            mini_fsm_state = MiniFSMStates::STABILIZE_1;
        }
        return;
    }

    if (mini_fsm_state == MiniFSMStates::FORWARD_1) {
        traveled_dist_mm = 0;
        reference_time = bsp::get_tick_ms();
        turn_tick_counter = 0;
        mini_fsm_state = MiniFSMStates::TURN;
        current_angular_acceleration = 0.0f;

        if (!is_search_mode(selected_mode)) {
            bsp::leds::stripe_set(Color::Blue);
        }
    } else {
        is_finished = true;
        mini_fsm_state = MiniFSMStates::FORWARD_1;
    }
}

void Navigation::step_turn_forward() {
    control->set_use_inplace_friction(false);
    const float max_speed = forward_params[current_movement].max_speed;
    const float acceleration = forward_params[current_movement].acceleration;
    const float deceleration = forward_params[current_movement].deceleration;
    float final_speed = forward_params[current_movement].max_speed;

    if (is_turn_around_movement() && mini_fsm_state == MiniFSMStates::FORWARD_1) {
        final_speed = 0.0f;
        control->set_wall_pid_enabled(false);
    } else if (mini_fsm_state == MiniFSMStates::FORWARD_1) {
        control->set_wall_pid_enabled(false);
    } else {
        control->set_wall_pid_enabled(true);
    }

    if (is_turn_from_diagonal() && mini_fsm_state == MiniFSMStates::FORWARD_1) {
        control->set_diagonal_pid_enabled(std::abs(traveled_dist_mm) < DIAGONAL_PID_START_DISTANCE_MM);
    }

    float control_linear_speed = control->get_target_linear_speed();
    update_turn_linear_speed(control_linear_speed, max_speed, acceleration, deceleration, final_speed);

    control->set_target_linear_speed(control_linear_speed);
    control->set_target_angular_speed(0);

    const bool reached_target = std::abs(traveled_dist_mm) >= target_travel_mm;
    const bool is_turn_around_stop = is_turn_around_movement() && (mini_fsm_state == MiniFSMStates::FORWARD_1);

    const bool should_transition = is_turn_around_stop ? (is_braking && control_linear_speed <= 0.0f) : reached_target;

    if (should_transition) {
        transition_after_turn_forward();
    }
}

void Navigation::update_turn_angular_acceleration(const TurnParams& turn, uint32_t elapsed_time) {
    const float max_angular_acceleration = turn.angular_accel;
    const float max_angular_deceleration = -turn.angular_accel;
    const float control_frequency = Config::CONTROL_FREQUENCY_HZ;

    if (elapsed_time < turn.t_start_deccel) {
        if (turn.accel_ramp_up_jerk == 0 || turn.time_to_decrease_jerk_1 == 0) {
            current_angular_acceleration = max_angular_acceleration;
        } else if (elapsed_time < turn.time_to_decrease_jerk_1) {
            current_angular_acceleration += turn.accel_ramp_up_jerk / control_frequency;
            current_angular_acceleration = std::min(current_angular_acceleration, max_angular_acceleration);
        } else {
            current_angular_acceleration -= turn.accel_ramp_down_jerk / control_frequency;
            current_angular_acceleration = std::max(current_angular_acceleration, 0.0f);
        }
        return;
    }

    if (turn.accel_ramp_down_jerk == 0 || turn.time_to_decrease_jerk_2 == 0) {
        current_angular_acceleration = max_angular_deceleration;
    } else if (elapsed_time < turn.time_to_decrease_jerk_2) {
        current_angular_acceleration -= turn.accel_ramp_down_jerk / control_frequency;
        current_angular_acceleration = std::max(current_angular_acceleration, max_angular_deceleration);
    } else {
        current_angular_acceleration += turn.accel_ramp_up_jerk / control_frequency;
        current_angular_acceleration = std::min(current_angular_acceleration, 0.0f);
    }
}

void Navigation::transition_after_turn_rotation() {
    control->set_use_inplace_friction(false);

    if (is_turn_around_movement()) {
        control->set_target_angular_speed(0.0f);
        control->set_motor_control_disabled(true);
        reference_time = bsp::get_tick_ms();
        is_braking = false;
        mini_fsm_state = MiniFSMStates::STABILIZE_2;
        return;
    }

    if (is_search_turn_movement()) {
        control->set_target_angular_speed(0.0f);
        // target_travel_mm for FORWARD_2 is based on the calculated position
        target_travel_mm = HALF_CELL_SIZE_MM - std::abs(current_position_mm.y);
        reference_time = bsp::get_tick_ms();
        traveled_dist_mm = 0;
        is_braking = false;
        mini_fsm_state = MiniFSMStates::FORWARD_2;
        return;
    }

    is_finished = true;
    mini_fsm_state = MiniFSMStates::FORWARD_1;
}

void Navigation::step_turn_rotation() {
    const auto current_turn_params = turn_params[current_movement];
    const float angular_max_speed = current_turn_params.max_angular_speed;
    float control_angular_speed_abs = std::abs(control->get_target_angular_speed());
    const int turn_sign = current_turn_params.sign;

    const uint32_t elapsed_time = turn_tick_counter;
    turn_tick_counter++;

    update_turn_angular_acceleration(current_turn_params, elapsed_time);

    control_angular_speed_abs += current_angular_acceleration / Config::CONTROL_FREQUENCY_HZ;
    control_angular_speed_abs = std::min(control_angular_speed_abs, angular_max_speed);
    control_angular_speed_abs = std::max(control_angular_speed_abs, 0.0f);

    control->set_target_angular_speed(control_angular_speed_abs * turn_sign);
    control->set_wall_pid_enabled(false);
    control->set_diagonal_pid_enabled(false);
    control->set_use_inplace_friction(is_turn_around_movement());

    if (turn_tick_counter >= turn_params[current_movement].t_stop) {
        transition_after_turn_rotation();
    }
}

void Navigation::step_turn_stabilize_1() {
    control->set_wall_pid_enabled(false);
    control->set_diagonal_pid_enabled(false);
    control->set_use_inplace_friction(false);

    const uint32_t elapsed_time = bsp::get_tick_ms() - reference_time;
    if (elapsed_time <= STABILIZE_FORWARD_TIME_MS) {
        return;
    }

    reference_time = bsp::get_tick_ms();
    control->reset(general_params);
    control->set_motor_control_disabled(false);
    traveled_dist_mm = 0;
    turn_tick_counter = 0;
    is_braking = false;
    mini_fsm_state = MiniFSMStates::TURN;
    current_angular_acceleration = 0.0f;
}

void Navigation::step_turn_stabilize_2() {
    control->set_wall_pid_enabled(false);
    control->set_diagonal_pid_enabled(false);
    control->set_use_inplace_friction(false);

    const uint32_t elapsed_time = bsp::get_tick_ms() - reference_time;
    if (elapsed_time <= STABILIZE_TURN_TIME_MS) {
        return;
    }

    reference_time = bsp::get_tick_ms();
    control->reset(general_params);
    control->set_motor_control_disabled(false);
    traveled_dist_mm = 0;
    is_braking = false;

    if (current_movement == Movement::TURN_AROUND_INPLACE) {
        is_finished = true;
        mini_fsm_state = MiniFSMStates::FORWARD_1;
    } else {
        target_travel_mm = std::abs(current_position_mm.x);
        mini_fsm_state = MiniFSMStates::FORWARD_2;
    }
}

void Navigation::step_turn_movement() {
    // Mini FSM: FORWARD_1 -> TURN -> FORWARD_2, with stabilization when needed.
    switch (mini_fsm_state) {
    case MiniFSMStates::FORWARD_1:
    case MiniFSMStates::FORWARD_2:
        step_turn_forward();
        break;
    case MiniFSMStates::TURN:
        step_turn_rotation();
        break;
    case MiniFSMStates::STABILIZE_1:
        step_turn_stabilize_1();
        break;
    case MiniFSMStates::STABILIZE_2:
        step_turn_stabilize_2();
        break;
    default:
        is_finished = true;
        mini_fsm_state = MiniFSMStates::FORWARD_1;
        break;
    }
}

bool Navigation::step() {
    if (is_linear_movement(current_movement)) {
        step_linear_movement();
    } else if (is_turn_movement(current_movement)) {
        step_turn_movement();
    }

    if (is_finished) {
        update_cell_position_and_dir();
    }

    control->update();
    return is_finished;
}

Point Navigation::get_robot_cell_position(void) {
    return current_cell;
}

Position Navigation::get_robot_position_mm(void) {
    return current_position_mm;
}

Direction Navigation::get_robot_direction(void) {
    return current_direction;
}

float Navigation::get_robot_travelled_dist_mm() {
    return traveled_dist_mm;
}

void Navigation::set_movement(Direction dir) {

    previous_movement = current_movement;

    target_direction = dir;
    current_movement = get_movement(dir, current_direction, true);
    reset_movement_variables();

    target_travel_mm = forward_params[current_movement].target_travel_mm;

    if (current_movement == Movement::STOP) {
        forward_end_speed = 0;
    } else {
        forward_end_speed = forward_params[Movement::FORWARD].max_speed;
    }
}

Movement Navigation::get_movement(Direction target_dir, Direction current_dir, bool search_mode) {
    using enum Direction;

    if (target_dir == Direction::STOP) {
        return Movement::STOP;
    }

    if (target_dir == current_dir) {
        return Movement::FORWARD;
    }

    if ((target_dir == NORTH && current_dir == WEST) || (target_dir == EAST && current_dir == NORTH) ||
        (target_dir == SOUTH && current_dir == EAST) || (target_dir == WEST && current_dir == SOUTH)) {
        return search_mode ? Movement::TURN_RIGHT_90_SEARCH_MODE : Movement::TURN_RIGHT_90;
    }

    if ((target_dir == NORTH && current_dir == SOUTH) || (target_dir == EAST && current_dir == WEST) ||
        (target_dir == SOUTH && current_dir == NORTH) || (target_dir == WEST && current_dir == EAST)) {
        return Movement::TURN_AROUND;
    }

    return search_mode ? Movement::TURN_LEFT_90_SEARCH_MODE : Movement::TURN_LEFT_90;
}

void Navigation::update_cell_position_and_dir() {
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

float Navigation::calculate_turn_end_offset(Movement movement) {
    using bsp::analog_sensors::ir_distance_mm;
    using bsp::analog_sensors::ir_reading_wall;
    using bsp::analog_sensors::SensingDirection;

    float lateral_error_mm = 0.0f;
    constexpr float MAX_ALLOWED_OFFSET_MM = 10.0f;
    constexpr float DEG_TO_RAD = 3.14159265358979323846f / 180.0f;

    if (movement == Movement::TURN_LEFT_90) {
        // Turning Left: check opposite (Right) wall
        if (ir_reading_wall(SensingDirection::RIGHT)) {
            const float cos_theta = std::cos(Config::right_sensor_angle_deg * DEG_TO_RAD);
            float measured_dist = ir_distance_mm(SensingDirection::RIGHT);
            float delta_l = measured_dist - Config::ir_wall_dist_ref_right; // positive when further from right wall
            lateral_error_mm = delta_l * cos_theta;
        }
    } else if (movement == Movement::TURN_RIGHT_90) {
        // Turning Right: check opposite (Left) wall
        if (ir_reading_wall(SensingDirection::LEFT)) {
            const float cos_theta = std::cos(Config::left_sensor_angle_deg * DEG_TO_RAD);
            float measured_dist = ir_distance_mm(SensingDirection::LEFT);
            float delta_l = measured_dist - Config::ir_wall_dist_ref_left; // positive when further from left wall
            lateral_error_mm = delta_l * cos_theta;
        }
    }

    // Clamp adjustment to guard against sensor anomalies
    lateral_error_mm = std::clamp(lateral_error_mm, -MAX_ALLOWED_OFFSET_MM, MAX_ALLOWED_OFFSET_MM);

    // If robot was further away from the opposite wall (+delta), the turn's ends +delta mm after
    return lateral_error_mm;
}

void Navigation::set_movement(Movement movement, Movement prev_movement, Movement next_movement, uint8_t count,
                              uint8_t next_move_count) {

    complete_prev_move_travel = -1 * turn_params[prev_movement].end;

    if (Config::enable_lateral_correction_90) {
        if (prev_movement == Movement::TURN_LEFT_90 || prev_movement == Movement::TURN_RIGHT_90) {
            complete_prev_move_travel -= turn_end_correction_mm;
            turn_end_correction_mm = 0.0f;
        }

        if (movement == Movement::TURN_LEFT_90 || movement == Movement::TURN_RIGHT_90) {
            turn_end_correction_mm = calculate_turn_end_offset(movement);
        }
    }

    previous_movement = prev_movement;
    current_movement = movement;

    bool continuous_start_to_forward = (prev_movement == Movement::START && movement == Movement::FORWARD && count > 3);
    reset_movement_variables(!continuous_start_to_forward);

    if (movement == Movement::FORWARD || movement == Movement::DIAGONAL) {
        if (waiting_for_fast_param) {
            waiting_for_fast_param = false;
            if (selected_mode == FAST || selected_mode == SUPER) {
                configure_mode(selected_mode);
            }
        }
        target_travel_mm = complete_prev_move_travel + (forward_params[movement].target_travel_mm * count) +
                           turn_params[next_movement].start;
    } else if (movement == Movement::START) {
        if (next_movement == Movement::TURN_LEFT_135 || next_movement == Movement::TURN_RIGHT_135 ||
            next_movement == Movement::TURN_LEFT_45 || next_movement == Movement::TURN_RIGHT_45) {
            if (selected_mode == FAST || selected_mode == SUPER) {
                // This can happen if the robot is turning right afer the start movement.
                // TODO: generalize this function, because now we are forcing medium parameters
                waiting_for_fast_param = true;
                configure_mode(MEDIUM);
            }
        }
        target_travel_mm = forward_params[movement].target_travel_mm + turn_params[next_movement].start;
    } else {
        target_travel_mm = complete_prev_move_travel + forward_params[movement].target_travel_mm;
    }

    // When target travel is 0, we directly go to turn state
    if (target_travel_mm <= 0) {
        mini_fsm_state = MiniFSMStates::TURN;
        current_angular_acceleration = 0.0f;
        turn_tick_counter = 0;
    }

    if (movement == Movement::STOP) {
        forward_end_speed = 0;
        bsp::leds::stripe_set(Color::Blue);
    } else if (movement == Movement::START) {
        if (next_movement == Movement::FORWARD && next_move_count > 3) { // continuous_start_to_forward condition
            forward_end_speed = forward_params[Movement::FORWARD].max_speed;
        } else {
            forward_end_speed = forward_params[Movement::START].max_speed;
        }
    } else if (next_movement == Movement::FORWARD || next_movement == Movement::DIAGONAL) {
        forward_end_speed = forward_params[next_movement].max_speed;
    } else if (next_movement == Movement::STOP) {
        forward_end_speed = forward_params[next_movement].max_speed;
    } else {
        forward_end_speed = turn_params[next_movement].turn_linear_speed;
    }
}

std::vector<std::pair<Movement, uint8_t>> Navigation::get_movements_to_goal(std::vector<Direction> target_directions,
                                                                            target_movement_mode_t mode) {

    std::vector<std::pair<Movement, uint8_t>> movements;

    switch (mode) {
    case target_movement_mode_t::NORMAL:
        movements = get_default_target_movements(target_directions);
        break;
    case target_movement_mode_t::SMOOTH:
        movements = get_smooth_movements(get_default_target_movements(target_directions));
        break;
    case target_movement_mode_t::DIAGONALS:
        movements = get_diagonal_movements(get_default_target_movements(target_directions));
        break;
    case target_movement_mode_t::HARD_CODED:
        movements = get_hardcoded_movements();
        print_movement_sequence(movements, "Hard Coded");
        break;
    }

    return movements;
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
    }

    output_movements.push_back({Movement::STOP, 1});
    print_movement_sequence(output_movements, "Diagonal");

    return output_movements;
}

std::vector<std::pair<Movement, uint8_t>> Navigation::get_hardcoded_movements() {
    return hardcoded_movements;
}

void Navigation::set_hardcoded_movements(std::vector<std::pair<Movement, uint8_t>> moves) {
    hardcoded_movements = moves;
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