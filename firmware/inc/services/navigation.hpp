#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "algorithms/pid.hpp"
#include "services/control.hpp"

namespace services {

class Navigation {
public:
    enum navigation_mode_t {
        SEARCH_SLOW,
        SEARCH_MEDIUM,
        SEARCH_FAST,
        CUSTOM,
        SLOW,
        MEDIUM,
        FAST,
        SUPER
    };

    enum target_movement_mode_t {
        NORMAL,
        SMOOTH,
        DIAGONALS,
        HARD_CODED,
    };

    static Navigation* instance();

    Navigation(const Navigation&) = delete;

    void init();
    void reset(navigation_mode_t mode);
    void update();
    bool step();

    Point get_robot_cell_position();
    Position get_robot_position_mm();
    Direction get_robot_direction();
    float get_robot_travelled_dist_mm();

    /// @brief Configure and reset movement variables based on a target direction. Update method will execute the
    /// movement Used on search mode, to set the next movement
    /// @param dir The direction to set
    void set_movement(Direction dir);

    /// @brief Configure and reset movement variables. Update method will execute the movement
    /// @param movement The movement to set
    /// @param prev_movement The previous movement
    /// @param next_movement The next movement
    /// @param count The number of steps to take on the current movement
    void set_movement(Movement movement, Movement prev_movement, Movement next_movement, uint8_t count, uint8_t next_move_count);

    std::vector<std::pair<Movement, uint8_t>> get_movements_to_goal(std::vector<Direction> target_directions,
                                                                    target_movement_mode_t mode);

    std::vector<std::pair<Movement, uint8_t>> get_hardcoded_movements();
    void set_hardcoded_movements(std::vector<std::pair<Movement, uint8_t>> moves);

    float get_encoder_imu_diff() const { return encoder_imu_diff; };


private:
    enum class PathState {
        Start,
        Ortho_F,  // Moving straight
        Ortho_R,  // Made a single Right 90 turn
        Ortho_L,  // Made a single Left 90 turn
        Ortho_RR, // Made two Right 90 turns (180)
        Ortho_LL, // Made two Left 90 turns (180)
        Diag_LR,  // On a diagonal path, last turn was Right
        Diag_RL,  // On a diagonal path, last turn was Left
        Diag_RR,  // In a diagonal turn sequence (R-R)
        Diag_LL,  // In a diagonal turn sequence (L-L)
        Stop,
    };

    enum class MiniFSMStates {
        FORWARD_1,
        TURN,
        FORWARD_2,
        STABILIZE_1,
        STABILIZE_2,
    };

    enum class WallBreak { LEFT, RIGHT, NONE };

    Navigation() {}

    // Lifecycle / configuration
    void configure_mode(navigation_mode_t mode);
    void update_cell_position_and_dir();

    // Main movement state handlers
    bool is_linear_movement(Movement movement) const;
    bool is_turn_movement(Movement movement) const;
    bool is_turn_around_movement() const;
    bool is_search_turn_movement() const;
    bool is_turn_from_diagonal() const;
    void step_linear_movement();
    void step_turn_movement();

    // Linear movement helpers
    void apply_wall_break_correction();
    float get_acceleration_ramp_distance_m(float current_speed, float acceleration, float brake_jerk) const;
    float get_required_brake_distance(float control_linear_speed, float deceleration,
                                      bool continuous_start_to_forward);
    void update_linear_target_speed(float& control_linear_speed, float max_speed, float max_acceleration,
                                    float deceleration, bool continuous_start_to_forward);
    void configure_linear_pid();
    void finish_linear_movement(float control_linear_speed);

    // Turn movement helpers
    void step_turn_forward();
    void step_turn_rotation();
    void step_turn_stabilize_1();
    void step_turn_stabilize_2();
    void update_turn_linear_speed(float& control_linear_speed, float max_speed, float acceleration,
                                  float deceleration, float final_speed);
    void transition_after_turn_forward();
    void update_turn_angular_acceleration(const TurnParams& turn, uint32_t elapsed_time);
    void transition_after_turn_rotation();
    bool is_front_emergency() const; 


    /// @brief Get the movement type to go to a target direction, based on the current direction and search mode
    /// @param target_dir The target direction
    /// @param current_dir The current direction
    /// @param search_mode Whether the search mode is active
    /// @return The movement type
    Movement get_movement(Direction target_dir, Direction current_dir, bool search_mode);

    float get_torricelli_distance(float final_speed, float initial_speed, float acceleration);
    float get_s_curve_brake_distance(float initial_speed, float final_speed, float deceleration, float jerk);
    bool start_accel_ramp_down(float current_speed, float current_accel, float max_speed, float jerk);
    bool start_brake_ramp_up(float current_speed, float current_accel, float final_speed, float jerk);
    float get_effective_max_acceleration(float current_speed, float base_max_accel);
    WallBreak process_wall_break();
    void reset_wall_break();
    void reset_movement_variables(bool reset_linear_accel = true);

    std::vector<std::pair<Movement, uint8_t>> get_default_target_movements(std::vector<Direction> target_directions);

    std::vector<std::pair<Movement, uint8_t>>
    get_smooth_movements(std::vector<std::pair<Movement, uint8_t>> default_target_movements);

    std::vector<std::pair<Movement, uint8_t>>
    get_diagonal_movements(std::vector<std::pair<Movement, uint8_t>> default_target_movements);

    void print_movement_sequence(std::vector<std::pair<Movement, uint8_t>> movements, std::string name);

    services::Control* control;

    float target_travel_mm;
    float forward_end_speed;

    bool is_initialized = false;
    bool is_finished = false;

    uint32_t reference_time;
    uint32_t turn_tick_counter = 0;
    float traveled_dist_mm = 0;
    int32_t encoder_right_counter;
    int32_t encoder_left_counter;
    Point current_cell;
    Position current_position_mm = {0, 0};
    float current_angle_rad = 0;
    Direction current_direction;
    Movement current_movement;
    Movement previous_movement;
    Direction target_direction;
    float complete_prev_move_travel;

    uint32_t wall_right_counter_on = 0;
    uint32_t wall_left_counter_on = 0;
    uint32_t wall_right_counter_off = 0;
    uint32_t wall_left_counter_off = 0;
    float wall_break_last_dist = 0;
    bool current_wall_break_detected = false;
    bool is_braking = false;
    float encoder_imu_diff = 0;

    float current_angular_acceleration = 0.0f;
    float current_linear_acceleration = 0.0f;

    MiniFSMStates mini_fsm_state = MiniFSMStates::FORWARD_1;

    std::vector<std::pair<Movement, uint8_t>> hardcoded_movements;

    bool waiting_for_fast_param = false;
    navigation_mode_t selected_mode;

};

}
