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
    float get_robot_travelled_dist_cm();

    /// @brief Configure and reset movement variables based on a target direction. Update method will execute the
    /// movement Used on search mode, to set the next movement
    /// @param dir The direction to set
    void set_movement(Direction dir);

    /// @brief Configure and reset movement variables. Update method will execute the movement
    /// @param movement The movement to set
    /// @param prev_movement The previous movement
    /// @param next_movement The next movement
    /// @param count The number of steps to take on the current movement
    void set_movement(Movement movement, Movement prev_movement, Movement next_movement, uint8_t count);

    std::vector<std::pair<Movement, uint8_t>> get_movements_to_goal(std::vector<Direction> target_directions,
                                                                    target_movement_mode_t mode);

    std::vector<std::pair<Movement, uint8_t>> get_hardcoded_movements();
    void set_hardcoded_movements(std::vector<std::pair<Movement, uint8_t>> moves);

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
    void update_cell_position_and_dir();

    /// @brief Get the movement type to go to a target direction, based on the current direction and search mode
    /// @param target_dir The target direction
    /// @param current_dir The current direction
    /// @param search_mode Whether the search mode is active
    /// @return The movement type
    Movement get_movement(Direction target_dir, Direction current_dir, bool search_mode);

    float get_torricelli_distance(float final_speed, float initial_speed, float acceleration);
    WallBreak process_wall_break();
    void reset_wall_break();
    void reset_movement_variables();

    std::vector<std::pair<Movement, uint8_t>> get_default_target_movements(std::vector<Direction> target_directions);

    std::vector<std::pair<Movement, uint8_t>>
    get_smooth_movements(std::vector<std::pair<Movement, uint8_t>> default_target_movements);

    std::vector<std::pair<Movement, uint8_t>>
    get_diagonal_movements(std::vector<std::pair<Movement, uint8_t>> default_target_movements);

    void print_movement_sequence(std::vector<std::pair<Movement, uint8_t>> movements, std::string name);

    services::Control* control;

    float target_travel_cm;
    float forward_end_speed;

    bool is_initialized = false;
    bool is_finished = false;

    uint32_t reference_time;
    float traveled_dist_cm = 0;
    int32_t encoder_right_counter;
    int32_t encoder_left_counter;
    Point current_cell;
    Position current_position_mm = {0, 0};
    float current_angle_rad = 0;
    Direction current_direction;
    Movement current_movement;
    Direction target_direction;
    float complete_prev_move_travel;

    uint32_t wall_right_counter_on = 0;
    uint32_t wall_left_counter_on = 0;
    uint32_t wall_right_counter_off = 0;
    uint32_t wall_left_counter_off = 0;
    uint32_t wall_break_last_dist = 0;

    MiniFSMStates mini_fsm_state = MiniFSMStates::FORWARD_1;

    std::vector<std::pair<Movement, uint8_t>> hardcoded_movements;

    bool waiting_for_fast_param = false;
    navigation_mode_t selected_mode;
};

}
