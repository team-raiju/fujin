#pragma once

#include <cstdint>
#include <vector>
#include <string>

#include "algorithms/pid.hpp"
#include "services/control.hpp"

namespace services {

class Navigation {
public:
    static Navigation* instance();

    Navigation(const Navigation&) = delete;

    void init();
    void reset(bool search_mode);
    void update();
    bool step();

    Point get_robot_position();
    Direction get_robot_direction();
    void move(Direction dir);
    void set_movement(Movement);

    std::vector<std::pair<Movement, uint8_t>> get_default_target_movements(std::vector<Direction> target_directions);

    std::vector<std::pair<Movement, uint8_t>>
    get_smooth_movements(std::vector<std::pair<Movement, uint8_t>> default_target_movements);

    std::vector<std::pair<Movement, uint8_t>>
    get_diagonal_movements(std::vector<std::pair<Movement, uint8_t>> default_target_movements);

    void print_movement_sequence(std::vector<std::pair<Movement, uint8_t>> movements, std::string name);

private:
    Navigation() {}
    void update_position();
    Movement get_movement(Direction target_dir, Direction current_dir);
    float get_torricelli_distance(float final_speed, float initial_speed, float acceleration);

    services::Control* control;

    float target_travel_cm;

    bool is_initialized = false;
    bool is_finished = false;

    uint32_t reference_time;
    float traveled_dist_cm = 0;
    int32_t encoder_right_counter;
    int32_t encoder_left_counter;
    Point current_position;
    Direction current_direction;
    Movement current_movement;
    Direction target_direction;
};

}
