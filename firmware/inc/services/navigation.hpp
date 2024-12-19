#pragma once

#include <cstdint>

#include "algorithms/pid.hpp"
#include "services/control.hpp"


namespace services {

class Navigation {
public:
    static Navigation* instance();

    Navigation(const Navigation&) = delete;

    void init();
    void reset();
    void update();
    bool step();

    void optimize(bool = true);

    Point get_robot_position();
    Direction get_robot_direction();
    void move(Direction dir, uint8_t cells);
    void stop();
    void stop_run_mode();

private:
    Navigation() {}
    void update_position();
    Movement get_movement(Direction);
    float get_torricelli_distance(float final_speed, float initial_speed, float acceleration);

    services::Control* control;

    float target_travel;

    bool is_initialized = false;
    bool is_finished = false;

    bool optimize_turns = false;

    // The service uses a mini-fsm to update it's movement on each iteration
    uint8_t state;
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
