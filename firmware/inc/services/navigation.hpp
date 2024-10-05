#pragma once

#include <cstdint>

#include "algorithms/pid.hpp"

namespace services {

class Navigation {
public:
    static Navigation* instance();

    Navigation(const Navigation&) = delete;

    void init();
    void reset();
    void update();
    bool step();

    Point get_robot_position();
    Direction get_robot_direction();
    void move(Direction);
    void stop();

private:
    Navigation() {}
    void update_position();
    Movement get_movement(Direction);

    algorithm::PID angular_vel_pid;
    algorithm::PID walls_pid;

    float target_speed;
    float rotation_ratio;
    float target_travel;

    bool is_initialized = false;
    bool is_finished = false;

    // The service uses a mini-fsm to update it's movement on each iteration
    uint8_t state;
    uint32_t reference_time;
    float traveled_dist = 0;
    int32_t encoder_right_counter;
    int32_t encoder_left_counter;
    Point current_position;
    Direction current_direction;
    Movement current_movement;
    Direction target_direction;
};

}
