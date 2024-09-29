#pragma once

#include "navigation.hpp"
#include "utils/math.hpp"

namespace services {

class Position {
public:
    Position(const Position&) = delete;

    static Position* instance();

    void init();
    void reset();
    void update();

    float get_traveled_cm();

    Direction get_robot_direction();
    void set_robot_direction(Direction);

    Point get_robot_position();
    void increase_robot_position(Direction);

private:
    Position() {}

    float traveled_dist = 0;
    int32_t encoder_right_counter;
    int32_t encoder_left_counter;
    Point current_position;
    Direction current_direction;
};

}
