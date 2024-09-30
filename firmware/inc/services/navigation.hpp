#pragma once

#include <cstdint>

namespace services {

class Navigation {
public:
    static Navigation* instance();

    Navigation(const Navigation&) = delete;

    void init();
    void reset();
    void update();

    float get_traveled_cm();

    Direction get_robot_direction();
    void set_robot_direction(Direction);

    Point get_robot_position();
    void move(Movement);

private:
    Navigation() {}

    float traveled_dist = 0;
    int32_t encoder_right_counter;
    int32_t encoder_left_counter;
    Point current_position;
    Direction current_direction;
};

}
