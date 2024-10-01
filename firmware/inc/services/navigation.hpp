#pragma once

#include <cstdint>

namespace services {

class Navigation {
public:
    static constexpr float CELL_SIZE_CM = 18.0;
    static constexpr float HALF_CELL_SIZE_CM = 9.0;

    static Navigation* instance();

    Navigation(const Navigation&) = delete;

    void init();
    void reset();
    void update();

    float get_traveled_cm();
    void set_traveled_cm(float);

    Direction get_robot_direction();
    void set_robot_direction(Direction);

    Point get_robot_position();
    void move(Movement);
    Movement get_target_movement(Direction robot_dir, Direction target_dir);

private:
    Navigation() {}

    float traveled_dist = 0;
    int32_t encoder_right_counter;
    int32_t encoder_left_counter;
    Point current_position;
    Direction current_direction;
};


}
