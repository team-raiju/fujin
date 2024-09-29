#pragma once

#include <cstdint>

namespace services {

enum Direction {
    NORTH,
    EAST,
    SOUTH,
    WEST,
};

enum Movement {
    FORWARD,
    RIGHT,
    BACKWARD,
    LEFT,
};

struct Point {
    int8_t x;
    int8_t y;
};

}
