#pragma once

#include <cstdint>

namespace navigation {

    enum Direction {
        NORTH,
        EAST,
        SOUTH,
        WEST,
        DIRECTION_MAX,
    };
    enum Movement {
        FORWARD,
        RIGHT,
        BACKWARD,
        LEFT,
        UNKNOWN_MOVEMENT,
    };

    struct Position {
        int x;
        int y;
    };

}
