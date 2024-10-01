/// @brief Simples types, this is included globally

#pragma once

enum Direction {
    NORTH,
    EAST,
    SOUTH,
    WEST,
};

static constexpr Direction Directions[] = {NORTH, EAST, SOUTH, WEST};

enum Movement {
    FORWARD,
    RIGHT,
    TURN_AROUND,
    LEFT,
};

struct Point {
    int x;
    int y;
};

inline bool operator==(Point const& p1, Point const& p2) {
    return p1.x == p2.x && p1.y == p2.y;
}

inline Point operator+(Point const& p1, Point const& p2) {
    return {p1.x + p2.x, p1.y + p2.y};
}
