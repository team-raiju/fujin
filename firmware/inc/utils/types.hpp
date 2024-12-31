/// @brief Simples types, this is included globally

#pragma once

#include <cstdint>
#include <utility>

enum class Direction : uint8_t {
    NORTH,
    WEST,
    SOUTH,
    EAST,
};

static constexpr Direction Directions[] = {
    Direction::NORTH,
    Direction::WEST,
    Direction::SOUTH,
    Direction::EAST,
};

/// @brief This is a circular 4 bit shift to be used with Directions
static constexpr uint8_t operator<<(uint8_t x, const Direction n) {
    const uint8_t _n = std::to_underlying(n);
    return 0xF & ((x << _n) | (x >> (4 - _n)));
}

enum Movement {
    FORWARD,
    RIGHT,
    TURN_AROUND,
    LEFT,
    TURN_RIGHT_180,
    TURN_LEFT_180,
    STOP,
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
