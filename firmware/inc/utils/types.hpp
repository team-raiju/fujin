/// @brief Simples types, this is included globally

#pragma once

#include <cstdint>
#include <utility>
#include <cmath>

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
    START,
    FORWARD,
    DIAGONAL,
    TURN_RIGHT_45,
    TURN_LEFT_45,
    TURN_RIGHT_90,
    TURN_LEFT_90,
    TURN_RIGHT_135,
    TURN_LEFT_135,
    TURN_RIGHT_180,
    TURN_LEFT_180,
    TURN_RIGHT_45_FROM_45,
    TURN_LEFT_45_FROM_45,
    TURN_RIGHT_90_FROM_45,
    TURN_LEFT_90_FROM_45,
    TURN_RIGHT_135_FROM_45,
    TURN_LEFT_135_FROM_45,
    TURN_AROUND,
    TURN_RIGHT_90_SEARCH_MODE,
    TURN_LEFT_90_SEARCH_MODE,
    TURN_AROUND_INPLACE,
    STOP,
};

// Initialize the array with address-name pairs
const std::pair<Movement, const char*> movementInfoMap[] = {
    {START, "START"},
    {FORWARD, "FORWARD"},
    {DIAGONAL, "DIAGONAL"},
    {TURN_RIGHT_45, "TURN_RIGHT_45"},
    {TURN_LEFT_45, "TURN_LEFT_45"},
    {TURN_RIGHT_90, "TURN_RIGHT_90"},
    {TURN_LEFT_90, "TURN_LEFT_90"},
    {TURN_RIGHT_135, "TURN_RIGHT_135"},
    {TURN_LEFT_135, "TURN_LEFT_135"},
    {TURN_RIGHT_180, "TURN_RIGHT_180"},
    {TURN_LEFT_180, "TURN_LEFT_180"},
    {TURN_RIGHT_45_FROM_45, "TURN_RIGHT_45_FROM_45"},
    {TURN_LEFT_45_FROM_45, "TURN_LEFT_45_FROM_45"},
    {TURN_RIGHT_90_FROM_45, "TURN_RIGHT_90_FROM_45"},
    {TURN_LEFT_90_FROM_45, "TURN_LEFT_90_FROM_45"},
    {TURN_RIGHT_135_FROM_45, "TURN_RIGHT_135_FROM_45"},
    {TURN_LEFT_135_FROM_45, "TURN_LEFT_135_FROM_45"},
    {TURN_AROUND, "TURN_AROUND"},
    {STOP, "STOP"},
};

struct Point {
    int x;
    int y;
};

struct Position {
    float x;
    float y;
};

inline bool operator==(Point const& p1, Point const& p2) {
    return p1.x == p2.x && p1.y == p2.y;
}

inline Point operator+(Point const& p1, Point const& p2) {
    return {p1.x + p2.x, p1.y + p2.y};
}


inline bool operator==(Position const& p1, Position const& p2) {
    return p1.x == p2.x && p1.y == p2.y;
}

inline Position operator+(Position const& p1, Position const& p2) {
    return {p1.x + p2.x, p1.y + p2.y};
}


inline Position operator*(Position const& p, float scalar) {
    return {p.x * scalar, p.y * scalar};
}

inline Position rotate(Position const& p, float radians) {
    return {
        p.x * std::cos(radians) - p.y * std::sin(radians),
        p.x * std::sin(radians) + p.y * std::cos(radians),
    };
}

