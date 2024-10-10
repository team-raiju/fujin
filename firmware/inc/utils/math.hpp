#pragma once

#include <cmath>
#include <stdint.h>

#ifndef M_TWOPI
#define M_TWOPI (M_PI * 2.0)
#endif

/**
 * @brief Constrains v between x and y, returning at least x and at most y.
 */
#define constrain(v, x, y) ((v) < (x) ? (x) : ((v) > (y) ? (y) : (v)))

/**
 * @brief Constrains v between x and y, returning at least x and at most y.
 */
#define len(arr) (sizeof(arr) / sizeof(arr[0]))

/**
 * @brief Maps an integer value from one scale to another.
 *
 * @param T Integer type
 * @param former_value Value in former scale.
 * @param former_min   Former scale minimum value.
 * @param former_max   Former scale maximum value.
 * @param new_min      New scale minimum value.
 * @param new_max      New scale maximum value.
 *
 * @return Value in new scale
 */
template <typename T>
static constexpr T map(T former_value, T former_min, T former_max, T new_min, T new_max) {
    long int new_value;

    new_value = (long int)(former_value - former_min) * (new_max - new_min);
    new_value /= (former_max - former_min);
    new_value += new_min;

    return (T)new_value;
}

static constexpr double deg2rad(double const& degrees) {
    static constexpr double pi_on_180 = M_PI / 180.0;
    return degrees * pi_on_180;
}

static constexpr uint8_t manhattan(Point const& p1, Point const& p2) {
    return std::abs(p1.x - p2.x) + std::abs(p1.y - p2.y);
}

static constexpr uint8_t bit_reverse(uint8_t n) {
#if defined(__arm__)
    uint32_t r;
    __asm__("rbit %0, %1" : "=r"(r) : "r"(n));
    return r >> 24;
#else
    n = (n & 0xF0 >> 4) | (n & 0x0F << 4);
    n = (n & 0xCC >> 2) | (n & 0x33 << 2);
    n = (n & 0xAA >> 1) | (n & 0x55 << 1);
    return n;
#endif
}
