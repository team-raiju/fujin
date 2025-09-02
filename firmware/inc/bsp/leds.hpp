#pragma once

#include <cstdint>

namespace bsp::leds {

/// @section Constants

#define LED_STRIPE_AMOUNT 2 ///< Number of LEDs in the stripe

enum Emitter {
    LEFT_SIDE,
    LEFT_FRONT,
    RIGHT_FRONT,
    RIGHT_SIDE,
};

/// @section Custom types

struct Color {
    static Color White;
    static Color Black; // Off
    static Color Red;
    static Color Green;
    static Color Blue;
    static Color Purple;
    static Color Yellow;
    static Color Orange;
    static Color Pink;
    static Color Cyan;

    uint32_t encode() const;

    Color(const Color&) = delete;

private:
    const uint8_t r, g, b;
    constexpr Color(uint8_t r, uint16_t g, uint8_t b) : r(r), g(g), b(b) {}
};

/// @section Interface definition

/// @brief Initializes LEDs peripherals
/// @note Doesn't need to be called directly as bsp_init() already calls this
void init(void);

void indication_on(void);
void indication_off(void);
void indication_toggle(void);

void ir_emitter_on(Emitter led);
void ir_emitter_off(Emitter led);
void ir_emitter_all_on(void);
void ir_emitter_all_off(void);

/// @brief Sets the color of a LED in the stripe
/// @param color_1 Color of led 1
/// @param color_2 Color of led 2
void stripe_set(Color const& color_1, Color const& color_2);

/// @brief Sets the color of all LEDs in the stripe and send the command
/// @param color the color
void stripe_set(Color const& color);

void stripe_send();

} // namespace
