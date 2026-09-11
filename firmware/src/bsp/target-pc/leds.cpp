#include "bsp/leds.hpp"

namespace bsp::leds {

Color Color::Red = Color(0x7F, 0x00, 0x00);
Color Color::Green = Color(0x00, 0x3F, 0x00);
Color Color::Blue = Color(0x00, 0x00, 0x7F);
Color Color::Purple = Color(0x7F, 0x00, 0x7F);
Color Color::Yellow = Color(0x3F, 0x3F, 0x00);
Color Color::Orange = Color(0x64, 0xC8, 0x00);
Color Color::White = Color(0x3F, 0x3F, 0x3F);
Color Color::Black = Color(0x00, 0x00, 0x00);
Color Color::Pink = Color(0x1F, 0xC8, 0x64);
Color Color::Cyan = Color(0xD2, 0x32, 0x12);

uint32_t Color::encode() const {
    return ((b) << 16) | ((r) << 8) | (g);
}

void init() {}

void indication_on() {}
void indication_off() {}
void indication_toggle() {}

void ir_emitter_on(Emitter) {}
void ir_emitter_off(Emitter) {}
void ir_emitter_all_on() {}
void ir_emitter_all_off() {}

void stripe_set(Color const& color_1, Color const& color_2) {
    (void)color_1;
    (void)color_2;
}

void stripe_set(Color const& color) {
    (void)color;
}

void stripe_send() {}

} // namespace bsp::leds
