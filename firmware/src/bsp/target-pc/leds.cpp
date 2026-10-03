#include "bsp/leds.hpp"
#include "amaterasu/amaterasu.hpp"

namespace bsp::leds {

static bool indication_state = false;

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

void init() {
    indication_off();
}

void indication_on() {
    indication_state = true;
    amaterasu::set_led_indication(true);
}

void indication_off() {
    indication_state = false;
    amaterasu::set_led_indication(false);
}

void indication_toggle() {
    indication_state = !indication_state;
    amaterasu::set_led_indication(indication_state);
}

void ir_emitter_on(Emitter) {}
void ir_emitter_off(Emitter) {}
void ir_emitter_all_on() {}
void ir_emitter_all_off() {}

void stripe_set(Color const& color_1, Color const& color_2) {
    (void)color_2;
    stripe_set(color_1);
}

void stripe_set(Color const& color) {
    uint32_t enc = color.encode();
    uint8_t r = (enc >> 8) & 0xFF;
    uint8_t g = enc & 0xFF;
    uint8_t b = (enc >> 16) & 0xFF;
    amaterasu::set_led_rgb(r > 0, g > 0, b > 0);
}

void stripe_send() {}

} // namespace bsp::leds
