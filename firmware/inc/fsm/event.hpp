#pragma once

#include <variant>
#include "bsp/ble.hpp"

namespace fsm {

struct BleCommand {
    uint8_t packet[bsp::ble::max_packet_size] = {0};
};
struct UsbCommand {};

struct ButtonPressed {
    enum Type { SHORT1, SHORT2, LONG1, LONG2, LONG3, LONG4, LONG5, LONG6 } button;
};

struct Timeout {};

using Event = std::variant<BleCommand, UsbCommand, ButtonPressed, Timeout>;

}
