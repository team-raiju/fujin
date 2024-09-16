#pragma once

#include <variant>

namespace fsm {

struct BleCommand {};
struct UsbCommand {};

struct ButtonPressed {
    enum { SHORT1, SHORT2, LONG1, LONG2 } button;
};

struct Timeout {};

using Event = std::variant<BleCommand, UsbCommand, ButtonPressed, Timeout>;

}
