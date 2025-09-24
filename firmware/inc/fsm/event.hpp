#pragma once

#include <variant>

namespace fsm {

struct BleCommand {};
struct UsbCommand {};

struct ButtonPressed {
    enum Type { SHORT1, SHORT2, LONG1, LONG2, LONG3 } button;
};

struct Timeout {};

using Event = std::variant<BleCommand, UsbCommand, ButtonPressed, Timeout>;

}
