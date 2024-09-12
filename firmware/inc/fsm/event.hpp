#pragma once

namespace fsm {

struct Event {};

struct BleCommand : Event {};

struct ButtonPressed : Event {
    enum { BUTTON1, BUTTON2 } button;
};

struct Timeout : Event {};

}
