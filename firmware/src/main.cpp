#include "bsp/core.h"
#include "bsp/eeprom.h"
#include "bsp/fan.h"
#include "bsp/leds.h"
#include "bsp/motors.h"
#include "bsp/timers.h"

int main() {
    bsp_init();

    while (1) {
        bsp_leds_indication_toggle();
        bsp_delay_us(500000);
    }
}
