#include "bsp/buttons.h"
#include "bsp/buzzer.h"
#include "bsp/core.h"
#include "bsp/eeprom.h"
#include "bsp/fan.h"
#include "bsp/leds.h"
#include "bsp/motors.h"
#include "bsp/timers.h"

static bool go = false;
static bool fan = false;

int main() {
    bsp_init();

    bsp_buttons_register_callback_button1([] { go = true; });

    bsp_buttons_register_callback_button2([] { fan = !fan; });

    bsp_buzzer_start();
    bsp_buzzer_set_volume(50);
    bsp_buzzer_set_frequency(3333);

    bsp_delay_ms(1000);
    bsp_buzzer_stop();

    while (1) {
        bsp_leds_indication_toggle();
        bsp_delay_ms(1000);

        if (fan) {
            for (int i = 0; i < 70; i++) {
                bsp_fan_set(i);
                bsp_delay_ms(30);
            }

            fan = false;
        }

        if (go) {
            bsp_buzzer_start();
            bsp_delay_ms(500);
            bsp_buzzer_stop();
            bsp_delay_ms(1500);
            bsp_motors_set(300, 300);
            bsp_delay_ms(500);
            bsp_motors_set(0, 0);
            go = false;
        }
    }
}
