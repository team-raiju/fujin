#include "buzzer_service.h"
#include "bsp_buzzer.h"
#include "utils.h"
#include <stdbool.h>

bool buzzer_on;

void buzzer_service_init()
{
    BSP_buzzerInit();
    buzzer_set_intensity(20);
    buzzer_set_frequency(3500);
    buzzer_on = false;
}

void buzzer_start()
{
    BSP_buzzerStart();
    buzzer_on = true;
}

void buzzer_stop()
{
    BSP_buzzerStop();
    buzzer_on = false;
}

void buzzer_toggle()
{
    if (buzzer_on) {
        buzzer_stop();
    } else {
        buzzer_start();
    }
}

void buzzer_set_intensity(uint8_t volume)
{
    volume = min(volume, 100);

    // uint8_t duty_cycle = map(volume, 0, 100, 0, 50);
    uint8_t duty_cycle = volume;

    BSP_buzzerSetPwmDutyCycle(duty_cycle);
}

void buzzer_set_frequency(uint16_t frequency_hz)
{
    BSP_buzzerSetFrequency(frequency_hz);
}
