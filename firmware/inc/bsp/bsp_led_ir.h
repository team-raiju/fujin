#ifndef BSP_LED_IR_H
#define BSP_LED_IR_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    BSP_LED_IR_L_S,
    BSP_LED_IR_L_F,
    BSP_LED_IR_R_F,
    BSP_LED_IR_R_S,
} bsp_led_ir_t;

void bsp_leds_ir_init(void);

void bsp_led_ir_on(bsp_led_ir_t led);
void bsp_led_ir_off(bsp_led_ir_t led);
void bsp_led_ir_all_off(void);
void bsp_led_ir_all_on(void);

#endif /* BSP_LED_H */
