/***************************************************************************************************
 * INCLUDES
 **************************************************************************************************/

#include "led_service.h"
#include "bsp_led.h"
#include "bsp_ws2812.h"
#include "utils.h"

/***************************************************************************************************
 * LOCAL DEFINES
 **************************************************************************************************/

/***************************************************************************************************
 * LOCAL TYPEDEFS
 **************************************************************************************************/

/***************************************************************************************************
 * LOCAL FUNCTION PROTOTYPES
 **************************************************************************************************/
static void led_stripe_prepare(uint8_t idx, color_rgb_t color);
static void convert_color_name_to_rgb(color_name_t name, color_rgb_t *rgb);
static void led_stripe_prepare_range(uint8_t from, uint8_t to, color_rgb_t color);
/***************************************************************************************************
 * LOCAL VARIABLES
 **************************************************************************************************/
static color_rgb_t red = { 0x7F, 0x00, 0x00 };
static color_rgb_t green = { 0x00, 0x3F, 0x00 };
static color_rgb_t blue = { 0x00, 0x00, 0x7F };
static color_rgb_t purple = { 0x7F, 0x00, 0x7F };
static color_rgb_t yellow = { 0x3F, 0x3F, 0x00 };
static color_rgb_t orange = { 0x64, 0xC8, 0x00 };
static color_rgb_t white = { 0x3F, 0x3F, 0x3F };
static color_rgb_t black = { 0x00, 0x00, 0x00 };
static color_rgb_t pink = { 0x1F, 0xC8, 0x64 };
static color_rgb_t light_blue = { 0xD2, 0x32, 0x12 };

/***************************************************************************************************
 * GLOBAL VARIABLES
 **************************************************************************************************/

/***************************************************************************************************
 * LOCAL FUNCTIONS
 **************************************************************************************************/

static void convert_color_name_to_rgb(color_name_t name, color_rgb_t *rgb)
{
    switch (name) {
    case COLOR_RED:
        *rgb = red;
        break;
    case COLOR_GREEN:
        *rgb = green;
        break;
    case COLOR_BLUE:
        *rgb = blue;
        break;
    case COLOR_PURPLE:
        *rgb = purple;
        break;
    case COLOR_YELLOW:
        *rgb = yellow;
        break;
    case COLOR_ORANGE:
        *rgb = orange;
        break;
    case COLOR_PINK:
        *rgb = pink;
        break;
    case COLOR_WHITE:
        *rgb = white;
        break;
    case COLOR_BLACK:
        *rgb = black;
        break;
    case COLOR_LIGHT_BLUE:
        *rgb = light_blue;
        break;
    default:
        *rgb = black;
        break;
    }
}

static void led_stripe_prepare(uint8_t idx, color_rgb_t color)
{
    BSP_ws2812_set(idx, color.R, color.G, color.B);
}

static void led_stripe_prepare_range(uint8_t from, uint8_t to, color_rgb_t color)
{
    to = min(to, WS2812_MAX_LED_AMOUNT);

    for (uint8_t i = from; i < to; i++) {
        led_stripe_prepare(i, color);
    }
}
/***************************************************************************************************
 * GLOBAL FUNCTIONS
 **************************************************************************************************/
void board_led_toggle()
{
    BSP_ledToggle();
}

void board_led_on()
{
    BSP_ledOn();
}

void board_led_off()
{
    BSP_ledOff();
}

/* Led Stripe */
void led_stripe_init()
{
    BSP_ws2812_init();
    led_stripe_reset();
}

void led_stripe_send()
{
    BSP_ws2812_send();
}

void led_stripe_set(uint8_t idx, color_rgb_t color)
{
    led_stripe_prepare(idx, color);
    led_stripe_send();
}

void led_stripe_set_range(uint8_t from, uint8_t to, color_rgb_t color)
{
    to = min(to, WS2812_MAX_LED_AMOUNT);

    for (uint8_t i = from; i < to; i++) {
        led_stripe_prepare(i, color);
    }

    led_stripe_send();
}

void led_stripe_set_all(color_rgb_t color)
{
    led_stripe_set_range(0, WS2812_MAX_LED_AMOUNT, color);
}

void led_stripe_reset()
{
    led_stripe_set_all(black);
}

void led_stripe_set_color(uint8_t idx, color_name_t color_name)
{
    color_rgb_t color_rgb;
    convert_color_name_to_rgb(color_name, &color_rgb);
    led_stripe_set(idx, color_rgb);
}

void led_stripe_set_range_color(uint8_t from, uint8_t to, color_name_t color_name)
{
    color_rgb_t color_rgb;
    convert_color_name_to_rgb(color_name, &color_rgb);
    led_stripe_set_range(from, to, color_rgb);
}

void led_stripe_set_all_color(color_name_t color_name)
{
    color_rgb_t color_rgb;
    convert_color_name_to_rgb(color_name, &color_rgb);
    led_stripe_set_all(color_rgb);
}

void led_stripe_prepare_range_color(uint8_t from, uint8_t to, color_name_t color_name)
{
    color_rgb_t color_rgb;
    convert_color_name_to_rgb(color_name, &color_rgb);
    led_stripe_prepare_range(from, to, color_rgb);
}

void led_stripe_send_color(color_name_t led0, color_name_t led1, color_name_t led2, color_name_t led3)
{
    color_rgb_t color_rgb;

    convert_color_name_to_rgb(led0, &color_rgb);
    led_stripe_prepare(0, color_rgb);

    convert_color_name_to_rgb(led1, &color_rgb);
    led_stripe_prepare(1, color_rgb);

    led_stripe_send();
}