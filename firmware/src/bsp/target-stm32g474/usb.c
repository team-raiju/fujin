#include "st/hal.h"

#include "bsp/usb.h"

void bsp_usb_init(void) {
    MX_USB_Device_Init();
}
void bsp_usb_reset_on_host(void) {}
