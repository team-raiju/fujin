#include "bsp.h"
#include "bsp_led.h"

int main() {

    BSP_init(); 
    

    while(1) {
        BSP_ledToggle();
        BSP_delay(500);
    }

}