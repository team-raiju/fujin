#ifndef BLE_SERVICE_H
#define BLE_SERVICE_H

#include "bsp_ble.h"
#include <stdint.h>

#define BLE_PACKET_TRANSMIT_SIZE 20

typedef enum {
    BLE_STOP_SIG,
    BLE_REQUEST_DATA,
    BLE_UPDATE_PARAMETERS,
    BLE_BUTTON_1_SHORT,
    BLE_BUTTON_2_SHORT,
    BLE_BUTTON_1_LONG,
    BLE_BUTTON_2_LONG,
} ble_header_t;

typedef union {
    uint8_t packet[BLE_RECEIVE_PACKET_SIZE];

    struct {
        uint8_t header;
        uint8_t data[BLE_RECEIVE_PACKET_SIZE - 2];
        uint8_t chk;
    };

} ble_rcv_packet_t;

void ble_service_init(void);

void ble_service_send_data(uint8_t *data, uint8_t size);
void ble_service_send_string(char *str);
void ble_service_last_packet(ble_rcv_packet_t *data);
ble_header_t ble_service_last_packet_type(void);

#endif /* BLE_SERVICE_H */
