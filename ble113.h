/*
 * ble113.h
 *
 *  Created on: 20-Mar-2017
 *      Author: mallesh
 */

#ifndef BLE113_H_
#define BLE113_H_

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "peripherals.h"
#include "cmd_def.h"
#include "parser.h"

#define MAX_DEVICES 64
#define UART_TIMEOUT 1000
#define GATT_HANDLE_C_RX_DATA   20  // 0x11, supports "write" operation
#define GATT_HANDLE_C_TX_DATA   22  // 0x14, supports "read" and "indicate" operations

#define FIRST_HANDLE 0x0001
#define LAST_HANDLE  0xffff

#define THERMOMETER_SERVICE_UUID            0x180D
#define THERMOMETER_MEASUREMENT_UUID        0x2a37
#define THERMOMETER_MEASUREMENT_CONFIG_UUID 0x2902
#define DEBUG


enum actions {
    action_none,
    action_scan,
    action_connect,
    action_info,
};

typedef enum {
    state_poweroff,
	state_standby,
	state_advertising,
    state_disconnected,
    state_connecting,
    state_connected,
	state_connected_master,
    state_finding_services,
    state_finding_attributes,
    state_listening_measurements,
    state_finish,
    state_last
} states;

states state;

void output(uint8 , uint8* , uint16 , uint8* );
void print_raw_packet(struct ble_header* , unsigned char* );
void change_state(states );
states get_state();
int cmp_bdaddr(bd_addr , bd_addr );
void print_bdaddr(bd_addr );
void enable_indications(uint8 , uint16 );
int read_message(int );



#endif /* BLE113_H_ */
