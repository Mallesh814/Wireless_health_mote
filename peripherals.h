/*
 * peripherals.h
 *
 *  Created on: May 10, 2017
 *      Author: edsys
 */

#ifndef PERIPHERALS_H_
#define PERIPHERALS_H_

#include <stdint.h>
#include <stdbool.h>

#include "driverlib/gpio.h"
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/ssi.h"
#include "driverlib/uart.h"
#include "driverlib/pin_map.h"
#include "driverlib/i2c.h"

#include "ble113.h"


typedef struct
{
    uint32_t ssiBase;
    uint32_t csPort;
    uint32_t csPin;
} ssi_deviceHandle;


typedef struct
{
    uint8_t* instBuffer;
    uint32_t instLen;
    uint8_t* dataBuffer;
    uint32_t dataLen;
} ssi_packetHandle;


typedef struct
{
    uint32_t i2cBase;
    uint32_t i2cAddr;
} i2c_deviceHandle;


typedef struct
{
    uint32_t uartBase;
    uint32_t portBase;
    uint32_t rstPin;
    uint32_t wakeupPin;
} ble_deviceHandle;


typedef struct
{
    uint32_t selBase;
    uint32_t selPins;
    uint32_t inBase;
    uint32_t inPin;
} deMux;

ssi_deviceHandle sram23LcvHandle;
ssi_deviceHandle flashM25pHandle;
ssi_deviceHandle ads1294Handle;

i2c_deviceHandle driver_dac7573Handle;
i2c_deviceHandle sensor_dac7573Handle;
i2c_deviceHandle tempSensorHandle;

ble_deviceHandle ble113Handle;

deMux deMuxLed;

typedef enum {
    selRed  = 0x00,
    selIr   = 0x03,
    sel810  = 0x02,
    sel1300 = 0x01
} ledSelect;

ledSelect sensChannel_A;
ledSelect sensChannel_B;

uint32_t debugConsole, bleConsole, decimal;


void configurePeripherals();

uint32_t InitSPI(uint32_t , uint32_t , uint32_t , uint32_t , uint32_t , bool );

uint32_t InitI2C(uint32_t , bool);

uint32_t InitConsole(uint32_t , uint32_t );


#endif /* PERIPHERALS_H_ */
