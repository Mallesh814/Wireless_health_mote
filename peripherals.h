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

#include "parser.h"

/* Compability */
#ifndef PACKSTRUCT
/*Default packed configuration*/
#ifdef __GNUC__
#ifdef _WIN32
#define PACKSTRUCT( decl ) decl __attribute__((__packed__,gcc_struct))
#else
#define PACKSTRUCT( decl ) decl __attribute__((__packed__))
#endif
#define ALIGNED __attribute__((aligned(0x4)))
#elif __IAR_SYSTEMS_ICC__

#define PACKSTRUCT( decl ) __packed decl

#define ALIGNED
#elif _MSC_VER  //msvc

#define PACKSTRUCT( decl ) __pragma( pack(push, 1) ) decl __pragma( pack(pop) )
#define ALIGNED
#else
//#define PACKSTRUCT(a) a PACKED
#define PACKSTRUCT(a) a __attribute__((packed))
#endif
#endif



#define RAW_RED_BASE    0x00000000  //Sector 0 in Flash
#define RAW_IR_BASE     0x00080000  //Sector 0 in Flash
#define RAW_810_BASE    0x00100000  //Sector 0 in Flash
#define RAW_1300_BASE   0x00180000  //Sector 0 in Flash

#define FILTER_DATA_BASE    0x00200000  //Sector 32 in Flash

#define FIR_RED_SIGNAL_BASE     0x00200000  //Sector 0 in Flash
#define FIR_RED_AMBIENT_BASE    0x00240000  //Sector 0 in Flash
#define FIR_IR_SIGNAL_BASE      0x00280000  //Sector 0 in Flash
#define FIR_IR_AMBIENT_BASE     0x002C0000  //Sector 0 in Flash
#define FIR_810_SIGNAL_BASE     0x00300000  //Sector 0 in Flash
#define FIR_810_AMBIENT_BASE    0x00340000  //Sector 0 in Flash
#define FIR_1300_SIGNAL_BASE    0x00380000  //Sector 0 in Flash
#define FIR_1300_AMBIENT_BASE   0x003C0000  //Sector 0 in Flash

#define MAX_ADC_FAIL_COUNT 4
// 1Mbit Ram => 131072 Bytes => 43690 Units of ADC Data(1 Unit = 3 Bytes) => 8738 Samples (Including Status and 4 Channels) => 4.3 Seconds
// (1024*1024)/(8*3*5) = 8738

// 1Mbit Ram => 131072 Bytes => 43690 Units of ADC Data(1 Unit = 3 Bytes) => 14563 Samples (Including Status and 2 Channels) => 7.2 Seconds
// (1024*1024)/(8*3*3) = 14563

// 1Mbit Ram => 131072 Bytes => 43690 Units of ADC Data(1 Unit = 3 Bytes) => 21845 Samples (Including 2 Channels) => 10.5 Seconds
// (1024*1024)/(8*3*2) = 21845

// 1Mbit Ram => 131072 Bytes => 43690 Units of ADC Data(1 Unit = 3 Bytes) => 43690 Samples (1 Channel Data Only) => 21 Seconds
// (1024*1024)/(8*3*1) = 21845

#define MAX_NO_OF_SAMPLES 8192

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


PACKSTRUCT(struct ads1294DataStruct
{
    uint8_t status[3];
    uint8_t ch1[3];
    uint8_t ch2[3];
    uint8_t ch3[3];
    uint8_t ch4[3];
});
struct ads1294DataStruct adcData;


PACKSTRUCT(struct channelDataStruct
{
    uint32_t rawMain;
    uint32_t ambientMain;
    uint32_t rawAlt;
    uint32_t ambientAlt;
});

struct channelDataStruct channelData;

PACKSTRUCT(struct filterPtrStruct
{
    uint32_t signalPtr;
    uint32_t ambientPtr;
});


ssi_deviceHandle sram23LcvHandle;
ssi_deviceHandle flashM25pHandle;
ssi_deviceHandle ads1294Handle;

i2c_deviceHandle driver_dac7573Handle;
i2c_deviceHandle sensor_dac7573Handle;
i2c_deviceHandle tempSensorHandle;

ble_deviceHandle ble113Handle;
uint32_t debugConsole, bleConsole, decimal;

deMux deMuxLed;

typedef enum {
    selRed  = 0x00,
    selIr   = 0x01,
    sel810  = 0x02,
    sel1300 = 0x03
} ledSelect;

ledSelect sensChannel_A;
ledSelect sensChannel_B;


uint32_t filterInput[256];
uint32_t filterOutput[256];
uint32_t filterState[256];

typedef enum {
    initialize,
    wait_for_ble,
    configuring,
    siganl_acquisition,
    filtering,
    data_transfer,
    deviceState_last
} deviceStateMachine;

deviceStateMachine deviceState;

void change_deviceState(deviceStateMachine);

void configurePeripherals();

uint32_t InitSPI(uint32_t , uint32_t , uint32_t , uint32_t , uint32_t , bool );

uint32_t InitI2C(uint32_t , bool);

uint32_t InitConsole(uint32_t , uint32_t );
int uart_tx(uint16_t len,unsigned char *data);
int uart_rx(int len,unsigned char *data,int timeout_ms);


#endif /* PERIPHERALS_H_ */
