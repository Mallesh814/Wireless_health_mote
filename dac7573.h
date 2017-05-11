/*
 * dac7573.h
 *
 *  Created on: May 10, 2017
 *      Author: edsys
 */

#ifndef DAC7573_H_
#define DAC7573_H_

#include "peripherals.h"
#include "communication.h"

#define CTRL_ADD_MASK 0xC0

#define LOAD_MODE_0 0x00
#define LOAD_MODE_1 0x10
#define LOAD_MODE_2 0x20
#define LOAD_MODE_3 0x30

#define POWER_DOWN 0x01

#define SEL_A   0x00
#define SEL_B   0x02
#define SEL_C   0x04
#define SEL_D   0x06

#define ADDRESS_BYTE 0x4C
#define ADDRESS_MASK 0x03

void dac7573_Send(i2c_deviceHandle , uint32_t , ledSelect );


#endif /* DAC7573_H_ */
