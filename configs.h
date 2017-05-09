/*
 * configs.h
 *
 *  Created on: 02-Mar-2016
 *      Author: Mallesh
 */

#ifndef CONFIGS_H_
#define CONFIGS_H_
#include <stdint.h>
#include <stdbool.h>
#include "inc/tm4c123gh6pm.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/i2c.h"
#include "driverlib/interrupt.h"
#include "driverlib/i2c.h"
#include "driverlib/ssi.h"

#define SSI_DUMMY_BYTE						 0x00

void InitSPI(uint32_t , uint32_t , uint32_t , uint32_t , uint32_t , bool );
uint32_t InitI2C(uint32_t , bool);
uint32_t InitConsole(uint32_t , uint32_t );


#endif /* CONFIGS_H_ */
