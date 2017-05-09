/*
 * ads1294Configs.h
 *
 *  Created on: Apr 20, 2017
 *      Author: edsys
 */

#ifndef ADS1294CONFIGS_H_
#define ADS1294CONFIGS_H_

#include <stdint.h>
#include <stdbool.h>
#include "inc/tm4c123gh6pm.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/systick.h"
#include "inc/hw_memmap.h"
#include "driverlib/timer.h"

#include "configs.h"
#include "parser.h"
#include "tlv5636.h"
#include "ADS129xInfo.h"
#include "communication.h"

uint32_t ADS1294_Init(uint32_t , uint32_t );
void TimerConfig(uint32_t );

ssi_deviceHandle ads1294Handle;

#endif /* ADS1294CONFIGS_H_ */
