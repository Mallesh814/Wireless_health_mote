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
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"

#include "peripherals.h"
#include "communication.h"
#include "ADS129xInfo.h"
#include "parser.h"

uint32_t ADS1294_Init(ssi_deviceHandle);
void ADS1294_readBytes(uint8_t*, uint8_t );
void ADS1294_stopConv(ssi_deviceHandle);

void Timer0Config(uint32_t );
void Timer1Config(uint32_t );

#endif /* ADS1294CONFIGS_H_ */
