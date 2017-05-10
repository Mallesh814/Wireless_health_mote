/*
 * ram_23lcv1024.h
 *
 * Created: 2014-04-06 09:30:05
 *  Author: Virtmedia
 */ 

#ifndef SRAM_23LCV1024_H_
#define SRAM_23LCV1024_H_

#include <stdint.h>
#include <stdbool.h>

#include "driverlib/ssi.h"
#include "driverlib/gpio.h"
#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"
#include "driverlib/sysctl.h"

#include "peripherals.h"
#include "communication.h"

//Instruction set
#define SRAM_INS_READ	0x03	//Read data from memory array beginning at selected address
#define SRAM_INS_WRITE	0x02	//Write data to memory array beginning at selected address
#define SRAM_INS_EDIO	0x3B	//Enter Dual I/O access
#define SRAM_INS_RSTIO	0xFF	//Reset Dual I/O access
#define SRAM_INS_RDMR	0x05	//Read Mode Register
#define SRAM_INS_WRMR	0x01	//Write Mode Register

//Mode Register
#define SRAM_MODE_BYTE	0x00
#define SRAM_MODE_PAGE	0x80
#define SRAM_MODE_SEQUENTIAL	0x40


#define SRAM_MAX_ADDRESS    0x1FFFFh

/*
//SSI Base and FSS Pin Info
#define LCV_SSI_BASE SSI0_BASE
#define LCV_FSS_PERIPH SYSCTL_PERIPH_GPIOC
#define LCV_FSS_GPIO GPIO_PORTC_BASE
#define LCV_FSS_PIN GPIO_PIN_6
*/

uint32_t ram_address;

uint32_t SRAM23LCV_Init(uint32_t ui32Base, uint32_t ui32BitRate);

void SRAMSetMode(uint8_t mode);

uint8_t SRAMReadMode(void);

void SRAMWriteByte(uint8_t data, uint32_t address);

void SRAMWriteData(uint8_t *buffer, uint8_t buffer_size, uint32_t address);

//void SRAMWriteDataPage(const uint8_t *buffer, uint8_t buffer_size, uint32_t address);

uint8_t SRAMReadByte(uint32_t address);

void SRAMReadData(uint8_t *buffer, uint8_t buffer_size, uint32_t address);

//void SRAMFillData(uint8_t value, uint32_t lenght, uint32_t address);

#endif /* SRAM_23LCV1024_H_ */
