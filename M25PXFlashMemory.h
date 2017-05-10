#ifndef M25PXFlashMemory_h
#define M25PXFlashMemory_h

#include <stdint.h>
#include <stdbool.h>

#include "driverlib/ssi.h"
#include "driverlib/gpio.h"
#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"
#include "driverlib/sysctl.h"

#include "peripherals.h"
#include "communication.h"

//Instructions
#define M25P_WRITE_ENABLE                    0x06
#define M25P_WRITE_DISABLE                   0x04
#define M25P_READ_IDENTIFICATION             0x9f
#define M25P_READ_IDENTIFICATION2            0x9e
#define M25P_READ_STATUS_REGISTER            0x05
#define M25P_WRITE_STATUS_REGISTER           0x01
#define M25P_READ_DATA_BYTES                 0x03
#define M25P_READ_DATA_BYTES_FAST            0x0b
#define M25P_PAGE_PROGRAM                    0x02
#define M25P_SECTOR_ERASE                    0xd8
#define M25P_BULK_ERASE                      0xc7
#define M25P_DEEP_POWER_DOWN                 0xb9
#define M25P_RELEASE_DEEP_POWER_DOWN         0xab


//Configuration
#define M25P_SSI_BASE SSI0_BASE
#define M25P_FSS_GPIO GPIO_PORTA_BASE
#define M25P_FSS_PIN GPIO_PIN_3
#define M25P_FSS_PERIPH SYSCTL_PERIPH_GPIOA

uint32_t FLASHM25P_Init(uint32_t ui32Base, uint32_t ui32BitRate);
uint32_t M25P_ReadID();
uint32_t M25P_readStatus();
bool M25P_isBusy();
bool M25P_isWritable();
void M25P_programByte(uint32_t addr,uint8_t b);
void M25P_programBytes(uint8_t *buffer, uint8_t buffer_size, uint32_t address);
uint8_t M25P_readByte(uint32_t addr);
void M25P_readBytes(uint8_t *buffer, uint8_t buffer_size, uint32_t address);

void M25P_eraseSector(uint32_t addr);
void M25P_bulkErase();

void M25P_enableWrite();
void M25P_disableWrite();
void M25P_sendAddress(uint32_t addr);
void M25P_programPage(uint8_t *buffer, uint8_t buffer_size, uint32_t address);

#endif
