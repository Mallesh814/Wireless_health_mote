/*
 * ram_23lcv1024.c
 *
 * Created: 2014-04-06 09:29:45
 *  Author: Virtmedia
 */ 

#include "SRAM_23LCV1024.h"

/*
uint32_t SRAM23LCV_Init(uint32_t ui32Base, uint32_t ui32BitRate)
{
    InitSPI(ui32Base, SSI_FRF_MOTO_MODE_0,SSI_MODE_MASTER, ui32BitRate, 8, false);

    sram23LcvHandle.ssiBase = ui32Base;
    sram23LcvHandle.csPort = GPIO_PORTC_BASE;
    sram23LcvHandle.csPin = GPIO_PIN_6;

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    GPIOPinTypeGPIOOutput(sram23LcvHandle.csPort, sram23LcvHandle.csPin);
    GPIOPinWrite(sram23LcvHandle.csPort, sram23LcvHandle.csPin, 0XFF);           // Pull Up FSS PIN of 23LCV1024

    return ui32Base;
}
*/
void SRAMSetMode(uint8_t mode)
{
    uint8_t buffer[2];

    buffer[0] = SRAM_INS_WRMR;
    buffer[1] = mode;

    SPI_Write(sram23LcvHandle, buffer, 2);

}

uint8_t SRAMReadMode(void)
{

	    uint8_t buffer[2];

	    buffer[0] = SRAM_INS_RDMR;
	    buffer[1] = 0x00;

	    SPI_Read(sram23LcvHandle, buffer, 2);
        return buffer[1];
}

uint8_t SRAMReadByte(uint32_t address)
{
    uint8_t buffer[5];

    buffer[0] = SRAM_INS_READ;
    buffer[1] = (address>>16) & 0xFF;
    buffer[2] = (address>>8) & 0xFF;
    buffer[3] = (address) & 0xFF;
    buffer[4] = 0;

    SPI_Read(sram23LcvHandle, buffer, 5);

    return buffer[4];
}


void SRAMWriteByte(uint8_t data, uint32_t address)
{
    uint8_t buffer[5];

    buffer[0] = SRAM_INS_WRITE;
    buffer[1] = (address>>16) & 0xFF;
    buffer[2] = (address>>8) & 0xFF;
    buffer[3] = (address) & 0xFF;
    buffer[4] = data;

    SPI_Write(sram23LcvHandle, buffer, 5);

}

void SRAMWriteData(uint8_t *buffer, uint8_t buffer_size, uint32_t address)
{
    ssi_packetHandle packetHandle;
    uint8_t instBuffer[4];

    instBuffer[0] = SRAM_INS_WRITE;
    instBuffer[1] = (address>>16) & 0xFF;
    instBuffer[2] = (address>>8) & 0xFF;
    instBuffer[3] = (address) & 0xFF;

    packetHandle.instBuffer = instBuffer;
    packetHandle.instLen = 4;
    packetHandle.dataBuffer = buffer;
    packetHandle.dataLen = buffer_size;

    SPI_Write_Packet(sram23LcvHandle, packetHandle);
}

/*
void SRAMWriteDataPage(const uint8_t *buffer, uint8_t buffer_size, uint32_t address)
{
	uint8_t i;
	GPIOPinWrite(LCV_FSS_GPIO,LCV_FSS_PIN, 0X00);			// Pull Down FSS PIN of 23LCV1024

	SSIDataPut(LCV_SSI_BASE, SRAM_INS_WRITE);
	SRAMSendAdderss(address);

	for(uint8_t i=0;i<buffer_size;++i)
	{
		SSIDataPut(LCV_SSI_BASE, *(buffer+i));
	}

	while(SSIBusy(LCV_SSI_BASE));
	GPIOPinWrite(LCV_FSS_GPIO,LCV_FSS_PIN, 0XFF);			// Pull Down FSS PIN of 23LCV1024
}
*/

void SRAMReadData(uint8_t *buffer, uint8_t buffer_size, uint32_t address)
{

    ssi_packetHandle packetHandle;
    uint8_t instBuffer[4];

    instBuffer[0] = SRAM_INS_READ;
    instBuffer[1] = (address>>16) & 0xFF;
    instBuffer[2] = (address>>8) & 0xFF;
    instBuffer[3] = (address) & 0xFF;

    packetHandle.instBuffer = instBuffer;
    packetHandle.instLen = 4;
    packetHandle.dataBuffer = buffer;
    packetHandle.dataLen = buffer_size;

    SPI_Read_Packet(sram23LcvHandle, packetHandle);

}

/*
void SRAMFillData(uint8_t value, uint32_t length, uint32_t address)
{
	GPIOPinWrite(LCV_FSS_GPIO,LCV_FSS_PIN, 0X00);			// Pull Down FSS PIN of 23LCV1024
	SSIDataPut(LCV_SSI_BASE, SRAM_INS_WRITE);
	SRAMSendAdderss(address);

	while(--length)
	{
		SSIDataPut(LCV_SSI_BASE, value);
	}

	while(SSIBusy(LCV_SSI_BASE));
	GPIOPinWrite(LCV_FSS_GPIO,LCV_FSS_PIN, 0XFF);			// Pull Down FSS PIN of 23LCV1024
}
*/
