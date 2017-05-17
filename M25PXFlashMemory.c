#include "M25PXFlashMemory.h"

//Public Methods


//*****************************************************************************
//
// This function sets up SSI External Flash ID Read
//
//*****************************************************************************
/*
uint32_t FLASHM25P_Init(uint32_t ui32Base, uint32_t ui32BitRate)
{
    InitSPI(ui32Base, SSI_FRF_MOTO_MODE_0,SSI_MODE_MASTER, ui32BitRate, 8, false);

    flashM25pHandle.ssiBase = ui32Base;
    flashM25pHandle.csPort = GPIO_PORTA_BASE;
    flashM25pHandle.csPin = GPIO_PIN_3;

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    GPIOPinTypeGPIOOutput(flashM25pHandle.csPort, flashM25pHandle.csPin);
    GPIOPinWrite(flashM25pHandle.csPort, flashM25pHandle.csPin, 0XFF);           // Pull Up FSS PIN of 23LCV1024

    return ui32Base;
}
*/

uint32_t M25P_ReadID()
{
    uint8_t buffer[4];
    uint32_t ui32ReadIDFlash;

    buffer[0] = M25P_READ_IDENTIFICATION;
    buffer[1] = 0;
    buffer[2] = 0;
    buffer[3] = 0;

    SPI_Read(flashM25pHandle, buffer, 4);

    ui32ReadIDFlash  = (buffer[1] << 16) | (buffer[2] << 8) | buffer[3];

	return ui32ReadIDFlash;
}

uint32_t M25P_readStatus() {
    uint8_t buffer[2];

    buffer[0] = M25P_READ_STATUS_REGISTER;
    buffer[1] = 0;

    SPI_Read(flashM25pHandle, buffer, 2);

    return buffer[1];
}

bool M25P_isBusy() {
    return M25P_readStatus() & (1 << 0); //LSB is WIP
}

bool M25P_isWritable() {
    return M25P_readStatus() & (1 << 1); //bit2 is WEL
}

uint8_t M25P_readByte(uint32_t address) {


    uint8_t buffer[5];

    buffer[0] = M25P_READ_DATA_BYTES;
    buffer[1] = (address>>16) & 0xFF;
    buffer[2] = (address>>8) & 0xFF;
    buffer[3] = (address) & 0xFF;
    buffer[4] = 0;

    while(M25P_isBusy()) ;

    SPI_Read(flashM25pHandle, buffer, 5);

    return buffer[4];

}

void M25P_programByte(uint32_t address,uint8_t data) {

    uint8_t buffer[5];

    buffer[0] = M25P_PAGE_PROGRAM;
    buffer[1] = (address>>16) & 0xFF;
    buffer[2] = (address>>8) & 0xFF;
    buffer[3] = (address) & 0xFF;
    buffer[4] = data;

    while(M25P_isBusy());
    M25P_enableWrite();

    SPI_Write(flashM25pHandle, buffer, 5);
}


void M25P_readBytes(uint8_t *buffer, uint8_t buffer_size, uint32_t address) {

    ssi_packetHandle packetHandle;
    uint8_t instBuffer[4];

    instBuffer[0] = M25P_READ_DATA_BYTES;
    instBuffer[1] = (address>>16) & 0xFF;
    instBuffer[2] = (address>>8) & 0xFF;
    instBuffer[3] = (address) & 0xFF;

    packetHandle.instBuffer = instBuffer;
    packetHandle.instLen = 4;
    packetHandle.dataBuffer = buffer;
    packetHandle.dataLen = buffer_size;

    while(M25P_isBusy());
    SPI_Read_Packet(flashM25pHandle, packetHandle);

}


void M25P_programBytes(uint8_t *buffer, uint32_t buffer_size, uint32_t address) {
	uint32_t i = 0;
    while (i < buffer_size) {
        uint32_t writeSize = 0x00100 - (address & 0x00ff);
        if (writeSize > buffer_size - i) writeSize = buffer_size - i;

        M25P_programPage(buffer, writeSize, address);

        address &= 0xffffff00; //start at the beginning of the page
        address += 0x00000100; //increment page by one
        buffer += writeSize;
        i += writeSize;
    }
}


void M25P_programPage(uint8_t *buffer, uint32_t buffer_size, uint32_t address) {

    ssi_packetHandle packetHandle;
    uint8_t instBuffer[4];

    instBuffer[0] = M25P_PAGE_PROGRAM;
    instBuffer[1] = (address>>16) & 0xFF;
    instBuffer[2] = (address>>8) & 0xFF;
    instBuffer[3] = (address) & 0xFF;

    packetHandle.instBuffer = instBuffer;
    packetHandle.instLen = 4;
    packetHandle.dataBuffer = buffer;
    packetHandle.dataLen = buffer_size;

    while(M25P_isBusy()) ;
    M25P_enableWrite(); //write is disabled automatically afterwards
    SPI_Write_Packet(flashM25pHandle, packetHandle);

}


void M25P_enableWrite() {
    uint8_t buffer[1];
    buffer[0] = M25P_WRITE_ENABLE;

    SPI_Write(flashM25pHandle, buffer, 1);

}

void M25P_disableWrite() {
    uint8_t buffer[1];
    buffer[0] = M25P_WRITE_DISABLE;

    SPI_Write(flashM25pHandle, buffer, 1);
}



void M25P_eraseSector(uint32_t address) {


    uint8_t buffer[4];

    address = address & 0xFF0000; //target Sector

    buffer[0] = M25P_SECTOR_ERASE;
    buffer[1] = (address>>16) & 0xFF;
    buffer[2] = (address>>8) & 0xFF;
    buffer[3] = (address) & 0xFF;

    while(M25P_isBusy());
    M25P_enableWrite();

    SPI_Write(flashM25pHandle, buffer, 4);

    while(M25P_isBusy());

}

void M25P_bulkErase() {


    uint8_t buffer[1];

    buffer[0] = M25P_BULK_ERASE;

    while(M25P_isBusy()) ;
    M25P_enableWrite(); //write is disabled automatically afterwards

    SPI_Write(flashM25pHandle, buffer, 1);

    while(M25P_isBusy()) ; //M25P_BULK_ERASE can take awhile, lets just wait until it completes

}
