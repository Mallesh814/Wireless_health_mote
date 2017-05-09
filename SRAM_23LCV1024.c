/*
 * ram_23lcv1024.c
 *
 * Created: 2014-04-06 09:29:45
 *  Author: Virtmedia
 */ 

#include "SRAM_23LCV1024.h"


void InitSRAM(void)
{
	SysCtlPeripheralEnable(LCV_FSS_PERIPH);
	GPIOPinTypeGPIOOutput(LCV_FSS_GPIO, LCV_FSS_PIN);
	GPIOPinWrite(LCV_FSS_GPIO,LCV_FSS_PIN, 0XFF);			// Pull Up FSS PIN of 23LCV1024
    InitSPI(LCV_SSI_BASE, SSI_FRF_MOTO_MODE_0,SSI_MODE_MASTER, 1000000, 8, false);
}


void SRAMSendAdderss(uint32_t address)
{
	SSIDataPut(LCV_SSI_BASE, (address>>16) & 0xFF);
	SSIDataPut(LCV_SSI_BASE, (address>>8) & 0xFF);
	SSIDataPut(LCV_SSI_BASE, (address) & 0xFF);
}

void SRAMSetMode(uint8_t mode)
{

	GPIOPinWrite(LCV_FSS_GPIO,LCV_FSS_PIN, 0X00);			// Pull Down FSS PIN of 23LCV1024

	SSIDataPut(LCV_SSI_BASE, SRAM_INS_WRMR);
	SSIDataPut(LCV_SSI_BASE, mode);
	while(SSIBusy(LCV_SSI_BASE));

	GPIOPinWrite(LCV_FSS_GPIO,LCV_FSS_PIN, 0XFF);			// Pull Down FSS PIN of 23LCV1024
}

uint8_t SRAMReadMode(void)
{
	   uint32_t ui32Status;
		uint32_t pui32Dummy;
		while(SSIDataGetNonBlocking(LCV_SSI_BASE, &pui32Dummy)); // Clear FIFO Before Initiation a read Operation

	//   HWREG(M25P_FSS_GPIO + (GPIO_O_DATA + (M25P_FSS_PIN << 2))) = 0x00; // To Prevent Function Call

		GPIOPinWrite(LCV_FSS_GPIO,LCV_FSS_PIN, 0X00);			// Pull Down FSS PIN of 23LCV1024

		SSIDataPut(LCV_SSI_BASE, SRAM_INS_RDMR);
		SSIDataPut(LCV_SSI_BASE, 0x00);
		while(SSIBusy(LCV_SSI_BASE));

		GPIOPinWrite(LCV_FSS_GPIO,LCV_FSS_PIN, 0XFF);			// Pull Down FSS PIN of 23LCV1024

		SSIDataGet(LCV_SSI_BASE, &ui32Status);
		SSIDataGet(LCV_SSI_BASE, &ui32Status);

		return ui32Status;
}

void SRAMWriteByte(uint8_t data, uint32_t address)
{
	GPIOPinWrite(LCV_FSS_GPIO,LCV_FSS_PIN, 0X00);			// Pull Down FSS PIN of 23LCV1024

	SSIDataPut(LCV_SSI_BASE, SRAM_INS_WRITE);
	SRAMSendAdderss(address);
	SSIDataPut(LCV_SSI_BASE, data);

	while(SSIBusy(LCV_SSI_BASE));

	GPIOPinWrite(LCV_FSS_GPIO,LCV_FSS_PIN, 0XFF);			// Pull Down FSS PIN of 23LCV1024
}

void SRAMWriteData(uint8_t *buffer, uint8_t buffer_size, uint32_t address)
{
	uint8_t i;
	GPIOPinWrite(LCV_FSS_GPIO,LCV_FSS_PIN, 0X00);			// Pull Down FSS PIN of 23LCV1024

	SSIDataPut(LCV_SSI_BASE, SRAM_INS_WRITE);
	SRAMSendAdderss(address);

	for(i=0; i<buffer_size; ++i)
	{
		SSIDataPut(LCV_SSI_BASE, *(buffer+i));
	}

	while(SSIBusy(LCV_SSI_BASE));
	GPIOPinWrite(LCV_FSS_GPIO,LCV_FSS_PIN, 0XFF);			// Pull Down FSS PIN of 23LCV1024
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


uint8_t SRAMReadByte(uint32_t address)
{
	uint8_t data;
	uint32_t pui32Dummy;

	while(SSIDataGetNonBlocking(LCV_SSI_BASE, &pui32Dummy)); // Clear FIFO Before Initiation a read Operation

	GPIOPinWrite(LCV_FSS_GPIO,LCV_FSS_PIN, 0X00);			// Pull Down FSS PIN of 23LCV1024

	SSIDataPut(LCV_SSI_BASE, SRAM_INS_READ);				// Send Instruction and adrress
	SRAMSendAdderss(address);
	SSIDataPut(LCV_SSI_BASE, SSI_DUMMY_BYTE);				// send Dummy bytes as long as u want to receive data in Sequential Mode

	while(SSIBusy(LCV_SSI_BASE));
	GPIOPinWrite(LCV_FSS_GPIO,LCV_FSS_PIN, 0XFF);			// Pull Down FSS PIN of 23LCV1024

	for (data = 4; data ; data--)							// Discard Junk Data From FIFO
		SSIDataGet(LCV_SSI_BASE, &pui32Dummy);

	SSIDataGet(LCV_SSI_BASE, &pui32Dummy);

	data = (0xFF &  pui32Dummy);

	return data;
}

void SRAMReadData(uint8_t *buffer, uint8_t buffer_size, uint32_t address)
{
	uint8_t i;
	uint32_t pui32Dummy;
	while(SSIDataGetNonBlocking(LCV_SSI_BASE, &pui32Dummy)); // Clear FIFO Before Initiation a read Operation

	GPIOPinWrite(LCV_FSS_GPIO,LCV_FSS_PIN, 0X00);			// Pull Down FSS PIN of 23LCV1024

	SSIDataPut(LCV_SSI_BASE, SRAM_INS_READ);
	SRAMSendAdderss(address);

	for (i = 4; i ; i--)							// Discard Junk Data From FIFO
		SSIDataGet(LCV_SSI_BASE, &pui32Dummy);

	while(buffer_size)
	{

		for(i=((buffer_size > 8) ?(8):(buffer_size)); i ; i--){		// Dealing with Chunks of 8 Bytes(FIFO LENGTH)
			SSIDataPutNonBlocking(LCV_SSI_BASE, SSI_DUMMY_BYTE);
		}

		while(SSIDataGetNonBlocking(LCV_SSI_BASE, buffer) && buffer_size){
			++buffer;
			--buffer_size;
		}
	}

//	while(SSIBusy(LCV_SSI_BASE));
	GPIOPinWrite(LCV_FSS_GPIO,LCV_FSS_PIN, 0XFF);			// Pull Down FSS PIN of 23LCV1024
}

void SRAMFillData(uint8_t value, uint32_t lenght, uint32_t address)
{
	GPIOPinWrite(LCV_FSS_GPIO,LCV_FSS_PIN, 0X00);			// Pull Down FSS PIN of 23LCV1024
	SSIDataPut(LCV_SSI_BASE, SRAM_INS_WRITE);
	SRAMSendAdderss(address);

	while(--lenght)
	{
		SSIDataPut(LCV_SSI_BASE, value);
	}

	while(SSIBusy(LCV_SSI_BASE));
	GPIOPinWrite(LCV_FSS_GPIO,LCV_FSS_PIN, 0XFF);			// Pull Down FSS PIN of 23LCV1024
}
