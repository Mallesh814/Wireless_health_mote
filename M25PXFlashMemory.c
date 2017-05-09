#include "M25PXFlashMemory.h"

//Public Methods


//*****************************************************************************
//
// This function sets up SSI External Flash ID Read
//
//*****************************************************************************

void InitFLASH(void)
{
	SysCtlPeripheralEnable(M25P_FSS_PERIPH);
	GPIOPinTypeGPIOOutput(M25P_FSS_GPIO, M25P_FSS_PIN);
	GPIOPinWrite(M25P_FSS_GPIO,M25P_FSS_PIN, 0XFF);			// Pull Up FSS PIN of 23LCV1024
    InitSPI(M25P_SSI_BASE, SSI_FRF_MOTO_MODE_0,SSI_MODE_MASTER, 1000000, 8, false);
}


uint32_t M25P_ReadID()
{
   uint32_t ui32Receive, ui32ReadIDFlash;

//   HWREG(M25P_FSS_GPIO + (GPIO_O_DATA + (M25P_FSS_PIN << 2))) = 0x00;

	while(SSIDataGetNonBlocking(M25P_SSI_BASE, &ui32Receive)); // Clear FIFO Before Initiation a read Operation

	GPIOPinWrite(M25P_FSS_GPIO,M25P_FSS_PIN, 0X00);	// PULL DOWN Slave Slect PIN

	SSIDataPut(M25P_SSI_BASE, M25P_READ_IDENTIFICATION);
	SSIDataPut(M25P_SSI_BASE, 0x00);
	SSIDataPut(M25P_SSI_BASE, 0x00);
	SSIDataPut(M25P_SSI_BASE, 0x00);

	while(SSIBusy(M25P_SSI_BASE));

	GPIOPinWrite(M25P_FSS_GPIO,M25P_FSS_PIN, 0XFF);	// PULL UP Slave Slect PIN

	SSIDataGet(M25P_SSI_BASE, &ui32Receive);
	SSIDataGet(M25P_SSI_BASE, &ui32Receive);
	ui32ReadIDFlash = ui32Receive;
	SSIDataGet(M25P_SSI_BASE, &ui32Receive);
	ui32ReadIDFlash  = (ui32ReadIDFlash << 8) | ui32Receive;
	SSIDataGet(M25P_SSI_BASE, &ui32Receive);
	ui32ReadIDFlash  = (ui32ReadIDFlash << 8) | ui32Receive;

	return ui32ReadIDFlash;
}

uint32_t M25P_readStatus() {
	   uint32_t ui32Status;

	//   HWREG(M25P_FSS_GPIO + (GPIO_O_DATA + (M25P_FSS_PIN << 2))) = 0x00;
		while(SSIDataGetNonBlocking(M25P_SSI_BASE, &ui32Status)); // Clear FIFO Before Initiation a read Operation

		GPIOPinWrite(M25P_FSS_GPIO,M25P_FSS_PIN, 0X00);	// PULL DOWN Slave Slect PIN

		SSIDataPut(M25P_SSI_BASE, M25P_READ_STATUS_REGISTER);
		SSIDataPut(M25P_SSI_BASE, 0x00);
		while(SSIBusy(M25P_SSI_BASE));

		GPIOPinWrite(M25P_FSS_GPIO,M25P_FSS_PIN, 0XFF);	// PULL UP Slave Slect PIN

		SSIDataGet(M25P_SSI_BASE, &ui32Status);
		SSIDataGet(M25P_SSI_BASE, &ui32Status);

		return ui32Status;
}

bool M25P_isBusy() {
    return M25P_readStatus() & (1 << 0); //LSB is WIP
}

bool M25P_isWritable() {
    return M25P_readStatus() & (1 << 1); //bit2 is WEL
}

uint8_t M25P_readByte(uint32_t address) {

	uint8_t data;
	uint32_t pui32Dummy;

	while(M25P_isBusy()) ;

	while(SSIDataGetNonBlocking(M25P_SSI_BASE, &address)); // Clear Rx FIFO After Operation

	GPIOPinWrite(M25P_FSS_GPIO,M25P_FSS_PIN, 0x00);	// PULL DOWN Slave Slect PIN
	SSIDataPut(M25P_SSI_BASE, M25P_READ_DATA_BYTES);
    M25P_sendAddress(address);
	SSIDataPut(M25P_SSI_BASE, SSI_DUMMY_BYTE);

	for (data = 4; data ; data--)							// Discard Junk Data From FIFO
		SSIDataGet(M25P_SSI_BASE, &pui32Dummy);

	SSIDataGet(M25P_SSI_BASE, &pui32Dummy);

    while(SSIBusy(M25P_SSI_BASE));
	GPIOPinWrite(M25P_FSS_GPIO,M25P_FSS_PIN, 0xFF);	// PULL UP Slave Slect PIN

	data = 0xff & pui32Dummy;

    return data;
}

void M25P_programByte(uint32_t addr,uint8_t b) {
    while(M25P_isBusy());

	M25P_enableWrite();

	GPIOPinWrite(M25P_FSS_GPIO,M25P_FSS_PIN, 0X00);	// PULL DOWN Slave Slect PIN

	SSIDataPut(M25P_SSI_BASE, M25P_PAGE_PROGRAM);
    M25P_sendAddress(addr);
    SSIDataPut(M25P_SSI_BASE, b);

    while(SSIBusy(M25P_SSI_BASE));
	GPIOPinWrite(M25P_FSS_GPIO,M25P_FSS_PIN, 0XFF);	// PULL UP Slave Slect PIN

	while(SSIDataGetNonBlocking(M25P_SSI_BASE, &addr)); // Clear Rx FIFO After Operation
}


void M25P_readBytes(uint8_t *buffer, uint8_t buffer_size, uint32_t address) {
	uint8_t i;
	uint32_t pui32Dummy;

	while(M25P_isBusy());
	while(SSIDataGetNonBlocking(M25P_SSI_BASE, &pui32Dummy)); // Clear FIFO Before Initiation a read Operation

	GPIOPinWrite(M25P_FSS_GPIO,M25P_FSS_PIN, 0X00);			// Pull Down FSS PIN of 23LCV1024

	SSIDataPut(M25P_SSI_BASE, M25P_READ_DATA_BYTES);
	M25P_sendAddress(address);

	for (i = 4; i ; i--)							// Discard Junk Data From FIFO
		SSIDataGet(M25P_SSI_BASE, &pui32Dummy);

	while(buffer_size)
	{

		for(i=((buffer_size > 8) ?(8):(buffer_size)); i ; i--){		// Dealing with Chunks of 8 Bytes(FIFO LENGTH)
			SSIDataPutNonBlocking(M25P_SSI_BASE, SSI_DUMMY_BYTE);
		}

		while(SSIDataGetNonBlocking(M25P_SSI_BASE, buffer) && buffer_size){
			++buffer;
			--buffer_size;
		}
	}

//	while(SSIBusy(LCV_SSI_BASE));
	GPIOPinWrite(M25P_FSS_GPIO,M25P_FSS_PIN, 0XFF);			// Pull Down FSS PIN of 23LCV1024
}

void M25P_programBytes(uint8_t *buffer, uint8_t buffer_size, uint32_t address) {
	int i=0;
    while (i < buffer_size) {
        int writeSize = 0x100 - (address & 0xff);
        if (writeSize > buffer_size - i) writeSize = buffer_size - i;
        M25P_programPage(buffer,writeSize,address);
        address &= 0xffffff00; //start at the beginning of the page
        address += 0x00000100; //increment page by one
        buffer += writeSize;
        i += writeSize;
    }
}


void M25P_programPage(uint8_t *buffer, uint8_t buffer_size, uint32_t address) {
	uint32_t ui32Dummy;
	int i=0;

	while(M25P_isBusy()) ;

    M25P_enableWrite(); //write is disabled automatically afterwards

    GPIOPinWrite(M25P_FSS_GPIO,M25P_FSS_PIN, 0X00);	// PULL DOWN Slave Slect PIN

    SSIDataPut(M25P_SSI_BASE, M25P_PAGE_PROGRAM);
    M25P_sendAddress(address);

    for (i=0; i < buffer_size; i++) {
    	SSIDataPut(M25P_SSI_BASE, buffer[i]);
    }

    while(SSIBusy(M25P_SSI_BASE));
    GPIOPinWrite(M25P_FSS_GPIO,M25P_FSS_PIN, 0XFF);	// PULL UP Slave Slect PIN

    while(SSIDataGetNonBlocking(M25P_SSI_BASE, &ui32Dummy)); // Clear Rx FIFO After Operation

}


void M25P_enableWrite() {
	uint32_t ui32Dummy;

	GPIOPinWrite(M25P_FSS_GPIO,M25P_FSS_PIN, 0X00);	// PULL DOWN Slave Slect PIN

	SSIDataPut(SSI0_BASE, M25P_WRITE_ENABLE);

    while(SSIBusy(M25P_SSI_BASE));
	GPIOPinWrite(M25P_FSS_GPIO,M25P_FSS_PIN, 0XFF);	// PULL UP Slave Slect PIN

	while(SSIDataGetNonBlocking(M25P_SSI_BASE, &ui32Dummy)); // Clear Rx FIFO After Operation

}

void M25P_disableWrite() {
	uint32_t ui32Dummy;

	GPIOPinWrite(M25P_FSS_GPIO,M25P_FSS_PIN, 0X00);	// PULL DOWN Slave Slect PIN

    SSIDataPut(M25P_SSI_BASE, M25P_WRITE_DISABLE);

    while(SSIBusy(M25P_SSI_BASE));
	GPIOPinWrite(M25P_FSS_GPIO,M25P_FSS_PIN, 0XFF);	// PULL UP Slave Slect PIN

	while(SSIDataGetNonBlocking(M25P_SSI_BASE, &ui32Dummy)); // Clear Rx FIFO After Operation
}

void M25P_sendAddress(uint32_t addr) {
	SSIDataPut(M25P_SSI_BASE, (addr & 0xff0000) >> 16);	//addr 0
	SSIDataPut(M25P_SSI_BASE, (addr & 0xff00) >> 8);	//addr 1
	SSIDataPut(M25P_SSI_BASE, (addr & 0xff));			//addr 2
}


void M25P_eraseSector(uint32_t addr) {

	uint32_t ui32Dummy;
    addr = addr & 0xFF0000; //target Sector

    while(M25P_isBusy()) ;

    M25P_enableWrite(); //write is disabled automatically afterwards

	GPIOPinWrite(M25P_FSS_GPIO,M25P_FSS_PIN, 0X00);	// PULL DOWN Slave Slect PIN

	SSIDataPut(M25P_SSI_BASE, M25P_SECTOR_ERASE);
    M25P_sendAddress(addr);

    while(SSIBusy(M25P_SSI_BASE));
	GPIOPinWrite(M25P_FSS_GPIO,M25P_FSS_PIN, 0XFF);	// PULL UP Slave Slect PIN

	while(SSIDataGetNonBlocking(M25P_SSI_BASE, &ui32Dummy)); // Clear Rx FIFO After Operation

}

void M25P_bulkErase() {

	uint32_t ui32Dummy;
    while(M25P_isBusy()) ;

    M25P_enableWrite(); //write is disabled automatically afterwards

	GPIOPinWrite(M25P_FSS_GPIO,M25P_FSS_PIN, 0X00);	// PULL DOWN Slave Slect PIN

	SSIDataPut(M25P_SSI_BASE, M25P_BULK_ERASE);

    while(SSIBusy(M25P_SSI_BASE));
	GPIOPinWrite(M25P_FSS_GPIO,M25P_FSS_PIN, 0XFF);	// PULL UP Slave Slect PIN

    while(M25P_isBusy()) ; //M25P_BULK_ERASE can take awhile, lets just wait until it completes

	while(SSIDataGetNonBlocking(M25P_SSI_BASE, &ui32Dummy)); // Clear Rx FIFO After Operation

}
