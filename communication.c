
/******************************************************************************/
/* Include Files                                                              */
/******************************************************************************/
#include "communication.h"

/***************************************************************************//**
 * @brief Reads data from SPI.
 *
 * @param deviceHandle - Contains SSI Port information of Device.
 * @param data - Data represents the write buffer as an input parameter and the
 *               read buffer as an output parameter.
 * @param bytesNumber - Number of bytes to read.
 *
 * @return Number of read bytes.
*******************************************************************************/
void SPI_Read(ssi_deviceHandle deviceHandle,
              unsigned char* data,
              unsigned char bytesNumber)
{
	uint8_t i = 0;
	uint32_t ui32Receive;

	while(SSIDataGetNonBlocking(deviceHandle.ssiBase, &ui32Receive));  // Clear FIFO Before Initiation a read Operation.
    GPIOPinWrite(deviceHandle.csPort,deviceHandle.csPin, 0X00);         // PULL DOWN Slave Select PIN

	for(i = 0; i < bytesNumber; i++){
		SSIDataPut(deviceHandle.ssiBase, data[i]);
		SSIDataGet(deviceHandle.ssiBase, &ui32Receive);
		data[i] = 0x00FF & ui32Receive;
	}

	while(SSIBusy(deviceHandle.ssiBase));

	GPIOPinWrite(deviceHandle.csPort,deviceHandle.csPin, 0XFF);         // PULL UP Slave Select PIN
	while(SSIDataGetNonBlocking(deviceHandle.ssiBase, &ui32Receive));  // Clear FIFO After a read Operation.
}


/***************************************************************************//**
 * @brief Writes data to SPI.
 *
 * @param deviceHandle - Contains SSI Port information of Device.
 * @param data - Data represents the write buffer.
 * @param bytesNumber - Number of bytes to write.
 *
 * @return Number of written bytes.
*******************************************************************************/
unsigned char SPI_Write(ssi_deviceHandle deviceHandle,
                        unsigned char* data,
                        unsigned char bytesNumber)
{
	uint8_t i		= 0;
	uint32_t ui32Receive;

	while(SSIDataGetNonBlocking(deviceHandle.ssiBase, &ui32Receive)); // Clear FIFO Before Initiation a read Operation
    GPIOPinWrite(deviceHandle.csPort,deviceHandle.csPin, 0X00); // PULL DOWN Slave Slect PIN

    for(i = 0; i < bytesNumber; i++){
		SSIDataPut(deviceHandle.ssiBase, data[i]);
	}

    while(SSIBusy(deviceHandle.ssiBase));

    GPIOPinWrite(deviceHandle.csPort,deviceHandle.csPin, 0XFF);	// PULL UP Slave Slect PIN
	while(SSIDataGetNonBlocking(deviceHandle.ssiBase, &ui32Receive)); // Clear FIFO After a Write Operation

	return i-1;
}



uint32_t SPI_Write_Packet(ssi_deviceHandle deviceHandle,
                          ssi_packetHandle packetHandle)
{
    uint32_t i;
    uint32_t ui32Receive;

    while(SSIDataGetNonBlocking(deviceHandle.ssiBase, &ui32Receive)); // Clear FIFO Before Initiation a read Operation
    GPIOPinWrite(deviceHandle.csPort,deviceHandle.csPin, 0X00); // PULL DOWN Slave Slect PIN

    for(i = 0; i < packetHandle.instLen; ++i){
        SSIDataPut(deviceHandle.ssiBase, packetHandle.instBuffer[i]);
    }

    for(i = 0; i < packetHandle.dataLen; ++i){
        SSIDataPut(deviceHandle.ssiBase, packetHandle.dataBuffer[i]);
    }
    while(SSIBusy(deviceHandle.ssiBase));

    GPIOPinWrite(deviceHandle.csPort,deviceHandle.csPin, 0XFF); // PULL UP Slave Slect PIN
    while(SSIDataGetNonBlocking(deviceHandle.ssiBase, &ui32Receive)); // Clear FIFO After a Write Operation

    return i-1;
}

uint32_t SPI_Read_Packet(ssi_deviceHandle deviceHandle,
                          ssi_packetHandle packetHandle)
{
    uint32_t i;
    uint32_t ui32Receive;

    while(SSIDataGetNonBlocking(deviceHandle.ssiBase, &ui32Receive)); // Clear FIFO Before Initiation a read Operation
    GPIOPinWrite(deviceHandle.csPort,deviceHandle.csPin, 0X00); // PULL DOWN Slave Slect PIN

    for(i = 0; i < packetHandle.instLen; i++){
        SSIDataPut(deviceHandle.ssiBase, packetHandle.instBuffer[i]);
        SSIDataGet(deviceHandle.ssiBase, &ui32Receive);
        packetHandle.instBuffer[i] = (uint8_t) (0x00FF & ui32Receive);
    }

    for(i = 0; i < packetHandle.dataLen; i++){
        SSIDataPut(deviceHandle.ssiBase, packetHandle.dataBuffer[i]);
        SSIDataGet(deviceHandle.ssiBase, &ui32Receive);
        packetHandle.dataBuffer[i] = (uint8_t) (0x00FF & ui32Receive);
    }
    while(SSIBusy(deviceHandle.ssiBase));

    GPIOPinWrite(deviceHandle.csPort,deviceHandle.csPin, 0XFF); // PULL UP Slave Slect PIN
    while(SSIDataGetNonBlocking(deviceHandle.ssiBase, &ui32Receive)); // Clear FIFO After a Write Operation

    return i-1;
}

/***************************************************************************//**
 * @brief Writes data to SPI.
 *
 * @param i2cBase - The ID of the selected slave device.
 * @param data - Data represents the write buffer.
 * @param bytesNumber - Number of bytes to write.
 *
 * @return Number of written bytes.
*******************************************************************************/
unsigned char I2C_Write(uint32_t i2cBase,
                        unsigned char* data,
                        unsigned char bytesNumber)
{

	uint8_t i;
	I2CMasterSlaveAddrSet(i2cBase,data[0],false);
	I2CMasterDataPut(i2cBase,data[1]);
	I2CMasterControl(i2cBase, I2C_MASTER_CMD_BURST_SEND_START);
	while(I2CMasterBusy(i2cBase));
	if(I2CMasterErr(i2cBase)!= 0)
	{
		return 1;
	}

	for(i = 2; i < bytesNumber - 1; i++){
		I2CMasterDataPut(i2cBase,data[i]);
		I2CMasterControl(i2cBase, I2C_MASTER_CMD_BURST_SEND_CONT);
		while(I2CMasterBusy(i2cBase));
		if(I2CMasterErr(i2cBase)!= 0)
		{
			return 1;
		}
	}

	I2CMasterDataPut(i2cBase,data[bytesNumber-1]);
	I2CMasterControl(i2cBase, I2C_MASTER_CMD_BURST_SEND_FINISH);
	while(I2CMasterBusy(i2cBase));
	while(I2CMasterBusBusy(i2cBase));
	if(I2CMasterErr(i2cBase)!= 0)
	{
		return 1;
	}

	return 0;
}




unsigned char I2C_Read(uint32_t i2cBase,
                        unsigned char* data,
                        unsigned char bytesNumber)
{
	uint32_t ui32Receive;
	uint8_t i;

	I2CMasterSlaveAddrSet(i2cBase,data[0],false);
	I2CMasterDataPut(i2cBase, data[1]);

	I2CMasterControl(i2cBase, I2C_MASTER_CMD_SINGLE_SEND);
	while(I2CMasterBusy(i2cBase));

	if(I2CMasterErr(i2cBase)!= 0)	return 1;

	I2CMasterSlaveAddrSet(i2cBase, data[0], true);
	I2CMasterControl(i2cBase, I2C_MASTER_CMD_BURST_RECEIVE_START);
	while(I2CMasterBusy(i2cBase));

	if(I2CMasterErr(i2cBase)!= 0)	return 1;

	ui32Receive = I2CMasterDataGet(i2cBase);
	data[2] = 0x00FF & ui32Receive;

	for(i = 3; i < (bytesNumber - 1) ; i++)
	{
		I2CMasterControl(i2cBase, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
		while(I2CMasterBusy(i2cBase));

		if(I2CMasterErr(i2cBase)!= 0)	return 1;
		ui32Receive = I2CMasterDataGet(i2cBase);
		data[i] = 0x00FF & ui32Receive;
	}

	I2CMasterControl(i2cBase, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
	while(I2CMasterBusy(i2cBase));

	if(I2CMasterErr(i2cBase)!= 0)	return 1;
	ui32Receive = I2CMasterDataGet(i2cBase);
	data[bytesNumber-1] = 0x00FF & ui32Receive;

	return i;
}
