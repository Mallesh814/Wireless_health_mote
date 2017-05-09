/***************************************************************************//**
 *   @file   Communication.c
 *   @brief  Implementation of Communication Driver.
 *   @author DBogdan (dragos.bogdan@analog.com)
********************************************************************************
 * Copyright 2012-2015(c) Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  - Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *  - The use of this software may or may not infringe the patent rights
 *    of one or more patent holders.  This license does not release you
 *    from the requirement that you obtain separate licenses from these
 *    patent holders to use this software.
 *  - Use of the software either in source or binary form, must be run
 *    on or directly connected to an Analog Devices Inc. component.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
*******************************************************************************/

/******************************************************************************/
/* Include Files                                                              */
/******************************************************************************/
#include "communication.h"

/***************************************************************************//**
 * @brief Initializes the SPI communication peripheral.
 *
 * @param lsbFirst - Transfer format (0 or 1).
 *                   Example: 0x0 - MSB first.
 *                            0x1 - LSB first.
 * @param clockFreq - SPI clock frequency (Hz).
 *                    Example: 1000 - SPI clock frequency is 1 kHz.
 * @param clockPol - SPI clock polarity (0 or 1).
 *                   Example: 0x0 - Idle state for clock is a low level; active
 *                                  state is a high level;
 *	                      0x1 - Idle state for clock is a high level; active
 *                                  state is a low level.
 * @param clockEdg - SPI clock edge (0 or 1).
 *                   Example: 0x0 - Serial output data changes on transition
 *                                  from idle clock state to active clock state;
 *                            0x1 - Serial output data changes on transition
 *                                  from active clock state to idle clock state.
 *
 * @return status - Result of the initialization procedure.
 *                  Example: 1 - if initialization was successful;
 *                           0 - if initialization was unsuccessful.
*******************************************************************************/
unsigned char SPI_Init(unsigned char lsbFirst,
                       unsigned long clockFreq,
                       unsigned char clockPol,
                       unsigned char clockEdg)
{
    /* Add your code here. */
	InitSPI(SSI3_BASE, SSI_FRF_MOTO_MODE_3, SSI_MODE_MASTER, 1000000, 8, 0);
}

/***************************************************************************//**
 * @brief Reads data from SPI.
 *
 * @param slaveDeviceId - The ID of the selected slave device.
 * @param data - Data represents the write buffer as an input parameter and the
 *               read buffer as an output parameter.
 * @param bytesNumber - Number of bytes to read.
 *
 * @return Number of read bytes.
*******************************************************************************/
unsigned char SPI_Read(uint32_t slaveDeviceId,
                       unsigned char* data,
                       unsigned char bytesNumber)
{
	uint8_t i         = 0;
	uint8_t count     = 0;
	int32_t i32Status;
	uint32_t ui32Receive;

	while(SSIDataGetNonBlocking(SSI3_BASE, &ui32Receive)); // Clear FIFO Before Initiation a read Operation

	GPIOPinWrite(GPIO_PORTD_BASE,GPIO_PIN_1, 0X00);	// PULL DOWN Slave Select PIN
	for(i=0; i <= bytesNumber; ++i){
		SSIDataPut(SSI3_BASE, data[i]);
	}
	while(SSIBusy(SSI3_BASE));
	GPIOPinWrite(GPIO_PORTD_BASE,GPIO_PIN_1, 0XFF);	// PULL UP Slave Select PIN

	count = 0;
	for(i=0; i <= bytesNumber; ++i){
		i32Status = SSIDataGetNonBlocking(SSI3_BASE, &ui32Receive);
		if(i32Status) {
			count++;
			data[i] = 0x00FF & ui32Receive;
		}
	}

	if(count != (bytesNumber+1))
		return -1;

	while(SSIDataGetNonBlocking(SSI3_BASE, &ui32Receive)); // Clear FIFO Before Initiation a read Operation
	return count;
}



void SPI_Read_mod(uint32_t slaveDeviceId,
                       unsigned char* data,
                       unsigned char bytesNumber)
{
	uint8_t i         = 0;
	uint32_t ui32Receive;

	while(SSIDataGetNonBlocking(SSI3_BASE, &ui32Receive)); // Clear FIFO Before Initiation a read Operation

	GPIOPinWrite(GPIO_PORTD_BASE,GPIO_PIN_1, 0X00);	// PULL DOWN Slave Select PIN
	for(i=0; i <= bytesNumber; ++i){
		SSIDataPut(SSI3_BASE, data[i]);
		SSIDataGet(SSI3_BASE, &ui32Receive);
		data[i] = 0x00FF & ui32Receive;
	}
	while(SSIBusy(SSI3_BASE));
	GPIOPinWrite(GPIO_PORTD_BASE,GPIO_PIN_1, 0XFF);	// PULL UP Slave Select PIN

	while(SSIDataGetNonBlocking(SSI3_BASE, &ui32Receive)); // Clear FIFO Before Initiation a read Operation
}



/***************************************************************************//**
 * @brief Writes data to SPI.
 *
 * @param slaveDeviceId - The ID of the selected slave device.
 * @param data - Data represents the write buffer.
 * @param bytesNumber - Number of bytes to write.
 *
 * @return Number of written bytes.
*******************************************************************************/
unsigned char SPI_Write(uint32_t slaveDeviceId,
                        unsigned char* data,
                        unsigned char bytesNumber)
{
	uint8_t i		= 0;
	uint32_t ui32Receive;

	while(SSIDataGetNonBlocking(SSI3_BASE, &ui32Receive)); // Clear FIFO Before Initiation a read Operation

	GPIOPinWrite(GPIO_PORTD_BASE,GPIO_PIN_1, 0X00);	// PULL DOWN Slave Slect PIN
	for(i=0; i <= bytesNumber; ++i){
		SSIDataPut(SSI3_BASE, data[i]);
	}
	while(SSIBusy(SSI3_BASE));
	GPIOPinWrite(GPIO_PORTD_BASE,GPIO_PIN_1, 0XFF);	// PULL UP Slave Slect PIN

	while(SSIDataGetNonBlocking(SSI3_BASE, &ui32Receive)); // Clear FIFO Before Initiation a read Operation

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