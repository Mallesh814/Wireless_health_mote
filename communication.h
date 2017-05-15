/***************************************************************************//**
 *   @file   Communication.h
 *   @brief  Header file of Communication Driver.
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
#ifndef _COMMUNICATION_H_
#define _COMMUNICATION_H_

/******************************************************************************/
/* Include Files                                                              */
/******************************************************************************/
#include "peripherals.h"

/******************************************************************************/
/* Functions Prototypes                                                       */
/******************************************************************************/

/*! Initializes the I2C communication peripheral. */
unsigned char I2C_Init(unsigned long clockFreq);

/*! Reads data from SPI. */
void SPI_Read(ssi_deviceHandle deviceHandle,
              unsigned char* data,
              unsigned char bytesNumber);

void SPI_Read_Dummy(ssi_deviceHandle deviceHandle,
              unsigned char* data,
              unsigned char bytesNumber);

/*! Writes data to SPI. */
unsigned char SPI_Write(ssi_deviceHandle deviceHandle,
                        unsigned char* data,
                        unsigned char bytesNumber);

uint32_t SPI_Write_Packet(ssi_deviceHandle deviceHandle,
                          ssi_packetHandle packetHandle);

uint32_t SPI_Read_Packet(ssi_deviceHandle deviceHandle,
                          ssi_packetHandle packetHandle);

uint32_t SPI_Read_Packet_Dummy(ssi_deviceHandle deviceHandle,
                          ssi_packetHandle packetHandle);

/*! Writes data to I2C slave device.. */
unsigned char I2C_Write(uint32_t i2cBase,
                        unsigned char* data,
                        unsigned char bytesNumber);

/*! Reads data from I2C slave device. */
unsigned char I2C_Read(uint32_t i2cBase,
                        unsigned char* data,
                        unsigned char bytesNumber);

#endif /* _COMMUNICATION_H_ */
