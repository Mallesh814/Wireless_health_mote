/*
 * dac7573.c
 *
 *  Created on: May 10, 2017
 *      Author: edsys
 */

#include "dac7573.h"


void dac7573_Send(i2c_deviceHandle deviceHandle, uint32_t data, ledSelect channel)
{
    uint32_t decimal;
    uint8_t buffer[4];

    decimal = data << 4;

    buffer[0] = ADDRESS_BYTE | (deviceHandle.i2cAddr & ADDRESS_MASK);

    switch(channel){
    case selRed:
        buffer[1] = ((deviceHandle.i2cAddr << 4) & CTRL_ADD_MASK) | LOAD_MODE_1 | SEL_A;
        break;
    case selIr:
        buffer[1] = ((deviceHandle.i2cAddr << 4) & CTRL_ADD_MASK) | LOAD_MODE_1 | SEL_B;
        break;
    case sel810:
        buffer[1] = ((deviceHandle.i2cAddr << 4) & CTRL_ADD_MASK) | LOAD_MODE_1 | SEL_D;
        break;
    case sel1300:
        buffer[1] = ((deviceHandle.i2cAddr << 4) & CTRL_ADD_MASK) | LOAD_MODE_1 | SEL_C;
        break;
    }

    buffer[2] = decimal >> 8;
    buffer[3] = decimal & 0x00FF;

    I2C_Write(deviceHandle.i2cBase, buffer, 4);
}

/*
uint32_t dac7573_Read(i2c_deviceHandle deviceHandle, ledSelect channel)
{
    uint32_t decimal;
    uint8_t buffer[4];

    decimal = data << 4;

    buffer[0] = ADDRESS_BYTE | (deviceHandle.i2cAddr & ADDRESS_MASK);

    switch(channel){
    case selRed:
        buffer[1] = ((deviceHandle.i2cAddr << 4) & CTRL_ADD_MASK) | LOAD_MODE_1 | SEL_A;
        break;
    case selIr:
        buffer[1] = ((deviceHandle.i2cAddr << 4) & CTRL_ADD_MASK) | LOAD_MODE_1 | SEL_B;
        break;
    case sel810:
        buffer[1] = ((deviceHandle.i2cAddr << 4) & CTRL_ADD_MASK) | LOAD_MODE_1 | SEL_C;
        break;
    case sel1300:
        buffer[1] = ((deviceHandle.i2cAddr << 4) & CTRL_ADD_MASK) | LOAD_MODE_1 | SEL_D;
        break;
    }

    buffer[2] = decimal >> 8;
    buffer[3] = decimal & 0x00FF;

    I2C_Write(deviceHandle.i2cBase, buffer, 4);
}
*/
