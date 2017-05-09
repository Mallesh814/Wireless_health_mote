/*
 * configs.c
 *
 *  Created on: 02-Mar-2016
 *      Author: Mallesh
 */
#include "configs.h"
//*****************************************************************************
// Values that can be passed as ui32Protocol.
//*****************************************************************************
/*
	SSI_FRF_MOTO_MODE_0	// Moto fmt, polarity 0, phase 0
	SSI_FRF_MOTO_MODE_1	// Moto fmt, polarity 0, phase 1
	SSI_FRF_MOTO_MODE_2	// Moto fmt, polarity 1, phase 0
	SSI_FRF_MOTO_MODE_3	// Moto fmt, polarity 1, phase 1
	SSI_FRF_TI			// TI frame format
	SSI_FRF_NMW			// National MicroWire frame format

// Values that can be passed as ui32Mode.

	SSI_MODE_MASTER		// SSI master
	SSI_MODE_SLAVE		// SSI slave
	SSI_MODE_SLAVE_OD	// SSI slave with output disabled
*/

void InitSPI(uint32_t ui32Base, uint32_t ui32Protocol, uint32_t ui32Mode, uint32_t ui32BitRate, uint32_t ui32DataWidth, bool enfss){

	uint32_t SYSCTL_PERIPH_SPI, SYSCTL_PERIPH_GPIO;
	uint32_t CONF_PIN_CLK, CONF_PIN_FSS, CONF_PIN_TX, CONF_PIN_RX;
	uint32_t GPIO_PORT_BASE;
	uint32_t PIN_CLK, PIN_FSS, PIN_TX, PIN_RX;
	uint32_t pui32Dummy;


	switch(ui32Base)
	{
		case SSI0_BASE:	SYSCTL_PERIPH_SPI  =  SYSCTL_PERIPH_SSI0;
						SYSCTL_PERIPH_GPIO =  SYSCTL_PERIPH_GPIOA;

						CONF_PIN_CLK = GPIO_PA2_SSI0CLK;
						CONF_PIN_FSS = GPIO_PA3_SSI0FSS;
						CONF_PIN_RX = GPIO_PA4_SSI0RX;
						CONF_PIN_TX = GPIO_PA5_SSI0TX;

						GPIO_PORT_BASE = GPIO_PORTA_BASE;

						PIN_CLK = GPIO_PIN_2;
						PIN_FSS = GPIO_PIN_3;
						PIN_RX = GPIO_PIN_4;
						PIN_TX = GPIO_PIN_5;

						break;

		case SSI1_BASE:	SYSCTL_PERIPH_SPI  =  SYSCTL_PERIPH_SSI1;
						SYSCTL_PERIPH_GPIO =  SYSCTL_PERIPH_GPIOF;

						CONF_PIN_CLK = GPIO_PF2_SSI1CLK;
						CONF_PIN_FSS = GPIO_PF3_SSI1FSS;
						CONF_PIN_RX = GPIO_PF0_SSI1RX;
						CONF_PIN_TX = GPIO_PF1_SSI1TX;

						GPIO_PORT_BASE = GPIO_PORTF_BASE;

						PIN_CLK = GPIO_PIN_2;
						PIN_FSS = GPIO_PIN_3;
						PIN_RX = GPIO_PIN_0;
						PIN_TX = GPIO_PIN_1;

						break;

		case SSI2_BASE:	SYSCTL_PERIPH_SPI  =  SYSCTL_PERIPH_SSI2;
						SYSCTL_PERIPH_GPIO =  SYSCTL_PERIPH_GPIOB;

						CONF_PIN_CLK = GPIO_PB4_SSI2CLK;
						CONF_PIN_FSS = GPIO_PB5_SSI2FSS;
						CONF_PIN_RX = GPIO_PB6_SSI2RX;
						CONF_PIN_TX = GPIO_PB7_SSI2TX;

						GPIO_PORT_BASE = GPIO_PORTB_BASE;

						PIN_CLK = GPIO_PIN_4;
						PIN_FSS = GPIO_PIN_5;
						PIN_RX = GPIO_PIN_6;
						PIN_TX = GPIO_PIN_7;
						break;

		case SSI3_BASE:	SYSCTL_PERIPH_SPI  =  SYSCTL_PERIPH_SSI3;
						SYSCTL_PERIPH_GPIO =  SYSCTL_PERIPH_GPIOD;

						CONF_PIN_CLK = GPIO_PD0_SSI3CLK;
						CONF_PIN_FSS = GPIO_PD1_SSI3FSS;
						CONF_PIN_RX = GPIO_PD2_SSI3RX;
						CONF_PIN_TX = GPIO_PD3_SSI3TX;

						GPIO_PORT_BASE = GPIO_PORTD_BASE;

						PIN_CLK = GPIO_PIN_0;
						PIN_FSS = GPIO_PIN_1;
						PIN_RX = GPIO_PIN_2;
						PIN_TX = GPIO_PIN_3;
						break;

		default:		while(1);

	}

    // The SSI peripheral must be enabled for use.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SPI);
	//Wait Until SPI is Ready
	while(!(SysCtlPeripheralReady(SYSCTL_PERIPH_SPI)));


    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIO);
	//Wait Until GPIO is Ready
	while(!(SysCtlPeripheralReady(SYSCTL_PERIPH_GPIO)));

    // Configure the pin muxing for SSI1 functions on port D0, D1, D2 and D3.
    // This step is not necessary if your part does not support pin muxing.

	GPIOPinConfigure(CONF_PIN_CLK);
    GPIOPinConfigure(CONF_PIN_RX);
    GPIOPinConfigure(CONF_PIN_TX);

	if(enfss){
		GPIOPinConfigure(CONF_PIN_FSS);
	    GPIOPinTypeSSI(GPIO_PORT_BASE, PIN_CLK | PIN_FSS | PIN_RX | PIN_TX);
	}
	else {
	    GPIOPinTypeSSI(GPIO_PORT_BASE, PIN_CLK | PIN_RX | PIN_TX);
	}

    // Configure and enable the SSI port for SPI master mode.  Use SSI,
    // system clock supply, idle clock level low and active low clock in
    // freescale SPI mode, master mode, 1MHz SSI frequency, and 8-bit data.
    // For SPI mode, you can set the polarity of the SSI clock when the SSI
    // unit is idle.  You can also configure what clock edge you want to
    // capture data on.  Please reference the datasheet for more information on
    // the different SPI modes.
    //

    SSIConfigSetExpClk(ui32Base, SysCtlClockGet(), ui32Protocol, ui32Mode, ui32BitRate, ui32DataWidth);
    SSIEnable(ui32Base);
    while(SSIDataGetNonBlocking(ui32Base, &pui32Dummy));
}





uint32_t InitI2C(uint32_t BASE, bool mode)
{
	uint32_t SYSCTL_PERIPH_I2C, SYSCTL_PERIPH_GPIO;
	uint32_t CONF_PIN_SCL, CONF_PIN_SDA;
	uint32_t GPIO_PORT_BASE;
	uint32_t PIN_SCL, PIN_SDA;
	switch(BASE)
	{
		case I2C0_BASE:	SYSCTL_PERIPH_I2C  =  SYSCTL_PERIPH_I2C0;
						SYSCTL_PERIPH_GPIO =  SYSCTL_PERIPH_GPIOB;
						CONF_PIN_SCL = GPIO_PB2_I2C0SCL;
						CONF_PIN_SDA = GPIO_PB3_I2C0SDA;
						GPIO_PORT_BASE = GPIO_PORTB_BASE;
						PIN_SCL = GPIO_PIN_2;
						PIN_SDA = GPIO_PIN_3;
						break;

		case I2C1_BASE:	SYSCTL_PERIPH_I2C  =  SYSCTL_PERIPH_I2C1;
						SYSCTL_PERIPH_GPIO =  SYSCTL_PERIPH_GPIOA;
						CONF_PIN_SCL = GPIO_PA6_I2C1SCL;
						CONF_PIN_SDA = GPIO_PA7_I2C1SDA;
						GPIO_PORT_BASE = GPIO_PORTA_BASE;
						PIN_SCL = GPIO_PIN_6;
						PIN_SDA = GPIO_PIN_7;
						break;

		case I2C2_BASE:	SYSCTL_PERIPH_I2C  =  SYSCTL_PERIPH_I2C2;
						SYSCTL_PERIPH_GPIO =  SYSCTL_PERIPH_GPIOE;
						CONF_PIN_SCL = GPIO_PE4_I2C2SCL;
						CONF_PIN_SDA = GPIO_PE5_I2C2SDA;
						GPIO_PORT_BASE = GPIO_PORTE_BASE;
						PIN_SCL = GPIO_PIN_4;
						PIN_SDA = GPIO_PIN_5;
						break;

		case I2C3_BASE:	SYSCTL_PERIPH_I2C  =  SYSCTL_PERIPH_I2C3;
						SYSCTL_PERIPH_GPIO =  SYSCTL_PERIPH_GPIOD;
						CONF_PIN_SCL = GPIO_PD0_I2C3SCL;
						CONF_PIN_SDA = GPIO_PD1_I2C3SDA;
						GPIO_PORT_BASE = GPIO_PORTD_BASE;
						PIN_SCL = GPIO_PIN_0;
						PIN_SDA = GPIO_PIN_1;
						break;

		default:		while(1);

	}

	// Enable I2C
	SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C);
	//Wait Until I2C is Ready
	while(!(SysCtlPeripheralReady(SYSCTL_PERIPH_I2C)));

	//Enable GPIO
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIO);
	//Wait Until GPIO is Ready
	while(!(SysCtlPeripheralReady(SYSCTL_PERIPH_GPIO)));


	GPIOPinConfigure(CONF_PIN_SCL);
	GPIOPinConfigure(CONF_PIN_SDA);

	GPIOPinTypeI2C(GPIO_PORT_BASE, PIN_SDA);
	GPIOPinTypeI2CSCL(GPIO_PORT_BASE, PIN_SCL);

	I2CMasterInitExpClk(BASE,SysCtlClockGet(),mode);

	return BASE;
}

uint32_t InitConsole(uint32_t BASE, uint32_t baudRate)
{

	uint32_t SYSCTL_PERIPH_UART, SYSCTL_PERIPH_GPIO;
	uint32_t CONF_PIN_RX, CONF_PIN_TX;
	uint32_t GPIO_PORT_BASE;
	uint32_t PIN_RX, PIN_TX;
	switch(BASE)
	{
		case UART0_BASE:SYSCTL_PERIPH_UART  =  SYSCTL_PERIPH_UART0;
						SYSCTL_PERIPH_GPIO =  SYSCTL_PERIPH_GPIOA;
						CONF_PIN_RX = GPIO_PA0_U0RX;
						CONF_PIN_TX = GPIO_PA1_U0TX;
						GPIO_PORT_BASE = GPIO_PORTA_BASE;
						PIN_RX = GPIO_PIN_0;
						PIN_TX = GPIO_PIN_1;
						break;

		case UART1_BASE:SYSCTL_PERIPH_UART  =  SYSCTL_PERIPH_UART1;
						SYSCTL_PERIPH_GPIO =  SYSCTL_PERIPH_GPIOB;
						CONF_PIN_RX = GPIO_PB0_U1RX;
						CONF_PIN_TX = GPIO_PB1_U1TX;
						GPIO_PORT_BASE = GPIO_PORTB_BASE;
						PIN_RX = GPIO_PIN_0;
						PIN_TX = GPIO_PIN_1;
						break;

		case UART2_BASE:SYSCTL_PERIPH_UART  =  SYSCTL_PERIPH_UART2;
						SYSCTL_PERIPH_GPIO =  SYSCTL_PERIPH_GPIOD;
						CONF_PIN_RX = GPIO_PD6_U2RX;
						CONF_PIN_TX = GPIO_PD7_U2TX;
						GPIO_PORT_BASE = GPIO_PORTD_BASE;
						PIN_RX = GPIO_PIN_6;
						PIN_TX = GPIO_PIN_7;
						break;

		case UART3_BASE:SYSCTL_PERIPH_UART  =  SYSCTL_PERIPH_UART3;
						SYSCTL_PERIPH_GPIO =  SYSCTL_PERIPH_GPIOC;
						CONF_PIN_RX = GPIO_PC6_U3RX;
						CONF_PIN_TX = GPIO_PC7_U3TX;
						GPIO_PORT_BASE = GPIO_PORTC_BASE;
						PIN_RX = GPIO_PIN_6;
						PIN_TX = GPIO_PIN_7;
						break;

		case UART4_BASE:SYSCTL_PERIPH_UART  =  SYSCTL_PERIPH_UART4;
						SYSCTL_PERIPH_GPIO =  SYSCTL_PERIPH_GPIOC;
						CONF_PIN_RX = GPIO_PC4_U4RX;
						CONF_PIN_TX = GPIO_PC5_U4TX;
						GPIO_PORT_BASE = GPIO_PORTC_BASE;
						PIN_RX = GPIO_PIN_4;
						PIN_TX = GPIO_PIN_5;
						break;

		case UART5_BASE:SYSCTL_PERIPH_UART  =  SYSCTL_PERIPH_UART5;
						SYSCTL_PERIPH_GPIO =  SYSCTL_PERIPH_GPIOE;
						CONF_PIN_RX = GPIO_PE4_U5RX;
						CONF_PIN_TX = GPIO_PE5_U5TX;
						GPIO_PORT_BASE = GPIO_PORTE_BASE;
						PIN_RX = GPIO_PIN_4;
						PIN_TX = GPIO_PIN_5;
						break;

		case UART6_BASE:SYSCTL_PERIPH_UART  =  SYSCTL_PERIPH_UART6;
						SYSCTL_PERIPH_GPIO =  SYSCTL_PERIPH_GPIOD;
						CONF_PIN_RX = GPIO_PD4_U6RX;
						CONF_PIN_TX = GPIO_PD5_U6TX;
						GPIO_PORT_BASE = GPIO_PORTD_BASE;
						PIN_RX = GPIO_PIN_4;
						PIN_TX = GPIO_PIN_5;
						break;

		case UART7_BASE:SYSCTL_PERIPH_UART  =  SYSCTL_PERIPH_UART7;
						SYSCTL_PERIPH_GPIO =  SYSCTL_PERIPH_GPIOE;
						CONF_PIN_RX = GPIO_PE0_U7RX;
						CONF_PIN_TX = GPIO_PE1_U7TX;
						GPIO_PORT_BASE = GPIO_PORTE_BASE;
						PIN_RX = GPIO_PIN_0;
						PIN_TX = GPIO_PIN_1;
						break;

		default:		while(1);
	}


	//
    // Enable GPIO port A which is used for UART0 pins.
    // TODO: change this to whichever GPIO port you are using.
    //
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIO);

	//
    // Enable UART0 so that we can configure the clock.
    //
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART);

    //
    // Configure the pin muxing for UART0 functions on port A0 and A1.
    // This step is not necessary if your part does not support pin muxing.
    // TODO: change this to select the port/pin you are using.
    //
	GPIOPinConfigure(CONF_PIN_RX);
	GPIOPinConfigure(CONF_PIN_TX);

    //
    // Select the alternate (UART) function for these pins.
    // TODO: change this to select the port/pin you are using.
    //
	GPIOPinTypeUART(GPIO_PORT_BASE,PIN_RX | PIN_TX);

    //
    // Initialize the UART for console I/O.
    //
	UARTConfigSetExpClk(BASE, SysCtlClockGet(), baudRate,(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
	UARTFIFODisable(BASE);

	return BASE;

}

