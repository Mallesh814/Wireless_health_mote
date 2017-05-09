/*
 * ads1294Configs.c
 *
 *  Created on: Apr 20, 2017
 *      Author: edsys
 */
#include "ads1294Configs.h"
extern uint32_t debugConsole;
extern void Timer0AIntHandler(void);

uint32_t ADS1294_Init(uint32_t ui32Base, uint32_t ui32BitRate){
	uint8_t buffer[6] = {0,0,0,0,0,0};
	uint32_t status,i=0;
	char num[10]="\0";

	InitSPI(ui32Base, SSI_FRF_MOTO_MODE_1, SSI_MODE_MASTER, ui32BitRate, 8, 0);
	GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_1);
	GPIOPinWrite(GPIO_PORTD_BASE,GPIO_PIN_1, 0XFF);

	ads1294Handle.ssiBase = ui32Base;
	ads1294Handle.csPort = GPIO_PORTD_BASE;
	ads1294Handle.csPin = GPIO_PIN_1;

	i = 65536 * 50;
	while(i--);
	buffer[0] = RESET;
	status = SPI_Write(ads1294Handle, buffer, 0);
	transfer("ADS1294 Reset Command Sent \n\r", debugConsole);
	i = 65536 * 50;
	while(i--);

	buffer[0] = SDATAC;
	status = SPI_Write(ads1294Handle, buffer, 0);
	transfer("ADS1294 Stop Command Sent \n\r", debugConsole);

	buffer[0] = RREG | ID;
	buffer[1] = 0x00;
	buffer[2] = 0x00;
	SPI_Read(ads1294Handle, buffer, 2);
	status = buffer[2];
	if((status == ID_ADS1294) || (status == 128)){
		transfer("ADS1294 Initialized \n\r", debugConsole);
	    int_hex_ascii(num, status);
	    transfer(num, debugConsole);
	    transfer("\n\r", debugConsole);
	}
	else {
		transfer("ADS1294 Initialization Failed \n\r", debugConsole);
	    int_hex_ascii(num, status);
	    transfer(num, debugConsole);
	    transfer("\n\r", debugConsole);
		return 0;
	}


	buffer[0] = WREG | CONFIG3;
	buffer[1] = 0x00;
	buffer[2] = CONFIG3_const | PD_REFBUF | VREF_4V;
	status = SPI_Write(ads1294Handle, buffer, 2);

	buffer[0] = RREG | CONFIG3;
	buffer[1] = 0x00;
	buffer[2] = 0x00;
	SPI_Read(ads1294Handle, buffer, 2);
	status = buffer[2];
    transfer("ADs1294 Config 3 register: ", debugConsole);
    int_hex_ascii(num, status);
    transfer(num, debugConsole);
    transfer("\n\r", debugConsole);

    /*
	buffer[0] = WREG | GPIO;
	buffer[1] = 0x00;
	buffer[2] = GPIO_const;
	status = SPI_Write(ads1294Handle, buffer, 2);

	buffer[0] = RREG | GPIO;
	buffer[1] = 0x00;
	buffer[2] = 0x00;
	SPI_Read(ads1294Handle, buffer, 2);
	status = buffer[2];
    transfer("ADs1294 GPIO register: ", debugConsole);
    int_hex_ascii(num, status);
    transfer(num, debugConsole);
    transfer("\n\r", debugConsole);
	*/

    buffer[0] = WREG | CONFIG1;
	buffer[1] = 0x00;
	buffer[2] = CONFIG1_const | HIGH_RES_32k_SPS | DAISY_EN;
	status = SPI_Write(ads1294Handle, buffer, 2);

	buffer[0] = RREG | CONFIG1;
	buffer[1] = 0x00;
	buffer[2] = 0x00;
	SPI_Read(ads1294Handle, buffer, 2);
	status = buffer[2];
    transfer("ADs1294 Config 1 register: ", debugConsole);
    int_hex_ascii(num, status);
    transfer(num, debugConsole);
    transfer("\n\r", debugConsole);



	buffer[0] = WREG | CONFIG2;
	buffer[1] = 0x00;
	buffer[2] = CONFIG2_const | INT_TEST_2HZ;
	status = SPI_Write(ads1294Handle, buffer, 2);

	buffer[0] = RREG | CONFIG2;
	buffer[1] = 0x00;
	buffer[2] = 0x00;
	SPI_Read(ads1294Handle, buffer, 2);
	status = buffer[2];
    transfer("ADs1294 Config 2 register: ", debugConsole);
    int_hex_ascii(num, status);
    transfer(num, debugConsole);
    transfer("\n\r", debugConsole);

    transfer("ADS1294 Config Commands Sent \n\r", debugConsole);

	buffer[0] = WREG | CH1SET;
	buffer[1] = 0x03;
	buffer[2] = CHnSET_const | ELECTRODE_INPUT | GAIN_X1;
	buffer[3] = CHnSET_const | ELECTRODE_INPUT | GAIN_X1;
	buffer[4] = CHnSET_const | ELECTRODE_INPUT | GAIN_X1;
	buffer[5] = CHnSET_const | ELECTRODE_INPUT | GAIN_X1;
	status = SPI_Write(ads1294Handle, buffer, 5);
	transfer("ADS1294 Channel Commands Sent \n\r", debugConsole);

	buffer[0] = RREG | CH1SET;
	buffer[1] = 0x03;
	buffer[2] = 0x00;
	buffer[3] = 0x00;
	buffer[4] = 0x00;
	buffer[5] = 0x00;
	SPI_Read(ads1294Handle, buffer, 5);
	for(i=2;i<6;i++){
		status = buffer[i];
	    transfer("ADs1294 Channel register: ", debugConsole);
	    int_hex_ascii(num, status);
	    transfer(num, debugConsole);
	    transfer("\n\r", debugConsole);
	}

	buffer[0] = STARTCON;
	status = SPI_Write(ads1294Handle, buffer, 0);
	transfer("ADS1294 Start Conversion Command Sent \n\r", debugConsole);

/*	buffer[0] = RDATAC;
	status = SPI_Write(ads1294Handle, buffer, 0);
	transfer("ADS1294 Read Data Continuous Command Sent \n\r", debugConsole);
*/
	return ui32Base;
}

void TimerConfig(uint32_t freq){

    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);

    //
    // The Timer0 peripheral must be enabled for use.
    //
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);

    //
    // Set the Timer0B load value to 0.625ms.
    //
    TimerLoadSet(TIMER0_BASE, TIMER_A, SysCtlClockGet() / freq);

    //
    // Configure the Timer0B interrupt for timer timeout.
    //
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    TimerIntRegister(TIMER0_BASE, TIMER_A, Timer0AIntHandler);
    //
    // Enable the Timer0B interrupt on the processor (NVIC).
    //
    IntEnable(INT_TIMER0A);
    //
    // Enable Timer0B.
    //
    TimerEnable(TIMER0_BASE, TIMER_A );

}


