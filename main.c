
//#include "arm_math.h"
#include "peripherals.h"

#include "parser.h"
#include "M25PXFlashMemory.h"
#include "SRAM_23LCV1024.h"
#include "ads1294Configs.h"
#include "ADS129xInfo.h"
#include "dac7573.h"
#include "communication.h"

void isr_debugConsole(void);
void isr_bleConsole();
void Timer0AIntHandler(void);
void Timer0BIntHandler(void);

uint8_t t0 = 0, t1 = 0, call_parser = 0, uart_char = 0, ble_data = 0, console_event = 0;
uint32_t status;
uint8_t timer_int = 0, timer_event = 0;
uint32_t toggle = 0, conversions = 0;
uint8_t tim0 = 0, tim1 = 0, mux = 0;


void TimerConfig2(uint32_t freq, uint32_t width){
    //
    // The Timer0 peripheral must be enabled for use.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);

    //
    // The Timer0 peripheral must be enabled for use.
    //
    TimerConfigure(TIMER0_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PERIODIC | TIMER_CFG_B_ONE_SHOT);

    //
    // Set the Timer0B load value to 0.625ms.
    //
    TimerLoadSet(TIMER0_BASE, TIMER_A, SysCtlClockGet() / freq);
    TimerLoadSet(TIMER0_BASE, TIMER_B, SysCtlClockGet() / width);	// 1/Pulse Width required = 6250 for 160us

    //
    // Configure the Timer0B interrupt for timer timeout.
    //
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMB_TIMEOUT);

    TimerIntRegister(TIMER0_BASE, TIMER_A, Timer0AIntHandler);
    TimerIntRegister(TIMER0_BASE, TIMER_B, Timer0BIntHandler);
    //
    // Enable the Timer0B interrupt on the processor (NVIC).
    //
    IntEnable(INT_TIMER0A);
    IntEnable(INT_TIMER0B);

    //
    // Enable Timer0B.
    //
    TimerEnable(TIMER0_BASE, TIMER_A );
    TimerEnable(TIMER0_BASE, TIMER_B );
}



int main(void) {

    bglib_output = output;
    uint8_t dat[8] = {0,0,0,0,0,0,0,0};

    uint32_t deci = 0, ble_active = 0;
    uint8_t ascii[6]="\0";
	uint8_t num[10]="\0";
	uint8_t tx_buf[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYY";
	uint8_t rx_buf[] = {'\0','\0','\0','\0','\0','\0','\0','\0','\0','\0','\0','\0','\0','\0','\0','\0','\0'};
	uint8_t buffer[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

	uint32_t i, adcHandle;
	uint32_t averageDC = 6710886;
	uint32_t correctionRed = 6710886;
	uint32_t correctionIR = 6710886;
	uint32_t correction810 = 6710886;
	uint32_t correction1300 = 6710886;
	uint32_t dcChannel, acChannel;
	uint32_t dac_val=0;

	uint32_t sramAddrPtr = 0, sramData = 0, temp = 0, mask = 0;

    uint8 adv_data[] = {
        0x02, // field length
		gap_ad_type_flags, // field type (0x01)
        0x06, // data (0x02 | 0x04 = 0x06, general discoverable + BLE only, no BR+EDR)

/*		0x07, // field length
		gap_ad_type_localname_complete, // field type (0x01)
		'B','g','D','E','M','O',

		0x03, // field length
		gap_ad_type_services_16bit_all, // field type (0x01)
        0x09,0x18, // data (0x02 | 0x04 = 0x06, general discoverable + BLE only, no BR+EDR)
*/
		0x11, // field length
		gap_ad_type_services_128bit_all, // field type (0x07)
        0xCF, 0xC7, 0xC8, 0x43, 0x2D, 0xD9, 0x6D, 0xA9, 0x8B, 0x42, 0xA4, 0xA7, 0x4A, 0x1C, 0x43, 0x00

    };

    uint8_t sr_data[] = {
		0x07, // field length
		gap_ad_type_localname_complete, // field type (0x01)
		'B','g','D','E','M','O',

		0x03, // field length
		gap_ad_type_services_16bit_all, // field type (0x01)
		0x09,0x18, // data (0x02 | 0x04 = 0x06, general discoverable + BLE only, no BR+EDR)

    };

    //SysCtlClockSet(SYSCTL_SYSDIV_8|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ);

	configurePeripherals();

	#ifdef DEBUG
        dec_ascii(num, SysCtlClockGet());
        transfer("Clock Freq:", debugConsole);
        transfer(num, debugConsole);
        transfer("\n\r", debugConsole);

        deci = M25P_ReadID();
        dec_ascii(ascii, deci);
        transfer("FLASH Status : ", debugConsole);
        transfer(ascii, debugConsole);
        transfer("\n\r", debugConsole);

        deci = M25P_readStatus();
        dec_ascii(ascii, deci);
        transfer("FLASH Status : ", debugConsole);
        transfer(ascii, debugConsole);
        transfer("\n\r", debugConsole);

        transfer("SRAM Initialized\n\r", debugConsole);
        SRAMSetMode(SRAM_MODE_SEQUENTIAL);
        deci = SRAMReadMode();
        dec_ascii(ascii, deci);
        transfer("SRAM Mode Status : ", debugConsole);
        transfer(ascii, debugConsole);
        transfer("\n\r", debugConsole);


        SRAMWriteByte(99, sramAddrPtr);

        deci = SRAMReadByte(sramAddrPtr);
        dec_ascii(ascii, deci);
        transfer("SRAM Read Byte : ", debugConsole);
        transfer(ascii, debugConsole);
        transfer("\n\r", debugConsole);

        sramAddrPtr++;
        SRAMWriteData(tx_buf, 36, sramAddrPtr);
        SRAMReadData(buffer, 36, sramAddrPtr);
        transfer("SRAM Read Data : ", debugConsole);
        transfer(buffer, debugConsole);
        transfer("\n\r", debugConsole);
    #endif

    sramData = 0;
    sramAddrPtr = 0;

    ble_cmd_gap_set_mode(gap_general_discoverable,gap_undirected_connectable);
    change_state(state_advertising);
    adcHandle = ADS1294_Init(ads1294Handle);
    while(adcHandle == 0){
        adcHandle = ADS1294_Init(ads1294Handle);
    }

    transfer("ADC Initialized\n\r", debugConsole);


	dac_val = 0x7FF;
    dac7573_Send(driver_dac7573Handle, dac_val, selRed);
    dac7573_Send(driver_dac7573Handle, dac_val, selIr);
    dac7573_Send(driver_dac7573Handle, dac_val, sel810);
    dac7573_Send(driver_dac7573Handle, dac_val, sel1300);

    GPIOPinWrite(deMuxLed.selBase, deMuxLed.selPins, selRed);    // Toggle LED0 everytime a key is pressed
    GPIOPinWrite(deMuxLed.inBase, deMuxLed.inPin, 0XFF); // Toggle LED0 everytime a key is pressed

    /*
    for(i = 0; i < 6; i++){

        SysCtlDelay(SysCtlClockGet()/3);
        buffer[0] = RDATA;
        SPI_Read(ads1294Handle, buffer,16);

        SysCtlDelay(SysCtlClockGet()/3);

        ADS1294_readBytes((uint8_t*)&adcData, 15);

        dcChannel = (buffer[1] << 16) | (buffer[2] << 8) | (buffer[3]);
        transfer("STAT:", debugConsole);
        int_hex_ascii_big(num, dcChannel);
        transfer(num, debugConsole);
        transfer("\n\r", debugConsole);

        dcChannel = (buffer[4] << 16) | (buffer[5] << 8) | (buffer[6]);
        transfer("CH1:", debugConsole);
        int_hex_ascii_big(num, dcChannel);
        transfer(num, debugConsole);
        transfer("\n\r", debugConsole);

        dcChannel = (buffer[7] << 16) | (buffer[8] << 8) | (buffer[9]);
        transfer("CH2:", debugConsole);
        int_hex_ascii_big(num, dcChannel);
        transfer(num, debugConsole);
        transfer("\n\r", debugConsole);

        dcChannel = (buffer[10] << 16) | (buffer[11] << 8) | (buffer[12]);
        transfer("CH3:", debugConsole);
        int_hex_ascii_big(num, dcChannel);
        transfer(num, debugConsole);
        transfer("\n\r", debugConsole);

        dcChannel = (buffer[13] << 16) | (buffer[14] << 8) | (buffer[15]);
        transfer("CH4:", debugConsole);
        int_hex_ascii_big(num, dcChannel);
        transfer(num, debugConsole);
        transfer("\n\r\n\r", debugConsole);
    }

    while (1);
    */

    TimerConfig2(2000, 5000);
    transfer("Timer Started\n\r", debugConsole);

	while (1) {

    	if(ble_data){
    		ble_data = 0;
    	    transfer("S\n\r", debugConsole);
    	    read_message(1000);
    	    transfer("\n\r", debugConsole);
    	}

    	if(console_event){
    		console_event = 0;
    		switch(uart_char){
    		case 'r':	ble_cmd_system_reset(0);
    					ble_active = 0;
    					break;
    		case 'm':	ble_cmd_system_address_get();
    					break;
    		case 'g':	ble_cmd_gap_set_mode(gap_general_discoverable,gap_undirected_connectable);
    					change_state(state_advertising);
    					ble_active = 1;
    					break;
    		case 'a':	ble_cmd_gap_set_adv_data(0, 0x15, adv_data);
    					break;
    		case 's':	ble_cmd_gap_set_adv_data(1, 0x15, sr_data);
    					break;
    		case 'u':	ble_cmd_gap_set_mode(gap_user_data, gap_undirected_connectable);
    					change_state(state_advertising);
    					ble_active = 1;
    					break;
    		case 'd':	dat[0]=0x07;
						dat[1]++;
						ble_cmd_attributes_write(0x20,0,2,dat);
    					break;
    		default :	ble_cmd_system_hello();
    					break;
    		}
    	}


		if(call_parser){
			call_parser = 0;

			M25P_readBytes(rx_buf, 15, 0x0170);
		    transfer("Data Read Successful : ", debugConsole);
		    transfer(rx_buf, debugConsole);
		    transfer("\n\r", debugConsole);

		    M25P_programBytes(tx_buf, 15, 0x0170);

		    M25P_readBytes(rx_buf, 15, 0x0170);
		    transfer("Data Read Successful : ", debugConsole);
		    transfer(rx_buf, debugConsole);
		    transfer("\n\r", debugConsole);

		    M25P_eraseSector(0x0170);

		    M25P_readBytes(rx_buf, 15, 0x0170);
		    transfer("Data Read Successful : ", debugConsole);
		    transfer(rx_buf, debugConsole);
		    transfer("\n\r", debugConsole);

		    M25P_programBytes(tx_buf, 15, 0x0170);

		    M25P_readBytes(rx_buf, 15, 0x0170);
		    transfer("Data Read Successful : ", debugConsole);
		    transfer(rx_buf, debugConsole);
		    transfer("\n\r", debugConsole);

/*		    M25P_programByte(0x0002,83);

			deci = M25P_readByte(0x0002);
		    dec_ascii(deci, ascii);
		    transfer("Data Read Successful : ");
		    transfer(ascii);
		    transfer("\n\r");
*/
		}

    	if(timer_int){
			timer_int = 0;

			/*
			sramData++;
			SRAMWriteByte(sramData, sramAddrPtr);
		    sramAddrPtr += 1;

		    if (sramAddrPtr == 0x10){
		        sramData = 1;
		        sramAddrPtr = 0;
		        while(sramAddrPtr != 0x10){
	                temp = SRAMReadByte(sramAddrPtr);
                    transfer("\n\r Expected : ", debugConsole);
                    dec_ascii(num, sramData);
                    transfer(num, debugConsole);
                    transfer("\n\r Received : ", debugConsole);
                    dec_ascii(num, temp);
                    transfer(num, debugConsole);
	                if(sramData - temp){
	                    transfer("Data MisMatch in location : ", debugConsole);
	                    dec_ascii(num, sramAddrPtr);
	                    transfer(num, debugConsole);
	                    transfer("\n\r Expected : ", debugConsole);
                        dec_ascii(num, sramData);
                        transfer(num, debugConsole);
                        transfer("\n\r Received : ", debugConsole);
                        dec_ascii(num, temp);
                        transfer(num, debugConsole);
	                }
	                sramData = (temp + 1)%256;
	                sramAddrPtr++;
		        }
		        while(1);
		    }
            */

			buffer[0] = RDATA;
			SPI_Read(ads1294Handle, buffer,16);
			GPIOPinWrite(deMuxLed.inBase, deMuxLed.inPin, 0X00);	// LED Control Signal OFF

			switch(mux){
			case 0:
				dcChannel = (buffer[4] << 16) | (buffer[5] << 8) | (buffer[6]);
                acChannel = (buffer[7] << 16) | (buffer[8] << 8) | (buffer[9]);

                correctionRed = (uint32_t)((int32_t)correctionRed + (((int32_t)dcChannel - (int32_t)correctionRed) >> 9));

                dac_val = ((correctionIR >> 13) * 5);
                dac7573_Send(sensor_dac7573Handle, dac_val, sensChannel_A);

				transfer("D10:", debugConsole);
				dec_ascii(num, dcChannel);
				transfer(num, debugConsole);

				transfer(" D20:", debugConsole);
				dec_ascii(num, acChannel);
				transfer(num, debugConsole);

				/*
				decimal = dac_val << 4;
				buffer[0] = 0x4F;
				buffer[1] = 0xD0;
				buffer[2] = decimal >> 8;
				buffer[3] = decimal & 0x00FF;
				I2C_Write(dacHandle, buffer, 4);
                */

				break;
			case 1:
				dcChannel = (buffer[4] << 16) | (buffer[5] << 8) | (buffer[6]);
                acChannel = (buffer[7] << 16) | (buffer[8] << 8) | (buffer[9]);
                correctionIR = (uint32_t)((int32_t)correctionIR + (((int32_t)dcChannel - (int32_t)correctionIR) >> 9));

                dac_val = ((correction810 >> 13) * 5);
                dac7573_Send(sensor_dac7573Handle, dac_val, sensChannel_A);

				transfer("D11:", debugConsole);
				dec_ascii(num, dcChannel);
				transfer(num, debugConsole);

				transfer(" D21:", debugConsole);
				dec_ascii(num, acChannel);
				transfer(num, debugConsole);

                /*
				decimal = dac_val << 4;
				buffer[0] = 0x4F;
				buffer[1] = 0xD0;
				buffer[2] = decimal >> 8;
				buffer[3] = decimal & 0x00FF;
				I2C_Write(dacHandle, buffer, 4);
				*/

				break;
			case 2:
				dcChannel = (buffer[4] << 16) | (buffer[5] << 8) | (buffer[6]);
                acChannel = (buffer[7] << 16) | (buffer[8] << 8) | (buffer[9]);
                correction810 = (uint32_t)((int32_t)correction810 + (((int32_t)dcChannel - (int32_t)correction810 ) >> 9));

                dac_val = ((correction1300 >> 13) * 5);
                dac7573_Send(sensor_dac7573Handle, dac_val, sensChannel_B);

				transfer("D12:", debugConsole);
				dec_ascii(num, dcChannel);
				transfer(num, debugConsole);


				transfer(" D22:", debugConsole);
				dec_ascii(num, acChannel);
				transfer(num, debugConsole);

                /*
				decimal = dac_val << 4;
				buffer[0] = 0x4F;
				buffer[1] = 0xD2;
				buffer[2] = decimal >> 8;
				buffer[3] = decimal & 0x00FF;
				I2C_Write(dacHandle, buffer, 4);
				*/
				break;
			case 3:
				dcChannel = (buffer[10] << 16) | (buffer[11] << 8) | (buffer[12]);
                acChannel = (buffer[13] << 16) | (buffer[14] << 8) | (buffer[15]);
                correction1300 = (uint32_t)((int32_t)correction1300 + (((int32_t)dcChannel - (int32_t)correction1300 ) >> 9));

                dac_val = ((correctionRed >> 13) * 5);
                dac7573_Send(sensor_dac7573Handle, dac_val, sensChannel_A);

				transfer("D3:", debugConsole);
				dec_ascii(num, dcChannel);
				transfer(num, debugConsole);


				transfer(" D4:", debugConsole);
				dec_ascii(num, acChannel);
				transfer(num, debugConsole);

                /*
				decimal = dac_val << 4;
				buffer[0] = 0x4F;
				buffer[1] = 0xD0;
				buffer[2] = decimal >> 8;
				buffer[3] = decimal & 0x00FF;
				I2C_Write(dacHandle, buffer, 4);
				*/

				break;
			default : mux = 0;
				break;
			}

			transfer("\n\r", debugConsole);

			/*
			transfer("W:", debugConsole);
			dec_ascii(num, dac_val);
			transfer(num, debugConsole);
			transfer("\n\r", debugConsole);
			*/
			for (i=1; i<16; i++)
				buffer[i] = 0;
		}

	}
}


void isr_debugConsole(void)
{
	uint32_t u0status;
	u0status = UARTIntStatus(UART0_BASE,true);
	UARTIntClear(UART0_BASE,u0status);
	if(UARTCharsAvail(UART0_BASE))
		{
			uart_char = UARTCharGet(UART0_BASE);
			UARTCharPut(UART0_BASE,uart_char);
			console_event = 1;
			//call_parser = 1;
		}
}

void isr_bleConsole()
{
	uint32_t u3status;
	u3status = UARTIntStatus(bleConsole, true);
	UARTIntClear(bleConsole, u3status);
	if(UARTCharsAvail(bleConsole))
		{
			ble_data = 1;
//			GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_3|GPIO_PIN_2|GPIO_PIN_1, uart_char3);	// Toggle LED0 everytime a key is pressed
		}
}


//*****************************************************************************
//
// The interrupt handler for the Timer0B interrupt.
//
//*****************************************************************************
void
Timer0AIntHandler(void)
{
    //
    // Clear the timer interrupt flag.
    //
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

//	tim0 ^= 0xFF;
//    tim1 = 0xFF;
//    timer_int = 1;

    if(mux < 3) mux++;
    else mux = 0;
    switch(mux){
    case 0:
    	GPIOPinWrite(deMuxLed.selBase, deMuxLed.selPins, selRed);	// Toggle LED0 everytime a key is pressed
    	GPIOPinWrite(deMuxLed.inBase, deMuxLed.inPin, deMuxLed.inPin);	// Toggle LED0 everytime a key is pressed
    	break;
    case 1:
    	GPIOPinWrite(deMuxLed.selBase, deMuxLed.selPins, selIr);	// Toggle LED0 everytime a key is pressed
    	GPIOPinWrite(deMuxLed.inBase, deMuxLed.inPin, deMuxLed.inPin);	// Toggle LED0 everytime a key is pressed
    	break;
    case 2:
    	GPIOPinWrite(deMuxLed.selBase, deMuxLed.selPins, sel810);	// Toggle LED0 everytime a key is pressed
    	GPIOPinWrite(deMuxLed.inBase, deMuxLed.inPin, deMuxLed.inPin);	// Toggle LED0 everytime a key is pressed
    	break;
    case 3:
    	GPIOPinWrite(deMuxLed.selBase, deMuxLed.selPins, sel1300);	// Toggle LED0 everytime a key is pressed
    	GPIOPinWrite(deMuxLed.inBase, deMuxLed.inPin, deMuxLed.inPin);	// Toggle LED0 everytime a key is pressed
    	break;
    }
//    GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1, tim0);
//	GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_2, tim1);
    TimerEnable(TIMER0_BASE, TIMER_B );
}

void
Timer0BIntHandler(void)
{
    //
    // Clear the timer interrupt flag.
    //
    TimerIntClear(TIMER0_BASE, TIMER_TIMB_TIMEOUT);
//    tim1 = 0x00;
    timer_int = 1;
    //timer_event = 1;

//	GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_2, tim1);

//	TimerLoadSet(TIMER1_BASE, TIMER_A, SysCtlClockGet() / 500);

}

