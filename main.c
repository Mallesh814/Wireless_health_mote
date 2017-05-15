
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
    // The Timer0 peripheral must be enabled for use.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);

    // The Timer0 peripheral must be enabled for use.
    TimerConfigure(TIMER0_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PERIODIC | TIMER_CFG_B_ONE_SHOT);

    // Set the Timer0B load value to 0.625ms.
    TimerLoadSet(TIMER0_BASE, TIMER_A, SysCtlClockGet() / freq);
    TimerLoadSet(TIMER0_BASE, TIMER_B, SysCtlClockGet() / width);	// 1/Pulse Width required = 6250 for 160us

    // Configure the Timer0B interrupt for timer timeout.
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMB_TIMEOUT);

    TimerIntRegister(TIMER0_BASE, TIMER_A, Timer0AIntHandler);
    TimerIntRegister(TIMER0_BASE, TIMER_B, Timer0BIntHandler);
    // Enable the Timer0B interrupt on the processor (NVIC).
    IntEnable(INT_TIMER0A);
    IntEnable(INT_TIMER0B);
    // Enable Timer0B.
    TimerEnable(TIMER0_BASE, TIMER_A );
    TimerEnable(TIMER0_BASE, TIMER_B );
}



int main(void) {

    deviceState = initialize;

    bglib_output = output;
    uint8_t dat[8] = {0,0,0,0,0,0,0,0};

    uint32_t deci = 0, ble_active = 0;
    uint8_t ascii[6]="\0";
	uint8_t num[10]="\0";
	uint8_t tx_buf[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYY";
	uint8_t rx_buf[] = {'\0','\0','\0','\0','\0','\0','\0','\0','\0','\0','\0','\0','\0','\0','\0','\0','\0'};
	uint8_t buffer[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

	uint32_t i, j, adcHandle;
	uint32_t dcChannel, acChannel;
	uint32_t dac_val=0;

	uint32_t sramWritePtr = 0, sramReadPtr = 0, sramData = 0, temp = 0;
	uint32_t numberOfSamples = 0;

	struct ads1294DataStruct adcData;
	uint8_t *adcDataPtr = (uint8_t *)&adcData;
	uint8_t adcFailCount = MAX_ADC_FAIL_COUNT;

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

    SysCtlClockSet(SYSCTL_SYSDIV_8|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ);

	configurePeripherals();

	#ifdef DEBUG
        dec_ascii(num, SysCtlClockGet());
        transfer("Clock Freq:", debugConsole);
        transfer(num, debugConsole);
        transfer("\n\r", debugConsole);

        deci = M25P_ReadID();
        dec_ascii(ascii, deci);
        transfer("FLASH ID : ", debugConsole);
        transfer(ascii, debugConsole);
        transfer("\n\r", debugConsole);

        deci = M25P_readStatus();
        dec_ascii(ascii, deci);
        transfer("FLASH Status : ", debugConsole);
        transfer(ascii, debugConsole);
        transfer("\n\r", debugConsole);

        SRAMSetMode(SRAM_MODE_SEQUENTIAL);
        transfer("SRAM Initialized\n\r", debugConsole);
        deci = SRAMReadMode();
        dec_ascii(ascii, deci);
        transfer("SRAM Mode : ", debugConsole);
        transfer(ascii, debugConsole);
        transfer("\n\r", debugConsole);

        SRAMWriteByte(99, sramWritePtr);
        deci = SRAMReadByte(sramReadPtr);
        dec_ascii(ascii, deci);
        transfer("SRAM Read Byte : ", debugConsole);
        transfer(ascii, debugConsole);
        transfer("\n\r", debugConsole);

        sramWritePtr++;
        SRAMWriteData(tx_buf, 36, sramWritePtr);

        sramReadPtr = sramWritePtr;
        SRAMReadData(buffer, 36, sramReadPtr);
        transfer("SRAM Read Data : ", debugConsole);
        transfer(buffer, debugConsole);
        transfer("\n\r", debugConsole);
    #endif

    sramData = 0;
    sramWritePtr = 0;
    sramReadPtr = 0;

    ble_cmd_gap_set_mode(gap_general_discoverable,gap_undirected_connectable);
    change_state(state_advertising);

    change_deviceState(wait_for_ble);

    while(1){
        switch(deviceState){
        case wait_for_ble:
            if(ble_data){
                ble_data = 0;
                transfer("S\n\r", debugConsole);
                read_message(1000);
                transfer("\n\r", debugConsole);
            }
            change_deviceState(configuring);
            break;

        case configuring:

            do{
                adcHandle = ADS1294_Init(ads1294Handle);
                adcFailCount--;
            }while((adcHandle == 0) & (adcFailCount != 0));

            if(adcHandle != 0)
                transfer("ADC Initialized\n\r", debugConsole);
            else{
                transfer("ADC Initialization Failed Check Cable\n\r", debugConsole);
                while(1);
            }

            dac_val = 0x7FF;
            dac7573_Send(driver_dac7573Handle, dac_val, selRed);
            dac7573_Send(driver_dac7573Handle, dac_val, selIr);
            dac7573_Send(driver_dac7573Handle, dac_val, sel810);
            dac7573_Send(driver_dac7573Handle, dac_val, sel1300);

            GPIOPinWrite(deMuxLed.selBase, deMuxLed.selPins, selRed);    // Toggle LED0 everytime a key is pressed
            GPIOPinWrite(deMuxLed.inBase, deMuxLed.inPin, deMuxLed.inPin); // Toggle LED0 everytime a key is pressed

            sramWritePtr = 0;
            sramReadPtr = 0;
            numberOfSamples = MAX_NO_OF_SAMPLES;
            change_deviceState(siganl_acquisition);

            TimerConfig2(2000, 5000);
            transfer("Timer Started\n\r", debugConsole);
            break;

        case siganl_acquisition:

            if(timer_int & (numberOfSamples != 0)){
                timer_int = 0;

                ADS1294_readBytes(adcDataPtr, 15);
                GPIOPinWrite(deMuxLed.inBase, deMuxLed.inPin, 0x00);  // Toggle LED0 everytime a key is pressed
                numberOfSamples--;

                acChannel = (adcData.ch1[0] << 16) | (adcData.ch1[1] << 8) | (adcData.ch1[2]);
                transfer("D10:", debugConsole);
                dec_ascii(num, acChannel);
                transfer(num, debugConsole);

                acChannel = (adcData.ch2[0] << 16) | (adcData.ch2[1] << 8) | (adcData.ch2[2]);
                transfer(" D11:", debugConsole);
                dec_ascii(num, acChannel);
                transfer(num, debugConsole);

                acChannel = (adcData.ch3[0] << 16) | (adcData.ch3[1] << 8) | (adcData.ch3[2]);
                transfer(" D12:", debugConsole);
                dec_ascii(num, acChannel);
                transfer(num, debugConsole);

                acChannel = (adcData.ch4[0] << 16) | (adcData.ch4[1] << 8) | (adcData.ch4[2]);
                transfer(" D3:", debugConsole);
                dec_ascii(num, acChannel);
                transfer(num, debugConsole);
                transfer("\n\r", debugConsole);

                for (j = 0; j < 15; j++)
                    adcDataPtr[j] = 0;
            }
            break;

        case data_transfer:
            SRAMSetMode(SRAM_MODE_SEQUENTIAL);
            transfer("SRAM Initialized\n\r", debugConsole);
            deci = SRAMReadMode();
            dec_ascii(ascii, deci);
            transfer("SRAM Mode : ", debugConsole);

            SRAMReadData(buffer, 36, sramReadPtr);
            transfer("SRAM Read Data : ", debugConsole);
            transfer(buffer, debugConsole);
            transfer("\n\r", debugConsole);

            break;
        default:
            while(1);
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
    // Clear the timer interrupt flag.
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);


    if(mux < 3) mux++;
    else mux = 0;

    switch(mux){
    case 0:
    	GPIOPinWrite(deMuxLed.selBase, deMuxLed.selPins, selRed);	// Toggle LED0 everytime a key is pressed
    	GPIOPinWrite(deMuxLed.inBase, deMuxLed.inPin, 0x00);	// Toggle LED0 everytime a key is pressed
    	break;
    case 1:
    	GPIOPinWrite(deMuxLed.selBase, deMuxLed.selPins, selIr);	// Toggle LED0 everytime a key is pressed
    	GPIOPinWrite(deMuxLed.inBase, deMuxLed.inPin, 0x00);	// Toggle LED0 everytime a key is pressed
    	break;
    case 2:
    	GPIOPinWrite(deMuxLed.selBase, deMuxLed.selPins, sel810);	// Toggle LED0 everytime a key is pressed
    	GPIOPinWrite(deMuxLed.inBase, deMuxLed.inPin, 0x00);	// Toggle LED0 everytime a key is pressed
    	break;
    case 3:
    	GPIOPinWrite(deMuxLed.selBase, deMuxLed.selPins, sel1300);	// Toggle LED0 everytime a key is pressed
    	GPIOPinWrite(deMuxLed.inBase, deMuxLed.inPin, 0x00);	// Toggle LED0 everytime a key is pressed
    	break;
    }
    TimerEnable(TIMER0_BASE, TIMER_B );
}

void
Timer0BIntHandler(void)
{
    // Clear the timer interrupt flag.
    TimerIntClear(TIMER0_BASE, TIMER_TIMB_TIMEOUT);
    timer_int = 1;
}

