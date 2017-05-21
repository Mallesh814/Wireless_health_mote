
//#include "arm_math.h"
#include "peripherals.h"

#include "parser.h"
#include "M25PXFlashMemory.h"
#include "SRAM_23LCV1024.h"
#include "ads1294Configs.h"
#include "ADS129xInfo.h"
#include "dac7573.h"
#include "communication.h"

#include "arm_math.h"
#include "math_helper.h"


/* ----------------------------------------------------------------------
** Macro Defines
** ------------------------------------------------------------------- */

#define TEST_LENGTH_SAMPLES  512
#define SNR_THRESHOLD_F32    140.0f
#define BLOCK_SIZE            128
#define NUM_TAPS              61

/* -------------------------------------------------------------------
 * The input signal and reference output (computed with MATLAB)
 * are defined externally in arm_fir_lpf_data.c.
 * ------------------------------------------------------------------- */

extern q31_t testInput_f32_1kHz_15kHz[TEST_LENGTH_SAMPLES];
extern q31_t refOutput[TEST_LENGTH_SAMPLES];

/* -------------------------------------------------------------------
 * Declare Test output buffer
 * ------------------------------------------------------------------- */

static q31_t testOutput[TEST_LENGTH_SAMPLES];

/* -------------------------------------------------------------------
 * Declare State buffer of size (numTaps + blockSize - 1)
 * ------------------------------------------------------------------- */

static q31_t firStateF32[BLOCK_SIZE + NUM_TAPS - 1];

/* ----------------------------------------------------------------------
** FIR Coefficients buffer generated using fir1() MATLAB function.
** fir1(28, 6/24)
** ------------------------------------------------------------------- */

const q31_t firCoeffs32[NUM_TAPS] = {
                                     2059646,     2328495,     2764568,     3401917,     4271626,     5400826,
                                     6811790,     8521114,    10539034,    12868869,    15506639,    18440846,
                                    21652437,    25114957,    28794887,    32652158,    36640842,    40709994,
                                    44804635,    48866849,    52836966,    56654814,    60260994,    63598160,
                                    66612273,    69253780,    71478717,    73249684,    74536684,    75317793,
                                    75579658,    75317793,    74536684,    73249684,    71478717,    69253780,
                                    66612273,    63598160,    60260994,    56654814,    52836966,    48866849,
                                    44804635,    40709994,    36640842,    32652158,    28794887,    25114957,
                                    21652437,    18440846,    15506639,    12868869,    10539034,     8521114,
                                     6811790,     5400826,     4271626,     3401917,     2764568,     2328495,
                                     2059646
                                    };

/* ------------------------------------------------------------------
 * Global variables for FIR LPF Example
 * ------------------------------------------------------------------- */

uint32_t blockSize = BLOCK_SIZE;
uint32_t numBlocks = TEST_LENGTH_SAMPLES/BLOCK_SIZE;

float32_t  snr;

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
    state = state_standby;

    uint32_t deci = 0;
    uint8_t ascii[6]="\0";
	uint8_t num[10]="\0";
	uint8_t tx_buf[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYY";
	uint8_t buffer[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};


	uint32_t i, j, k, adcHandle;
	uint32_t dcChannel, acChannel, filteredData;
	uint32_t dac_val=0;

	uint32_t rawWritePtr = 0;
    uint32_t rawReadPtr = 0;
    uint32_t filterWritePtr = 0;
    uint32_t filterReadPtr = 0;

	uint32_t sramWritePtr = 0, sramReadPtr = 0, sramData = 0, temp = 0;
	uint32_t flashWritePtr = 0, flashReadPtr = 0;
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

    arm_fir_instance_q31 S,S2;
    arm_status status;
    q31_t  *inputQ32, *outputQ32;

    /* Initialize input and output buffer pointers */
    inputQ32 = &testInput_f32_1kHz_15kHz[0];
    outputQ32 = &testOutput[0];


    /* Call FIR init function to initialize the instance structure. */
    arm_fir_init_q31(&S, NUM_TAPS, (q31_t *)&firCoeffs32[0], &firStateF32[0], blockSize);

    /* ----------------------------------------------------------------------
    ** Call the FIR process function for every blockSize samples
    ** ------------------------------------------------------------------- */

        for(i=0; i < numBlocks; i++)
        {
            arm_fir_q31(&S, inputQ32 + (i * blockSize), outputQ32 + (i * blockSize), blockSize);
        }

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

        sramReadPtr = sramWritePtr;
        SRAMReadData(buffer, 36, sramReadPtr);
        transfer("SRAM Read Data : ", debugConsole);
        transfer(buffer, debugConsole);
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
//            if(ble_data){
            if(console_event){
                console_event = 0;
                if(uart_char=='S'){
                    change_deviceState(configuring);
                    uart_char = '\0';

                }
                /*
                ble_data = 0;
                transfer("S\n\r", debugConsole);
                read_message(1000);
                transfer("\n\r", debugConsole);
                */
            }
            //change_deviceState(configuring);
            break;

        case configuring:

            adcFailCount = MAX_ADC_FAIL_COUNT;
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

            //while(1);

            rawWritePtr = RAW_DATA_BASE;
            rawReadPtr  = RAW_DATA_BASE;

            for(i=0; i < 2; i++){
                M25P_eraseSector(rawWritePtr + (i * M25P_SECTOR_SIZE));
            }

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

                M25P_programBytes(adcDataPtr, 15, rawWritePtr);
                rawWritePtr += 15;

                for (j = 0; j < 15; j++)
                    adcDataPtr[j] = 0;

                if(numberOfSamples == 0){
                    change_deviceState(filtering);
                }
            }
            break;

        case filtering:
            ADS1294_stopConv(ads1294Handle);
            TimerDisable(TIMER0_BASE, TIMER_A );
            TimerDisable(TIMER0_BASE, TIMER_B );
            GPIOPinWrite(deMuxLed.inBase, deMuxLed.inPin, 0x00); // Toggle LED0 everytime a key is pressed

            /* Initialize input and output buffer pointers */
            inputQ32 = &testInput_f32_1kHz_15kHz[0];
            outputQ32 = &testOutput[0];
            blockSize = 128;
            numBlocks = MAX_NO_OF_SAMPLES/blockSize;

            rawReadPtr  = RAW_DATA_BASE;
            filterWritePtr = FILTER_DATA_BASE;

            for(i=0; i < 2; i++){
                M25P_eraseSector(filterWritePtr + (i * M25P_SECTOR_SIZE));
            }

            transfer("Filter Initialization\n\r ", debugConsole);

            /* Call FIR init function to initialize the instance structure. */
            arm_fir_init_q31(&S2, NUM_TAPS, (q31_t *)&firCoeffs32[0], &firStateF32[0], blockSize);
            transfer("Filter Initialized\n\r ", debugConsole);

            /* ----------------------------------------------------------------------
            ** Call the FIR process function for every blockSize samples
            ** ------------------------------------------------------------------- */

            for(i=0; i < numBlocks; i++)
            {
                transfer("Filter Block\n\r", debugConsole);
                for(j=0; j < blockSize; j++){
                    M25P_readBytes(adcDataPtr, 15, rawReadPtr);
                    rawReadPtr += 15;
                    inputQ32[j] = (adcData.ch1[0] << 16) | (adcData.ch1[1] << 8) | (adcData.ch1[2]);

                    for (k = 0; k < 15; k++)
                        adcDataPtr[k] = 0;
                }
                arm_fir_q31(&S2, inputQ32, outputQ32, blockSize);

                M25P_programBytes((uint8_t *)outputQ32, (blockSize*4), filterWritePtr);
                filterWritePtr += (blockSize*4);
            }

            dec_ascii(ascii, filterWritePtr);
            transfer("Flash WritePointer At: ", debugConsole);
            transfer(ascii, debugConsole);
            transfer("\n\r", debugConsole);
            transfer("Filtering Complete\n\r", debugConsole);
            change_deviceState(data_transfer);
            break;

        case data_transfer:

            numberOfSamples = MAX_NO_OF_SAMPLES;
            rawReadPtr  = RAW_DATA_BASE;

            while(numberOfSamples){

                M25P_readBytes(adcDataPtr, 15, rawReadPtr);
                rawReadPtr += 15;
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

            numberOfSamples = MAX_NO_OF_SAMPLES;
            filterReadPtr = FILTER_DATA_BASE;

            while(numberOfSamples){
                filteredData = 0;
                M25P_readBytes((uint8_t*)&filteredData, 4, filterReadPtr);
                filterReadPtr += 4;
                numberOfSamples--;

                transfer("F10:", debugConsole);
                dec_ascii(num, filteredData);
                transfer(num, debugConsole);
                transfer("\n\r", debugConsole);
            }

            if(numberOfSamples == 0){
                transfer("Transfer Complete\n\r", debugConsole);
                change_deviceState(wait_for_ble);
            }

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
    	GPIOPinWrite(deMuxLed.inBase, deMuxLed.inPin, deMuxLed.inPin);	// Toggle LED0 everytime a key is pressed
    	break;
    case 1:
    	GPIOPinWrite(deMuxLed.selBase, deMuxLed.selPins, selIr);	// Toggle LED0 everytime a key is pressed
        GPIOPinWrite(deMuxLed.inBase, deMuxLed.inPin, deMuxLed.inPin);  // Toggle LED0 everytime a key is pressed
    	break;
    case 2:
    	GPIOPinWrite(deMuxLed.selBase, deMuxLed.selPins, sel810);	// Toggle LED0 everytime a key is pressed
        GPIOPinWrite(deMuxLed.inBase, deMuxLed.inPin, deMuxLed.inPin);  // Toggle LED0 everytime a key is pressed
    	break;
    case 3:
    	GPIOPinWrite(deMuxLed.selBase, deMuxLed.selPins, sel1300);	// Toggle LED0 everytime a key is pressed
        GPIOPinWrite(deMuxLed.inBase, deMuxLed.inPin, deMuxLed.inPin);  // Toggle LED0 everytime a key is pressed
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

