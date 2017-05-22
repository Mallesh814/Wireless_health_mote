
//#include "arm_math.h"
#include "peripherals.h"
#include "ble113.h"
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
 * Declare State buffer of size (numTaps + blockSize - 1)
 * ------------------------------------------------------------------- */

static q31_t firStateF32[BLOCK_SIZE + NUM_TAPS - 1];

static q31_t firStateSignal[BLOCK_SIZE + NUM_TAPS - 1];
static q31_t firStateAmbient[BLOCK_SIZE + NUM_TAPS - 1];
static q31_t firStateAlt[BLOCK_SIZE + NUM_TAPS - 1];
static q31_t firStateAltAmb[BLOCK_SIZE + NUM_TAPS - 1];

static q31_t signalInputBuffer[BLOCK_SIZE];
static q31_t signalOutputBuffer[BLOCK_SIZE];
static q31_t ambientInputBuffer[BLOCK_SIZE];
static q31_t ambientOutputBuffer[BLOCK_SIZE];
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

extern void (*bglib_output)(uint8 len1,uint8* data1,uint16 len2,uint8* data2);
extern void output(uint8 , uint8* , uint16 , uint8* );


int main(void) {

    deviceState = initialize;

    bglib_output = output;
    state = state_poweroff;

    uint32_t deci = 0;
    uint8_t ascii[6]="\0";
	uint8_t num[10]="\0";
	uint8_t tx_buf[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYY";
	uint8_t buffer[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};


	uint32_t i, j, k, l, p, adcHandle;
	uint32_t acChannel, filteredData;
	uint32_t dac_val=0;

	uint32_t rawDataPtrs[4];
	rawDataPtrs[selRed] = RAW_RED_BASE;
    rawDataPtrs[selIr]  = RAW_IR_BASE;
    rawDataPtrs[sel810] = RAW_810_BASE;
    rawDataPtrs[sel1300]= RAW_1300_BASE;

    struct filterPtrStruct filterDataStructs[4];

    filterDataStructs[selRed].signalPtr = FIR_RED_SIGNAL_BASE;
    filterDataStructs[selRed].ambientPtr = FIR_RED_AMBIENT_BASE;
    filterDataStructs[selIr].signalPtr = FIR_IR_SIGNAL_BASE;
    filterDataStructs[selIr].ambientPtr = FIR_IR_AMBIENT_BASE;
    filterDataStructs[sel810].signalPtr = FIR_810_SIGNAL_BASE;
    filterDataStructs[sel810].ambientPtr = FIR_810_AMBIENT_BASE;
    filterDataStructs[sel1300].signalPtr = FIR_1300_SIGNAL_BASE;
    filterDataStructs[sel1300].ambientPtr = FIR_1300_AMBIENT_BASE;

	uint32_t sramWritePtr = 0, sramReadPtr = 0;
	uint32_t numberOfSamples = 0;

	uint8_t *adcDataPtr = (uint8_t *)&adcData;
    uint8_t *channelDataPtr = (uint8_t *)&channelData;

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

    arm_fir_instance_q31 SignalState, AmbientState;
    q31_t  *SignalInput, *SignalOutput;
    q31_t  *AmbientInput, *AmbientOutput;

    /* Initialize input and output buffer pointers */
    SignalInput = &signalInputBuffer[0];
    SignalOutput = &signalOutputBuffer[0];

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

    sramWritePtr = 0;
    sramReadPtr = 0;

    //ble_cmd_gap_set_mode(gap_general_discoverable,gap_undirected_connectable);
    //change_state(state_advertising);

    change_deviceState(wait_for_ble);

    while(1){
        switch(deviceState){
        case wait_for_ble:
            if(console_event){
                console_event = 0;
                if(uart_char=='S'){
                    change_deviceState(configuring);
                    uart_char = '\0';

                }
            }

            if(ble_data){
                ble_data = 0;
                transfer("D\n\r", debugConsole);
                read_message(1000);
                transfer("\n\r", debugConsole);
                if (get_state() == state_standby){
                    ble_cmd_gap_set_mode(gap_general_discoverable,gap_undirected_connectable);
                    change_state(state_advertising);
                }
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
            dac_val = 0x7FF;
            dac7573_Send(driver_dac7573Handle, dac_val, selIr);
            dac_val = 0x7FF;
            dac7573_Send(driver_dac7573Handle, dac_val, sel810);
            dac_val = 0x7FF;
            dac7573_Send(driver_dac7573Handle, dac_val, sel1300);

            GPIOPinWrite(deMuxLed.selBase, deMuxLed.selPins, selRed);    // Toggle LED0 everytime a key is pressed
            GPIOPinWrite(deMuxLed.inBase, deMuxLed.inPin, deMuxLed.inPin); // Toggle LED0 everytime a key is pressed

            rawDataPtrs[selRed] = RAW_RED_BASE;
            rawDataPtrs[selIr]  = RAW_IR_BASE;
            rawDataPtrs[sel810] = RAW_810_BASE;
            rawDataPtrs[sel1300]= RAW_1300_BASE;


            for (j = 0 ; j < 4; j++){
                for(i = 0; i < 2; i++){
                    M25P_eraseSector(rawDataPtrs[j] + (i * M25P_SECTOR_SIZE));
                }
            }

            for (j = 0; j < 15; j++)
                adcDataPtr[j] = 0;

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

                switch(mux){
                case 0:
                    channelData.rawMain     =  (adcData.ch1[0] << 16) | (adcData.ch1[1] << 8) | (adcData.ch1[2]);
                    channelData.rawAlt      =  (adcData.ch3[0] << 16) | (adcData.ch3[1] << 8) | (adcData.ch3[2]);
                    M25P_programBytes(channelDataPtr, 16, rawDataPtrs[selRed]);
                    rawDataPtrs[selRed] += 16;

                    transfer("TR:", debugConsole);
                    dec_ascii(num, channelData.rawMain);
                    transfer(num, debugConsole);

                    transfer("\n\r", debugConsole);
                    break;
                case 1:
                    channelData.rawMain     =  (adcData.ch1[0] << 16) | (adcData.ch1[1] << 8) | (adcData.ch1[2]);
                    channelData.rawAlt      =  (adcData.ch3[0] << 16) | (adcData.ch3[1] << 8) | (adcData.ch3[2]);
                    M25P_programBytes(channelDataPtr, 16, rawDataPtrs[selIr]);
                    rawDataPtrs[selIr] += 16;
                    break;
                case 2:
                    channelData.rawMain     =  (adcData.ch1[0] << 16) | (adcData.ch1[1] << 8) | (adcData.ch1[2]);
                    channelData.rawAlt      =  (adcData.ch3[0] << 16) | (adcData.ch3[1] << 8) | (adcData.ch3[2]);
                    M25P_programBytes(channelDataPtr, 16, rawDataPtrs[sel810]);
                    rawDataPtrs[sel810] += 16;
                    break;
                case 3:
                    channelData.rawMain     =  (adcData.ch3[0] << 16) | (adcData.ch3[1] << 8) | (adcData.ch3[2]);
                    channelData.rawAlt      =  (adcData.ch1[0] << 16) | (adcData.ch1[1] << 8) | (adcData.ch1[2]);
                    M25P_programBytes(channelDataPtr, 16, rawDataPtrs[sel1300]);
                    rawDataPtrs[sel1300] += 16;
                    break;
                default:
                    transfer("Unknown Error Mux in State : ", debugConsole);
                    dec_ascii(ascii, mux);
                    transfer(ascii, debugConsole);
                    transfer("\n\r",debugConsole);
                    while(1);
                }

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
            SignalInput = &signalInputBuffer[0];
            SignalOutput = &signalOutputBuffer[0];
            AmbientInput = &ambientInputBuffer[0];
            AmbientOutput = &ambientOutputBuffer[0];

            blockSize = BLOCK_SIZE;
            numBlocks = (MAX_NO_OF_SAMPLES/4)/blockSize;

            rawDataPtrs[selRed] = RAW_RED_BASE;
            rawDataPtrs[selIr]  = RAW_IR_BASE;
            rawDataPtrs[sel810] = RAW_810_BASE;
            rawDataPtrs[sel1300]= RAW_1300_BASE;

            filterDataStructs[selRed].signalPtr = FIR_RED_SIGNAL_BASE;
            filterDataStructs[selRed].ambientPtr = FIR_RED_AMBIENT_BASE;
            filterDataStructs[selIr].signalPtr = FIR_IR_SIGNAL_BASE;
            filterDataStructs[selIr].ambientPtr = FIR_IR_AMBIENT_BASE;
            filterDataStructs[sel810].signalPtr = FIR_810_SIGNAL_BASE;
            filterDataStructs[sel810].ambientPtr = FIR_810_AMBIENT_BASE;
            filterDataStructs[sel1300].signalPtr = FIR_1300_SIGNAL_BASE;
            filterDataStructs[sel1300].ambientPtr = FIR_1300_AMBIENT_BASE;

            for (j = 0; j < 4; j++){

                for(i = 0; i < 2; i++){
                    M25P_eraseSector(filterDataStructs[j].signalPtr + (i * M25P_SECTOR_SIZE));
                }

                for(i = 0; i < 2; i++){
                    M25P_eraseSector(filterDataStructs[j].ambientPtr + (i * M25P_SECTOR_SIZE));
                }
            }

            transfer("Filter Initialization\n\r ", debugConsole);
            /* Call FIR init function to initialize the instance structure. */

            arm_fir_init_q31(&SignalState, NUM_TAPS, (q31_t *)&firCoeffs32[0], &firStateSignal[0], blockSize);
            arm_fir_init_q31(&AmbientState, NUM_TAPS, (q31_t *)&firCoeffs32[0], &firStateAmbient[0], blockSize);
            transfer("Signal and Ambient Filters Initialized\n\r ", debugConsole);

            /* ----------------------------------------------------------------------
            ** Call the FIR process function for every blockSize samples
            ** ------------------------------------------------------------------- */
            for(l = 0; l < 4; l++){

                for(p = 0; p < (BLOCK_SIZE + NUM_TAPS - 1); p++){
                    firStateSignal[p] = 0;
                    firStateAmbient[p] = 0;
                }

                for(i = 0; i < numBlocks; i++)
                {
                    for(j = 0; j < blockSize; j++){

                        for (k = 0; k < 16; k++)
                            channelDataPtr[k] = 0;

                        M25P_readBytes(channelDataPtr, 16, rawDataPtrs[l]);
                        rawDataPtrs[l] += 16;

                        SignalInput[j] = channelData.rawMain;
                        AmbientInput[j] = channelData.ambientMain;
                    }
                    transfer("Filter Block : \n\r", debugConsole);
                    arm_fir_q31(&SignalState, SignalInput, SignalOutput, blockSize);
                    arm_fir_q31(&AmbientState, AmbientInput, AmbientOutput, blockSize);

                    M25P_programBytes((uint8_t *)SignalOutput, (blockSize*4), filterDataStructs[l].signalPtr);
                    filterDataStructs[l].signalPtr += (blockSize*4);

                    M25P_programBytes((uint8_t *)AmbientOutput, (blockSize*4), filterDataStructs[l].ambientPtr);
                    filterDataStructs[l].ambientPtr += (blockSize*4);

                }
            }
            /*
            dec_ascii(ascii, filterData.signalPtr);
            transfer("Flash WritePointer At: ", debugConsole);
            transfer(ascii, debugConsole);
            transfer("\n\r", debugConsole);
            */
            transfer("Filtering Complete\n\r", debugConsole);
            change_deviceState(data_transfer);
            break;

        case data_transfer:

            rawDataPtrs[selRed] = RAW_RED_BASE;
            rawDataPtrs[selIr]  = RAW_IR_BASE;
            rawDataPtrs[sel810] = RAW_810_BASE;
            rawDataPtrs[sel1300]= RAW_1300_BASE;

            for(i = 0; i < 4 ; i++){
                numberOfSamples = MAX_NO_OF_SAMPLES/4;

                while(numberOfSamples){

                    for (j = 0; j < 16; j++)
                        channelDataPtr[j] = 0;

                    M25P_readBytes(channelDataPtr, 16, rawDataPtrs[i]);
                    rawDataPtrs[i] += 16;
                    numberOfSamples--;

                    transfer("D", debugConsole);
                    dec_ascii(num, i);
                    transfer(num, debugConsole);

                    transfer(":", debugConsole);
                    dec_ascii(num, channelData.rawMain);
                    transfer(num, debugConsole);

                    transfer(":", debugConsole);
                    dec_ascii(num, channelData.ambientMain);
                    transfer(num, debugConsole);

                    transfer(":", debugConsole);
                    dec_ascii(num, channelData.rawAlt);
                    transfer(num, debugConsole);

                    transfer(":", debugConsole);
                    dec_ascii(num, channelData.ambientAlt);
                    transfer(num, debugConsole);

                    transfer("\n\r", debugConsole);
                }
            }

            filterDataStructs[selRed].signalPtr = FIR_RED_SIGNAL_BASE;
            filterDataStructs[selRed].ambientPtr = FIR_RED_AMBIENT_BASE;
            filterDataStructs[selIr].signalPtr = FIR_IR_SIGNAL_BASE;
            filterDataStructs[selIr].ambientPtr = FIR_IR_AMBIENT_BASE;
            filterDataStructs[sel810].signalPtr = FIR_810_SIGNAL_BASE;
            filterDataStructs[sel810].ambientPtr = FIR_810_AMBIENT_BASE;
            filterDataStructs[sel1300].signalPtr = FIR_1300_SIGNAL_BASE;
            filterDataStructs[sel1300].ambientPtr = FIR_1300_AMBIENT_BASE;

            for(i = 0; i < 4; i++){
                numberOfSamples = MAX_NO_OF_SAMPLES/4;

                while(numberOfSamples){

                    filteredData = 0;
                    M25P_readBytes((uint8_t*)&filteredData, 4, filterDataStructs[i].signalPtr);
                    filterDataStructs[i].signalPtr += 4;

                    transfer("F", debugConsole);
                    dec_ascii(num, i);
                    transfer(num, debugConsole);

                    transfer(":", debugConsole);
                    dec_ascii(num, filteredData);
                    transfer(num, debugConsole);

                    filteredData = 0;
                    M25P_readBytes((uint8_t*)&filteredData, 4, filterDataStructs[i].ambientPtr);
                    filterDataStructs[i].ambientPtr += 4;

                    transfer(":", debugConsole);
                    dec_ascii(num, filteredData);
                    transfer(num, debugConsole);

                    transfer("\n\r", debugConsole);

                    numberOfSamples--;
                }
            }

            if(numberOfSamples == 0){
                transfer("Transfer Complete\n\r", debugConsole);
                change_deviceState(wait_for_ble);
            }

            break;
        default:
            transfer("Unknown Device State: ", debugConsole);
            dec_ascii(num, deviceState);
            transfer(num, debugConsole);
            transfer("\n\r", debugConsole);
            while(1);
            break;
        }
    }
}


void isr_debugConsole(void)
{
	uint32_t u0status;
	u0status = UARTIntStatus(debugConsole,true);
	UARTIntClear(debugConsole,u0status);
	if(UARTCharsAvail(debugConsole))
		{
			uart_char = UARTCharGet(debugConsole);
			UARTCharPut(debugConsole,uart_char);
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
    uint8_t j;
    uint8_t *adcDataPtr = (uint8_t *)&adcData;

    // Clear the timer interrupt flag.
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    if(mux < 3) mux++;
    else mux = 0;

    ADS1294_readBytes(adcDataPtr, 15);

    switch(mux){
    case 0:
        GPIOPinWrite(deMuxLed.selBase, deMuxLed.selPins, selRed);   // Toggle LED0 everytime a key is pressed
        channelData.ambientMain     =  (adcData.ch1[0] << 16) | (adcData.ch1[1] << 8) | (adcData.ch1[2]);
        channelData.ambientAlt      =  (adcData.ch3[0] << 16) | (adcData.ch3[1] << 8) | (adcData.ch3[2]);
    	break;
    case 1:
    	GPIOPinWrite(deMuxLed.selBase, deMuxLed.selPins, selIr);	// Toggle LED0 everytime a key is pressed
        channelData.ambientMain     =  (adcData.ch1[0] << 16) | (adcData.ch1[1] << 8) | (adcData.ch1[2]);
        channelData.ambientAlt      =  (adcData.ch3[0] << 16) | (adcData.ch3[1] << 8) | (adcData.ch3[2]);
    	break;
    case 2:
    	GPIOPinWrite(deMuxLed.selBase, deMuxLed.selPins, sel810);	// Toggle LED0 everytime a key is pressed
        channelData.ambientMain     =  (adcData.ch1[0] << 16) | (adcData.ch1[1] << 8) | (adcData.ch1[2]);
        channelData.ambientAlt      =  (adcData.ch3[0] << 16) | (adcData.ch3[1] << 8) | (adcData.ch3[2]);
    	break;
    case 3:
    	GPIOPinWrite(deMuxLed.selBase, deMuxLed.selPins, sel1300);	// Toggle LED0 everytime a key is pressed
        channelData.ambientMain     =  (adcData.ch3[0] << 16) | (adcData.ch3[1] << 8) | (adcData.ch3[2]);
        channelData.ambientAlt      =  (adcData.ch1[0] << 16) | (adcData.ch1[1] << 8) | (adcData.ch1[2]);
    	break;
    }

    GPIOPinWrite(deMuxLed.inBase, deMuxLed.inPin, deMuxLed.inPin);  // Toggle LED0 everytime a key is pressed
    TimerEnable(TIMER0_BASE, TIMER_B );
    for (j = 0; j < 15; j++)
        adcDataPtr[j] = 0;
}

void
Timer0BIntHandler(void)
{
    // Clear the timer interrupt flag.
    TimerIntClear(TIMER0_BASE, TIMER_TIMB_TIMEOUT);
    timer_int = 1;
}

