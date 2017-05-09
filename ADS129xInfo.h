/* Teensy ADS129xADC library
 * Copyright (C) 2014 by Valentin Goverdovsky
 *
 * This file is part of the Teensy ADS129xADC Library
 *
 * This Library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This Library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the Teensy ADS129xADC Library.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
#ifndef ADS129xInfo_h
#define ADS129xInfo_h
// Based on:
//
// Low-Power, 8-Channel, 24-Bit AnalogFront-Endfor Biopotential Measurements
// SBAS459J – JANUARY 2010 – REVISED JANUARY 2014
// Rev. J
// Define function to send the command
// http://www.ti.com/lit/gpn/ads1294
//------------------------------------------------------------------------------
// ADC commands
//------------------------------------------------------------------------------
/** System commands */
#define WAKEUP             0x02
#define STANDBY            0x04
#define RESET              0x06
#define STARTCON           0x08
#define STOPCON            0x0A
/** Read data commands */
#define RDATAC             0x10
#define SDATAC             0x11
#define RDATA              0x12
/** Write/Read register commands */
#define RREG               0x20
#define WREG               0x40
//------------------------------------------------------------------------------
// ADC register settings
//------------------------------------------------------------------------------
/** ID Control Register Address (Factory-Programmed, Read-Only). */
#define ID                 0x00
/** Device family identification*/
#define ID_ADS1294         0x90
#define ID_ADS1296         0x91
#define ID_ADS1298         0x92
#define ID_ADS1294R        0xD0
#define ID_ADS1296R        0xD1
#define ID_ADS1298R        0xD2
//------------------------------------------------------------------------------
/** CONFIG1: Configuration Register 1 address. Configure resolution and power mode. */
#define CONFIG1            0x01
/** CONFIG1 register configuration bits */
#define CONFIG1_const      0x00 // Bits[4:3] must always bet set to 0, always use this const when configuring!
#define DAISY_EN           0x40 // Multiple readback mode
#define CLK_EN             0x20 // Internal oscillator signal is connected to the CLK pin
/** For High-Resolution mode: fMOD  fCLK/4 */
#define HIGH_RES_32k_SPS   0x80 // Data rate is fMOD/16
#define HIGH_RES_16k_SPS   0x81 // Data rate is fMOD/32
#define HIGH_RES_8k_SPS    0x82 // Data rate is fMOD/64
#define HIGH_RES_4k_SPS    0x83 // Data rate is fMOD/128
#define HIGH_RES_2k_SPS    0x84 // Data rate is fMOD/256
#define HIGH_RES_1k_SPS    0x85 // Data rate is fMOD/512
#define HIGH_RES_500_SPS   0x86 // Data rate is fMOD/1024
/** For Low Power mode: fMOD  fCLK/8 */
#define LOW_POWR_16k_SPS   0x00 // Data rate is fMOD/16
#define LOW_POWR_8k_SPS    0x01 // Data rate is fMOD/32
#define LOW_POWR_4k_SPS    0x02 // Data rate is fMOD/64
#define LOW_POWR_2k_SPS    0x03 // Data rate is fMOD/128
#define LOW_POWR_1k_SPS    0x04 // Data rate is fMOD/256
#define LOW_POWR_500_SPS   0x05 // Data rate is fMOD/512
#define LOW_POWR_250_SPS   0x06 // Data rate is fMOD/1024
//------------------------------------------------------------------------------
/** CONFIG2: Configuration Register 2 address. Configure the test signal generation. */
#define CONFIG2            0x02
/** CONFIG2 register configuration bits */
#define CONFIG2_const      0x00 // Bits[7:6,3] must always bet set to 0, always use this const when configuring!
#define WCT_CHOP           0x20 // Chopping frequency constant at fMOD/16
#define TEST_AMP           0x04 // Increase test Amp to 2 × –(VREFP – VREFN)/2400 V

#define INT_TEST_1HZ       0x10 // Internal test with Amp  1 × –(VREFP – VREFN)/2400 V and pulsed at fCLK/(2^21)
#define INT_TEST_2HZ       0x11 // Internal test with Amp  1 × –(VREFP – VREFN)/2400 V and pulsed at fCLK/(2^20)
#define INT_TEST_DC        0x13 // Internal test with Amp  1 × –(VREFP – VREFN)/2400 V and at DC
//------------------------------------------------------------------------------
/** CONFIG3: Configuration Register 3 address. Configure multi-reference and RLD operation. */
#define CONFIG3            0x03
/** CONFIG3 register configuration bits */
#define CONFIG3_const      0x40 // Bit[6] must always bet set to 1, always use this const when configuring!
#define PD_REFBUF          0x80 // Enable internal reference buffer
#define VREF_4V            0x20 // VREFP is set to 4V (use only with a 5V analog supply)
#define RLD_MEAS           0x10 // RLD_IN signal is routed to the channel that has the MUX_Setting 010 (VREF)
#define RLDREF_INT         0x08 // RLDREF signal (AVDD – AVSS)/2 generated internally
#define PD_RLD             0x04 // RLD buffer is enabled
#define RLD_LOFF_SENS      0x02 // RLD sense is enabled
#define RLD_STAT           0x01 // RLD is not connected
//------------------------------------------------------------------------------
/** LOFF: Lead-Off Control Register address. Configure the Lead-Off detection operation. */
#define LOFF               0x04
/** LOFF register configuration bits */
#define LOFF_const         0x00 // No constant bits, nevertheless always use this const when configuring!

#define COMP_TH_95         0x00 // Comparator positive side 95%, Comparator negative side 5%
#define COMP_TH_92_5       0x20 // Comparator positive side 92.5%, Comparator negative side 7.5%
#define COMP_TH_90         0x40 // Comparator positive side 90%, Comparator negative side 10%
#define COMP_TH_87_5       0x60 // Comparator positive side 87.5%, Comparator negative side 12.5%
#define COMP_TH_85         0x80 // Comparator positive side 85%, Comparator negative side 15%
#define COMP_TH_80         0xA0 // Comparator positive side 80%, Comparator negative side 20%
#define COMP_TH_75         0xC0 // Comparator positive side 75%, Comparator negative side 25%
#define COMP_TH_70         0xE0 // Comparator positive side 70%, Comparator negative side 30%

#define VLEAD_OFF_EN       0x10 // Pull-up/pull-down resistor mode lead-off

#define ILEAD_OFF_6nA      0x00 // Magnitude of current for the current lead-off mode: 6 nA
#define ILEAD_OFF_12nA     0x04 // Magnitude of current for the current lead-off mode: 12 nA
#define ILEAD_OFF_18nA     0x08 // Magnitude of current for the current lead-off mode: 18 nA
#define ILEAD_OFF_24nA     0x0C // Magnitude of current for the current lead-off mode: 24 nA

#define FLEAD_OFF_AC       0x01 // AC lead-off detection at fDR/4
#define FLEAD_OFF_DC       0x03 // DC lead-off detection
//------------------------------------------------------------------------------
/** CHnSET: Individual Channel addresses (n  1 : 8). Configure the power mode, PGA gain, and multiplexer settings. */
#define CH1SET             0x05 // Channel 1 configuration register address
#define CH2SET             0x06 // Channel 2 configuration register address
#define CH3SET             0x07 // Channel 3 configuration register address
#define CH4SET             0x08 // Channel 4 configuration register address
#define CH5SET             0x09 // Channel 5 configuration register address
#define CH6SET             0x0A // Channel 6 configuration register address
#define CH7SET             0x0B // Channel 7 configuration register address
#define CH8SET             0x0C // Channel 8 configuration register address
/** CHnSET register configuration bits */
#define CHnSET_const       0x00 // Bit[3] must always bet set to 1, always use this const when configuring!

#define PD_CH              0x80 // Channel power-down. Recommended that the channel be set to input short MUXn[2:0]  001.

#define GAIN_X6            0x00 // PGA gain is 6 (default)
#define GAIN_X1            0x10 // PGA gain is 1
#define GAIN_X2            0x20 // PGA gain is 2
#define GAIN_X3            0x30 // PGA gain is 3
#define GAIN_X4            0x40 // PGA gain is 4
#define GAIN_X8            0x50 // PGA gain is 8
#define GAIN_X12           0x60 // PGA gain is 12

#define ELECTRODE_INPUT    0x00 // Normal electrode input
#define SHORTED            0x01 // Input shorted (for offset or noise measurements)
#define RLD_INPUT          0x02 // Used in conjunction with RLD_MEAS bit for RLD measurements.
#define MVDD               0x03 // MVDD for supply measurement
#define TEMP               0x04 // Temperature sensor
#define TEST_SIGNAL        0x05 // Test signal
#define RLD_DRP            0x06 // RLD_DRP (positive electrode is the driver)
#define RLD_DRN            0x07 // RLD_DRN (negative electrode is the driver)
//------------------------------------------------------------------------------
/** Control the selection of the positive signals from each channel for right leg drive derivation. */
#define RLD_SENSP          0x0D
/** RLD_SENSP register configuration bits */
#define RLD_SENSP_const    0x00 // No positive signal selected for RLD
#define RLD1P              0x01 // Channel 1+  is selected for RLD
#define RLD2P              0x02 // Channel 2+  is selected for RLD
#define RLD3P              0x04 // Channel 3+  is selected for RLD
#define RLD4P              0x08 // Channel 4+  is selected for RLD
#define RLD5P              0x10 // Channel 5+  is selected for RLD
#define RLD6P              0x20 // Channel 6+  is selected for RLD
#define RLD7P              0x40 // Channel 7+  is selected for RLD
#define RLD8P              0x80 // Channel 8+  is selected for RLD
//------------------------------------------------------------------------------
/** Control the selection of the negative signals from each channel for right leg drive derivation. */
#define RLD_SENSN          0x0E
/** RLD_SENSN register configuration bits */
#define RLD_SENSN_const    0x00 // No negative signal selected for RLD
#define RLD1N              0x01 // Channel 1-  is selected for RLD
#define RLD2N              0x02 // Channel 2-  is selected for RLD
#define RLD3N              0x04 // Channel 3-  is selected for RLD
#define RLD4N              0x08 // Channel 4-  is selected for RLD
#define RLD5N              0x10 // Channel 5-  is selected for RLD
#define RLD6N              0x20 // Channel 6-  is selected for RLD
#define RLD7N              0x40 // Channel 7-  is selected for RLD
#define RLD8N              0x80 // Channel 8-  is selected for RLD
//------------------------------------------------------------------------------
/** Control the selection of the the positive side from each channel for lead-off detection. */
#define LOFF_SENSP         0x0F
/** LOFF_SENSP register configuration bits */
#define LOFF_SENSP_const   0x00 // No positive side selected for LOFF
#define LOFF1P             0x01 // Channel 1+  is selected
#define LOFF2P             0x02 // Channel 2+  is selected
#define LOFF3P             0x04 // Channel 3+  is selected
#define LOFF4P             0x08 // Channel 4+  is selected
#define LOFF5P             0x10 // Channel 5+  is selected
#define LOFF6P             0x20 // Channel 6+  is selected
#define LOFF7P             0x40 // Channel 7+  is selected
#define LOFF8P             0x80 // Channel 8+  is selected
//------------------------------------------------------------------------------
/** Control the selection of the the negative side from each channel for lead-off detection. */
#define LOFF_SENSN         0x10
/** LOFF_SENSN register configuration bits */
#define LOFF_SENSN_const   0x00 // No negative side selected for LOFF
#define LOFF1N             0x01 // Channel 1-  is selected
#define LOFF2N             0x02 // Channel 2-  is selected
#define LOFF3N             0x04 // Channel 3-  is selected
#define LOFF4N             0x08 // Channel 4-  is selected
#define LOFF5N             0x10 // Channel 5-  is selected
#define LOFF6N             0x20 // Channel 6-  is selected
#define LOFF7N             0x40 // Channel 7-  is selected
#define LOFF8N             0x80 // Channel 8-  is selected
//------------------------------------------------------------------------------
/** Control the direction of the current used for lead-off derivation. */
#define LOFF_FLIP          0x11
/** LOFF_FLIP register configuration bits */
#define LOFF_FLIP_const    0x00
#define LOFF_FLIP1         0x01
#define LOFF_FLIP2         0x02
#define LOFF_FLIP3         0x04
#define LOFF_FLIP4         0x08
#define LOFF_FLIP5         0x10
#define LOFF_FLIP6         0x20
#define LOFF_FLIP7         0x40
#define LOFF_FLIP8         0x80
//------------------------------------------------------------------------------
/** LOFF_STATP: Read-Only Register stores the status of whether the positive electrode on each channel is on or off. */
#define LOFF_STATP         0x12
/** LOFF_STATP register indicator bits */
#define IN1P_OFF           0x01
#define IN2P_OFF           0x02
#define IN3P_OFF           0x04
#define IN4P_OFF           0x08
#define IN5P_OFF           0x10
#define IN6P_OFF           0x20
#define IN7P_OFF           0x40
#define IN8P_OFF           0x80
//------------------------------------------------------------------------------
/** LOFF_STATN: Read-Only Register stores the status of whether the negative electrode on each channel is on or off. */
#define LOFF_STATN         0x13
/** LOFF_STATN register indicator bits */
#define IN1N_OFF           0x01
#define IN2N_OFF           0x02
#define IN3N_OFF           0x04
#define IN4N_OFF           0x08
#define IN5N_OFF           0x10
#define IN6N_OFF           0x20
#define IN7N_OFF           0x40
#define IN8N_OFF           0x80
//------------------------------------------------------------------------------
/** The General-Purpose I/O Register controls the action of the three GPIO pins. */
#define GPIO               0x14
/** GPIO register indicator/configuration bits */
#define GPIO_const         0x00 // Set all GPIO pins to output
#define GPIOD4             0x80 // Read/write data to the GPIO port 4
#define GPIOD3             0x40 // Read/write data to the GPIO port 3
#define GPIOD2             0x20 // Read/write data to the GPIO port 2
#define GPIOD1             0x10 // Read/write data to the GPIO port 1

#define GPIOC4             0x08 // GPIO port 4 is input
#define GPIOC3             0x04 // GPIO port 3 is input
#define GPIOC2             0x02 // GPIO port 2 is input
#define GPIOC1             0x01 // GPIO port 1 is input
//------------------------------------------------------------------------------
/** PACE: PACE Detect Register address. Configure the channel signal used to feed the external PACE detect circuitry. */
#define PACE               0x15
/** PACE register configuration bits */
#define PACE_const         0x00 // Bit[7:5] must always bet set to 0, always use this const when configuring!

#define PACEE_CHAN2        0x00 // Channel 2 (default) available on TEST_PACE_OUT1
#define PACEE_CHAN4        0x08 // Channel 4 available on TEST_PACE_OUT1
#define PACEE_CHAN6        0x10 // Channel 6 available on TEST_PACE_OUT1
#define PACEE_CHAN8        0x18 // Channel 8 available on TEST_PACE_OUT1

#define PACEE_CHAN1        0x00 // Channel 1 (default) available on TEST_PACE_OUT2
#define PACEE_CHAN3        0x02 // Channel 3 available on TEST_PACE_OUT2
#define PACEE_CHAN5        0x04 // Channel 6 available on TEST_PACE_OUT2
#define PACEE_CHAN7        0x06 // Channel 7 available on TEST_PACE_OUT2

#define PD_PACE            0x01 // PACE detect buffer turned on
//------------------------------------------------------------------------------
/** RESP: Respiration Control Register address. Control the respiration circuitry. */
#define RESP               0x16
/** RESP register configuration bits */
#define RESP_const         0x20 // Bit[5] must always bet set to 1, always use this const when configuring!
#define RESP_DEMOD_EN1     0x80 // RESP demodulation circuitry turned on
#define RESP_MOD_EN1       0x40 // RESP modulation circuitry turned on

#define RESP_PH_22_5       0x00 // 22.5 phase of the respiration demodulation control signal
#define RESP_PH_45         0x04 // 45 phase of the respiration demodulation control signal
#define RESP_PH_67_5       0x08 // 67.5 phase of the respiration demodulation control signal
#define RESP_PH_90         0x0C // 90 phase of the respiration demodulation control signal
#define RESP_PH_112_5      0x10 // 112.5 phase of the respiration demodulation control signal
#define RESP_PH_135        0x14 // 135 phase of the respiration demodulation control signal
#define RESP_PH_157_5      0x18 // 157.5 phase of the respiration demodulation control signal

#define RESP_NONE          0x00 // No respiration
#define RESP_EXT           0x01 // External respiration
#define RESP_INT_SIG_INT   0x02 // Internal respiration with internal signals
#define RESP_INT_SIG_EXT   0x03 // Internal respiration with user-generated signals
//------------------------------------------------------------------------------
/** CONFIG4: Configuration Register 4 address. Miscellaneous controls. */
#define CONFIG4            0x17
/** CONFIG4 register configuration bits */
#define CONFIG4_const      0x00 // Bits[4,0] must always bet set to 0, always use this const when configuring!

#define RESP_FREQ_64k_Hz   0x00 // 64kHz modulation clock
#define RESP_FREQ_32k_Hz   0x20 // 32kHz modulation clock
#define RESP_FREQ_16k_Hz   0x40 // 16kHz modulation clock
#define RESP_FREQ_8k_Hz    0x60 // 8kHz modulation clock
#define RESP_FREQ_4k_Hz    0x80 // 4kHz modulation clock
#define RESP_FREQ_2k_Hz    0xA0 // 2kHz modulation clock
#define RESP_FREQ_1k_Hz    0xC0 // 1kHz modulation clock
#define RESP_FREQ_500_Hz   0xE0 // 0.5kHz modulation clock

#define SINGLE_SHOT        0x08 // Single-shot mode

#define WCT_TO_RLD         0x04 // WCT to RLD connection on

#define PD_LOFF_COMP       0x02 // Lead-off comparators enabled
//------------------------------------------------------------------------------
/** WCT1: Wilson Central Terminal and Augmented Lead Control Register address. */
#define WCT1               0x18
/** WCT1 register configuration bits */
#define WCT1_const         0x00 // No constant bits, nevertheless always use this const when configuring!

#define aVF_CH6            0x80 // Enable (WCTA + WCTB)/2 to the negative input of channel 6

#define aVL_CH5            0x40 // Enable (WCTA + WCTC)/2 to the negative input of channel 5

#define aVR_CH7            0x20 // Enable (WCTB + WCTC)/2 to the negative input of channel 7

#define avR_CH4            0x10 // Enable (WCTB + WCTC)/2 to the negative input of channel 4

#define PD_WCTA            0x08 // Powered on WCTA

#define WCTA_CH1P          0x00 // Channel 1 positive input connected to WCTA amplifier
#define WCTA_CH1N          0x01 // Channel 1 negative input connected to WCTA amplifier
#define WCTA_CH2P          0x02 // Channel 2 positive input connected to WCTA amplifier
#define WCTA_CH2N          0x03 // Channel 2 negative input connected to WCTA amplifier
#define WCTA_CH3P          0x04 // Channel 3 positive input connected to WCTA amplifier
#define WCTA_CH3N          0x05 // Channel 3 negative input connected to WCTA amplifier
#define WCTA_CH4P          0x06 // Channel 4 positive input connected to WCTA amplifier
#define WCTA_CH4N          0x07 // Channel 4 negative input connected to WCTA amplifier
//------------------------------------------------------------------------------
/** WCT2: Wilson Central Terminal Control Register address. */
#define WCT2               0x19
/** WCT2 register configuration bits */
#define WCT2_const         0x00 // No constant bits, nevertheless always use this const when configuring!

#define PD_WCTC            0x80 // Powered on WCTC

#define PD_WCTB            0x40 // Powered on WCTB

#define WCTB_CH1P          0x00 // Channel 1 positive input connected to WCTB amplifier
#define WCTB_CH1N          0x08 // Channel 1 negative input connected to WCTB amplifier
#define WCTB_CH2P          0x10 // Channel 2 positive input connected to WCTB amplifier
#define WCTB_CH2N          0x18 // Channel 2 negative input connected to WCTB amplifier
#define WCTB_CH3P          0x20 // Channel 3 positive input connected to WCTB amplifier
#define WCTB_CH3N          0x28 // Channel 3 negative input connected to WCTB amplifier
#define WCTB_CH4P          0x30 // Channel 4 positive input connected to WCTB amplifier
#define WCTB_CH4N          0x38 // Channel 4 negative input connected to WCTB amplifier

#define WCTC_CH1P          0x00 // Channel 1 positive input connected to WCTC amplifier
#define WCTC_CH1N          0x01 // Channel 1 negative input connected to WCTC amplifier
#define WCTC_CH2P          0x02 // Channel 2 positive input connected to WCTC amplifier
#define WCTC_CH2N          0x03 // Channel 2 negative input connected to WCTC amplifier
#define WCTC_CH3P          0x04 // Channel 3 positive input connected to WCTC amplifier
#define WCTC_CH3N          0x05 // Channel 3 negative input connected to WCTC amplifier
#define WCTC_CH4P          0x06 // Channel 4 positive input connected to WCTC amplifier
#define WCTC_CH4N          0x07 // Channel 4 negative input connected to WCTC amplifier
//------------------------------------------------------------------------------
#endif  /* ADS129xInfo_h */
