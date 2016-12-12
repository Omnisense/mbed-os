/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/*
 * TEXT BELOW IS USED AS SETTING FOR THE PINS TOOL *****************************
PinsProfile:
- !!product 'Pins v2.0'
- !!processor 'MK20DN512xxx10'
- !!package 'MK20DN512VMC10'
- !!mcu_data 'ksdk2_0'
- !!processor_version '1.1.0'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR THE PINS TOOL ***
 */

#include "fsl_common.h"
#include "fsl_port.h"
#include "pin_mux.h"

#define PCR_DSE_HIGH                  0x01u   /*!< Drive Strength Enable: High drive strength is configured on the corresponding pin, if pin is configured as a digital output. */
#define PCR_PS_UP                     0x01u   /*!< Pull Select: Internal pull-up resistor is enabled on the corresponding pin, if the corresponding Port Pull Enable Register bit is set. */
#define PIN0_IDX                         0u   /*!< Pin number for pin 0 in a port */
#define PIN1_IDX                         1u   /*!< Pin number for pin 1 in a port */
#define PIN2_IDX                         2u   /*!< Pin number for pin 2 in a port */
#define PIN3_IDX                         3u   /*!< Pin number for pin 3 in a port */
#define PIN4_IDX                         4u   /*!< Pin number for pin 4 in a port */
#define PIN5_IDX                         5u   /*!< Pin number for pin 5 in a port */
#define PIN6_IDX                         6u   /*!< Pin number for pin 6 in a port */
#define PIN7_IDX                         7u   /*!< Pin number for pin 7 in a port */
#define PIN8_IDX                         8u   /*!< Pin number for pin 8 in a port */
#define PIN9_IDX                         9u   /*!< Pin number for pin 9 in a port */
#define PIN10_IDX                       10u   /*!< Pin number for pin 10 in a port */
#define PIN11_IDX                       11u   /*!< Pin number for pin 11 in a port */
#define PIN12_IDX                       12u   /*!< Pin number for pin 12 in a port */
#define PIN13_IDX                       13u   /*!< Pin number for pin 13 in a port */
#define PIN14_IDX                       14u   /*!< Pin number for pin 14 in a port */
#define PIN15_IDX                       15u   /*!< Pin number for pin 15 in a port */
#define PIN16_IDX                       16u   /*!< Pin number for pin 16 in a port */
#define PIN17_IDX                       17u   /*!< Pin number for pin 17 in a port */
#define PIN18_IDX                       18u   /*!< Pin number for pin 18 in a port */
#define PIN19_IDX                       19u   /*!< Pin number for pin 19 in a port */
#define PIN20_IDX                       20u   /*!< Pin number for pin 20 in a port */
#define PIN21_IDX                       21u   /*!< Pin number for pin 21 in a port */
#define PIN22_IDX                       22u   /*!< Pin number for pin 22 in a port */
#define PIN23_IDX                       23u   /*!< Pin number for pin 23 in a port */
#define PIN26_IDX                       26u   /*!< Pin number for pin 26 in a port */
#define PIN29_IDX                       29u   /*!< Pin number for pin 29 in a port */
#define SOPT4_FTM2CH0SRC_FTM          0x00u   /*!< FTM2 channel 0 input capture source select: FTM2_CH0 signal */
#define SOPT4_FTM2CLKSEL_CLK0         0x00u   /*!< FlexTimer 2 External Clock Pin Select: FTM2 external clock driven by FTM_CLK0 pin. */
#define SOPT5_UART0RXSRC_UART_RX      0x00u   /*!< UART 0 receive data source select: UART0_RX pin */
#define SOPT5_UART0TXSRC_UART_TX      0x00u   /*!< UART 0 transmit data source select: UART0_TX pin */

/*
 * TEXT BELOW IS USED AS SETTING FOR THE PINS TOOL *****************************
BOARD_InitPins:
- options: {coreID: singlecore, enableClock: 'true'}
- pin_list:
  - {pin_num: L1, peripheral: ADC1, signal: 'DP, 0', pin_signal: PGA1_DP/ADC1_DP0/ADC0_DP3}
  - {pin_num: J1, peripheral: ADC1, signal: 'DP, 1', pin_signal: ADC1_DP1}
  - {pin_num: L3, peripheral: VREF, signal: OUT, pin_signal: VREF_OUT/CMP1_IN5/CMP0_IN5/ADC1_SE18}
  - {pin_num: F2, peripheral: USB0, signal: DM, pin_signal: USB0_DM}
  - {pin_num: F1, peripheral: USB0, signal: DP, pin_signal: USB0_DP}
  - {pin_num: G1, peripheral: USB0, signal: VOUT33, pin_signal: VOUT33}
  - {pin_num: G2, peripheral: USB0, signal: VREGIN, pin_signal: VREGIN}
  - {pin_num: J6, peripheral: JTAG, signal: JTAG_TCLK, pin_signal: TSI0_CH1/PTA0/UART0_CTS_b/UART0_COL_b/FTM0_CH5/JTAG_TCLK/SWD_CLK/EZP_CLK}
  - {pin_num: H8, peripheral: JTAG, signal: JTAG_TDI, pin_signal: TSI0_CH2/PTA1/UART0_RX/FTM0_CH6/JTAG_TDI/EZP_DI}
  - {pin_num: J7, peripheral: JTAG, signal: JTAG_TDO, pin_signal: TSI0_CH3/PTA2/UART0_TX/FTM0_CH7/JTAG_TDO/TRACE_SWO/EZP_DO}
  - {pin_num: H9, peripheral: JTAG, signal: JTAG_TMS, pin_signal: TSI0_CH4/PTA3/UART0_RTS_b/FTM0_CH0/JTAG_TMS/SWD_DIO}
  - {pin_num: K7, peripheral: JTAG, signal: JTAG_TRST, pin_signal: PTA5/USB_CLKIN/FTM0_CH2/CMP2_OUT/I2S0_TX_BCLK/JTAG_TRST_b}
  - {pin_num: J6, peripheral: JTAG, signal: SWD_CLK, pin_signal: TSI0_CH1/PTA0/UART0_CTS_b/UART0_COL_b/FTM0_CH5/JTAG_TCLK/SWD_CLK/EZP_CLK}
  - {pin_num: H9, peripheral: JTAG, signal: SWD_DIO, pin_signal: TSI0_CH4/PTA3/UART0_RTS_b/FTM0_CH0/JTAG_TMS/SWD_DIO}
  - {pin_num: E10, peripheral: UART3, signal: CTS, pin_signal: PTB9/SPI1_PCS1/UART3_CTS_b/FB_AD20}
  - {pin_num: D11, peripheral: UART3, signal: RTS, pin_signal: PTB8/UART3_RTS_b/FB_AD21}
  - {pin_num: D10, peripheral: UART3, signal: RX, pin_signal: ADC1_SE14/PTB10/SPI1_PCS0/UART3_RX/FB_AD19/FTM0_FLT1}
  - {pin_num: C10, peripheral: UART3, signal: TX, pin_signal: ADC1_SE15/PTB11/SPI1_SCK/UART3_TX/FB_AD18/FTM0_FLT2, direction: OUTPUT, drive_strength: high}
  - {pin_num: A10, peripheral: UART5, signal: RX, pin_signal: PTD8/I2C0_SCL/UART5_RX/FB_A16}
  - {pin_num: A9, peripheral: UART5, signal: TX, pin_signal: PTD9/I2C0_SDA/UART5_TX/FB_A17, direction: OUTPUT, drive_strength: high}
  - {pin_num: J8, peripheral: SystemControl, signal: NMI, pin_signal: TSI0_CH5/PTA4/LLWU_P3/FTM0_CH1/NMI_b/EZP_CS_b}
  - {pin_num: J9, peripheral: FTM2, signal: 'CH, 0', pin_signal: PTA10/FTM2_CH0/FTM2_QD_PHA/TRACE_D0, direction: INPUT}
  - {pin_num: J4, peripheral: FTM2, signal: 'CH, 1', pin_signal: PTA11/FTM2_CH1/FTM2_QD_PHB, direction: INPUT}
  - {pin_num: K9, peripheral: n/a, signal: disabled, pin_signal: PTA14/SPI0_PCS0/UART0_TX/I2S0_RX_BCLK/I2S0_TXD1}
  - {pin_num: L11, peripheral: FTM2, signal: CLKIN, pin_signal: EXTAL0/PTA18/FTM0_FLT2/FTM_CLKIN0}
  - {pin_num: H11, peripheral: GPIOA, signal: 'GPIO, 29', pin_signal: PTA29/FB_A24, direction: OUTPUT, pull_select: up}
  - {pin_num: K11, peripheral: GPIOA, signal: 'GPIO, 19', pin_signal: XTAL0/PTA19/FTM1_FLT0/FTM_CLKIN1/LPTMR0_ALT1, direction: OUTPUT}
  - {pin_num: J10, peripheral: n/a, signal: disabled, pin_signal: PTA16/SPI0_SOUT/UART0_CTS_b/UART0_COL_b/I2S0_RX_FS/I2S0_RXD1}
  - {pin_num: L9, peripheral: n/a, signal: disabled, pin_signal: PTA15/SPI0_SCK/UART0_RX/I2S0_RXD0}
  - {pin_num: G11, peripheral: LLWU, signal: 'P, 5', pin_signal: ADC0_SE8/ADC1_SE8/TSI0_CH0/PTB0/LLWU_P5/I2C0_SCL/FTM1_CH0/FTM1_QD_PHA}
  - {pin_num: G11, peripheral: GPIOB, signal: 'GPIO, 0', pin_signal: ADC0_SE8/ADC1_SE8/TSI0_CH0/PTB0/LLWU_P5/I2C0_SCL/FTM1_CH0/FTM1_QD_PHA}
  - {pin_num: G8, peripheral: GPIOB, signal: 'GPIO, 3', pin_signal: ADC0_SE13/TSI0_CH8/PTB3/I2C0_SDA/UART0_CTS_b/UART0_COL_b/FTM0_FLT0, direction: OUTPUT}
  - {pin_num: F11, peripheral: GPIOB, signal: 'GPIO, 6', pin_signal: ADC1_SE12/PTB6/FB_AD23, direction: OUTPUT}
  - {pin_num: E11, peripheral: GPIOB, signal: 'GPIO, 7', pin_signal: ADC1_SE13/PTB7/FB_AD22, direction: OUTPUT}
  - {pin_num: B10, peripheral: UART0, signal: RX, pin_signal: TSI0_CH9/PTB16/SPI1_SOUT/UART0_RX/FB_AD17/EWM_IN}
  - {pin_num: E9, peripheral: UART0, signal: TX, pin_signal: TSI0_CH10/PTB17/SPI1_SIN/UART0_TX/FB_AD16/EWM_OUT_b, direction: OUTPUT, drive_strength: high}
  - {pin_num: F10, peripheral: GPIOB, signal: 'GPIO, 20', pin_signal: PTB20/SPI2_PCS0/FB_AD31/CMP0_OUT, direction: INPUT}
  - {pin_num: F9, peripheral: GPIOB, signal: 'GPIO, 21', pin_signal: PTB21/SPI2_SCK/FB_AD30/CMP1_OUT, direction: OUTPUT}
  - {pin_num: F8, peripheral: GPIOB, signal: 'GPIO, 22', pin_signal: PTB22/SPI2_SOUT/FB_AD29/CMP2_OUT, direction: OUTPUT}
  - {pin_num: E8, peripheral: GPIOB, signal: 'GPIO, 23', pin_signal: PTB23/SPI2_SIN/SPI0_PCS5/FB_AD28, direction: INPUT}
  - {pin_num: B9, peripheral: GPIOC, signal: 'GPIO, 0', pin_signal: ADC0_SE14/TSI0_CH13/PTC0/SPI0_PCS4/PDB0_EXTRG/FB_AD14/I2S0_TXD1, direction: INPUT}
  - {pin_num: D8, peripheral: LLWU, signal: 'P, 6', pin_signal: ADC0_SE15/TSI0_CH14/PTC1/LLWU_P6/SPI0_PCS3/UART1_RTS_b/FTM0_CH0/FB_AD13/I2S0_TXD0}
  - {pin_num: D8, peripheral: GPIOC, signal: 'GPIO, 1', pin_signal: ADC0_SE15/TSI0_CH14/PTC1/LLWU_P6/SPI0_PCS3/UART1_RTS_b/FTM0_CH0/FB_AD13/I2S0_TXD0}
  - {pin_num: C8, peripheral: GPIOC, signal: 'GPIO, 2', pin_signal: ADC0_SE4b/CMP1_IN0/TSI0_CH15/PTC2/SPI0_PCS2/UART1_CTS_b/FTM0_CH1/FB_AD12/I2S0_TX_FS, direction: INPUT}
  - {pin_num: B8, peripheral: GPIOC, signal: 'GPIO, 3', pin_signal: CMP1_IN1/PTC3/LLWU_P7/SPI0_PCS1/UART1_RX/FTM0_CH2/CLKOUT/I2S0_TX_BCLK}
  - {pin_num: B8, peripheral: LLWU, signal: 'P, 7', pin_signal: CMP1_IN1/PTC3/LLWU_P7/SPI0_PCS1/UART1_RX/FTM0_CH2/CLKOUT/I2S0_TX_BCLK}
  - {pin_num: A8, peripheral: GPIOC, signal: 'GPIO, 4', pin_signal: PTC4/LLWU_P8/SPI0_PCS0/UART1_TX/FTM0_CH3/FB_AD11/CMP1_OUT}
  - {pin_num: A8, peripheral: LLWU, signal: 'P, 8', pin_signal: PTC4/LLWU_P8/SPI0_PCS0/UART1_TX/FTM0_CH3/FB_AD11/CMP1_OUT}
  - {pin_num: D7, peripheral: LLWU, signal: 'P, 9', pin_signal: PTC5/LLWU_P9/SPI0_SCK/LPTMR0_ALT2/I2S0_RXD0/FB_AD10/CMP0_OUT}
  - {pin_num: D7, peripheral: GPIOC, signal: 'GPIO, 5', pin_signal: PTC5/LLWU_P9/SPI0_SCK/LPTMR0_ALT2/I2S0_RXD0/FB_AD10/CMP0_OUT}
  - {pin_num: C7, peripheral: GPIOC, signal: 'GPIO, 6', pin_signal: CMP0_IN0/PTC6/LLWU_P10/SPI0_SOUT/PDB0_EXTRG/I2S0_RX_BCLK/FB_AD9/I2S0_MCLK}
  - {pin_num: C7, peripheral: LLWU, signal: 'P, 10', pin_signal: CMP0_IN0/PTC6/LLWU_P10/SPI0_SOUT/PDB0_EXTRG/I2S0_RX_BCLK/FB_AD9/I2S0_MCLK}
  - {pin_num: B7, peripheral: GPIOC, signal: 'GPIO, 7', pin_signal: CMP0_IN1/PTC7/SPI0_SIN/USB_SOF_OUT/I2S0_RX_FS/FB_AD8, direction: INPUT}
  - {pin_num: A7, peripheral: GPIOC, signal: 'GPIO, 8', pin_signal: ADC1_SE4b/CMP0_IN2/PTC8/I2S0_MCLK/FB_AD7, direction: INPUT}
  - {pin_num: D6, peripheral: GPIOC, signal: 'GPIO, 9', pin_signal: ADC1_SE5b/CMP0_IN3/PTC9/I2S0_RX_BCLK/FB_AD6/FTM2_FLT0, direction: OUTPUT}
  - {pin_num: C6, peripheral: I2C1, signal: SCL, pin_signal: ADC1_SE6b/PTC10/I2C1_SCL/I2S0_RX_FS/FB_AD5}
  - {pin_num: C5, peripheral: I2C1, signal: SDA, pin_signal: ADC1_SE7b/PTC11/LLWU_P11/I2C1_SDA/I2S0_RXD1/FB_RW_b}
  - {pin_num: B6, peripheral: GPIOC, signal: 'GPIO, 12', pin_signal: PTC12/UART4_RTS_b/FB_AD27, direction: INPUT}
  - {pin_num: A6, peripheral: GPIOC, signal: 'GPIO, 13', pin_signal: PTC13/UART4_CTS_b/FB_AD26, direction: OUTPUT}
  - {pin_num: A5, peripheral: UART4, signal: RX, pin_signal: PTC14/UART4_RX/FB_AD25}
  - {pin_num: B5, peripheral: UART4, signal: TX, pin_signal: PTC15/UART4_TX/FB_AD24, direction: OUTPUT, drive_strength: high}
  - {pin_num: D5, peripheral: GPIOC, signal: 'GPIO, 16', pin_signal: PTC16/CAN1_RX/UART3_RX/FB_CS5_b/FB_TSIZ1/FB_BE23_16_b, direction: OUTPUT}
  - {pin_num: C4, peripheral: GPIOC, signal: 'GPIO, 17', pin_signal: PTC17/CAN1_TX/UART3_TX/FB_CS4_b/FB_TSIZ0/FB_BE31_24_b, direction: OUTPUT}
  - {pin_num: B4, peripheral: GPIOC, signal: 'GPIO, 18', pin_signal: PTC18/UART3_RTS_b/FB_TBST_b/FB_CS2_b/FB_BE15_8_b, direction: OUTPUT}
  - {pin_num: A4, peripheral: GPIOC, signal: 'GPIO, 19', pin_signal: PTC19/UART3_CTS_b/FB_CS3_b/FB_BE7_0_b/FB_TA_b, direction: OUTPUT}
  - {pin_num: D4, peripheral: SPI0, signal: PCS0_SS, pin_signal: PTD0/LLWU_P12/SPI0_PCS0/UART2_RTS_b/FB_ALE/FB_CS1_b/FB_TS_b, direction: OUTPUT}
  - {pin_num: D3, peripheral: SPI0, signal: SCK, pin_signal: ADC0_SE5b/PTD1/SPI0_SCK/UART2_CTS_b/FB_CS0_b, direction: OUTPUT}
  - {pin_num: C3, peripheral: SPI0, signal: SOUT, pin_signal: PTD2/LLWU_P13/SPI0_SOUT/UART2_RX/FB_AD4}
  - {pin_num: B3, peripheral: SPI0, signal: SIN, pin_signal: PTD3/SPI0_SIN/UART2_TX/FB_AD3}
  - {pin_num: A3, peripheral: GPIOD, signal: 'GPIO, 4', pin_signal: PTD4/LLWU_P14/SPI0_PCS1/UART0_RTS_b/FTM0_CH4/FB_AD2/EWM_IN}
  - {pin_num: A3, peripheral: LLWU, signal: 'P, 14', pin_signal: PTD4/LLWU_P14/SPI0_PCS1/UART0_RTS_b/FTM0_CH4/FB_AD2/EWM_IN}
  - {pin_num: A2, peripheral: SPI0, signal: PCS2, pin_signal: ADC0_SE6b/PTD5/SPI0_PCS2/UART0_CTS_b/UART0_COL_b/FTM0_CH5/FB_AD1/EWM_OUT_b}
  - {pin_num: B2, peripheral: LLWU, signal: 'P, 15', pin_signal: ADC0_SE7b/PTD6/LLWU_P15/SPI0_PCS3/UART0_RX/FTM0_CH6/FB_AD0/FTM0_FLT0}
  - {pin_num: B2, peripheral: GPIOD, signal: 'GPIO, 6', pin_signal: ADC0_SE7b/PTD6/LLWU_P15/SPI0_PCS3/UART0_RX/FTM0_CH6/FB_AD0/FTM0_FLT0}
  - {pin_num: A1, peripheral: GPIOD, signal: 'GPIO, 7', pin_signal: PTD7/CMT_IRO/UART0_TX/FTM0_CH7/FTM0_FLT1, direction: INPUT}
  - {pin_num: B1, peripheral: GPIOD, signal: 'GPIO, 10', pin_signal: PTD10/UART5_RTS_b/FB_A18, direction: OUTPUT}
  - {pin_num: C2, peripheral: SPI2, signal: PCS0_SS, pin_signal: PTD11/SPI2_PCS0/UART5_CTS_b/SDHC0_CLKIN/FB_A19, direction: OUTPUT}
  - {pin_num: C1, peripheral: SPI2, signal: SCK, pin_signal: PTD12/SPI2_SCK/SDHC0_D4/FB_A20, direction: OUTPUT}
  - {pin_num: D2, peripheral: SPI2, signal: SOUT, pin_signal: PTD13/SPI2_SOUT/SDHC0_D5/FB_A21}
  - {pin_num: D1, peripheral: SPI2, signal: SIN, pin_signal: PTD14/SPI2_SIN/SDHC0_D6/FB_A22}
  - {pin_num: E1, peripheral: n/a, signal: disabled, pin_signal: PTD15/SPI2_PCS1/SDHC0_D7/FB_A23}
  - {pin_num: E4, peripheral: SPI1, signal: PCS1, pin_signal: ADC1_SE4a/PTE0/SPI1_PCS1/UART1_TX/SDHC0_D1/I2C1_SDA/RTC_CLKOUT}
  - {pin_num: E3, peripheral: SPI1, signal: SIN, pin_signal: ADC1_SE5a/PTE1/LLWU_P0/SPI1_SOUT/UART1_RX/SDHC0_D0/I2C1_SCL/SPI1_SIN}
  - {pin_num: E2, peripheral: SPI1, signal: SCK, pin_signal: ADC1_SE6a/PTE2/LLWU_P1/SPI1_SCK/UART1_CTS_b/SDHC0_DCLK, direction: OUTPUT}
  - {pin_num: F4, peripheral: SPI1, signal: SOUT, pin_signal: ADC1_SE7a/PTE3/SPI1_SIN/UART1_RTS_b/SDHC0_CMD/SPI1_SOUT}
  - {pin_num: H7, peripheral: SPI1, signal: PCS0_SS, pin_signal: PTE4/LLWU_P2/SPI1_PCS0/UART3_TX/SDHC0_D3, direction: OUTPUT}
  - {pin_num: G4, peripheral: GPIOE, signal: 'GPIO, 5', pin_signal: PTE5/SPI1_PCS2/UART3_RX/SDHC0_D2, direction: INPUT}
  - {pin_num: F3, peripheral: GPIOE, signal: 'GPIO, 6', pin_signal: PTE6/SPI1_PCS3/UART3_CTS_b/I2S0_MCLK/USB_SOF_OUT, direction: OUTPUT}
  - {pin_num: H6, peripheral: n/a, signal: disabled, pin_signal: PTE26/UART4_CTS_b/RTC_CLKOUT/USB_CLKIN}
  - {pin_num: K10, peripheral: SUPPLY, signal: 'VSS, 82', pin_signal: VSS82}
  - {pin_num: G6, peripheral: ADC0, signal: VREFL, pin_signal: VREFL}
  - {pin_num: G5, peripheral: ADC0, signal: VREFH, pin_signal: VREFH}
  - {pin_num: J2, peripheral: ADC1, signal: 'DM, 1', pin_signal: ADC1_DM1}
  - {pin_num: L10, peripheral: SUPPLY, signal: 'VDD, 81', pin_signal: VDD81}
  - {pin_num: G7, peripheral: SUPPLY, signal: 'VSS, 17', pin_signal: VSS17}
  - {pin_num: K6, peripheral: RTC, signal: VBAT, pin_signal: VBAT}
  - {pin_num: E6, peripheral: SUPPLY, signal: 'VDD, 16', pin_signal: VDD16}
  - {pin_num: G3, peripheral: SUPPLY, signal: 'VSS, 68', pin_signal: VSS68}
  - {pin_num: L5, peripheral: RTC, signal: EXTAL32, pin_signal: EXTAL32}
  - {pin_num: F5, peripheral: SUPPLY, signal: 'VDDA, 0', pin_signal: VDDA}
  - {pin_num: J11, peripheral: RCM, signal: RESET, pin_signal: RESET_b}
  - {pin_num: E7, peripheral: SUPPLY, signal: 'VDD, 5', pin_signal: VDD5}
  - {pin_num: F6, peripheral: SUPPLY, signal: 'VSSA, 0', pin_signal: VSSA}
  - {pin_num: F7, peripheral: SUPPLY, signal: 'VSS, 6', pin_signal: VSS6}
  - {pin_num: L6, peripheral: SUPPLY, signal: 'VSS, 22', pin_signal: VSS22}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR THE PINS TOOL ***
 */

/*FUNCTION**********************************************************************
 *
 * Function Name : BOARD_InitPins
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 *END**************************************************************************/
void BOARD_InitPins(void) {
  CLOCK_EnableClock(kCLOCK_PortA);                           /* Port A Clock Gate Control: Clock enabled */
  CLOCK_EnableClock(kCLOCK_PortB);                           /* Port B Clock Gate Control: Clock enabled */
  CLOCK_EnableClock(kCLOCK_PortC);                           /* Port C Clock Gate Control: Clock enabled */
  CLOCK_EnableClock(kCLOCK_PortD);                           /* Port D Clock Gate Control: Clock enabled */
  CLOCK_EnableClock(kCLOCK_PortE);                           /* Port E Clock Gate Control: Clock enabled */

  PORT_SetPinMux(PORTA, PIN0_IDX, kPORT_MuxAlt7);            /* PORTA0 (pin J6) is configured as JTAG_TCLK, SWD_CLK */
  PORT_SetPinMux(PORTA, PIN1_IDX, kPORT_MuxAlt7);            /* PORTA1 (pin H8) is configured as JTAG_TDI */
  PORT_SetPinMux(PORTA, PIN10_IDX, kPORT_MuxAlt3);           /* PORTA10 (pin J9) is configured as FTM2_CH0 */
  PORT_SetPinMux(PORTA, PIN11_IDX, kPORT_MuxAlt3);           /* PORTA11 (pin J4) is configured as FTM2_CH1 */
  PORT_SetPinMux(PORTA, PIN14_IDX, kPORT_PinDisabledOrAnalog); /* PORTA14 (pin K9) is disabled */
  PORT_SetPinMux(PORTA, PIN15_IDX, kPORT_PinDisabledOrAnalog); /* PORTA15 (pin L9) is disabled */
  PORT_SetPinMux(PORTA, PIN16_IDX, kPORT_PinDisabledOrAnalog); /* PORTA16 (pin J10) is disabled */
  PORT_SetPinMux(PORTA, PIN18_IDX, kPORT_MuxAlt4);           /* PORTA18 (pin L11) is configured as FTM_CLKIN0 */
  PORT_SetPinMux(PORTA, PIN19_IDX, kPORT_MuxAsGpio);         /* PORTA19 (pin K11) is configured as PTA19 */
  PORT_SetPinMux(PORTA, PIN2_IDX, kPORT_MuxAlt7);            /* PORTA2 (pin J7) is configured as JTAG_TDO */
  PORT_SetPinMux(PORTA, PIN29_IDX, kPORT_MuxAsGpio);         /* PORTA29 (pin H11) is configured as PTA29 */
  PORTA->PCR[29] = ((PORTA->PCR[29] &
    (~(PORT_PCR_PS_MASK | PORT_PCR_ISF_MASK)))               /* Mask bits to zero which are setting */
      | PORT_PCR_PS(PCR_PS_UP)                               /* Pull Select: Internal pull-up resistor is enabled on the corresponding pin, if the corresponding Port Pull Enable Register bit is set. */
    );
  PORTA->PCR[29] = ((PORTA->PCR[29] &
    (~(PORT_PCR_PE_MASK | PORT_PCR_ISF_MASK)))               /* Mask bits to zero which are setting */
      | PORT_PCR_PE(PCR_PS_UP)                               /* Pull Eanable */
    );
  PORT_SetPinMux(PORTA, PIN3_IDX, kPORT_MuxAlt7);            /* PORTA3 (pin H9) is configured as JTAG_TMS, SWD_DIO */
  PORT_SetPinMux(PORTA, PIN4_IDX, kPORT_MuxAlt7);            /* PORTA4 (pin J8) is configured as NMI_b */
  PORT_SetPinMux(PORTA, PIN5_IDX, kPORT_MuxAlt7);            /* PORTA5 (pin K7) is configured as JTAG_TRST_b */
  PORT_SetPinMux(PORTB, PIN0_IDX, kPORT_MuxAsGpio);          /* PORTB0 (pin G11) is configured as LLWU_P5, PTB0 */
  PORT_SetPinMux(PORTB, PIN10_IDX, kPORT_MuxAlt3);           /* PORTB10 (pin D10) is configured as UART3_RX */
  PORT_SetPinMux(PORTB, PIN11_IDX, kPORT_MuxAlt3);           /* PORTB11 (pin C10) is configured as UART3_TX */
  PORTB->PCR[11] = ((PORTB->PCR[11] &
    (~(PORT_PCR_DSE_MASK | PORT_PCR_ISF_MASK)))              /* Mask bits to zero which are setting */
      | PORT_PCR_DSE(PCR_DSE_HIGH)                           /* Drive Strength Enable: High drive strength is configured on the corresponding pin, if pin is configured as a digital output. */
    );
  PORT_SetPinMux(PORTB, PIN16_IDX, kPORT_MuxAlt3);           /* PORTB16 (pin B10) is configured as UART0_RX */
  PORT_SetPinMux(PORTB, PIN17_IDX, kPORT_MuxAlt3);           /* PORTB17 (pin E9) is configured as UART0_TX */
  PORTB->PCR[17] = ((PORTB->PCR[17] &
    (~(PORT_PCR_DSE_MASK | PORT_PCR_ISF_MASK)))              /* Mask bits to zero which are setting */
      | PORT_PCR_DSE(PCR_DSE_HIGH)                           /* Drive Strength Enable: High drive strength is configured on the corresponding pin, if pin is configured as a digital output. */
    );
  PORT_SetPinMux(PORTB, PIN20_IDX, kPORT_MuxAsGpio);         /* PORTB20 (pin F10) is configured as PTB20 */
  PORT_SetPinMux(PORTB, PIN21_IDX, kPORT_MuxAsGpio);         /* PORTB21 (pin F9) is configured as PTB21 */
  PORT_SetPinMux(PORTB, PIN22_IDX, kPORT_MuxAsGpio);         /* PORTB22 (pin F8) is configured as PTB22 */
  PORT_SetPinMux(PORTB, PIN23_IDX, kPORT_MuxAsGpio);         /* PORTB23 (pin E8) is configured as PTB23 */
  PORT_SetPinMux(PORTB, PIN3_IDX, kPORT_MuxAsGpio);          /* PORTB3 (pin G8) is configured as PTB3 */
  PORT_SetPinMux(PORTB, PIN6_IDX, kPORT_MuxAsGpio);          /* PORTB6 (pin F11) is configured as PTB6 */
  PORT_SetPinMux(PORTB, PIN7_IDX, kPORT_MuxAsGpio);          /* PORTB7 (pin E11) is configured as PTB7 */
  PORT_SetPinMux(PORTB, PIN8_IDX, kPORT_MuxAlt3);            /* PORTB8 (pin D11) is configured as UART3_RTS_b */
  PORT_SetPinMux(PORTB, PIN9_IDX, kPORT_MuxAlt3);            /* PORTB9 (pin E10) is configured as UART3_CTS_b */
  PORT_SetPinMux(PORTC, PIN0_IDX, kPORT_MuxAsGpio);          /* PORTC0 (pin B9) is configured as PTC0 */
  PORT_SetPinMux(PORTC, PIN1_IDX, kPORT_MuxAsGpio);          /* PORTC1 (pin D8) is configured as LLWU_P6, PTC1 */
  PORT_SetPinMux(PORTC, PIN10_IDX, kPORT_MuxAlt2);           /* PORTC10 (pin C6) is configured as I2C1_SCL */
  PORTC->PCR[10] = ((PORTC->PCR[10] &
    (~(PORT_PCR_DSE_MASK | PORT_PCR_ISF_MASK)))              /* Mask bits to zero which are setting */
      | PORT_PCR_DSE(PCR_DSE_HIGH)                           /* Drive Strength Enable: High drive strength is configured on the corresponding pin, if pin is configured as a digital output. */
    );
  PORTC->PCR[10] = ((PORTC->PCR[10] & ~(PORT_PCR_ISF_MASK)) | (PORT_PCR_PE_MASK | PORT_PCR_PS_MASK));   /* enable pullup */
  PORT_SetPinMux(PORTC, PIN11_IDX, kPORT_MuxAlt2);           /* PORTC11 (pin C5) is configured as I2C1_SDA */
  PORTC->PCR[11] = ((PORTC->PCR[11] &
    (~(PORT_PCR_DSE_MASK | PORT_PCR_ISF_MASK)))              /* Mask bits to zero which are setting */
      | PORT_PCR_DSE(PCR_DSE_HIGH)                           /* Drive Strength Enable: High drive strength is configured on the corresponding pin, if pin is configured as a digital output. */
    );
  PORTC->PCR[11] = ((PORTC->PCR[11] & ~(PORT_PCR_ISF_MASK)) | (PORT_PCR_PE_MASK | PORT_PCR_PS_MASK));   /* enable pullup */
  PORT_SetPinMux(PORTC, PIN12_IDX, kPORT_MuxAsGpio);         /* PORTC12 (pin B6) is configured as PTC12 */
  PORT_SetPinMux(PORTC, PIN13_IDX, kPORT_MuxAsGpio);         /* PORTC13 (pin A6) is configured as PTC13 */
  PORT_SetPinMux(PORTC, PIN14_IDX, kPORT_MuxAlt3);           /* PORTC14 (pin A5) is configured as UART4_RX */
  PORT_SetPinMux(PORTC, PIN15_IDX, kPORT_MuxAlt3);           /* PORTC15 (pin B5) is configured as UART4_TX */
  PORTC->PCR[15] = ((PORTC->PCR[15] &
    (~(PORT_PCR_DSE_MASK | PORT_PCR_ISF_MASK)))              /* Mask bits to zero which are setting */
      | PORT_PCR_DSE(PCR_DSE_HIGH)                           /* Drive Strength Enable: High drive strength is configured on the corresponding pin, if pin is configured as a digital output. */
    );
  PORT_SetPinMux(PORTC, PIN16_IDX, kPORT_MuxAsGpio);         /* PORTC16 (pin D5) is configured as PTC16 */
  PORT_SetPinMux(PORTC, PIN17_IDX, kPORT_MuxAsGpio);         /* PORTC17 (pin C4) is configured as PTC17 */
  PORT_SetPinMux(PORTC, PIN18_IDX, kPORT_MuxAsGpio);         /* PORTC18 (pin B4) is configured as PTC18 */
  PORT_SetPinMux(PORTC, PIN19_IDX, kPORT_MuxAsGpio);         /* PORTC19 (pin A4) is configured as PTC19 */
  PORT_SetPinMux(PORTC, PIN2_IDX, kPORT_MuxAsGpio);          /* PORTC2 (pin C8) is configured as PTC2 */
  PORT_SetPinMux(PORTC, PIN3_IDX, kPORT_MuxAsGpio);          /* PORTC3 (pin B8) is configured as LLWU_P7, PTC3 */
  PORT_SetPinMux(PORTC, PIN4_IDX, kPORT_MuxAsGpio);          /* PORTC4 (pin A8) is configured as PTC4, LLWU_P8 */
  PORT_SetPinMux(PORTC, PIN5_IDX, kPORT_MuxAsGpio);          /* PORTC5 (pin D7) is configured as LLWU_P9, PTC5 */
  PORT_SetPinMux(PORTC, PIN6_IDX, kPORT_MuxAsGpio);          /* PORTC6 (pin C7) is configured as PTC6, LLWU_P10 */
  PORT_SetPinMux(PORTC, PIN7_IDX, kPORT_MuxAsGpio);          /* PORTC7 (pin B7) is configured as PTC7 */
  PORT_SetPinMux(PORTC, PIN8_IDX, kPORT_MuxAsGpio);          /* PORTC8 (pin A7) is configured as PTC8 */
  PORT_SetPinMux(PORTC, PIN9_IDX, kPORT_MuxAsGpio);          /* PORTC9 (pin D6) is configured as PTC9 */
//  PORT_SetPinMux(PORTD, PIN0_IDX, kPORT_MuxAsGpio);            /* PORTD0 (pin D4) is configured as SPI0_PCS0 as PTD0 */
  PORT_SetPinMux(PORTD, PIN0_IDX, kPORT_MuxAlt2);            /* PORTD0 (pin D4) is configured as SPI0_PCS0 */
  PORT_SetPinMux(PORTD, PIN1_IDX, kPORT_MuxAlt2);            /* PORTD1 (pin D3) is configured as SPI0_SCK */
  PORT_SetPinMux(PORTD, PIN10_IDX, kPORT_MuxAsGpio);         /* PORTD10 (pin B1) is configured as PTD10 */
//  PORT_SetPinMux(PORTD, PIN11_IDX, kPORT_MuxAsGpio);           /* PORTD11 (pin C2) is configured as SPI2_PCS0 as PTD11 */
  PORT_SetPinMux(PORTD, PIN11_IDX, kPORT_MuxAlt2);           /* PORTD11 (pin C2) is configured as SPI2_PCS0 */
  PORT_SetPinMux(PORTD, PIN12_IDX, kPORT_MuxAlt2);           /* PORTD12 (pin C1) is configured as SPI2_SCK */
  PORT_SetPinMux(PORTD, PIN13_IDX, kPORT_MuxAlt2);           /* PORTD13 (pin D2) is configured as SPI2_SOUT */
  PORT_SetPinMux(PORTD, PIN14_IDX, kPORT_MuxAlt2);           /* PORTD14 (pin D1) is configured as SPI2_SIN */
  PORT_SetPinMux(PORTD, PIN15_IDX, kPORT_PinDisabledOrAnalog); /* PORTD15 (pin E1) is disabled */
  PORT_SetPinMux(PORTD, PIN2_IDX, kPORT_MuxAlt2);            /* PORTD2 (pin C3) is configured as SPI0_SOUT */
  PORT_SetPinMux(PORTD, PIN3_IDX, kPORT_MuxAlt2);            /* PORTD3 (pin B3) is configured as SPI0_SIN */
  PORT_SetPinMux(PORTD, PIN4_IDX, kPORT_MuxAsGpio);          /* PORTD4 (pin A3) is configured as PTD4, LLWU_P14 */
  PORT_SetPinMux(PORTD, PIN5_IDX, kPORT_MuxAlt2);            /* PORTD5 (pin A2) is configured as SPI0_PCS2 */
  PORT_SetPinMux(PORTD, PIN6_IDX, kPORT_MuxAsGpio);          /* PORTD6 (pin B2) is configured as LLWU_P15, PTD6 */
  PORT_SetPinMux(PORTD, PIN7_IDX, kPORT_MuxAsGpio);          /* PORTD7 (pin A1) is configured as PTD7 */
  PORT_SetPinMux(PORTD, PIN8_IDX, kPORT_MuxAlt3);            /* PORTD8 (pin A10) is configured as UART5_RX */
  PORT_SetPinMux(PORTD, PIN9_IDX, kPORT_MuxAlt3);            /* PORTD9 (pin A9) is configured as UART5_TX */
  PORTD->PCR[9] = ((PORTD->PCR[9] &
    (~(PORT_PCR_DSE_MASK | PORT_PCR_ISF_MASK)))              /* Mask bits to zero which are setting */
      | PORT_PCR_DSE(PCR_DSE_HIGH)                           /* Drive Strength Enable: High drive strength is configured on the corresponding pin, if pin is configured as a digital output. */
    );
  PORT_SetPinMux(PORTE, PIN0_IDX, kPORT_MuxAlt2);            /* PORTE0 (pin E4) is configured as SPI1_PCS1 */
  PORT_SetPinMux(PORTE, PIN1_IDX, kPORT_MuxAlt7);            /* PORTE1 (pin E3) is configured as SPI1_SIN */
  PORT_SetPinMux(PORTE, PIN2_IDX, kPORT_MuxAlt2);            /* PORTE2 (pin E2) is configured as SPI1_SCK */
  PORT_SetPinMux(PORTE, PIN26_IDX, kPORT_PinDisabledOrAnalog); /* PORTE26 (pin H6) is disabled */
  PORT_SetPinMux(PORTE, PIN3_IDX, kPORT_MuxAlt7);            /* PORTE3 (pin F4) is configured as SPI1_SOUT */
  PORT_SetPinMux(PORTE, PIN4_IDX, kPORT_MuxAlt2);            /* PORTE4 (pin H7) is configured as SPI1_PCS0 */
  PORT_SetPinMux(PORTE, PIN5_IDX, kPORT_MuxAsGpio);          /* PORTE5 (pin G4) is configured as PTE5 */
  PORT_SetPinMux(PORTE, PIN6_IDX, kPORT_MuxAsGpio);          /* PORTE6 (pin F3) is configured as PTE6 */
  SIM->SOPT4 = ((SIM->SOPT4 &
    (~(SIM_SOPT4_FTM2CH0SRC_MASK | SIM_SOPT4_FTM2CLKSEL_MASK))) /* Mask bits to zero which are setting */
      | SIM_SOPT4_FTM2CH0SRC(SOPT4_FTM2CH0SRC_FTM)           /* FTM2 channel 0 input capture source select: FTM2_CH0 signal */
      | SIM_SOPT4_FTM2CLKSEL(SOPT4_FTM2CLKSEL_CLK0)          /* FlexTimer 2 External Clock Pin Select: FTM2 external clock driven by FTM_CLK0 pin. */
    );
  SIM->SOPT5 = ((SIM->SOPT5 &
    (~(SIM_SOPT5_UART0TXSRC_MASK | SIM_SOPT5_UART0RXSRC_MASK))) /* Mask bits to zero which are setting */
      | SIM_SOPT5_UART0TXSRC(SOPT5_UART0TXSRC_UART_TX)       /* UART 0 transmit data source select: UART0_TX pin */
      | SIM_SOPT5_UART0RXSRC(SOPT5_UART0RXSRC_UART_RX)       /* UART 0 receive data source select: UART0_RX pin */
    );
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
