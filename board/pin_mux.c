/***********************************************************************************************************************
 * This file was generated by the MCUXpresso Config Tools. Any manual edits made to this file
 * will be overwritten if the respective MCUXpresso Config Tools is used to update this file.
 **********************************************************************************************************************/

/* clang-format off */
/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
!!GlobalInfo
product: Pins v9.0
processor: MK66FX1M0xxx18
package_id: MK66FX1M0VMD18
mcu_data: ksdk2_0
processor_version: 9.0.0
pin_labels:
- {pin_num: K10, pin_signal: CMP3_IN2/PTA16/SPI0_SOUT/UART0_CTS_b/UART0_COL_b/RMII0_TXD0/MII0_TXD0/I2S0_RX_FS/I2S0_RXD1, label: BARO_CS, identifier: BARO_CS}
- {pin_num: A3, pin_signal: ADC0_SE6b/PTD5/SPI0_PCS2/UART0_CTS_b/UART0_COL_b/FTM0_CH5/FB_AD1/SDRAM_A9/EWM_OUT_b/SPI1_SCK, label: IMU_CS, identifier: IMU_CS}
- {pin_num: A8, pin_signal: ADC1_SE4b/CMP0_IN2/PTC8/FTM3_CH4/I2S0_MCLK/FB_AD7/SDRAM_A15, label: IMU_DR, identifier: IMU_DR}
- {pin_num: K4, pin_signal: PTE26/ENET_1588_CLKIN/UART4_CTS_b/RTC_CLKOUT/USB0_CLKIN, label: PYRO_TEST, identifier: PYRO_TEST;PYRO_TEST_PB}
- {pin_num: K5, pin_signal: ADC0_SE18/PTE25/LLWU_P21/CAN1_RX/UART4_RX/I2C0_SDA/EWM_IN, label: PYRO_ENABLE, identifier: PYRO_ENABLE}
- {pin_num: K11, pin_signal: ADC1_SE17/PTA17/SPI0_SIN/UART0_RTS_b/RMII0_TXD1/MII0_TXD1/I2S0_MCLK, label: BATT_PYRO_SENSE, identifier: BATT_PYRO_SENSE}
- {pin_num: M4, pin_signal: ADC0_SE17/PTE24/CAN1_TX/UART4_TX/I2C0_SCL/EWM_OUT_b, label: RECO_CS, identifier: RECO_CS}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */
/* clang-format on */

#include "fsl_common.h"
#include "fsl_port.h"
#include "fsl_gpio.h"
#include "pin_mux.h"

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitBootPins
 * Description   : Calls initialization functions.
 *
 * END ****************************************************************************************************************/
void BOARD_InitBootPins(void)
{
    BOARD_InitPins();
}

/* clang-format off */
/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
BOARD_InitPins:
- options: {callFromInitBoot: 'true', coreID: core0, enableClock: 'true'}
- pin_list:
  - {pin_num: K10, peripheral: GPIOA, signal: 'GPIO, 16', pin_signal: CMP3_IN2/PTA16/SPI0_SOUT/UART0_CTS_b/UART0_COL_b/RMII0_TXD0/MII0_TXD0/I2S0_RX_FS/I2S0_RXD1,
    direction: OUTPUT, gpio_init_state: 'true'}
  - {pin_num: C8, peripheral: SPI0, signal: SOUT, pin_signal: CMP0_IN0/PTC6/LLWU_P10/SPI0_SOUT/PDB0_EXTRG/I2S0_RX_BCLK/FB_AD9/SDRAM_A17/I2S0_MCLK}
  - {pin_num: B8, peripheral: SPI0, signal: SIN, pin_signal: CMP0_IN1/PTC7/SPI0_SIN/USB0_SOF_OUT/I2S0_RX_FS/FB_AD8/SDRAM_A16}
  - {pin_num: D8, peripheral: SPI0, signal: SCK, pin_signal: PTC5/LLWU_P9/SPI0_SCK/LPTMR0_ALT2/I2S0_RXD0/FB_AD10/SDRAM_A18/CMP0_OUT/FTM0_CH2}
  - {pin_num: A9, peripheral: SPI0, signal: PCS0_SS, pin_signal: PTC4/LLWU_P8/SPI0_PCS0/UART1_TX/FTM0_CH3/FB_AD11/SDRAM_A19/CMP1_OUT}
  - {pin_num: L10, peripheral: UART0, signal: TX, pin_signal: PTA14/SPI0_PCS0/UART0_TX/RMII0_CRS_DV/MII0_RXDV/I2C2_SCL/I2S0_RX_BCLK/I2S0_TXD1}
  - {pin_num: L11, peripheral: UART0, signal: RX, pin_signal: CMP3_IN1/PTA15/SPI0_SCK/UART0_RX/RMII0_TXEN/MII0_TXEN/I2S0_RXD0}
  - {pin_num: A3, peripheral: GPIOD, signal: 'GPIO, 5', pin_signal: ADC0_SE6b/PTD5/SPI0_PCS2/UART0_CTS_b/UART0_COL_b/FTM0_CH5/FB_AD1/SDRAM_A9/EWM_OUT_b/SPI1_SCK,
    direction: OUTPUT, gpio_init_state: 'true'}
  - {pin_num: A8, peripheral: GPIOC, signal: 'GPIO, 8', pin_signal: ADC1_SE4b/CMP0_IN2/PTC8/FTM3_CH4/I2S0_MCLK/FB_AD7/SDRAM_A15, direction: INPUT, gpio_interrupt: kPORT_InterruptRisingEdge}
  - {pin_num: K5, peripheral: GPIOE, signal: 'GPIO, 25', pin_signal: ADC0_SE18/PTE25/LLWU_P21/CAN1_RX/UART4_RX/I2C0_SDA/EWM_IN, direction: OUTPUT}
  - {pin_num: K11, peripheral: GPIOA, signal: 'GPIO, 17', pin_signal: ADC1_SE17/PTA17/SPI0_SIN/UART0_RTS_b/RMII0_TXD1/MII0_TXD1/I2S0_MCLK, direction: INPUT}
  - {pin_num: M4, peripheral: GPIOE, signal: 'GPIO, 24', pin_signal: ADC0_SE17/PTE24/CAN1_TX/UART4_TX/I2C0_SCL/EWM_OUT_b, direction: OUTPUT, gpio_init_state: 'true'}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */
/* clang-format on */

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitPins
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
void BOARD_InitPins(void)
{
    /* Port A Clock Gate Control: Clock enabled */
    CLOCK_EnableClock(kCLOCK_PortA);
    /* Port C Clock Gate Control: Clock enabled */
    CLOCK_EnableClock(kCLOCK_PortC);
    /* Port D Clock Gate Control: Clock enabled */
    CLOCK_EnableClock(kCLOCK_PortD);
    /* Port E Clock Gate Control: Clock enabled */
    CLOCK_EnableClock(kCLOCK_PortE);

    gpio_pin_config_t BARO_CS_config = {
        .pinDirection = kGPIO_DigitalOutput,
        .outputLogic = 1U
    };
    /* Initialize GPIO functionality on pin PTA16 (pin K10)  */
    GPIO_PinInit(BOARD_INITPINS_BARO_CS_GPIO, BOARD_INITPINS_BARO_CS_PIN, &BARO_CS_config);

    gpio_pin_config_t BATT_PYRO_SENSE_config = {
        .pinDirection = kGPIO_DigitalInput,
        .outputLogic = 0U
    };
    /* Initialize GPIO functionality on pin PTA17 (pin K11)  */
    GPIO_PinInit(BOARD_INITPINS_BATT_PYRO_SENSE_GPIO, BOARD_INITPINS_BATT_PYRO_SENSE_PIN, &BATT_PYRO_SENSE_config);

    gpio_pin_config_t IMU_DR_config = {
        .pinDirection = kGPIO_DigitalInput,
        .outputLogic = 0U
    };
    /* Initialize GPIO functionality on pin PTC8 (pin A8)  */
    GPIO_PinInit(BOARD_INITPINS_IMU_DR_GPIO, BOARD_INITPINS_IMU_DR_PIN, &IMU_DR_config);

    gpio_pin_config_t IMU_CS_config = {
        .pinDirection = kGPIO_DigitalOutput,
        .outputLogic = 1U
    };
    /* Initialize GPIO functionality on pin PTD5 (pin A3)  */
    GPIO_PinInit(BOARD_INITPINS_IMU_CS_GPIO, BOARD_INITPINS_IMU_CS_PIN, &IMU_CS_config);

    gpio_pin_config_t RECO_CS_config = {
        .pinDirection = kGPIO_DigitalOutput,
        .outputLogic = 1U
    };
    /* Initialize GPIO functionality on pin PTE24 (pin M4)  */
    GPIO_PinInit(BOARD_INITPINS_RECO_CS_GPIO, BOARD_INITPINS_RECO_CS_PIN, &RECO_CS_config);

    gpio_pin_config_t PYRO_ENABLE_config = {
        .pinDirection = kGPIO_DigitalOutput,
        .outputLogic = 0U
    };
    /* Initialize GPIO functionality on pin PTE25 (pin K5)  */
    GPIO_PinInit(BOARD_INITPINS_PYRO_ENABLE_GPIO, BOARD_INITPINS_PYRO_ENABLE_PIN, &PYRO_ENABLE_config);

    /* PORTA14 (pin L10) is configured as UART0_TX */
    PORT_SetPinMux(PORTA, 14U, kPORT_MuxAlt3);

    /* PORTA15 (pin L11) is configured as UART0_RX */
    PORT_SetPinMux(PORTA, 15U, kPORT_MuxAlt3);

    /* PORTA16 (pin K10) is configured as PTA16 */
    PORT_SetPinMux(BOARD_INITPINS_BARO_CS_PORT, BOARD_INITPINS_BARO_CS_PIN, kPORT_MuxAsGpio);

    /* PORTA17 (pin K11) is configured as PTA17 */
    PORT_SetPinMux(BOARD_INITPINS_BATT_PYRO_SENSE_PORT, BOARD_INITPINS_BATT_PYRO_SENSE_PIN, kPORT_MuxAsGpio);

    /* PORTC4 (pin A9) is configured as SPI0_PCS0 */
    PORT_SetPinMux(PORTC, 4U, kPORT_MuxAlt2);

    /* PORTC5 (pin D8) is configured as SPI0_SCK */
    PORT_SetPinMux(PORTC, 5U, kPORT_MuxAlt2);

    /* PORTC6 (pin C8) is configured as SPI0_SOUT */
    PORT_SetPinMux(PORTC, 6U, kPORT_MuxAlt2);

    /* PORTC7 (pin B8) is configured as SPI0_SIN */
    PORT_SetPinMux(PORTC, 7U, kPORT_MuxAlt2);

    /* PORTC8 (pin A8) is configured as PTC8 */
    PORT_SetPinMux(BOARD_INITPINS_IMU_DR_PORT, BOARD_INITPINS_IMU_DR_PIN, kPORT_MuxAsGpio);

    /* Interrupt configuration on PORTC8 (pin A8): Interrupt on rising edge */
    PORT_SetPinInterruptConfig(BOARD_INITPINS_IMU_DR_PORT, BOARD_INITPINS_IMU_DR_PIN, kPORT_InterruptRisingEdge);

    /* PORTD5 (pin A3) is configured as PTD5 */
    PORT_SetPinMux(BOARD_INITPINS_IMU_CS_PORT, BOARD_INITPINS_IMU_CS_PIN, kPORT_MuxAsGpio);

    /* PORTE24 (pin M4) is configured as PTE24 */
    PORT_SetPinMux(BOARD_INITPINS_RECO_CS_PORT, BOARD_INITPINS_RECO_CS_PIN, kPORT_MuxAsGpio);

    /* PORTE25 (pin K5) is configured as PTE25 */
    PORT_SetPinMux(BOARD_INITPINS_PYRO_ENABLE_PORT, BOARD_INITPINS_PYRO_ENABLE_PIN, kPORT_MuxAsGpio);

    SIM->SOPT5 = ((SIM->SOPT5 &
                   /* Mask bits to zero which are setting */
                   (~(SIM_SOPT5_UART0TXSRC_MASK)))

                  /* UART 0 transmit data source select: UART0_TX pin. */
                  | SIM_SOPT5_UART0TXSRC(SOPT5_UART0TXSRC_UART_TX));
}
/***********************************************************************************************************************
 * EOF
 **********************************************************************************************************************/