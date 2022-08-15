/***********************************************************************************************************************
 * This file was generated by the MCUXpresso Config Tools. Any manual edits made to this file
 * will be overwritten if the respective MCUXpresso Config Tools is used to update this file.
 **********************************************************************************************************************/

#ifndef _PIN_MUX_H_
#define _PIN_MUX_H_

/*!
 * @addtogroup pin_mux
 * @{
 */

/***********************************************************************************************************************
 * API
 **********************************************************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Calls initialization functions.
 *
 */
void BOARD_InitBootPins(void);

#define SOPT5_UART0TXSRC_UART_TX 0x00u /*!<@brief UART 0 transmit data source select: UART0_TX pin */

/*! @name PORTA16 (coord K10), BARO_CS
  @{ */

/* Symbols to be used with GPIO driver */
#define BOARD_INITPINS_BARO_CS_GPIO GPIOA                /*!<@brief GPIO peripheral base pointer */
#define BOARD_INITPINS_BARO_CS_GPIO_PIN_MASK (1U << 16U) /*!<@brief GPIO pin mask */

/* Symbols to be used with PORT driver */
#define BOARD_INITPINS_BARO_CS_PORT PORTA                /*!<@brief PORT peripheral base pointer */
#define BOARD_INITPINS_BARO_CS_PIN 16U                   /*!<@brief PORT pin number */
#define BOARD_INITPINS_BARO_CS_PIN_MASK (1U << 16U)      /*!<@brief PORT pin mask */
                                                         /* @} */

/*! @name PORTD5 (coord A3), IMU_CS
  @{ */

/* Symbols to be used with GPIO driver */
#define BOARD_INITPINS_IMU_CS_GPIO GPIOD               /*!<@brief GPIO peripheral base pointer */
#define BOARD_INITPINS_IMU_CS_GPIO_PIN_MASK (1U << 5U) /*!<@brief GPIO pin mask */

/* Symbols to be used with PORT driver */
#define BOARD_INITPINS_IMU_CS_PORT PORTD               /*!<@brief PORT peripheral base pointer */
#define BOARD_INITPINS_IMU_CS_PIN 5U                   /*!<@brief PORT pin number */
#define BOARD_INITPINS_IMU_CS_PIN_MASK (1U << 5U)      /*!<@brief PORT pin mask */
                                                       /* @} */

/*! @name PORTC8 (coord A8), IMU_DR
  @{ */

/* Symbols to be used with GPIO driver */
#define BOARD_INITPINS_IMU_DR_GPIO GPIOC               /*!<@brief GPIO peripheral base pointer */
#define BOARD_INITPINS_IMU_DR_GPIO_PIN_MASK (1U << 8U) /*!<@brief GPIO pin mask */

/* Symbols to be used with PORT driver */
#define BOARD_INITPINS_IMU_DR_PORT PORTC               /*!<@brief PORT peripheral base pointer */
#define BOARD_INITPINS_IMU_DR_PIN 8U                   /*!<@brief PORT pin number */
#define BOARD_INITPINS_IMU_DR_PIN_MASK (1U << 8U)      /*!<@brief PORT pin mask */
                                                       /* @} */

/*! @name PORTE25 (coord K5), PYRO_ENABLE
  @{ */

/* Symbols to be used with GPIO driver */
#define BOARD_INITPINS_PYRO_ENABLE_GPIO GPIOE                /*!<@brief GPIO peripheral base pointer */
#define BOARD_INITPINS_PYRO_ENABLE_GPIO_PIN_MASK (1U << 25U) /*!<@brief GPIO pin mask */

/* Symbols to be used with PORT driver */
#define BOARD_INITPINS_PYRO_ENABLE_PORT PORTE                /*!<@brief PORT peripheral base pointer */
#define BOARD_INITPINS_PYRO_ENABLE_PIN 25U                   /*!<@brief PORT pin number */
#define BOARD_INITPINS_PYRO_ENABLE_PIN_MASK (1U << 25U)      /*!<@brief PORT pin mask */
                                                             /* @} */

/*! @name PORTA17 (coord K11), BATT_PYRO_SENSE
  @{ */

/* Symbols to be used with GPIO driver */
#define BOARD_INITPINS_BATT_PYRO_SENSE_GPIO GPIOA                /*!<@brief GPIO peripheral base pointer */
#define BOARD_INITPINS_BATT_PYRO_SENSE_GPIO_PIN_MASK (1U << 17U) /*!<@brief GPIO pin mask */

/* Symbols to be used with PORT driver */
#define BOARD_INITPINS_BATT_PYRO_SENSE_PORT PORTA                /*!<@brief PORT peripheral base pointer */
#define BOARD_INITPINS_BATT_PYRO_SENSE_PIN 17U                   /*!<@brief PORT pin number */
#define BOARD_INITPINS_BATT_PYRO_SENSE_PIN_MASK (1U << 17U)      /*!<@brief PORT pin mask */
                                                                 /* @} */

/*! @name PORTE24 (coord M4), RECO_CS
  @{ */

/* Symbols to be used with GPIO driver */
#define BOARD_INITPINS_RECO_CS_GPIO GPIOE                /*!<@brief GPIO peripheral base pointer */
#define BOARD_INITPINS_RECO_CS_GPIO_PIN_MASK (1U << 24U) /*!<@brief GPIO pin mask */

/* Symbols to be used with PORT driver */
#define BOARD_INITPINS_RECO_CS_PORT PORTE                /*!<@brief PORT peripheral base pointer */
#define BOARD_INITPINS_RECO_CS_PIN 24U                   /*!<@brief PORT pin number */
#define BOARD_INITPINS_RECO_CS_PIN_MASK (1U << 24U)      /*!<@brief PORT pin mask */
                                                         /* @} */

/*!
 * @brief Configures pin routing and optionally pin electrical features.
 *
 */
void BOARD_InitPins(void);

#if defined(__cplusplus)
}
#endif

/*!
 * @}
 */
#endif /* _PIN_MUX_H_ */

/***********************************************************************************************************************
 * EOF
 **********************************************************************************************************************/
