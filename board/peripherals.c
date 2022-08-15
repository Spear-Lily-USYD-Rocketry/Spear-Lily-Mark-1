/***********************************************************************************************************************
 * This file was generated by the MCUXpresso Config Tools. Any manual edits made to this file
 * will be overwritten if the respective MCUXpresso Config Tools is used to update this file.
 **********************************************************************************************************************/

/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
!!GlobalInfo
product: Peripherals v9.0
processor: MK66FX1M0xxx18
package_id: MK66FX1M0VMD18
mcu_data: ksdk2_0
processor_version: 9.0.0
functionalGroups:
- name: BOARD_InitPeripherals
  UUID: ef4b784b-fdb7-40bb-b69b-f30dfc44e546
  called_from_default_init: true
  selectedCore: core0
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/

/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
component:
- type: 'system'
- type_id: 'system_54b53072540eeeb8f8e9343e71f28176'
- global_system_definitions:
  - user_definitions: ''
  - user_includes: ''
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */

/***********************************************************************************************************************
 * Included files
 **********************************************************************************************************************/
#include "peripherals.h"

/***********************************************************************************************************************
 * BOARD_InitPeripherals functional group
 **********************************************************************************************************************/
/***********************************************************************************************************************
 * NVIC initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'NVIC'
- type: 'nvic'
- mode: 'general'
- custom_name_enabled: 'false'
- type_id: 'nvic_57b5eef3774cc60acaede6f5b8bddc67'
- functional_group: 'BOARD_InitPeripherals'
- peripheral: 'NVIC'
- config_sets:
  - nvic:
    - interrupt_table: []
    - interrupts: []
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */

/* Empty initialization function (commented out)
static void NVIC_init(void) {
} */

/***********************************************************************************************************************
 * SPI0 initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'SPI0'
- type: 'dspi'
- mode: 'DSPI_Polling'
- custom_name_enabled: 'false'
- type_id: 'dspi_305e5b03c593d065f61ded8061d15797'
- functional_group: 'BOARD_InitPeripherals'
- peripheral: 'SPI0'
- config_sets:
  - fsl_dspi:
    - dspi_mode: 'kDSPI_Master'
    - clockSource: 'BusInterfaceClock'
    - clockSourceFreq: 'GetFreq'
    - dspi_master_config:
      - whichCtar: 'kDSPI_Ctar0'
      - ctarConfig:
        - baudRate: '500000'
        - bitsPerFrame: '8'
        - cpol: 'kDSPI_ClockPolarityActiveLow'
        - cpha: 'kDSPI_ClockPhaseSecondEdge'
        - direction: 'kDSPI_MsbFirst'
        - pcsToSckDelayInNanoSec: '1000'
        - lastSckToPcsDelayInNanoSec: '1000'
        - betweenTransferDelayInNanoSec: '1000'
      - whichPcs: 'PCS0_SS'
      - pcsActiveHighOrLow: 'kDSPI_PcsActiveLow'
      - enableContinuousSCK: 'false'
      - enableRxFifoOverWrite: 'false'
      - enableModifiedTimingFormat: 'false'
      - samplePoint: 'kDSPI_SckToSin0Clock'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */
const dspi_master_config_t SPI0_config = {
  .whichCtar = kDSPI_Ctar0,
  .ctarConfig = {
    .baudRate = 500000UL,
    .bitsPerFrame = 8UL,
    .cpol = kDSPI_ClockPolarityActiveLow,
    .cpha = kDSPI_ClockPhaseSecondEdge,
    .direction = kDSPI_MsbFirst,
    .pcsToSckDelayInNanoSec = 1000UL,
    .lastSckToPcsDelayInNanoSec = 1000UL,
    .betweenTransferDelayInNanoSec = 1000UL
  },
  .whichPcs = kDSPI_Pcs0,
  .pcsActiveHighOrLow = kDSPI_PcsActiveLow,
  .enableContinuousSCK = false,
  .enableRxFifoOverWrite = false,
  .enableModifiedTimingFormat = false,
  .samplePoint = kDSPI_SckToSin0Clock
};

static void SPI0_init(void) {
  /* Initialization function */
  DSPI_MasterInit(SPI0_PERIPHERAL, &SPI0_config, SPI0_CLK_FREQ);
}

/***********************************************************************************************************************
 * UART0 initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'UART0'
- type: 'uart'
- mode: 'polling'
- custom_name_enabled: 'false'
- type_id: 'uart_88ab1eca0cddb7ee407685775de016d5'
- functional_group: 'BOARD_InitPeripherals'
- peripheral: 'UART0'
- config_sets:
  - uartConfig_t:
    - uartConfig:
      - clockSource: 'BusInterfaceClock'
      - clockSourceFreq: 'GetFreq'
      - baudRate_Bps: '9600'
      - parityMode: 'kUART_ParityDisabled'
      - stopBitCount: 'kUART_OneStopBit'
      - txFifoWatermark: '0'
      - rxFifoWatermark: '1'
      - idleType: 'kUART_IdleTypeStartBit'
      - enableTx: 'true'
      - enableRx: 'true'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */
const uart_config_t UART0_config = {
  .baudRate_Bps = 9600UL,
  .parityMode = kUART_ParityDisabled,
  .stopBitCount = kUART_OneStopBit,
  .txFifoWatermark = 0U,
  .rxFifoWatermark = 1U,
  .idleType = kUART_IdleTypeStartBit,
  .enableTx = true,
  .enableRx = true
};

static void UART0_init(void) {
  UART_Init(UART0_PERIPHERAL, &UART0_config, UART0_CLOCK_SOURCE);
}

/***********************************************************************************************************************
 * Initialization functions
 **********************************************************************************************************************/
void BOARD_InitPeripherals(void)
{
  /* Initialize components */
  SPI0_init();
  UART0_init();
}

/***********************************************************************************************************************
 * BOARD_InitBootPeripherals function
 **********************************************************************************************************************/
void BOARD_InitBootPeripherals(void)
{
  BOARD_InitPeripherals();
}