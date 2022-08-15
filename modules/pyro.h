
#ifndef PYRO_H_
#define PYRO_H_

#include "boolean.h"
#include "fsl_dspi.h"
#include "fsl_gpio.h"
#include "pin_mux.h"
#include "peripherals.h"

//const uint8_t MCP23S08_PIN_COUNT = 8;

#define IODIR		0x00
#define IPOL		0x01
#define GPINTEN		0x02
#define DEFVAL		0x03
#define INTCON		0x04
#define IOCON		0x05
#define GPPU		0x06
#define INTF		0x07
#define INTCAP		0x08
#define GPIO		0x09
#define OLAT		0x0A

#define OUTPUT 			0
#define INPUT			1
#define INPUT_PULLUP	3

#define PYRO_FIRE_0_DROGUE	7
#define PYRO_FIRE_1_MAIN	6
#define PYRO_FIRE_2_DROGUE	5
#define PYRO_FIRE_3_MAIN	4

#define PYRO_SENSE_0_DROGUE 3
#define PYRO_SENSE_1_MAIN	2
#define	PYRO_SENSE_2_DROGUE	1
#define	PYRO_SENSE_3_MAIN	0

// #define bool _Bool
// #define true 1
// #define false 0

#define HIGH 	1
#define LOW		0

#define _deviceOpcode	0x40

void reco_cs_enable();
void reco_cs_disable();

uint8_t getPullup();
uint8_t getMode();
uint8_t getOutput();
uint8_t getInput();
void setPullup(uint8_t state);
void setMode(uint8_t mode);
void setOutput(uint8_t state);
void pinMode(uint8_t pin, uint8_t mode);
void expandr_reset();
void expandr_setup();
void pyro_enable();
uint8_t read_batt_pyro_sense();

uint8_t RECO_Read_Reg(uint8_t regAddr);
void RECO_Write_Reg(uint8_t regAddr, uint8_t data);

uint8_t pyro_gpio_read(uint8_t pin);
void pyro_gpio_write(uint8_t pin, bool state);


#endif /* PYRO_H_ */
