#ifndef BARO_H_
#define BARO_H_

#include "fsl_dspi.h"
#include "fsl_gpio.h"
#include "pin_mux.h"
#include "peripherals.h"


//baro
#define CMD_RESET 		0x1E // ADC reset command
#define CMD_ADC_READ 	0x00 // ADC read command
#define CMD_ADC_CONV 	0x40 // ADC conversion command
#define CMD_ADC_D1 		0x00 // ADC D1 conversion
#define CMD_ADC_D2 		0x10 // ADC D2 conversion
#define CMD_ADC_256 	0x00 // ADC OSR=256
#define CMD_ADC_512 	0x02 // ADC OSR=512
#define CMD_ADC_1024 	0x04 // ADC OSR=1024
#define CMD_ADC_2048 	0x06 // ADC OSR=2056
#define CMD_ADC_4096 	0x08 // ADC OSR=4096
#define CMD_PROM_RD 	0xA0 // Prom read command

unsigned long cmd_adc(char cmd);
uint16_t cmd_prom(char coef_num);
unsigned char crc4(uint16_t n_prom[]);
void baro_cs_enable();
void baro_cs_disable();
void barometer(double *temperature, double *pressure, double *altitude);
void barometer_reset(void);

#endif /* BARO_H_ */
