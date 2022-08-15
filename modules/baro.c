
#include <math.h>
#include "baro.h"

//CS-BARO# PTA16 GPIO - Initial Logical 1
//SPI0-MOSI	11	PTC6	SPI	SPI0
//SPI0-MISO	12	PTC7	SPI	SPI0
//SPI0-SCK	13	PTC5	SPI	SPI0

void delay_ms(uint16_t ms)
{
	uint32_t ticks = 14705 * ms;
	while (ticks--) {
		__asm("nop");
	}
}

static void Delay(uint32_t ticks) {
	while (ticks--) {
		__asm("nop");
	}
}

void baro_cs_enable() {
	GPIO_PinWrite(BOARD_INITPINS_BARO_CS_GPIO, BOARD_INITPINS_BARO_CS_PIN, 0);
}

void baro_cs_disable() {
	GPIO_PinWrite(BOARD_INITPINS_BARO_CS_GPIO, BOARD_INITPINS_BARO_CS_PIN, 1);
}

status_t SPI_Read_Baro(SPI_Type *base, dspi_half_duplex_transfer_t *xfer) {
	assert(NULL != xfer);
	dspi_transfer_t tempXfer = { 0 };
	status_t status;

	if (true == xfer->isTransmitFirst) {
		tempXfer.txData = xfer->txData;
		tempXfer.rxData = xfer->rxData; //NULL;
		tempXfer.dataSize = xfer->txDataSize;
	} else {
		tempXfer.txData = NULL;
		tempXfer.rxData = xfer->rxData;
		tempXfer.dataSize = xfer->rxDataSize;
	}
	/* If the pcs pin keep assert between transmit and receive. */
	if (true == xfer->isPcsAssertInTransfer) {
		tempXfer.configFlags = (xfer->configFlags)
				| (uint32_t) kDSPI_MasterActiveAfterTransfer;
	} else {
		tempXfer.configFlags = (xfer->configFlags)
				& (~(uint32_t) kDSPI_MasterActiveAfterTransfer);
	}

	status = DSPI_MasterTransferBlocking(base, &tempXfer);

	return status;
}

void barometer_reset(void) {

	dspi_half_duplex_transfer_t xfer = { 0 };
	uint8_t txData[] = { CMD_RESET };

	xfer.txData = txData;
	xfer.rxData = NULL;
	xfer.txDataSize = 1; //1byte
	xfer.rxDataSize = 0;
	xfer.isTransmitFirst = true;
	xfer.isPcsAssertInTransfer = true;
	xfer.configFlags = kDSPI_MasterCtar0 | kDSPI_MasterPcs0
			| kDSPI_MasterPcsContinuous;

	baro_cs_enable();

	SPI_Read_Baro(SPI0_PERIPHERAL, &xfer);
	//delay 3ms
	Delay(60000);
//	delay_ms(3);

	baro_cs_disable();

}

unsigned long cmd_adc(char cmd) {
	unsigned long temp = 0;

	dspi_half_duplex_transfer_t xfer = { 0 };
	uint8_t txData[] = { CMD_ADC_CONV + cmd };
	uint8_t ret[1] = { 0 };

	xfer.txData = txData;
	xfer.rxData = NULL;
	xfer.txDataSize = 1; //1byte
	xfer.rxDataSize = 0;
	xfer.isTransmitFirst = true;
	xfer.isPcsAssertInTransfer = true;
	xfer.configFlags = kDSPI_MasterCtar0 | kDSPI_MasterPcs0
			| kDSPI_MasterPcsContinuous;

	baro_cs_enable();
	//send conversion command
	SPI_Read_Baro(SPI0_PERIPHERAL, &xfer); //spi_send(CMD_ADC_CONV+cmd);//transfer continuous

	switch (cmd & 0x0f) // wait necessary conversion time
	{
	case CMD_ADC_256:
		Delay(13300); //_delay_us(900);
		break;
	case CMD_ADC_512:
		Delay(44200); //_delay_ms(3);
		break;
	case CMD_ADC_1024:
		Delay(58900); //_delay_ms(4);
		break;
	case CMD_ADC_2048:
		Delay(88300); //_delay_ms(6);
		break;
	case CMD_ADC_4096:
		Delay(147100); //_delay_ms(10);
		break;
	}

	baro_cs_disable();

	baro_cs_enable();

	//send ADC read command
	txData[0] = CMD_ADC_READ;
	xfer.txData = txData;
	xfer.rxData = 0;
	xfer.rxDataSize = 0;
	SPI_Read_Baro(SPI0_PERIPHERAL, &xfer);

	//read MSB
	xfer.txData = 0x00;
	xfer.rxData = ret;
	xfer.rxDataSize = 1;
	SPI_Read_Baro(SPI0_PERIPHERAL, &xfer);
	temp = 65536 * ret[0];

	//read 2nd byte
	xfer.txData = 0x00;
	xfer.rxData = ret;
	xfer.rxDataSize = 1;
	SPI_Read_Baro(SPI0_PERIPHERAL, &xfer);
	temp = temp + 256 * ret[0];

	//read LSB
	xfer.txData = 0x00;
	xfer.rxData = ret;
	xfer.rxDataSize = 1;
	SPI_Read_Baro(SPI0_PERIPHERAL, &xfer);
	temp = temp + ret[0];

	baro_cs_disable();

	return temp;

}

/**
 * CRC check for data validation
 * The LS 4 bits of the C[7] register is the CRC value
 * Compare the output of this function with LS 4 bits of the C[7] register
 */
unsigned char crc4(uint16_t n_prom[]) {
	int cnt; // simple counter
	unsigned int n_rem; // crc reminder
	unsigned int crc_read; // original value of the crc
	unsigned char n_bit;
	n_rem = 0x00;
	crc_read = n_prom[7]; //save read CRC
	n_prom[7] = (0xFF00 & (n_prom[7])); //CRC byte is replaced by 0
	for (cnt = 0; cnt < 16; cnt++) // operation is performed on bytes
			{ // choose LSB or MSB
		if (cnt % 2 == 1)
			n_rem ^= (unsigned short) ((n_prom[cnt >> 1]) & 0x00FF);
		else
			n_rem ^= (unsigned short) (n_prom[cnt >> 1] >> 8);
		for (n_bit = 8; n_bit > 0; n_bit--) {
			if (n_rem & (0x8000)) {
				n_rem = (n_rem << 1) ^ 0x3000;
			} else {
				n_rem = (n_rem << 1);
			}
		}
	}
	n_rem = (0x000F & (n_rem >> 12)); // // final 4-bit reminder is CRC code
	n_prom[7] = crc_read; // restore the crc_read to its original place
	return (n_rem ^ 0x00);
}

/**
 *
 */
uint16_t cmd_prom(char coef_num) {
	uint16_t rC = 0;

	dspi_half_duplex_transfer_t xfer = { 0 };
	uint8_t txData[] = { CMD_PROM_RD + coef_num * 2 };
	uint8_t ret[2] = { 0 };

	xfer.txData = txData;
	xfer.rxData = ret;
	xfer.txDataSize = 1; //1byte
	xfer.rxDataSize = 2;
	xfer.isTransmitFirst = true;
	xfer.isPcsAssertInTransfer = false;
	xfer.configFlags = kDSPI_MasterCtar0 | kDSPI_MasterPcs0
			| kDSPI_MasterPcsContinuous;

	baro_cs_enable();
	SPI_Read_Baro(SPI0_PERIPHERAL, &xfer);

	xfer.txData = 0x00;
	xfer.rxData = ret;
	xfer.txDataSize = 1; //1byte
	xfer.rxDataSize = 1;
	SPI_Read_Baro(SPI0_PERIPHERAL, &xfer);

	rC = 256 * ret[0];
	SPI_Read_Baro(SPI0_PERIPHERAL, &xfer);

	baro_cs_disable();
	rC = rC + ret[0];

	return rC;

}

void barometer(double *temperature, double *pressure, double *altitude) {
//	unsigned char n_crc;
	unsigned long D1;
	unsigned long D2; // ADC value of the temperature conversion
//	unsigned int C[8]; // calibration coefficients
	double P; // compensated pressure value
	double T; // compensated temperature value
	double dT; // difference between actual and measured temperature
	double OFF; // offset at actual temperature
	double SENS; // sensitivity at actual temperature
	//barometer
	//barometer_reset();

	uint16_t C[8];

	int index = 0;
	for (index = 0; index < 8; index++) {
		C[index] = cmd_prom(index);
	}

	/*validate n_crc value with C[8] 4LSB
	 * TODO: tie this to sensor data error signal
	 */
	//	n_crc = crc4(C);

	D1 = cmd_adc(CMD_ADC_D1 + CMD_ADC_4096); //pressure
	D2 = cmd_adc(CMD_ADC_D2 + CMD_ADC_4096); //temperture

	// calcualte 1st order pressure and temperature (MS5607 1st order algorithm)
	dT = D2 - C[5] * pow(2, 8);
	OFF = C[2] * pow(2, 17) + dT * C[4] / pow(2, 6);
	SENS = C[1] * pow(2, 16) + dT * C[3] / pow(2, 7);
	T = (2000 + (dT * C[6]) / pow(2, 23)) / 100;
	P = (((D1 * SENS) / pow(2, 21) - OFF) / pow(2, 15)) / 100;

	*temperature = T;
	*pressure = P;
	*altitude = ((pow((1013.25 / P), 1 / 5.257) - 1.0) * (T + 273.15)) / 0.0065;



}

