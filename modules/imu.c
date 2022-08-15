#include "imu.h"

//CS-IMU#	20	PTD5	GPIO Initial Logical 1
//SPI0-MOSI	11	PTC6	SPI	SPI0 - Common for all SPI
//SPI0-MISO	12	PTC7	SPI	SPI0 - Common for all SPI
//SPI0-SCK	13	PTC5	SPI	SPI0 - common for all SPI

//IMU-DR	35	PTC8
//IMU-SYNC	36	PTC9 - ****Disabled and SYNC feature not in use,  the pin is not initialized in peripheral level

static void Delay(uint32_t ticks) {
	while (ticks--) {
		__asm("nop");
	}
}

void imu_cs_enable() {
	GPIO_PinWrite(BOARD_INITPINS_IMU_CS_GPIO, BOARD_INITPINS_IMU_CS_PIN, 0);
}

void imu_cs_disable() {
	GPIO_PinWrite(BOARD_INITPINS_IMU_CS_GPIO, BOARD_INITPINS_IMU_CS_PIN, 1);
}

float IMU_Accel_Scale(int16_t sensorData) {
	float finalData = sensorData * 0.00125; // Multiply by accel sensitivity (0.00125g/LSB)
	return finalData;
}

float IMU_Gyro_Scale(int16_t sensorData) {
	float finalData = sensorData * 0.1; // Multiply by gyro sensitivity (0.1 deg/LSB)
	return finalData;
}

status_t SPI_Read(SPI_Type *base, dspi_half_duplex_transfer_t *xfer) {
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

int IMU_Write_Reg(uint8_t regAddr, int16_t regData) {

	uint16_t addr = (((regAddr & 0x7F) | 0x80) << 8); // Toggle sign bit, and check that the address is 8 bits
	uint16_t lowWord = (addr | (regData & 0xFF)); // OR Register address (A) with data(D) (AADD)
	uint16_t highWord = ((addr | 0x100) | ((regData >> 8) & 0xFF)); // OR Register address with data and increment address

	// Split words into chars
	uint8_t highBytehighWord = (highWord >> 8);
	uint8_t lowBytehighWord = (highWord & 0xFF);
	uint8_t highBytelowWord = (lowWord >> 8);
	uint8_t lowBytelowWord = (lowWord & 0xFF);

	dspi_half_duplex_transfer_t xfer = { 0 };
	uint8_t txData[1] = { highBytelowWord };

	xfer.txData = txData;
	xfer.rxData = NULL;
	xfer.txDataSize = 1;
	xfer.rxDataSize = 0;
	xfer.isTransmitFirst = true;
	xfer.isPcsAssertInTransfer = true;
	xfer.configFlags = kDSPI_MasterCtar0 | kDSPI_MasterPcs0
			| kDSPI_MasterPcsContinuous;

	imu_cs_enable();

	// Write highWord to SPI bus
	SPI_Read(SPI0_PERIPHERAL, &xfer);
	txData[0] = lowBytelowWord;
	xfer.txData = txData;
	SPI_Read(SPI0_PERIPHERAL, &xfer);
	imu_cs_disable();

	//delay stall
	Delay(250);

	imu_cs_enable();
	// Write lowWord to SPI bus
	txData[0] = highBytehighWord;
	xfer.txData = txData;
	SPI_Read(SPI0_PERIPHERAL, &xfer);
	txData[0] = lowBytehighWord;
	xfer.txData = txData;
	SPI_Read(SPI0_PERIPHERAL, &xfer);
	imu_cs_disable();

	//delay stall
	Delay(250);

	return 1;
}

int16_t IMU_Read_Reg(uint8_t regAddr) {

	dspi_half_duplex_transfer_t xfer = { 0 };

	uint8_t txData[] = { regAddr };
	uint8_t rxData[1] = { 0 };

	/*Start Transfer by polling mode. */
	xfer.txData = txData;
	xfer.rxData = NULL;
	xfer.txDataSize = sizeof(txData);
	xfer.rxDataSize = 0;
	xfer.isTransmitFirst = true;
	xfer.isPcsAssertInTransfer = true;
	xfer.configFlags = kDSPI_MasterCtar0 | kDSPI_MasterPcs0
			| kDSPI_MasterPcsContinuous;

	imu_cs_enable();
	SPI_Read(SPI0_PERIPHERAL, &xfer);
	txData[0] = 0x00;
	xfer.txData = txData;
	SPI_Read(SPI0_PERIPHERAL, &xfer);
	imu_cs_disable();

	Delay(250);

	imu_cs_enable();
	xfer.rxData = rxData;
	xfer.rxDataSize = 1;
	SPI_Read(SPI0_PERIPHERAL, &xfer);
	uint8_t _msbData = rxData[0];
	SPI_Read(SPI0_PERIPHERAL, &xfer);
	uint8_t _lsbData = rxData[0];
	imu_cs_disable();

	int16_t _dataOut = (_msbData << 8) | (_lsbData & 0xFF);

	return _dataOut;

}

void imu_reset()
{
	uint16_t FLTR = 0;
	uint16_t DECR = 0;

	//startup delay (>252ms)
	Delay(5001000);

	//software reset sequence
    IMU_Write_Reg(GLOB_CMD,  0xE880);
    Delay(250); //stall period

    IMU_Write_Reg(GLOB_CMD, 0xE900);

    //software reset recovery is 193ms, set to 200ms
	Delay(3001000);

	IMU_Write_Reg(FILT_CTRL, 0x04); //filter control
	IMU_Write_Reg(DEC_RATE, 0x00); //disable decimation

	//validate
	FLTR = IMU_Read_Reg(FILT_CTRL);
	DECR = IMU_Read_Reg(DEC_RATE);

	assert(FLTR == 0x0004);
	assert(DECR == 0x0000);

}
