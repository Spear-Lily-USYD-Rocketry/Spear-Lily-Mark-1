#include "pyro.h"


void reco_cs_enable() {
	GPIO_PinWrite(BOARD_INITPINS_RECO_CS_GPIO, BOARD_INITPINS_RECO_CS_PIN, 0);
}

void reco_cs_disable() {
	GPIO_PinWrite(BOARD_INITPINS_RECO_CS_GPIO, BOARD_INITPINS_RECO_CS_PIN, 1);
}

status_t SPI_EXPR_Read(SPI_Type *base, dspi_half_duplex_transfer_t *xfer) {
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

uint8_t RECO_Read_Reg(uint8_t regAddr) {

	dspi_half_duplex_transfer_t xfer = { 0 };

	uint8_t txData[] = { _deviceOpcode | 1 };
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

	reco_cs_enable();
	SPI_EXPR_Read(SPI0_PERIPHERAL, &xfer);
	txData[0] = regAddr;
	xfer.txData = txData;
	SPI_EXPR_Read(SPI0_PERIPHERAL, &xfer);

	txData[0] = 0x00;
	xfer.txData = txData;
	xfer.rxData = rxData;
	xfer.rxDataSize = 1;
	SPI_EXPR_Read(SPI0_PERIPHERAL, &xfer);
	uint8_t data = rxData[0];

	reco_cs_disable();

	return data;

}

void RECO_Write_Reg(uint8_t regAddr, uint8_t data) {

	dspi_half_duplex_transfer_t xfer = { 0 };

	uint8_t txData[] = { _deviceOpcode };

	xfer.txData = txData;
	xfer.rxData = NULL;
	xfer.txDataSize = 1;
	xfer.rxDataSize = 0;
	xfer.isTransmitFirst = true;
	xfer.isPcsAssertInTransfer = true;
	xfer.configFlags = kDSPI_MasterCtar0 | kDSPI_MasterPcs0
			| kDSPI_MasterPcsContinuous;

	reco_cs_enable();

	SPI_EXPR_Read(SPI0_PERIPHERAL, &xfer);
	txData[0] = regAddr;
	xfer.txData = txData;
	SPI_EXPR_Read(SPI0_PERIPHERAL, &xfer);
	txData[0] = data;
	SPI_EXPR_Read(SPI0_PERIPHERAL, &xfer);

	reco_cs_disable();

}

uint8_t getPullup() {
	return RECO_Read_Reg(GPPU);
}

uint8_t getMode() {
	return ~(RECO_Read_Reg(IODIR));
}

uint8_t getOutput() {
	return RECO_Read_Reg(OLAT);
}

uint8_t getInput() {
	return RECO_Read_Reg(GPIO);
}

void setPullup(uint8_t state) {
	RECO_Write_Reg(GPPU, state);
}

void setMode(uint8_t mode) {
	RECO_Write_Reg(IODIR, ~(mode));
}

void setOutput(uint8_t state) {
	RECO_Write_Reg(OLAT, state);
}

void pinMode(uint8_t pin, uint8_t mode) {

	switch (mode) {
		case INPUT:
			setMode(getMode() & ~(1 << pin));		// set pin direction to input
			setPullup(getPullup() & ~(1 << pin));	// disable pullup for pin
			break;
		case OUTPUT:
			setMode(getMode() | (1 << pin));		// set pin direction to output
			setPullup(getPullup() & ~(1 << pin));	// disable pullup for pin
			break;
		case INPUT_PULLUP:
			setMode(getMode() & ~(1 << pin));		// set pin direction to input
			setPullup(getPullup() | (1 << pin));	// enable pullup for pin
			break;
	}
}

void expandr_reset() {
	RECO_Write_Reg(IODIR, 0xFF);
	for (uint8_t i = IPOL; i <= OLAT; i++) {
		RECO_Write_Reg(i, 0x00);
	}
}

void expandr_setup()
{

	//GP0 - sense3 GP1 - sense2 GP3 - sense0 GP2 - sense1
	pinMode(0, INPUT_PULLUP);
	pinMode(1, INPUT_PULLUP);
	pinMode(2, INPUT_PULLUP);
	pinMode(3, INPUT_PULLUP);

	//GP4 - fire3 GP5 - fire
	pinMode(4, OUTPUT);
	pinMode(5, OUTPUT);
	pinMode(6, OUTPUT);
	pinMode(7, OUTPUT);

}

void pyro_enable()
{
	GPIO_PinWrite(BOARD_INITPINS_PYRO_ENABLE_GPIO, BOARD_INITPINS_PYRO_ENABLE_PIN, 1);
}

uint8_t read_batt_pyro_sense()
{
	return GPIO_PinRead(BOARD_INITPINS_BATT_PYRO_SENSE_GPIO, BOARD_INITPINS_BATT_PYRO_SENSE_PIN);
}

uint8_t pyro_gpio_read(uint8_t pin)
{
	return (getInput() >> pin) & 1;
}

void pyro_gpio_write(uint8_t pin, bool state)
{
	setOutput((getOutput() & ~(1 << pin)) | (state << pin));
}
