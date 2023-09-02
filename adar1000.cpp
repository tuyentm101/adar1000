/*
 * Adar1000.cpp
 *
 *  Created on: Apr 24, 2023
 *      Author: tuyentm4
 */

#include "adar1000.h"
#include "utils/bit_manipulation.h"
#include "logger.h"

/* ADAR1000 Chip Addressing -------------------------------------------------*/
#define ADAR1000_CHIP_ADDR1			1
#define ADAR1000_CHIP_ADDR2			2
#define ADAR1000_CHIP_ADDR3			3
#define ADAR1000_CHIP_ADDR4			4
#define	ADAR1000_CHIP_ADDRALL		255

/* ADAR1000 Registers Address -------------------------------------------------*/
#define ADAR1000_INTERFACE_CONFIG_A		0x000
#define ADAR1000_INTERFACE_CONFIG_B		0x001
#define ADAR1000_DEV_CONFIG				0x002
#define ADAR1000_CHIP_TYPE				0x003
#define ADAR1000_PRODUCT_ID_H			0x004
#define ADAR1000_PRODUCT_ID_L			0x005
#define ADAR1000_SCRATCH_PAD			0x00A
#define ADAR1000_SPI_REV				0x00B
#define ADAR1000_VENDOR_ID_H			0x00C
#define ADAR1000_VENDOR_ID_L			0x00D
#define	ADAR1000_TRANSFER_REG			0x00F

#define	ADAR1000_CH1_RX_GAIN			0x010
#define	ADAR1000_CH2_RX_GAIN			0x011
#define	ADAR1000_CH3_RX_GAIN			0x012
#define	ADAR1000_CH4_RX_GAIN			0x013
#define ADAR1000_RX_GAIN(channel)		(0x010 + (channel))

#define	ADAR1000_CH1_RX_PHASE_I			0x014
#define	ADAR1000_CH1_RX_PHASE_Q			0x015
#define	ADAR1000_CH2_RX_PHASE_I			0x016
#define	ADAR1000_CH2_RX_PHASE_Q			0x017
#define	ADAR1000_CH3_RX_PHASE_I			0x018
#define	ADAR1000_CH3_RX_PHASE_Q			0x019
#define	ADAR1000_CH4_RX_PHASE_I			0x01A
#define	ADAR1000_CH4_RX_PHASE_Q			0x01B
#define ADAR1000_RX_PHASE_I(channel)	(0x014 + 2 * (channel))
#define ADAR1000_RX_PHASE_Q(channel)	(0x015 + 2 * (channel))

#define ADAR1000_CH1_TX_GAIN			0x01C
#define ADAR1000_CH2_TX_GAIN			0x01D
#define ADAR1000_CH3_TX_GAIN			0x01E
#define ADAR1000_CH4_TX_GAIN			0x01F
#define ADAR1000_TX_GAIN(channel)		(0x01C + (channel))

#define ADAR1000_CH1_TX_PHASE_I			0x020
#define ADAR1000_CH1_TX_PHASE_Q			0x021
#define ADAR1000_CH2_TX_PHASE_I			0x022
#define ADAR1000_CH2_TX_PHASE_Q			0x023
#define ADAR1000_CH3_TX_PHASE_I			0x024
#define ADAR1000_CH3_TX_PHASE_Q			0x025
#define ADAR1000_CH4_TX_PHASE_I			0x026
#define ADAR1000_CH4_TX_PHASE_Q			0x027
#define ADAR1000_TX_PHASE_I(channel)	(0x020 + 2 * (channel))
#define ADAR1000_TX_PHASE_Q(channel)	(0x021 + 2 * (channel))

#define ADAR1000_LD_WRK_REGS			0x028

#define	ADAR1000_CH1_PA_BIAS_ON			0x029
#define	ADAR1000_CH2_PA_BIAS_ON			0x02A
#define	ADAR1000_CH3_PA_BIAS_ON			0x02B
#define	ADAR1000_CH4_PA_BIAS_ON			0x02C
#define ADAR1000_PA_BIAS_ON(channel)    (0x29 + (channel))

#define ADAR1000_LNA_BIAS_ON			0x02D
#define	ADAR1000_RX_ENABLES				0x02E
#define	ADAR1000_TX_ENABLES				0x02F
#define	ADAR1000_MISC_ENABLES			0x030
#define	ADAR1000_SW_CTRL				0x031
#define	ADAR1000_ADC_CTRL				0x032
#define	ADAR1000_ADC_OUTPUT				0x033

#define	ADAR1000_BIAS_CURRENT_RX_LNA	0x034
#define	ADAR1000_BIAS_CURRENT_RX		0x035
#define	ADAR1000_BIAS_CURRENT_TX		0x036
#define	ADAR1000_BIAS_CURRENT_TX_DRV	0x037

#define ADAR1000_MEM_CTRL				0x038

#define ADAR1000_RX_CHX_MEM				0x039
#define ADAR1000_RX_CH1_MEM				0x03D
#define ADAR1000_RX_CH2_MEM				0x03E
#define ADAR1000_RX_CH3_MEM				0x03F
#define	ADAR1000_RX_CH4_MEM				0x040
#define ADAR1000_RX_MEM(channel)		(0x03D + (channel))

#define ADAR1000_TX_CHX_MEM				0x03A
#define ADAR1000_TX_CH1_MEM				0x041
#define ADAR1000_TX_CH2_MEM				0x042
#define ADAR1000_TX_CH3_MEM				0x043
#define	ADAR1000_TX_CH4_MEM				0x044
#define ADAR1000_TX_MEM(channel)		(0x041 + (channel))

#define	ADAR1000_REV_ID					0x045

#define	ADAR1000_CH1_PA_BIAS_OFF		0x046
#define	ADAR1000_CH2_PA_BIAS_OFF		0x047
#define	ADAR1000_CH3_PA_BIAS_OFF		0x048
#define	ADAR1000_CH4_PA_BIAS_OFF		0x049
#define ADAR1000_PA_BIAS_OFF(channel)	(0x046 + (channel))

#define	ADAR1000_LNA_BIAS_OFF			0x04A
#define	ADAR1000_TX_TO_RX_DELAY_CTRL	0x04B
#define	ADAR1000_RX_TO_TX_DELAY_CTRL	0x04C

#define	ADAR1000_TX_BEAM_STEP_START		0x04D
#define	ADAR1000_TX_BEAM_STEP_STOP		0x04E
#define	ADAR1000_RX_BEAM_STEP_START		0x04F
#define	ADAR1000_RX_BEAM_STEP_STOP		0x050

#define ADAR1000_RX_BIAS_RAM_CTL		0x051
#define	ADAR1000_TX_BIAS_RAM_CTL		0x052

#define	ADAR1000_LDO_TRIM_CTL_0			0x400
#define	ADAR1000_LDO_TROL_CTL_1			0x401

#define ADAR1000_BEAMPOSITION_RX_GAIN(channel, pos)			(0x1000 | (channel << 2) | (pos << 4))
#define ADAR1000_BEAMPOSITION_RX_PHASE_I(channel, pos)		(0x1001 | (channel << 2) | (pos << 4))
#define ADAR1000_BEAMPOSITION_RX_PHASE_Q(channel, pos)		(0x1002 | (channel << 2) | (pos << 4))

#define ADAR1000_BEAMPOSITION_TX_GAIN(channel, pos)			(0x1800 | (channel << 2) | (pos << 4))
#define ADAR1000_BEAMPOSITION_TX_PHASE_I(channel, pos)		(0x1801 | (channel << 2) | (pos << 4))
#define ADAR1000_BEAMPOSITION_TX_PHASE_Q(channel, pos)		(0x1802 | (channel << 2) | (pos << 4))

#define ADAR1000_BIASSETTING_TX_CH1_PABIAS_OFF(n)			(0x1F90 + (n << 4)) // n = 0 ... 6
#define ADAR1000_BIASSETTING_TX_CH2_PABIAS_OFF(n)			(0x1F91 + (n << 4)) // n = 0 ... 6
#define ADAR1000_BIASSETTING_TX_CH3_PABIAS_OFF(n)			(0x1F92 + (n << 4)) // n = 0 ... 6
#define ADAR1000_BIASSETTING_TX_CH4_PABIAS_OFF(n)			(0x1F98 + (n << 4)) // n = 0 ... 6

#define ADAR1000_BIASSETTING_TX_CH1_PABIAS_ON(n)			(0x1F94 + (n << 4)) // n = 0 ... 6
#define ADAR1000_BIASSETTING_TX_CH2_PABIAS_ON(n)			(0x1F95 + (n << 4)) // n = 0 ... 6
#define ADAR1000_BIASSETTING_TX_CH3_PABIAS_ON(n)			(0x1F96 + (n << 4)) // n = 0 ... 6
#define ADAR1000_BIASSETTING_TX_CH4_PABIAS_ON(n)			(0x1F99 + (n << 4)) // n = 0 ... 6

/* ADAR1000 Registers Mask bit -------------------------------------------------*/
// ADAR1000_RX_ENABLE bits
#define ADAR1000_RX_ENABLES_CH1_RX_EN_BIT 	(6) // Enables Receive Channel 1 Subcircuits
#define ADAR1000_RX_ENABLES_CH2_RX_EN_BIT 	(5) // Enables Receive Channel 2 Subcircuits
#define ADAR1000_RX_ENABLES_CH3_RX_EN_BIT 	(4) // Enables Receive Channel 3 Subcircuits
#define ADAR1000_RX_ENABLES_CH4_RX_EN_BIT	(3) // Enables Receive Channel 4 Subcircuits
#define ADAR1000_RX_ENABLES_RX_LNA_EN_BIT	(2) // Enables the Receive Channel LNAs
#define ADAR1000_RX_ENABLES_RX_VM_EN_BIT	(1) // Enables the Receive Channel Vector Modulators
#define ADAR1000_RX_ENABLES_RX_VGA_EN_BIT	(0) // Enables the Receive Channel VGAs

// ADAR1000_RX_ENABLE bits
#define ADAR1000_TX_ENABLES_CH1_TX_EN_BIT 	(6) // Enables Transmit Channel 1 Subcircuits
#define ADAR1000_TX_ENABLES_CH2_TX_EN_BIT 	(5) // Enables Transmit Channel 2 Subcircuits
#define ADAR1000_TX_ENABLES_CH3_TX_EN_BIT 	(4) // Enables Transmit Channel 3 Subcircuits
#define ADAR1000_TX_ENABLES_CH4_TX_EN_BIT	(3) // Enables Transmit Channel 4 Subcircuits
#define ADAR1000_TX_ENABLES_TX_DRV_EN_BIT	(2) // Enables the Transmit Channel Drivers
#define ADAR1000_TX_ENABLES_TX_VM_EN_BIT	(1) // Enables the Transmit Channel Vector Modulators
#define ADAR1000_TX_ENABLES_TX_VGA_EN_BIT	(0) // Enables the Transmit Channel VGAs

// ADAR1000_ADC_CTRL mask bit
#define ADAR1000_ADC_CTRL_MUX_SEL								(0x0E)
#define ADAR1000_ADC_CTRL_MUX_SEL_TEMPERATURE					(0)
#define ADAR1000_ADC_CTRL_MUX_SEL_DETECTOR(channel)  		    ((channel + 1) << 1)
#define ADAR1000_ADC_CTRL_ST_CONV								(1 << 4)
#define ADAR1000_ADC_CTRL_EOC									(1)

// ADAR1000_SW_CTRL bits
#define ADAR1000_SW_CTRL_POL_BIT								(0) // Control for External Polarity Switch Drivers.
																	// 0 = outputs 0 V, and 1 = outputs −5 V, if switch is enabled.
#define ADAR1000_SW_CTRL_TR_SPI_BIT								(1) // Transmit or Receive mode while in SPI Control. 0 = receive and 1 = transmit.
#define ADAR1000_SW_CTRL_TR_SOURCE_BIT							(2) // Source for Transmit/Receive Control. 0 = TR_SPI, 1 = TR input.
#define ADAR1000_SW_CTRL_SW_DRV_EN_POL_BIT						(3) // Enables Switch Driver for External Polarization Switch. 1 = enabled.
#define ADAR1000_SW_CTRL_SW_DRV_EN_TR_BIT						(4) // Enables Switch Driver for External Transmit/Receive Switch. 1 = enabled.
#define ADAR1000_SW_CTRL_RX_EN_BIT								(5) // Enables Receive Channel Subcircuits when Under SPI Control. 1 = enabled.
#define ADAR1000_SW_CTRL_TX_EN_BIT								(6) // Enables Transmit Channel Subcircuits when Under SPI Control. 1 = enabled.
#define ADAR1000_SW_CTRL_SW_DRV_TR_STATE_BIT					(7) // Controls Polarity of Transmit/Receive Switch Driver Output.
																	// If 0, the driver outputs 0 V in receive mode.
// ADAR1000_SW_CTRL_TR_SOURCE bit value
#define ADAR1000_SW_CTRL_TR_SOURCE_BIT_SPI						(0)
#define ADAR1000_SW_CTRL_TR_SOURCE_BIT_TR_PIN					(1)

// ADAR1000_MISC_ENABLES bits
#define ADAR1000_MISC_ENABLES_CH4_DET_EN_BIT						(0) // Enables Channel 4 Power Detector
#define ADAR1000_MISC_ENABLES_CH3_DET_EN_BIT						(1)
#define ADAR1000_MISC_ENABLES_CH2_DET_EN_BIT						(2)
#define ADAR1000_MISC_ENABLES_CH1_DET_EN_BIT						(3)
#define ADAR1000_MISC_ENABLES_LNA_BIAS_OUT_EN_BIT					(4) // Enables Output of LNA Bias DAC. 0 = open and 1 = bias connected
#define ADAR1000_MISC_ENABLES_BIAS_EN_BIT							(5) // Enables PA and LNA Bias DACs. 0 = enabled.
#define ADAR1000_MISC_ENABLES_BIAS_CTRL_BIT							(6) // External Amplifier Bias Control.
																		// If 0, DACs assume the on register values.
																		// If 1, DACs vary with device mode (transmit and receive).
#define ADAR1000_MISC_ENABLES_SW_DRV_TR_MODE_SEL_BIT				(7) // Transmit/Receive Output Driver Select.
																		// If 0, TR_SW_POS is enabled, and if 1, TR_SW_NEG is enabled.

// ADAR1000_MEM_CTRL bits
#define ADAR1000_MEM_CTRL_RX_CHX_RAM_BYPASS_BIT					(0)
#define ADAR1000_MEM_CTRL_TX_CHX_RAM_BYPASS_BIT					(1)
#define ADAR1000_MEM_CTRL_RX_BEAM_STEP_EN_BIT					(2)
#define ADAR1000_MEM_CTRL_TX_BEAM_STEP_EN_BIT					(3)
// bit 4 reserved
#define ADAR1000_MEM_CTRL_BIAS_RAM_BYPASS_BIT					(5)
#define ADAR1000_MEM_CTRL_BEAM_RAM_BYPASS_BIT					(6)
#define ADAR1000_MEM_CTRL_SCAN_MODE_EN_BIT						(7)


/* ADAR1000 SPI -------------------------------------------------*/
#define SPI_READ_CMD			(1)
#define SPI_WRITE_CMD			(0)
#define ADAR1000_FRAME(cmd, chipAddr, regAddr, data) 		(((uint32_t)cmd << 23) | ((uint32_t)chipAddr << 21) | ((uint32_t)regAddr << 8) | data)

struct Adar1000Phase_s {
	uint16_t integerPart;
	uint16_t fractionalPart;
	uint8_t regI;
	uint8_t regQ;
};

#define N_PHASEMAP			(128)
static const struct Adar1000Phase_s g_adar1000PhaseMap[N_PHASEMAP] = ADAR1000_PHASEMAP;

bool Adar1000::searchPhaseReg(uint16_t integralPhase, uint16_t fractionalPhase, uint8_t &regI, uint8_t &regQ)
{
	int l = 0;
	int r = N_PHASEMAP - 1;

	while (l <= r) {
		int m = (l + r) >> 1;
		if (integralPhase == g_adar1000PhaseMap[m].integerPart
				&& fractionalPhase == g_adar1000PhaseMap[m].fractionalPart)
		{
			regI = g_adar1000PhaseMap[m].regI;
			regQ = g_adar1000PhaseMap[m].regQ;
			return true;
		}
		if (integralPhase < g_adar1000PhaseMap[m].integerPart) {
			r = m - 1;
		}
		else {
			l = m + 1;
		}
	}
	return false;
}

bool Adar1000::searchPhase(uint8_t regI, uint8_t regQ, uint16_t &integralPhase, uint16_t &fractionalPhase)
{
	bool ret = false;
	for (int i = 0; i < N_PHASEMAP; ++i) {
		if (g_adar1000PhaseMap[i].regI == regI && g_adar1000PhaseMap[i].regQ == regQ) {
			integralPhase = g_adar1000PhaseMap[i].integerPart;
			fractionalPhase = g_adar1000PhaseMap[i].fractionalPart;
			ret = true;
		}
	}
	return ret;
}

bool Adar1000::writeRegister(uint32_t addr, uint8_t data)
{
	uint32_t spiPayload = 0;
	bool spiSendRet = true;

	spiPayload = ADAR1000_FRAME(SPI_WRITE_CMD, m_chipAddress, addr, data);
	m_pSpi->send(m_chipAddress, (uint8_t*)&spiPayload, 3);

	return spiSendRet;
}

bool Adar1000::readRegister(uint32_t addr, uint8_t &data)
{
	uint32_t spiPayload = 0;
	bool spiSendRet = true;
	uint32_t getData = 0;

	spiPayload = ADAR1000_FRAME(SPI_READ_CMD, m_chipAddress, addr, 0);
	m_pSpi->read(getData, (uint8_t*)&spiPayload, (uint8_t*)&getData, 3);
	data = (uint8_t)getData;

	return spiSendRet;
}

bool Adar1000::configBeamDataLoadFromRegister()
{
	uint8_t memCtrlRegVal = 0;

	if (!readRegister(ADAR1000_MEM_CTRL, memCtrlRegVal)) return false;
	BIT_SET(memCtrlRegVal, ADAR1000_MEM_CTRL_BEAM_RAM_BYPASS_BIT);

	return writeRegister(ADAR1000_MEM_CTRL, memCtrlRegVal);
}

/* follow Sequencing Through Memory Beam Positions section (page 33/79) */
bool Adar1000::configBeamDataLoadFromRAM(Adar1000Direction_t direction, bool isSingleChannelSet)
{
	uint8_t memCtrlRegVal = 0;

	if (!readRegister(ADAR1000_MEM_CTRL, memCtrlRegVal)) return false;
	BIT_CLR(memCtrlRegVal, ADAR1000_MEM_CTRL_BEAM_RAM_BYPASS_BIT);

	switch (direction) {

		case ADAR1000_DIRECTION_TX:
			if (isSingleChannelSet) BIT_SET(memCtrlRegVal, ADAR1000_MEM_CTRL_TX_CHX_RAM_BYPASS_BIT);
			else BIT_CLR(memCtrlRegVal, ADAR1000_MEM_CTRL_TX_CHX_RAM_BYPASS_BIT);
		break;

		case ADAR1000_DIRECTION_RX:
			if (isSingleChannelSet) BIT_SET(memCtrlRegVal, ADAR1000_MEM_CTRL_RX_CHX_RAM_BYPASS_BIT);
			else BIT_CLR(memCtrlRegVal, ADAR1000_MEM_CTRL_RX_CHX_RAM_BYPASS_BIT);
	}

	if (!writeRegister(ADAR1000_MEM_CTRL, memCtrlRegVal)) return false;

	return true;

//	/* provide at least six additional clock cycles */
//	return readRegister(ADAR1000_MEM_CTRL, memCtrlRegVal);
}

Adar1000::Adar1000(const char* name,
		uint8_t chipAddress,
		Spi* const p_spi,
		Gpio* const p_addr0Pin,
		Gpio* const p_addr1Pin,
		Gpio* const p_paOnPin)
: 	m_name(name),
	m_chipAddress(chipAddress & 3), // make sure chip address range from 0 to 3
	m_pAddr0Pin(p_addr0Pin),
	m_pAddr1Pin(p_addr1Pin),
	m_pPaOnPin(p_paOnPin),
	m_pSpi(p_spi)
{
}

Adar1000::~Adar1000() {}

const char *Adar1000::getName() const
{
	return m_name;
}

bool Adar1000::init()
{
	if (!m_pAddr0Pin || !m_pAddr1Pin) return false;

	/* Set device address */
	m_pAddr0Pin->writeOutputPin(m_chipAddress & 1);
	m_pAddr1Pin->writeOutputPin((m_chipAddress >> 1) & 1);

	uint8_t writeDataReg;
	uint8_t readDataReg;

	/* Reset whole chip, use SDO line for readback, address auto incrementing in block write mode. */
	writeDataReg = 0xBD;
	if (!writeRegister(ADAR1000_INTERFACE_CONFIG_A, writeDataReg)) return false;
	readDataReg = 0;
	if (!readRegister(ADAR1000_INTERFACE_CONFIG_A, readDataReg)) return false;
	if (readDataReg != writeDataReg) {
		LOG_ERROR("writeDataReg %d, readDataReg %d", writeDataReg, readDataReg);
		return false;
	}

	/* Init internal ADC:
	 * - Turn on ADC clock oscillator
	 * - Turn on and reset ADC
	 *  */
	if (!readRegister(ADAR1000_ADC_CTRL, readDataReg)) return false;
	writeDataReg = readDataReg;
	BIT_SET(writeDataReg, 6); // ADC_EN
	BIT_SET(writeDataReg, 5); // CLK_EN
	if (!writeRegister(ADAR1000_ADC_CTRL, writeDataReg)) return false;
	readDataReg = 0;
	if (!readRegister(ADAR1000_ADC_CTRL, readDataReg)) return false;
	if (readDataReg != writeDataReg) {
		LOG_ERROR("writeDataReg %d, readDataReg %d", writeDataReg, readDataReg);
		return false;
	}

	/* Set bit SW_DRV_TR_STATE for Controls Polarity of Transmit/Receive Switch Driver Output
	 * ----------- Rx mode -------------- Tx mode
	 * TR_SW_POS	3.3V					0V
	 * TR_SW_NEG	-5V						0V
	 *  */
	writeDataReg = 1 << ADAR1000_SW_CTRL_SW_DRV_TR_STATE_BIT;
	if (!writeRegister(ADAR1000_SW_CTRL, writeDataReg)) return false;
	readDataReg = 0;
	if (!readRegister(ADAR1000_SW_CTRL, readDataReg)) return false;
	if (readDataReg != writeDataReg) {
		LOG_ERROR("writeDataReg %d, readDataReg %d", writeDataReg, readDataReg);
		return false;
	}

	return true;
}

bool Adar1000::controlPowerDetector(Adar1000Channel_t channel, Adar1000ControlState_t state)
{
	uint8_t miscEnablesRegVal = 0;

	if (!readRegister(ADAR1000_MISC_ENABLES, miscEnablesRegVal)) return false;

	switch (channel) {
		case ADAR1000_CHANNEL_1:
			if (state == ADAR1000_CONTROL_STATE_ON)
				BIT_SET(miscEnablesRegVal, ADAR1000_MISC_ENABLES_CH1_DET_EN_BIT);
			else BIT_CLR(miscEnablesRegVal, ADAR1000_MISC_ENABLES_CH1_DET_EN_BIT);
		break;

		case ADAR1000_CHANNEL_2:
			if (state == ADAR1000_CONTROL_STATE_ON)
				BIT_SET(miscEnablesRegVal, ADAR1000_MISC_ENABLES_CH2_DET_EN_BIT);
			else BIT_CLR(miscEnablesRegVal, ADAR1000_MISC_ENABLES_CH2_DET_EN_BIT);
		break;

		case ADAR1000_CHANNEL_3:
			if (state == ADAR1000_CONTROL_STATE_ON)
				BIT_SET(miscEnablesRegVal, ADAR1000_MISC_ENABLES_CH3_DET_EN_BIT);
			else BIT_CLR(miscEnablesRegVal, ADAR1000_MISC_ENABLES_CH3_DET_EN_BIT);
		break;

		case ADAR1000_CHANNEL_4:
			if (state == ADAR1000_CONTROL_STATE_ON)
				BIT_SET(miscEnablesRegVal, ADAR1000_MISC_ENABLES_CH4_DET_EN_BIT);
			else BIT_CLR(miscEnablesRegVal, ADAR1000_MISC_ENABLES_CH4_DET_EN_BIT);
	}

	if (!writeRegister(ADAR1000_MISC_ENABLES, miscEnablesRegVal)) return false;

	return true;
}

bool Adar1000::getAdcTemperature(uint8_t &adcValue)
{
	uint8_t data = 0;
	uint8_t nTry = 0;

	if (!readRegister(ADAR1000_ADC_CTRL, data)) return false;
	data = BIT_MASK(data, ADAR1000_ADC_CTRL_MUX_SEL, ADAR1000_ADC_CTRL_MUX_SEL_TEMPERATURE | ADAR1000_ADC_CTRL_ST_CONV);
	if (!writeRegister(ADAR1000_ADC_CTRL, data)) return false;

	/* wait for ADC_EOC bit asserts high */
	do {
		if (!readRegister(ADAR1000_ADC_CTRL, data)) return false;
		++nTry;
	}
	while (!(data & ADAR1000_ADC_CTRL_EOC) && (nTry < 10));

	if (!(data & ADAR1000_ADC_CTRL_EOC)) return false;

	if (!readRegister(ADAR1000_ADC_OUTPUT, data)) return false;
	adcValue = data;

	return true;
}

bool Adar1000::getAdcPowerDetector(Adar1000Channel_t channel, uint8_t &adcValue)
{
	uint8_t data = 0;
	uint8_t nTry = 0;

	if (!readRegister(ADAR1000_ADC_CTRL, data)) return false;
	data = BIT_MASK(data, ADAR1000_ADC_CTRL_MUX_SEL, ADAR1000_ADC_CTRL_MUX_SEL_DETECTOR(channel) | ADAR1000_ADC_CTRL_ST_CONV);
	if (!writeRegister(ADAR1000_ADC_CTRL, data)) return false;

	do {
		if (!readRegister(ADAR1000_ADC_CTRL, data)) return false;
		++nTry;
	}
	while (!(data & ADAR1000_ADC_CTRL_EOC) && (nTry < 10));

	if (!(data & ADAR1000_ADC_CTRL_EOC)) return false;

	if (!readRegister(ADAR1000_ADC_OUTPUT, data)) return false;
	adcValue = data;

	return true;
}

bool Adar1000::getDeviceInfo(uint8_t &chipType, uint16_t &productId, uint16_t &vendorId)
{
	uint8_t data1 = 0, data2 = 0;

	if (!readRegister(ADAR1000_CHIP_TYPE, data1)) return false;
	chipType = data1;

	if (!readRegister(ADAR1000_PRODUCT_ID_H, data1)) return false;
	if (!readRegister(ADAR1000_PRODUCT_ID_L, data2)) return false;
	productId = ((uint16_t)data1 << 8) | data2;

	if (!readRegister(ADAR1000_VENDOR_ID_H, data1)) return false;
	if (!readRegister(ADAR1000_VENDOR_ID_L, data2)) return false;
	vendorId = ((uint16_t)data1 << 8) | data2;

	return true;
}

bool Adar1000::setTrCtrlByTrPin()
{
	uint8_t writeRegData = 0;

	if (!readRegister(ADAR1000_SW_CTRL, writeRegData)) return false;
	BIT_SET(writeRegData, ADAR1000_SW_CTRL_TR_SOURCE_BIT);
	if (!writeRegister(ADAR1000_SW_CTRL, writeRegData)) return false;

	uint8_t readRegData = 0;
	if (!readRegister(ADAR1000_SW_CTRL, readRegData)) return false;
	if (readRegData != writeRegData) return false;

	return true;
}

bool Adar1000::setTrCtrlBySpi()
{
	uint8_t writeRegData = 0;

	if (!readRegister(ADAR1000_SW_CTRL, writeRegData)) return false;
	BIT_CLR(writeRegData, ADAR1000_SW_CTRL_TR_SOURCE_BIT);
	if (!writeRegister(ADAR1000_SW_CTRL, writeRegData)) return false;

	uint8_t readRegData = 0;
	if (!readRegister(ADAR1000_SW_CTRL, readRegData)) return false;
	if (readRegData != writeRegData) return false;

	return true;
}

bool Adar1000::setPhaseToRegister(Adar1000Channel_t channel, Adar1000Direction_t direction,
		uint16_t integralPhaseDegree, uint16_t fractionalPhaseDegree)
{
	uint8_t regI = 0;
	uint8_t regQ = 0;
	uint32_t regIAddr = 0;
	uint32_t regQAddr = 0;

	if (!searchPhaseReg(integralPhaseDegree, fractionalPhaseDegree, regI, regQ)) return false;

	switch (direction) {
		case ADAR1000_DIRECTION_RX:
			regIAddr = ADAR1000_RX_PHASE_I(channel);
			regQAddr = ADAR1000_RX_PHASE_Q(channel);
		break;
		case ADAR1000_DIRECTION_TX:
			regIAddr = ADAR1000_TX_PHASE_I(channel);
			regQAddr = ADAR1000_TX_PHASE_Q(channel);
	}

	if (!writeRegister(regIAddr, regI)) return false;
	if (!writeRegister(regQAddr, regQ)) return false;

	return true;
}

bool Adar1000::setPhaseToRAM(Adar1000Channel_t channel, Adar1000Direction_t direction, uint8_t beamPosition,
		uint16_t integralPhaseDegree, uint16_t fractionalPhaseDegree)
{
	uint8_t regI = 0;
	uint8_t regQ = 0;
	uint32_t regIAddr = 0;
	uint32_t regQAddr = 0;

	if (beamPosition > 120) return false;

	if (!searchPhaseReg(integralPhaseDegree, fractionalPhaseDegree, regI, regQ)) return false;

	switch (direction) {
		case ADAR1000_DIRECTION_RX:
			regIAddr = ADAR1000_BEAMPOSITION_RX_PHASE_I(channel, beamPosition);
			regQAddr = ADAR1000_BEAMPOSITION_RX_PHASE_Q(channel, beamPosition);
		break;
		case ADAR1000_DIRECTION_TX:
			regIAddr = ADAR1000_BEAMPOSITION_TX_PHASE_I(channel, beamPosition);
			regQAddr = ADAR1000_BEAMPOSITION_TX_PHASE_Q(channel, beamPosition);
	}

	if (!writeRegister(regIAddr, regI)) return false;
	if (!writeRegister(regQAddr, regQ)) return false;

	return true;
}

bool Adar1000::setPhaseToRAM(Adar1000Channel_t channel, Adar1000Direction_t direction,
		uint8_t beamPosition, uint8_t phaseI, uint8_t phaseQ)
{
	uint32_t regIAddr = 0;
	uint32_t regQAddr = 0;

	if (beamPosition > 120) return false;

	switch (direction) {
		case ADAR1000_DIRECTION_RX:
			regIAddr = ADAR1000_BEAMPOSITION_RX_PHASE_I(channel, beamPosition);
			regQAddr = ADAR1000_BEAMPOSITION_RX_PHASE_Q(channel, beamPosition);
		break;
		case ADAR1000_DIRECTION_TX:
			regIAddr = ADAR1000_BEAMPOSITION_TX_PHASE_I(channel, beamPosition);
			regQAddr = ADAR1000_BEAMPOSITION_TX_PHASE_Q(channel, beamPosition);
	}

	if (!writeRegister(regIAddr, phaseI)) return false;
	if (!writeRegister(regQAddr, phaseQ)) return false;

	return true;
}

bool Adar1000::getPhaseFromRegister(Adar1000Channel_t channel, Adar1000Direction_t direction,
		uint16_t &integralPhase, uint16_t &fractionalPhase)
{
	uint8_t regI = 0;
	uint8_t regQ = 0;
	uint32_t regIAddr = 0;
	uint32_t regQAddr = 0;

	switch (direction) {
		case ADAR1000_DIRECTION_RX:
			regIAddr = ADAR1000_RX_PHASE_I(channel);
			regQAddr = ADAR1000_RX_PHASE_Q(channel);
		break;
		case ADAR1000_DIRECTION_TX:
			regIAddr = ADAR1000_TX_PHASE_I(channel);
			regQAddr = ADAR1000_TX_PHASE_Q(channel);
	}

	if (!readRegister(regIAddr, regI)) return false;
	if (!readRegister(regQAddr, regQ)) return false;
	if (!searchPhase(regI, regQ, integralPhase, fractionalPhase)) return false;

	return true;
}

bool Adar1000::getPhaseFromRAM(Adar1000Channel_t channel, Adar1000Direction_t direction, uint8_t beamPosition,
		uint16_t &integralPhase, uint16_t &fractionalPhase)
{
	uint8_t regI = 0;
	uint8_t regQ = 0;
	uint32_t regIAddr = 0;
	uint32_t regQAddr = 0;

	if (beamPosition > 120) return false;

	switch (direction) {
		case ADAR1000_DIRECTION_RX:
			regIAddr = ADAR1000_BEAMPOSITION_RX_PHASE_I(channel, beamPosition);
			regQAddr = ADAR1000_BEAMPOSITION_RX_PHASE_Q(channel, beamPosition);
		break;
		case ADAR1000_DIRECTION_TX:
			regIAddr = ADAR1000_BEAMPOSITION_TX_PHASE_I(channel, beamPosition);
			regQAddr = ADAR1000_BEAMPOSITION_TX_PHASE_Q(channel, beamPosition);
	}

	if (!readRegister(regIAddr, regI)) return false;
	if (!readRegister(regQAddr, regQ)) return false;
	if (!searchPhase(regI, regQ, integralPhase, fractionalPhase)) return false;

	return true;
}

bool Adar1000::getPhaseFromRAM(Adar1000Channel_t channel, Adar1000Direction_t direction, uint8_t beamPosition,
		uint8_t &phaseI, uint8_t &phaseQ)
{
	uint32_t regIAddr = 0;
	uint32_t regQAddr = 0;

	if (beamPosition > 120) return false;

	switch (direction) {
		case ADAR1000_DIRECTION_RX:
			regIAddr = ADAR1000_BEAMPOSITION_RX_PHASE_I(channel, beamPosition);
			regQAddr = ADAR1000_BEAMPOSITION_RX_PHASE_Q(channel, beamPosition);
		break;
		case ADAR1000_DIRECTION_TX:
			regIAddr = ADAR1000_BEAMPOSITION_TX_PHASE_I(channel, beamPosition);
			regQAddr = ADAR1000_BEAMPOSITION_TX_PHASE_Q(channel, beamPosition);
	}

	if (!readRegister(regIAddr, phaseI)) return false;
	if (!readRegister(regQAddr, phaseQ)) return false;

	return true;
}

bool Adar1000::setGainToRegister(Adar1000Channel_t channel, Adar1000Direction_t direction, uint8_t gainControlCode)
{
	uint32_t gainRegAddr = 0;

	switch (direction) {
		case ADAR1000_DIRECTION_RX:
			gainRegAddr = ADAR1000_RX_GAIN(channel);
		break;
		case ADAR1000_DIRECTION_TX:
			gainRegAddr = ADAR1000_TX_GAIN(channel);
	}

	return writeRegister(gainRegAddr, gainControlCode);
}

bool Adar1000::setGainToRAM(Adar1000Channel_t channel, Adar1000Direction_t direction,
		uint8_t beamPosition, uint8_t gainControlCode)
{
	uint32_t gainRegAddr = 0;

	switch (direction) {
		case ADAR1000_DIRECTION_RX:
			gainRegAddr = ADAR1000_BEAMPOSITION_RX_GAIN(channel, beamPosition);
		break;
		case ADAR1000_DIRECTION_TX:
			gainRegAddr = ADAR1000_BEAMPOSITION_TX_GAIN(channel, beamPosition);
	}

	return writeRegister(gainRegAddr, gainControlCode);
}

bool Adar1000::getGainFromRegister(Adar1000Channel_t channel, Adar1000Direction_t direction,
		uint8_t &gainControlCode)
{
	uint8_t data = 0;
	uint32_t regAddr = 0;

	switch (direction) {
		case ADAR1000_DIRECTION_RX:
			regAddr = ADAR1000_RX_GAIN(channel);
		break;
		case ADAR1000_DIRECTION_TX:
			regAddr = ADAR1000_TX_GAIN(channel);
	}

	if (!readRegister(regAddr, data)) return false;
	gainControlCode = data;

	return true;
}

bool Adar1000::getGainFromRAM(Adar1000Channel_t channel, Adar1000Direction_t direction, uint8_t beamPosition,
				uint8_t &gainControlCode)
{
	uint8_t data = 0;
	uint32_t regAddr = 0;

	switch (direction) {
		case ADAR1000_DIRECTION_RX:
			regAddr = ADAR1000_BEAMPOSITION_RX_GAIN(channel, beamPosition);
		break;
		case ADAR1000_DIRECTION_TX:
			regAddr = ADAR1000_BEAMPOSITION_TX_GAIN(channel, beamPosition);
	}

	if (!readRegister(regAddr, data)) return false;
	gainControlCode = data;

	return true;
}

bool Adar1000::loadGainPhaseFromRegister(Adar1000Direction_t direction)
{
	if (!configBeamDataLoadFromRegister()) return false;
	return writeRegister(ADAR1000_LD_WRK_REGS, (uint8_t)(direction + 1));
}

/* Refer to section Single Memory Fetch (page 32/79) */
bool Adar1000::loadGainPhaseFromRAM(Adar1000Channel_t channel, Adar1000Direction_t direction, uint8_t beamPosition)
{
	uint32_t memRegAddr = 0;
	uint8_t memRegVal = 0;

	if (!configBeamDataLoadFromRAM(direction, true)) return false;

	/* assert the fetch bit high (Bit 7), write the desired 7-bit beam position (0 through 120) */
	memRegVal = (1 << 7) | beamPosition;

	switch (direction) {
		case ADAR1000_DIRECTION_RX:
			memRegAddr = ADAR1000_RX_MEM(channel);
		break;
		case ADAR1000_DIRECTION_TX:
			memRegAddr = ADAR1000_TX_MEM(channel);
	}
	if (!writeRegister(memRegAddr, memRegVal)) return false;

	/* Provide at least six additional clock cycles on SCLK to load the new data from the RAM */
	if (!readRegister(memRegAddr, memRegVal)) return false;

	return writeRegister(ADAR1000_LD_WRK_REGS, (uint8_t)(direction + 1));
}

/* Refer to section Single Memory Fetch (page 32/79) */
bool Adar1000::loadGainPhaseAllFromRAM(Adar1000Direction_t direction, uint8_t beamPosition)
{
	uint32_t chxMemRegAddr = 0;
	uint8_t chxMemRegVal = 0;

	if (!configBeamDataLoadFromRAM(direction, false)) return false;

	/* assert the fetch bit high (Bit 7), write the desired 7-bit beam position (0 through 120) */
	chxMemRegVal = (1 << 7) | beamPosition;

	switch (direction) {
		case ADAR1000_DIRECTION_RX:
			chxMemRegAddr = ADAR1000_RX_CHX_MEM;
		break;
		case ADAR1000_DIRECTION_TX:
			chxMemRegAddr = ADAR1000_TX_CHX_MEM;
	}
	if (!writeRegister(chxMemRegAddr, chxMemRegVal)) return false;

	/* Provide at least six additional clock cycles on SCLK to load the new data from the RAM */
	if (!readRegister(chxMemRegAddr, chxMemRegVal)) return false;

	return writeRegister(ADAR1000_LD_WRK_REGS, (uint8_t)(direction + 1));
}

bool Adar1000::controlTxStateBySpi(Adar1000Channel_t channel, Adar1000ControlState_t state)
{
	uint8_t rxEnablesRegVal = 0;
	uint8_t txEnablesRegVal = 0;
	uint8_t swCtrlRegVal = 0;
	uint8_t readRegData = 0;

	/* Verify if device is in SPI control (TR_SOURCE bit = 0 in SW_CTRL register) */
	if (!readRegister(ADAR1000_SW_CTRL, swCtrlRegVal)) return false;
	if (BIT_READ(swCtrlRegVal, ADAR1000_SW_CTRL_TR_SOURCE_BIT) != ADAR1000_SW_CTRL_TR_SOURCE_BIT_SPI) {
		return false;
	}

	if (!readRegister(ADAR1000_TX_ENABLES, txEnablesRegVal)) return false;

	if (state == ADAR1000_CONTROL_STATE_ON) {
		/* Make sure to disable direction Rx in channel */
		if (!readRegister(ADAR1000_RX_ENABLES, rxEnablesRegVal)) return false;
		if (rxEnablesRegVal != 0) {
			switch (channel) {
				case ADAR1000_CHANNEL_1:
					BIT_CLR(rxEnablesRegVal, ADAR1000_RX_ENABLES_CH1_RX_EN_BIT);
				break;
				case ADAR1000_CHANNEL_2:
					BIT_CLR(rxEnablesRegVal, ADAR1000_RX_ENABLES_CH2_RX_EN_BIT);
				break;
				case ADAR1000_CHANNEL_3:
					BIT_CLR(rxEnablesRegVal, ADAR1000_RX_ENABLES_CH3_RX_EN_BIT);
				break;
				case ADAR1000_CHANNEL_4:
					BIT_CLR(rxEnablesRegVal, ADAR1000_RX_ENABLES_CH4_RX_EN_BIT);
			}
			/* Disable RF circuits in Rx mode: Receive LNAs, Vector Modulators, VGAs */
			BIT_CLR(rxEnablesRegVal, ADAR1000_RX_ENABLES_RX_LNA_EN_BIT);
			BIT_CLR(rxEnablesRegVal, ADAR1000_RX_ENABLES_RX_VGA_EN_BIT);
			BIT_CLR(rxEnablesRegVal, ADAR1000_RX_ENABLES_RX_VM_EN_BIT);
			if (!writeRegister(ADAR1000_RX_ENABLES, rxEnablesRegVal)) return false;
			readRegData = 0;
			if (!readRegister(ADAR1000_RX_ENABLES, readRegData)) return false;
			if (readRegData != rxEnablesRegVal) return false;
		}

		/* Enables Tx Channel subcircuits */
		switch (channel) {
			case ADAR1000_CHANNEL_1:
				BIT_SET(txEnablesRegVal, ADAR1000_TX_ENABLES_CH1_TX_EN_BIT);
			break;
			case ADAR1000_CHANNEL_2:
				BIT_SET(txEnablesRegVal, ADAR1000_TX_ENABLES_CH2_TX_EN_BIT);
			break;
			case ADAR1000_CHANNEL_3:
				BIT_SET(txEnablesRegVal, ADAR1000_TX_ENABLES_CH3_TX_EN_BIT);
			break;
			case ADAR1000_CHANNEL_4:
				BIT_SET(txEnablesRegVal, ADAR1000_TX_ENABLES_CH4_TX_EN_BIT);
		}
		/* Enable RF circuits in Tx mode: Transmit Drivers,  Vector Modulators, VGAs */
		BIT_SET(txEnablesRegVal, ADAR1000_TX_ENABLES_TX_DRV_EN_BIT);
		BIT_SET(txEnablesRegVal, ADAR1000_TX_ENABLES_TX_VGA_EN_BIT);
		BIT_SET(txEnablesRegVal, ADAR1000_TX_ENABLES_TX_VM_EN_BIT);
	}
	else {
		/* Disables Tx Channel subcircuits */
		switch (channel) {
			case ADAR1000_CHANNEL_1:
				BIT_CLR(txEnablesRegVal, ADAR1000_TX_ENABLES_CH1_TX_EN_BIT);
			break;
			case ADAR1000_CHANNEL_2:
				BIT_CLR(txEnablesRegVal, ADAR1000_TX_ENABLES_CH2_TX_EN_BIT);
			break;
			case ADAR1000_CHANNEL_3:
				BIT_CLR(txEnablesRegVal, ADAR1000_TX_ENABLES_CH3_TX_EN_BIT);
			break;
			case ADAR1000_CHANNEL_4:
				BIT_CLR(txEnablesRegVal, ADAR1000_TX_ENABLES_CH4_TX_EN_BIT);
		}
		/* Disable RF circuits in Tx mode: Transmit Drivers,  Vector Modulators, VGAs */
		BIT_CLR(txEnablesRegVal, ADAR1000_TX_ENABLES_TX_DRV_EN_BIT);
		BIT_CLR(txEnablesRegVal, ADAR1000_TX_ENABLES_TX_VGA_EN_BIT);
		BIT_CLR(txEnablesRegVal, ADAR1000_TX_ENABLES_TX_VM_EN_BIT);
	}

	if (!writeRegister(ADAR1000_TX_ENABLES, txEnablesRegVal)) return false;
	readRegData = 0;
	if (!readRegister(ADAR1000_TX_ENABLES, readRegData)) return false;
	if (readRegData != txEnablesRegVal) return false;

	return true;
}

bool Adar1000::controlRxStateBySpi(Adar1000Channel_t channel, Adar1000ControlState_t state)
{
	uint8_t rxEnablesRegVal = 0;
	uint8_t txEnablesRegVal = 0;
	uint8_t swCtrlRegVal = 0;
	uint8_t readRegData = 0;

	/* Verify if device is in SPI control (TR_SOURCE bit = 0 in SW_CTRL register) */
	if (!readRegister(ADAR1000_SW_CTRL, swCtrlRegVal)) return false;
	if (BIT_READ(swCtrlRegVal, ADAR1000_SW_CTRL_TR_SOURCE_BIT) != ADAR1000_SW_CTRL_TR_SOURCE_BIT_SPI) {
		return false;
	}

	if (!readRegister(ADAR1000_RX_ENABLES, rxEnablesRegVal)) return false;

	if (state == ADAR1000_CONTROL_STATE_ON) {
		/* Make sure to disable direction Tx in channel */
		if (!readRegister(ADAR1000_TX_ENABLES, txEnablesRegVal)) return false;
		if (txEnablesRegVal != 0) {
			switch (channel) {
				case ADAR1000_CHANNEL_1:
					BIT_CLR(txEnablesRegVal, ADAR1000_TX_ENABLES_CH1_TX_EN_BIT);
				break;
				case ADAR1000_CHANNEL_2:
					BIT_CLR(txEnablesRegVal, ADAR1000_TX_ENABLES_CH2_TX_EN_BIT);
				break;
				case ADAR1000_CHANNEL_3:
					BIT_CLR(txEnablesRegVal, ADAR1000_TX_ENABLES_CH3_TX_EN_BIT);
				break;
				case ADAR1000_CHANNEL_4:
					BIT_CLR(txEnablesRegVal, ADAR1000_TX_ENABLES_CH4_TX_EN_BIT);
			}
			/* Disable RF circuits in Tx mode: Transmit Drivers,  Vector Modulators, VGAs */
			BIT_CLR(txEnablesRegVal, ADAR1000_TX_ENABLES_TX_DRV_EN_BIT);
			BIT_CLR(txEnablesRegVal, ADAR1000_TX_ENABLES_TX_VGA_EN_BIT);
			BIT_CLR(txEnablesRegVal, ADAR1000_TX_ENABLES_TX_VM_EN_BIT);
			if (!writeRegister(ADAR1000_TX_ENABLES, txEnablesRegVal)) return false;
			readRegData = 0;
			if (!readRegister(ADAR1000_TX_ENABLES, readRegData)) return false;
			if (readRegData != txEnablesRegVal) return false;
		}

		/* Enables Rx Channel subcircuits */
		switch (channel) {
			case ADAR1000_CHANNEL_1:
				BIT_SET(rxEnablesRegVal, ADAR1000_RX_ENABLES_CH1_RX_EN_BIT);
			break;
			case ADAR1000_CHANNEL_2:
				BIT_SET(rxEnablesRegVal, ADAR1000_RX_ENABLES_CH2_RX_EN_BIT);
			break;
			case ADAR1000_CHANNEL_3:
				BIT_SET(rxEnablesRegVal, ADAR1000_RX_ENABLES_CH3_RX_EN_BIT);
			break;
			case ADAR1000_CHANNEL_4:
				BIT_SET(rxEnablesRegVal, ADAR1000_RX_ENABLES_CH4_RX_EN_BIT);
		}
		/* Enable RF circuits in Rx mode: Receive LNAs, Vector Modulators, VGAs */
		BIT_SET(rxEnablesRegVal, ADAR1000_RX_ENABLES_RX_LNA_EN_BIT);
		BIT_SET(rxEnablesRegVal, ADAR1000_RX_ENABLES_RX_VGA_EN_BIT);
		BIT_SET(rxEnablesRegVal, ADAR1000_RX_ENABLES_RX_VM_EN_BIT);
	}
	else {
		/* Disables Rx Channel subcircuits */
		switch (channel) {
			case ADAR1000_CHANNEL_1:
				BIT_CLR(rxEnablesRegVal, ADAR1000_RX_ENABLES_CH1_RX_EN_BIT);
			break;
			case ADAR1000_CHANNEL_2:
				BIT_CLR(rxEnablesRegVal, ADAR1000_RX_ENABLES_CH2_RX_EN_BIT);
			break;
			case ADAR1000_CHANNEL_3:
				BIT_CLR(rxEnablesRegVal, ADAR1000_RX_ENABLES_CH3_RX_EN_BIT);
			break;
			case ADAR1000_CHANNEL_4:
				BIT_CLR(rxEnablesRegVal, ADAR1000_RX_ENABLES_CH4_RX_EN_BIT);
		}
		/* Disable RF circuits in Rx mode: Receive LNAs, Vector Modulators, VGAs */
		BIT_CLR(rxEnablesRegVal, ADAR1000_RX_ENABLES_RX_LNA_EN_BIT);
		BIT_CLR(rxEnablesRegVal, ADAR1000_RX_ENABLES_RX_VGA_EN_BIT);
		BIT_CLR(rxEnablesRegVal, ADAR1000_RX_ENABLES_RX_VM_EN_BIT);
	}

	if (!writeRegister(ADAR1000_RX_ENABLES, rxEnablesRegVal)) return false;
	readRegData = 0;
	if (!readRegister(ADAR1000_RX_ENABLES, readRegData)) return false;
	if (readRegData != rxEnablesRegVal) return false;

	return true;
}

bool Adar1000::switchToTxBySpi()
{
	uint8_t swCtrlRegVal = 0;
	uint8_t rxEnablesRegVal = 0;
	uint8_t txEnablesRegVal = 0;

	/* Verify if device is in SPI control (TR_SOURCE bit = 0 in SW_CTRL register) */
	if (!readRegister(ADAR1000_SW_CTRL, swCtrlRegVal)) return false;
	if (BIT_READ(swCtrlRegVal, ADAR1000_SW_CTRL_TR_SOURCE_BIT) != ADAR1000_SW_CTRL_TR_SOURCE_BIT_SPI) {
		return false;
	}

	/* Make sure a specific channel is in Tx direction */
	if (!readRegister(ADAR1000_TX_ENABLES, txEnablesRegVal)) return false;
	if (!readRegister(ADAR1000_RX_ENABLES, rxEnablesRegVal)) return false;
	if (txEnablesRegVal == 0 || rxEnablesRegVal != 0) return false;

	/* Transmit or Receive mode while in SPI Control. 0 = receive and 1 = transmit */
	BIT_SET(swCtrlRegVal, ADAR1000_SW_CTRL_TR_SPI_BIT);
	/* Enables Transmit Channel Subcircuits when Under SPI Control. 1 = enabled */
	BIT_SET(swCtrlRegVal, ADAR1000_SW_CTRL_TX_EN_BIT);
	/* Enables Receive Channel Subcircuits when Under SPI Control. 1 = enabled. */
	BIT_CLR(swCtrlRegVal, ADAR1000_SW_CTRL_RX_EN_BIT);
	if (!writeRegister(ADAR1000_SW_CTRL, swCtrlRegVal)) return false;

	uint8_t readRegData = 0;
	if (!readRegister(ADAR1000_SW_CTRL, readRegData)) return false;
	if (readRegData != swCtrlRegVal) return false;

	return true;
}

bool Adar1000::switchToRxBySpi()
{
	uint8_t swCtrlRegVal = 0;
	uint8_t rxEnablesRegVal = 0;
	uint8_t txEnablesRegVal = 0;

	/* Verify if device is in SPI control (TR_SOURCE bit = 0 in SW_CTRL register) */
	if (!readRegister(ADAR1000_SW_CTRL, swCtrlRegVal)) return false;
	if (BIT_READ(swCtrlRegVal, ADAR1000_SW_CTRL_TR_SOURCE_BIT) != ADAR1000_SW_CTRL_TR_SOURCE_BIT_SPI) {
		return false;
	}

	/* Make sure a specific channel is in Rx direction */
	if (!readRegister(ADAR1000_TX_ENABLES, txEnablesRegVal)) return false;
	if (!readRegister(ADAR1000_RX_ENABLES, rxEnablesRegVal)) return false;
	if (txEnablesRegVal != 0 || rxEnablesRegVal == 0) return false;

	/* Transmit or Receive mode while in SPI Control. 0 = receive and 1 = transmit */
	BIT_CLR(swCtrlRegVal, ADAR1000_SW_CTRL_TR_SPI_BIT);
	/* Enables Transmit Channel Subcircuits when Under SPI Control. 1 = enabled */
	BIT_CLR(swCtrlRegVal, ADAR1000_SW_CTRL_TX_EN_BIT);
	/* Enables Receive Channel Subcircuits when Under SPI Control. 1 = enabled. */
	BIT_SET(swCtrlRegVal, ADAR1000_SW_CTRL_RX_EN_BIT);
	if (!writeRegister(ADAR1000_SW_CTRL, swCtrlRegVal)) return false;

	uint8_t readRegData = 0;
	if (!readRegister(ADAR1000_SW_CTRL, readRegData)) return false;
	if (readRegData != swCtrlRegVal) return false;

	return true;
}

/* Follow Table 14. Controlling TR_SW_POS and TR_SW_NEG Output */
bool Adar1000::disableAllTrSwOutputs()
{
	uint8_t writeRegData = 0;
	uint8_t readRegData = 0;

	/* Disable Switch Driver for External Transmit/Receive Switch */
	if (!readRegister(ADAR1000_SW_CTRL, writeRegData)) return false;
	BIT_CLR(writeRegData, ADAR1000_SW_CTRL_SW_DRV_EN_TR_BIT);
	if (!writeRegister(ADAR1000_SW_CTRL, writeRegData)) return false;

	if (!readRegister(ADAR1000_SW_CTRL, readRegData)) return false;
	if (readRegData != writeRegData) return false;

	return true;
}

/*
 * Setting follows Table 14. Controlling TR_SW_POS and TR_SW_NEG Output (page 38)
 */
bool Adar1000::enableTrSwPosOutput()
{
	uint8_t miscEnablesRegVal = 0;
	uint8_t swCtrlRegVal = 0;
	uint8_t readRegData = 0;

	/* Enable TR_SW_POS pin in MISC_ENABLES register */
	if (!readRegister(ADAR1000_MISC_ENABLES, miscEnablesRegVal)) return false;
	BIT_CLR(miscEnablesRegVal, ADAR1000_MISC_ENABLES_SW_DRV_TR_MODE_SEL_BIT);
	if (!writeRegister(ADAR1000_MISC_ENABLES, miscEnablesRegVal)) return false;
	if (!readRegister(ADAR1000_MISC_ENABLES, readRegData)) return false;
	if (readRegData != miscEnablesRegVal) return false;

	/* Enables Switch Driver for External Transmit/Receive Switch */
	if (!readRegister(ADAR1000_SW_CTRL, swCtrlRegVal)) return false;
	BIT_SET(swCtrlRegVal, ADAR1000_SW_CTRL_SW_DRV_EN_TR_BIT);
	if (!writeRegister(ADAR1000_SW_CTRL, swCtrlRegVal)) return false;
	readRegData = 0;
	if (!readRegister(ADAR1000_SW_CTRL, readRegData)) return false;
	if (readRegData != swCtrlRegVal) return false;

	return true;
}

/*
 * Setting follows Table 14. Controlling TR_SW_POS and TR_SW_NEG Output (page 38)
 */
bool Adar1000::enableTrSwNegOutput()
{
	uint8_t miscEnablesRegVal = 0;
	uint8_t swCtrlRegVal = 0;
	uint8_t readRegData = 0;

	/* Enable TR_SW_NEG pin in MISC_ENABLES register */
	if (!readRegister(ADAR1000_MISC_ENABLES, miscEnablesRegVal)) return false;
	BIT_SET(miscEnablesRegVal, ADAR1000_MISC_ENABLES_SW_DRV_TR_MODE_SEL_BIT);
	if (!writeRegister(ADAR1000_MISC_ENABLES, miscEnablesRegVal)) return false;
	if (!readRegister(ADAR1000_MISC_ENABLES, readRegData)) return false;
	if (readRegData != miscEnablesRegVal) return false;

	/* Enables Switch Driver for External Transmit/Receive Switch */
	if (!readRegister(ADAR1000_SW_CTRL, swCtrlRegVal)) return false;
	BIT_SET(swCtrlRegVal, ADAR1000_SW_CTRL_SW_DRV_EN_TR_BIT);
	if (!writeRegister(ADAR1000_SW_CTRL, swCtrlRegVal)) return false;
	readRegData = 0;
	if (!readRegister(ADAR1000_SW_CTRL, readRegData)) return false;
	if (readRegData != swCtrlRegVal) return false;

	return true;
}

/*
 * Setting follows Table 14. Controlling TR_SW_POS and TR_SW_NEG Output (page 38)
 */
//bool Adar1000::controlTrSwPosOutput(Adar1000Direction_t direction, Adar1000ControlState_t state)
//{
//	uint8_t swCtrlRegVal = 0;
//	uint8_t miscEnablesRegVal = 0;
//
//	if (!readRegister(ADAR1000_SW_CTRL, swCtrlRegVal)) return false;
//
//	if (state == ADAR1000_CONTROL_STATE_OFF) {
//		BIT_CLR(swCtrlRegVal, ADAR1000_SW_CTRL_SW_DRV_EN_TR_BIT);
//		if (!writeRegister(ADAR1000_SW_CTRL, swCtrlRegVal)) return false;
//		return true;
//	}
//
//	/* Enables Switch Driver for External Transmit/Receive Switch */
//	BIT_SET(swCtrlRegVal, ADAR1000_SW_CTRL_SW_DRV_EN_TR_BIT);
//
//	/* Enable TR_SW_POS pin in MISC_ENABLES register */
//	if (!readRegister(ADAR1000_MISC_ENABLES, miscEnablesRegVal)) return false;
//	BIT_CLR(miscEnablesRegVal, ADAR1000_MISC_ENABLES_SW_DRV_TR_MODE_SEL_BIT);
//	if (!writeRegister(ADAR1000_MISC_ENABLES, miscEnablesRegVal)) return false;
//
//	/* Need to config TR_SPI bit in SW_CTRL register when in SPI control (TR_SOURCE bit = 0) */
//	if (BIT_READ(swCtrlRegVal,  ADAR1000_SW_CTRL_TR_SOURCE_BIT) == ADAR1000_SW_CTRL_TR_SOURCE_BIT_SPI) {
//		switch (direction) {
//			case ADAR1000_DIRECTION_RX:
//				BIT_CLR(swCtrlRegVal, ADAR1000_SW_CTRL_TR_SPI_BIT);
//			break;
//			case ADAR1000_DIRECTION_TX:
//				BIT_SET(swCtrlRegVal, ADAR1000_SW_CTRL_TR_SPI_BIT);
//		}
//	}
//
//	if (!writeRegister(ADAR1000_SW_CTRL, swCtrlRegVal)) return false;
//
//	return true;
//}

/*
 * Setting follows Table 14. Controlling TR_SW_POS and TR_SW_NEG Output (page 38)
 */
//bool Adar1000::controlTrSwNegOutput(Adar1000Direction_t direction, Adar1000ControlState_t state)
//{
//	uint8_t swCtrlRegVal = 0;
//	uint8_t miscEnablesRegVal = 0;
//
//	if (!readRegister(ADAR1000_SW_CTRL, swCtrlRegVal)) return false;
//
//	if (state == ADAR1000_CONTROL_STATE_OFF) {
//		BIT_CLR(swCtrlRegVal, ADAR1000_SW_CTRL_SW_DRV_EN_TR_BIT);
//		if (!writeRegister(ADAR1000_SW_CTRL, swCtrlRegVal)) return false;
//		return true;
//	}
//
//	BIT_SET(swCtrlRegVal, ADAR1000_SW_CTRL_SW_DRV_EN_TR_BIT);
//
//	/* Enable TR_SW_NEG pin in MISC_ENABLES register */
//	if (!readRegister(ADAR1000_MISC_ENABLES, miscEnablesRegVal)) return false;
//	BIT_SET(miscEnablesRegVal, ADAR1000_MISC_ENABLES_SW_DRV_TR_MODE_SEL_BIT);
//	if (!writeRegister(ADAR1000_MISC_ENABLES, miscEnablesRegVal)) return false;
//
//	/* Need to config TR_SPI bit in SW_CTRL register when in SPI control (TR_SOURCE bit = 0) */
//	if (BIT_READ(swCtrlRegVal,  ADAR1000_SW_CTRL_TR_SOURCE_BIT) == ADAR1000_SW_CTRL_TR_SOURCE_BIT_SPI) {
//		switch (direction) {
//			case ADAR1000_DIRECTION_RX:
//				BIT_CLR(swCtrlRegVal, ADAR1000_SW_CTRL_TR_SPI_BIT);
//			break;
//			case ADAR1000_DIRECTION_TX:
//				BIT_SET(swCtrlRegVal, ADAR1000_SW_CTRL_TR_SPI_BIT);
//		}
//	}
//
//	if (!writeRegister(ADAR1000_SW_CTRL, swCtrlRegVal)) return false;
//
//	return true;
//}

/*
 * Follow settings in Table 6. SPI Settings for Nominal and Low Power Modes
 */
bool Adar1000::initNominalBiasCurrentTx()
{
	uint8_t biasCurrentTxRegVal = 0;
	uint8_t biasCurrentTxDrvRegVal = 0;

	/* Config Transmit Channel VGA Bias and Vector Modulator Bias Current in Nominal Setting */
	if (!readRegister(ADAR1000_BIAS_CURRENT_TX, biasCurrentTxRegVal)) return false;
	biasCurrentTxRegVal &= 0x80; // keep 7th reserved bit
	biasCurrentTxRegVal |= (5 << 3) | 5; // TX_VM_BIAS[2:0] = 5, TX_VGA_BIAS[6:3] = 5
	if (!writeRegister(ADAR1000_BIAS_CURRENT_TX, biasCurrentTxRegVal)) return false;

	/* Config Transmit Driver Bias Current in Nominal Setting */
	if (!readRegister(ADAR1000_BIAS_CURRENT_TX_DRV, biasCurrentTxDrvRegVal)) return false;
	biasCurrentTxDrvRegVal &= 0xF8; // keep [7:3] reserved bits
	biasCurrentTxDrvRegVal |= 6; // TX_DRV_BIAS[2:0] = 6
	if (!writeRegister(ADAR1000_BIAS_CURRENT_TX_DRV, biasCurrentTxDrvRegVal)) return false;

	uint8_t readReg1Data = 0;
	uint8_t readReg2Data = 0;
	if (!readRegister(ADAR1000_BIAS_CURRENT_TX, readReg1Data)) return false;
	if (!readRegister(ADAR1000_BIAS_CURRENT_TX_DRV, readReg2Data)) return false;
	if ((readReg1Data != biasCurrentTxRegVal) || (readReg2Data != biasCurrentTxDrvRegVal)) return false;

	return true;
}

/*
 * Follow settings in Table 6. SPI Settings for Nominal and Low Power Modes
 */
bool Adar1000::initLowPowerBiasCurrentTx()
{
	uint8_t biasCurrentTxRegVal = 0;
	uint8_t biasCurrentTxDrvRegVal = 0;

	/* Config Transmit Channel VGA Bias and Vector Modulator Bias Current in Low Power Setting */
	if (!readRegister(ADAR1000_BIAS_CURRENT_TX, biasCurrentTxRegVal)) return false;
	biasCurrentTxRegVal &= 0x80; // keep 7th reserved bit
	biasCurrentTxRegVal |= (5 << 3) | 2; // TX_VM_BIAS[2:0] = 2, TX_VGA_BIAS[6:3] = 5
	if (!writeRegister(ADAR1000_BIAS_CURRENT_TX, biasCurrentTxRegVal)) return false;

	/* Config Transmit Driver Bias Current in Low Power Setting */
	if (!readRegister(ADAR1000_BIAS_CURRENT_TX_DRV, biasCurrentTxDrvRegVal)) return false;
	biasCurrentTxDrvRegVal &= 0xF8; // keep [7:3] reserved bits
	biasCurrentTxDrvRegVal |= 3; // TX_DRV_BIAS[2:0] = 3
	if (!writeRegister(ADAR1000_BIAS_CURRENT_TX_DRV, biasCurrentTxDrvRegVal)) return false;

	uint8_t readReg1Data = 0;
	uint8_t readReg2Data = 0;
	if (!readRegister(ADAR1000_BIAS_CURRENT_TX, readReg1Data)) return false;
	if (!readRegister(ADAR1000_BIAS_CURRENT_TX_DRV, readReg2Data)) return false;
	if ((readReg1Data != biasCurrentTxRegVal) || (readReg2Data != biasCurrentTxDrvRegVal)) return false;

	return true;
}

/*
 * Follow settings in Table 6. SPI Settings for Nominal and Low Power Modes (Page 30/79)
 */
bool Adar1000::initNominalBiasCurrentRx()
{
	uint8_t biasCurrentRxLnaRegVal = 0;
	uint8_t biasCurrentRxRegVal = 0;

	/* Config Receive Channel VGA and Receive Channel Vector Modulator Bias Current in Nominal Setting */
	if (!readRegister(ADAR1000_BIAS_CURRENT_RX, biasCurrentRxRegVal)) return false;
	biasCurrentRxRegVal &= 0x80; // keep 7th reserved bit
	biasCurrentRxRegVal |= (10 << 3) | 5; // RX_VM_BIAS[2:0] = 5, RX_VGA_BIAS[6:3] = 10
	if (!writeRegister(ADAR1000_BIAS_CURRENT_RX, biasCurrentRxRegVal)) return false;

	/* Config LNA Bias Current Setting in Nominal Settings */
	if (!readRegister(ADAR1000_BIAS_CURRENT_RX_LNA, biasCurrentRxLnaRegVal)) return false;
	biasCurrentRxLnaRegVal &= 0xF0; // keep [7:4] reserved bits
	biasCurrentRxLnaRegVal |= 8; // LNA_BIAS[3:0] = 8;
	if (!writeRegister(ADAR1000_BIAS_CURRENT_RX_LNA, biasCurrentRxLnaRegVal)) return false;

	uint8_t readReg1Data = 0;
	uint8_t readReg2Data = 0;
	if (!readRegister(ADAR1000_BIAS_CURRENT_RX, readReg1Data)) return false;

//	m_logger->sendDebugMsg(DEBUG_FORMAT("biasCurrentRxRegVal: %d readReg1Data: %d", biasCurrentRxRegVal, readReg1Data));
	if (!readRegister(ADAR1000_BIAS_CURRENT_RX_LNA, readReg2Data)) return false;
	if ((readReg1Data != biasCurrentRxRegVal) || (readReg2Data != biasCurrentRxLnaRegVal)) return false;

	return true;
}

/*
 * Follow settings in Table 6. SPI Settings for Nominal and Low Power Modes
 */
bool Adar1000::initLowPowerBiasCurrentRx()
{
	uint8_t biasCurrentRxLnaRegVal = 0;
	uint8_t biasCurrentRxRegVal = 0;

	/* Config Receive Channel VGA and Receive Channel Vector Modulator Bias Current in Low Power Setting */
	if (!readRegister(ADAR1000_BIAS_CURRENT_RX, biasCurrentRxRegVal)) return false;
	biasCurrentRxRegVal &= 0x80; // keep 7th reserved bit
	biasCurrentRxRegVal |= (3 << 3) | 2; // RX_VM_BIAS[2:0] = 2, RX_VGA_BIAS[6:3] = 3
	if (!writeRegister(ADAR1000_BIAS_CURRENT_RX, biasCurrentRxRegVal)) return false;

	/* Config LNA Bias Current Setting in Low Power Settings */
	if (!readRegister(ADAR1000_BIAS_CURRENT_RX_LNA, biasCurrentRxLnaRegVal)) return false;
	biasCurrentRxLnaRegVal &= 0xF0; // keep [7:4] reserved bits
	biasCurrentRxLnaRegVal |= 5; // LNA_BIAS[3:0] = 5;
	if (!writeRegister(ADAR1000_BIAS_CURRENT_RX_LNA, biasCurrentRxLnaRegVal)) return false;

	uint8_t readReg1Data = 0;
	uint8_t readReg2Data = 0;
	if (!readRegister(ADAR1000_BIAS_CURRENT_RX, readReg1Data)) return false;
	if (!readRegister(ADAR1000_BIAS_CURRENT_RX_LNA, readReg2Data)) return false;
	if ((readReg1Data != biasCurrentRxRegVal) || (readReg2Data != biasCurrentRxLnaRegVal)) return false;

	return true;
}

bool Adar1000::setLnaBiasToRegister(uint8_t onValue, uint8_t offValue)
{
	if (!writeRegister(ADAR1000_LNA_BIAS_ON, onValue)) return false;
	if (!writeRegister(ADAR1000_LNA_BIAS_OFF, offValue)) return false;
	return true;
}

bool Adar1000::getLnaBiasFromRegister(uint8_t &onValue, uint8_t &offValue)
{
	if (!readRegister(ADAR1000_LNA_BIAS_ON, onValue)) return false;
	if (!readRegister(ADAR1000_LNA_BIAS_OFF, offValue)) return false;
	return true;
}

/* Table 18. Control of LNA_BIAS Output */
bool Adar1000::loadLnaBiasFromRegister()
{
	uint8_t miscEnablesRegVal = 0;
	uint8_t memCtrlRegVal = 0;

	if (!readRegister(ADAR1000_MISC_ENABLES, miscEnablesRegVal)) return false;
	/* External Amplifier Bias Control. If 0, DACs assume the on register values. If 1, DACs vary
	with device mode (transmit and receive). */
	BIT_SET(miscEnablesRegVal, ADAR1000_MISC_ENABLES_BIAS_CTRL_BIT);
	/* Enables PA and LNA Bias DACs. 0 = enabled */
	BIT_CLR(miscEnablesRegVal, ADAR1000_MISC_ENABLES_BIAS_EN_BIT);
	/* Enables Output of LNA Bias DAC. 0 = open and 1 = bias connected */
	BIT_SET(miscEnablesRegVal, ADAR1000_MISC_ENABLES_LNA_BIAS_OUT_EN_BIT);
	if (!writeRegister(ADAR1000_MISC_ENABLES, miscEnablesRegVal)) return false;

	/* Source all bias data from registers */
	if (!readRegister(ADAR1000_MEM_CTRL, memCtrlRegVal)) return false;
	BIT_SET(memCtrlRegVal, ADAR1000_MEM_CTRL_BIAS_RAM_BYPASS_BIT);
	if (!writeRegister(ADAR1000_MEM_CTRL, memCtrlRegVal)) return false;

	return true;
}

bool Adar1000::setPaBiasToRegister(Adar1000Channel_t channel, uint8_t onValue, uint8_t offValue)
{
	/* if in Tx mode (TR pin = 1 under TR pin control or ADAR1000_SW_CTRL_TX_EN_BIT = 1 under SPI control):
	 * load value from EXT_PAx_BIAS_ON to generate PA_BIASx pin voltage
	 */
	if (!writeRegister(ADAR1000_PA_BIAS_ON(channel), onValue)) return false;

	/* if in Rx mode (TR pin = 0 under TR pin control or ADAR1000_SW_CTRL_TX_EN_BIT = 0 under SPI control):
	 * load value from EXT_PAx_BIAS_OFF to generate PA_BIASx pin voltage
	 */
	if (!writeRegister(ADAR1000_PA_BIAS_OFF(channel), offValue)) return false;

	return true;
}

bool Adar1000::setPaBiasToRAM(Adar1000Channel_t channel, uint8_t onValue, uint8_t offValue, uint8_t nSetting)
{
	uint32_t biasSettingTxOnRegAddr = 0;
	uint32_t biasSettingTxOffRegAddr = 0;

	if (nSetting > 6) return false;

	switch (channel) {
		case ADAR1000_CHANNEL_1:
			biasSettingTxOnRegAddr = ADAR1000_BIASSETTING_TX_CH1_PABIAS_ON(nSetting);
			biasSettingTxOffRegAddr = ADAR1000_BIASSETTING_TX_CH1_PABIAS_OFF(nSetting);
		break;
		case ADAR1000_CHANNEL_2:
			biasSettingTxOnRegAddr = ADAR1000_BIASSETTING_TX_CH2_PABIAS_ON(nSetting);
			biasSettingTxOffRegAddr = ADAR1000_BIASSETTING_TX_CH2_PABIAS_OFF(nSetting);
		break;
		case ADAR1000_CHANNEL_3:
			biasSettingTxOnRegAddr = ADAR1000_BIASSETTING_TX_CH3_PABIAS_ON(nSetting);
			biasSettingTxOffRegAddr = ADAR1000_BIASSETTING_TX_CH3_PABIAS_OFF(nSetting);
		break;
		case ADAR1000_CHANNEL_4:
			biasSettingTxOnRegAddr = ADAR1000_BIASSETTING_TX_CH4_PABIAS_ON(nSetting);
			biasSettingTxOffRegAddr = ADAR1000_BIASSETTING_TX_CH4_PABIAS_OFF(nSetting);
	}

	/* if in Rx mode (TR pin = 0 under TR pin control or ADAR1000_SW_CTRL_TX_EN_BIT = 0 under SPI control):
	 * load value from EXT_PAx_BIAS_OFF to generate PA_BIASx pin voltage
	 */
	if (!writeRegister(biasSettingTxOffRegAddr, offValue)) return false;

	/* if in Tx mode (TR pin = 1 under TR pin control or ADAR1000_SW_CTRL_TX_EN_BIT = 1 under SPI control):
	 * load value from EXT_PAx_BIAS_ON to generate PA_BIASx pin voltage
	 */
	if (!writeRegister(biasSettingTxOnRegAddr, onValue)) return false;

	return true;
}

bool Adar1000::getPaBiasFromRegister(Adar1000Channel_t channel, uint8_t &onValue, uint8_t &offValue)
{
	if (!readRegister(ADAR1000_PA_BIAS_ON(channel), onValue)) return false;
	if (!readRegister(ADAR1000_PA_BIAS_OFF(channel), offValue)) return false;
	return true;
}

bool Adar1000::getPaBiasFromRAM(Adar1000Channel_t channel, uint8_t nSetting,
				uint8_t &onValue, uint8_t &offValue)
{
	uint32_t biasSettingTxOnRegAddr = 0;
	uint32_t biasSettingTxOffRegAddr = 0;

	if (nSetting > 6) return false;

	switch (channel) {
		case ADAR1000_CHANNEL_1:
			biasSettingTxOnRegAddr = ADAR1000_BIASSETTING_TX_CH1_PABIAS_ON(nSetting);
			biasSettingTxOffRegAddr = ADAR1000_BIASSETTING_TX_CH1_PABIAS_OFF(nSetting);
		break;
		case ADAR1000_CHANNEL_2:
			biasSettingTxOnRegAddr = ADAR1000_BIASSETTING_TX_CH2_PABIAS_ON(nSetting);
			biasSettingTxOffRegAddr = ADAR1000_BIASSETTING_TX_CH2_PABIAS_OFF(nSetting);
		break;
		case ADAR1000_CHANNEL_3:
			biasSettingTxOnRegAddr = ADAR1000_BIASSETTING_TX_CH3_PABIAS_ON(nSetting);
			biasSettingTxOffRegAddr = ADAR1000_BIASSETTING_TX_CH3_PABIAS_OFF(nSetting);
		break;
		case ADAR1000_CHANNEL_4:
			biasSettingTxOnRegAddr = ADAR1000_BIASSETTING_TX_CH4_PABIAS_ON(nSetting);
			biasSettingTxOffRegAddr = ADAR1000_BIASSETTING_TX_CH4_PABIAS_OFF(nSetting);
	}

	if (!readRegister(biasSettingTxOffRegAddr, offValue)) return false;
	if (!readRegister(biasSettingTxOnRegAddr, onValue)) return false;

	return true;
}

bool Adar1000::disableAllBiasOutputs()
{
	uint8_t writeRegData = 0;
	uint8_t readRegData = 0;

	if (!readRegister(ADAR1000_MISC_ENABLES, writeRegData)) return false;
	BIT_CLR(writeRegData, ADAR1000_MISC_ENABLES_LNA_BIAS_OUT_EN_BIT);
	BIT_SET(writeRegData, ADAR1000_MISC_ENABLES_BIAS_EN_BIT);
	if (!writeRegister(ADAR1000_MISC_ENABLES, writeRegData)) return false;

	if (!readRegister(ADAR1000_MISC_ENABLES, readRegData)) return false;
	if (readRegData != writeRegData) return false;

	return true;
}

/*
 * Follow settings in Table 16. Control of PA Bias Outputs (page 39/79)
 *  */
bool Adar1000::loadPaBiasFromRegister()
{
	uint8_t miscEnablesRegVal = 0;
	uint8_t memCtrlRegVal = 0;

	/* Config bias DACs vary between EXT_PAx_BIAS_ON/EXT_PAx_BIAS_OFF according to device mode (Tx, Rx) (BIAS_CTRL = 1) */
	if (!readRegister(ADAR1000_MISC_ENABLES, miscEnablesRegVal)) return false;
	/* External Amplifier Bias Control. If 0, DACs assume the on register values. If 1, DACs vary
	with device mode (transmit and receive). */
	BIT_SET(miscEnablesRegVal, ADAR1000_MISC_ENABLES_BIAS_CTRL_BIT);
	/* Enables PA and LNA Bias DACs. 0 = enabled */
	BIT_CLR(miscEnablesRegVal, ADAR1000_MISC_ENABLES_BIAS_EN_BIT);
	if (!writeRegister(ADAR1000_MISC_ENABLES, miscEnablesRegVal)) return false;

	/* Source all bias data from registers */
	if (!readRegister(ADAR1000_MEM_CTRL, memCtrlRegVal)) return false;
	BIT_SET(memCtrlRegVal, ADAR1000_MEM_CTRL_BIAS_RAM_BYPASS_BIT);
	if (!writeRegister(ADAR1000_MEM_CTRL, memCtrlRegVal)) return false;

	return true;
}

/*
 * Follow settings in Table 16. Control of PA Bias Outputs (page 39/79)
 * and section Bias Setting Memory Fetch (page 33/79)
 *  */
bool Adar1000::loadPaBiasFromRAM(Adar1000Direction_t direction, uint8_t nSetting)
{
	uint8_t miscEnablesRegVal = 0;
	uint8_t biasRamCtrlRegVal = 0;
	uint32_t biasRamCtrlRegAddr = 0;
	uint8_t memCtrlRegVal = 0;

	if (nSetting > 6) return false;

	/* Config bias DACs vary between EXT_PAx_BIAS_ON/EXT_PAx_BIAS_OFF according to device mode (Tx, Rx) (BIAS_CTRL = 1) */
	if (!readRegister(ADAR1000_MISC_ENABLES, miscEnablesRegVal)) return false;
	if (BIT_READ(miscEnablesRegVal, ADAR1000_MISC_ENABLES_BIAS_CTRL_BIT) == 0) {
		BIT_SET(miscEnablesRegVal, ADAR1000_MISC_ENABLES_BIAS_CTRL_BIT);
	}
	if (!writeRegister(ADAR1000_MISC_ENABLES, miscEnablesRegVal)) return false;

	/* Source all bias data from RAM */
	if (!readRegister(ADAR1000_MEM_CTRL, memCtrlRegVal)) return false;
	BIT_CLR(memCtrlRegVal, ADAR1000_MEM_CTRL_BIAS_RAM_BYPASS_BIT);
	if (!writeRegister(ADAR1000_MEM_CTRL, memCtrlRegVal)) return false;

	switch (direction) {
		case ADAR1000_DIRECTION_RX:
			biasRamCtrlRegAddr = ADAR1000_RX_BIAS_RAM_CTL;
		break;
		case ADAR1000_DIRECTION_TX:
			biasRamCtrlRegAddr = ADAR1000_TX_BIAS_RAM_CTL;
	}
	if (!readRegister(biasRamCtrlRegAddr, biasRamCtrlRegVal)) return false;
	/* assert the fetch bit high (Bit 3),
	 * write 3-bit bias setting (Value 0 through Value 6 maps to bias Setting 1 through Setting 7)
	 * */
	biasRamCtrlRegVal &= 0xF0; // keep [7:4] reserved bits
	biasRamCtrlRegVal |= (1 << 3) | nSetting;
	if (!writeRegister(biasRamCtrlRegAddr, biasRamCtrlRegVal)) return false;

	/* Provide at least six additional clock cycles on SCLK to load the new data from the RAM */
	if (!readRegister(biasRamCtrlRegAddr, biasRamCtrlRegVal)) return false;

	return true;
}

bool Adar1000::controlPaOnPin(Adar1000ControlState_t state)
{
	if (!m_pPaOnPin) return false;
	m_pPaOnPin->writeOutputPin((uint8_t)state);

	return true;
}
