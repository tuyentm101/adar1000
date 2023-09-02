/*
 * Adar1000.h
 *
 *  Created on: Apr 24, 2023
 *      Author: tuyentm4
 *
 *  Follow ADAR1000 RevB datasheet
 */

#ifndef ADAR1000_H_
#define ADAR1000_H_

#include "gpio.h"
#include "spi.h"

/* Exported define ------------------------------------------------------------*/
/* Function returns -------------------------------------------------*/

typedef enum
{
	ADAR1000_CHANNEL_1 = 0,
	ADAR1000_CHANNEL_2,
	ADAR1000_CHANNEL_3,
	ADAR1000_CHANNEL_4
} Adar1000Channel_t;

typedef enum
{
	ADAR1000_DIRECTION_RX = 0,
	ADAR1000_DIRECTION_TX
} Adar1000Direction_t;

typedef enum
{
	ADAR1000_CONTROL_STATE_OFF = 0,
	ADAR1000_CONTROL_STATE_ON
} Adar1000ControlState_t;

/* Phase values Table 10, 11, 12, 13 page 36, 37 of the datasheet - these values keep the vector modulator constant.
 phase_integer, phase_fraction, phase_i, phase_q
 * */
#define ADAR1000_PHASEMAP {{0, 0, 0x3f, 0x20}, {2, 8125, 0x3f, 0x21}, {5, 6250, 0x3f, 0x23},\
		{8, 4375, 0x3F, 0x24}, {11, 2500, 0x3F, 0x26}, {14, 625, 0x3E, 0x27},\
		{16, 8750, 0x3E, 0x28}, {19, 6875, 0x3D, 0x2A}, {22, 5000, 0x3D, 0x2B},\
		{25, 3125, 0x3C, 0x2D}, {28, 1250, 0x3C, 0x2E}, {30, 9375, 0x3B, 0x2F},\
		{33, 7500, 0x3A, 0x30}, {36, 5625, 0x39, 0x31}, {39, 3750, 0x38, 0x33},\
		{42, 1875, 0x37, 0x34}, {45, 0, 0x36, 0x35}, {47, 8125, 0x35, 0x36},\
		{50, 6250, 0x34, 0x37}, {53, 4375, 0x33, 0x38}, {56, 2500, 0x32, 0x38},\
		{59, 625, 0x30, 0x39}, {61, 8750, 0x2F, 0x3A}, {64, 6875, 0x2E, 0x3A},\
		{67, 5000, 0x2C, 0x3B}, {70, 3125, 0x2B, 0x3C}, {73, 1250, 0x2A, 0x3C},\
		{75, 9375, 0x28, 0x3C}, {78, 7500, 0x27, 0x3D}, {81, 5625, 0x25, 0x3D},\
		{84, 3750, 0x24, 0x3D}, {87, 1875, 0x22, 0x3D}, {90, 0, 0x21, 0x3D},\
		{92, 8125, 0x01, 0x3D}, {95, 6250, 0x03, 0x3D}, {98, 4375, 0x04, 0x3D},\
		{101, 2500, 0x06, 0x3D}, {104, 625, 0x07, 0x3C}, {106, 8750, 0x08, 0x3C},\
		{109, 6875, 0x0A, 0x3C}, {112, 5000, 0x0B, 0x3B}, {115, 3125, 0x0D, 0x3A},\
		{118, 1250, 0x0E, 0x3A}, {120, 9375, 0x0F, 0x39}, {123, 7500, 0x11, 0x38},\
		{126, 5625, 0x12, 0x38}, {129, 3750, 0x13, 0x37}, {132, 1875, 0x14, 0x36},\
		{135, 0, 0x16, 0x35}, {137, 8125, 0x17, 0x34}, {140, 6250, 0x18, 0x33},\
		{143, 4375, 0x19, 0x31}, {146, 2500, 0x19, 0x30}, {149, 625, 0x1A, 0x2F},\
		{151, 8750, 0x1B, 0x2E}, {154, 6875, 0x1C, 0x2D}, {157, 5000, 0x1C, 0x2B},\
		{160, 3125, 0x1D, 0x2A}, {163, 1250, 0x1E, 0x28}, {165, 9375, 0x1E, 0x27},\
		{168, 7500, 0x1E, 0x26}, {171, 5625, 0x1F, 0x24}, {174, 3750, 0x1F, 0x23},\
		{177, 1875, 0x1F, 0x21}, {180, 0, 0x1F, 0x20}, {182, 8125, 0x1F, 0x01},\
		{185, 6250, 0x1F, 0x03}, {188, 4375, 0x1F, 0x04}, {191, 2500, 0x1F, 0x06},\
		{194, 625, 0x1E, 0x07}, {196, 8750, 0x1E, 0x08}, {199, 6875, 0x1D, 0x0A},\
		{202, 5000, 0x1D, 0x0B}, {205, 3125, 0x1C, 0x0D}, {208, 1250, 0x1C, 0x0E},\
		{210, 9375, 0x1B, 0x0F}, {213, 7500, 0x1A, 0x10}, {216, 5625, 0x19, 0x11},\
		{219, 3750, 0x18, 0x13}, {222, 1875, 0x17, 0x14}, {225, 0, 0x16, 0x15},\
		{227, 8125, 0x15, 0x16}, {230, 6250, 0x14, 0x17}, {233, 4375, 0x13, 0x18},\
		{236, 2500, 0x12, 0x18}, {239, 625, 0x10, 0x19}, {241, 8750, 0x0F, 0x1A},\
		{244, 6875, 0x0E, 0x1A}, {247, 5000, 0x0C, 0x1B}, {250, 3125, 0x0B, 0x1C},\
		{253, 1250, 0x0A, 0x1C}, {255, 9375, 0x08, 0x1C}, {258, 7500, 0x07, 0x1D},\
		{261, 5625, 0x05, 0x1D}, {264, 3750, 0x04, 0x1D}, {267, 1875, 0x02, 0x1D},\
		{270, 0, 0x01, 0x1D}, {272, 8125, 0x21, 0x1D}, {275, 6250, 0x23, 0x1D},\
		{278, 4375, 0x24, 0x1D}, {281, 2500, 0x26, 0x1D}, {284, 625, 0x27, 0x1C},\
		{286, 8750, 0x28, 0x1C}, {289, 6875, 0x2A, 0x1C}, {292, 5000, 0x2B, 0x1B},\
		{295, 3125, 0x2D, 0x1A}, {298, 1250, 0x2E, 0x1A}, {300, 9375, 0x2F, 0x19},\
		{303, 7500, 0x31, 0x18}, {306, 5625, 0x32, 0x18}, {309, 3750, 0x33, 0x17},\
		{312, 1875, 0x34, 0x16}, {315, 0, 0x36, 0x15}, {317, 8125, 0x37, 0x14},\
		{320, 6250, 0x38, 0x13}, {323, 4375, 0x39, 0x11}, {326, 2500, 0x39, 0x10},\
		{329, 625, 0x3A, 0x0F}, {331, 8750, 0x3B, 0x0E}, {334, 6875, 0x3C, 0x0D},\
		{337, 5000, 0x3C, 0x0B}, {340, 3125, 0x3D, 0x0A}, {343, 1250, 0x3E, 0x08},\
		{345, 9375, 0x3E, 0x07}, {348, 7500, 0x3E, 0x06}, {351, 5625, 0x3F, 0x04},\
		{354, 3750, 0x3F, 0x03}, {357, 1875, 0x3F, 0x01}}

class Adar1000
{
	private:
		const char *const m_name;
		uint8_t m_chipAddress;
		Gpio *const m_pAddr0Pin;
		Gpio *const m_pAddr1Pin;
		Gpio *const m_pPaOnPin;
		Spi *const m_pSpi;

		bool searchPhaseReg(uint16_t integralPhase, uint16_t fractionalPhase,
				uint8_t &regI, uint8_t &regQ);
		bool searchPhase(uint8_t regI, uint8_t regQ,
				uint16_t &integralPhase, uint16_t &fractionalPhase);

		bool writeRegister(uint32_t addr, uint8_t data);
		bool readRegister(uint32_t addr, uint8_t &data);

		/**
		 * @fn bool configBeamDataLoadFromRegister()
		 *
		 * @brief config MEM_CTRL (0x38) register to load beam positions from registers
		 *
		 * @return 0 if successful
		 */
		bool configBeamDataLoadFromRegister();

		/**
		 * @fn bool configBeamDataLoadFromRAM(Adar1000Direction_t, bool)
		 *
		 * @brief config MEM_CTRL (0x38) register to load beam positions from RAM
		 * @brief and whether to load different beam position indices for each Tx/Rx channel
		 * @brief or to load the same beam position for all Tx/Rx channels
		 *
		 * @param direction: ADAR1000_CHANNEL_1 or ADAR1000_CHANNEL_2 or ADAR1000_CHANNEL_3 or ADAR1000_CHANNEL_4
		 * @param isSingleChannelSet: true if load different beam position indices for each Tx/Rx channel
		 *
		 * @return 0 if successful
		 */
		bool configBeamDataLoadFromRAM(Adar1000Direction_t direction, bool isSingleChannelSet);

	public:
		Adar1000(const char* name, uint8_t chipAddress, Spi *const p_spi, Gpio *const p_addr0Pin,
				Gpio *const p_addr1Pin, Gpio *const p_paOnPin);

		virtual ~Adar1000();

		const char *getName() const;

		/**
		 * @fn int init()
		 *
		 * @brief Assign chip address using addr0Pin and addr1Pin
		 * @brief Reset whole chip, use SDO line for readback, address auto incrementing in block write mode
		 * @brief Init internal ADC: turn on clock oscillator, turn on and reset ADC
		 * @brief Controls Polarity of Transmit/Receive Switch Driver Output
		 *
		 * @return 0 if successful
		 */
		int init();

		/**
		 * @fn int controlPowerDetector(Adar1000Channel_t, Adar1000ControlState_t)
		 *
		 * @brief
		 *
		 * @param channel: ADAR1000_CHANNEL_1 or ADAR1000_CHANNEL_2 or ADAR1000_CHANNEL_3 or ADAR1000_CHANNEL_4
		 * @param state
		 * @return 0 if successful
		 */
		int controlPowerDetector(Adar1000Channel_t channel, Adar1000ControlState_t state);

		/**
		 * @fn int readAdcTemperature(uint8_t&)
		 *
		 * @brief
		 *
		 * @param adcValue
		 *
		 * @return 0 if successful
		 */
		int getAdcTemperature(uint8_t &adcValue);

		/**
		 * @fn int getAdcPowerDetector(Adar1000Channel_t, uint8_t&)
		 *
		 * @brief
		 *
		 * @param channel: ADAR1000_CHANNEL_1 or ADAR1000_CHANNEL_2 or ADAR1000_CHANNEL_3 or ADAR1000_CHANNEL_4
		 * @param adcValue
		 *
		 * @return 0 if successful
		 */
		int getAdcPowerDetector(Adar1000Channel_t channel, uint8_t &adcValue);

		/**
		 * @fn int getDeviceInfo(uint8_t&, uint16_t&, uint16_t&)
		 *
		 * @brief
		 *
		 * @param chipType
		 * @param productId
		 * @param vendorId
		 *
		 * @return 0 if successful
		 */
		int getDeviceInfo(uint8_t &chipType, uint16_t &productId, uint16_t &vendorId);

		/**
		 * @fn int setTrCtrlByTrPin()
		 *
		 * @brief use this function to config TR control mode under TR pin
		 *
		 * @return 0 if successful
		 */
		int setTrCtrlByTrPin();

		/**
		 * @fn int setTrCtrlBySpi()
		 *
		 * @brief use this function to config TR control mode under SPI access
		 *
		 * @return 0 if successful
		 */
		int setTrCtrlBySpi();

		/**
		 * @fn int setPhaseToRegister(Adar1000Channel_t, Adar1000Direction_t, uint16_t, uint16_t)
		 *
		 * @brief set phase to register
		 *
		 * @param channel: ADAR1000_CHANNEL_1 or ADAR1000_CHANNEL_2 or ADAR1000_CHANNEL_3 or ADAR1000_CHANNEL_4
		 * @param direction: ADAR1000_DIRECTION_RX or ADAR1000_DIRECTION_TX
		 * @param integralPhaseDegree
		 * @param fractionalPhaseDegree
		 *
		 * @return 0 if successful
		 */
		int setPhaseToRegister(Adar1000Channel_t channel, Adar1000Direction_t direction,
				uint16_t integralPhaseDegree, uint16_t fractionalPhaseDegree);

		/**
		 * @fn int setPhaseToRegister(Adar1000Channel_t, Adar1000Direction_t, uint8_t, uint8_t)
		 * @brief
		 *
		 * @param channel: ADAR1000_CHANNEL_1 or ADAR1000_CHANNEL_2 or ADAR1000_CHANNEL_3 or ADAR1000_CHANNEL_4
		 * @param direction: ADAR1000_DIRECTION_RX or ADAR1000_DIRECTION_TX
		 * @param phaseI
		 * @param phaseQ
		 *
		 * @return 0 if successful
		 */
		int setPhaseToRegister(Adar1000Channel_t channel, Adar1000Direction_t direction,
				uint8_t phaseI, uint8_t phaseQ);

		/**
		 * @fn int setPhaseToRAM(Adar1000Channel_t, Adar1000Direction_t, uint8_t, uint16_t, uint16_t)
		 *
		 * @brief set phase to RAM by beam position
		 *
		 * @param channel: ADAR1000_CHANNEL_1 or ADAR1000_CHANNEL_2 or ADAR1000_CHANNEL_3 or ADAR1000_CHANNEL_4
		 * @param direction: ADAR1000_DIRECTION_RX or ADAR1000_DIRECTION_TX
		 * @param beamPosition: beam position in RAM memory (0 ... 120)
		 * @param integralPhaseDegree
		 * @param fractionalPhaseDegree
		 *
		 * @return 0 if successful
		 */
		int setPhaseToRAM(Adar1000Channel_t channel, Adar1000Direction_t direction,
				uint8_t beamPosition, uint16_t integralPhaseDegree, uint16_t fractionalPhaseDegree);

		/**
		 * @fn int setPhaseToRAM(Adar1000Channel_t, Adar1000Direction_t, uint8_t, uint8_t, uint8_t)
		 * @brief
		 *
		 * @param channel: ADAR1000_CHANNEL_1 or ADAR1000_CHANNEL_2 or ADAR1000_CHANNEL_3 or ADAR1000_CHANNEL_4
		 * @param direction: ADAR1000_DIRECTION_RX or ADAR1000_DIRECTION_TX
		 * @param beamPosition: beam position in RAM memory (0 ... 120)
		 * @param phaseI
		 * @param phaseQ
		 *
		 * @return 0 if successful
		 */
		int setPhaseToRAM(Adar1000Channel_t channel, Adar1000Direction_t direction,
				uint8_t beamPosition, uint8_t phaseI, uint8_t phaseQ);

		/**
		 * @fn int getPhaseFromRegister(Adar1000Channel_t, Adar1000Direction_t, uint16_t&, uint16_t&)
		 *
		 * @brief get phase from register
		 *
		 * @param channel: ADAR1000_CHANNEL_1 or ADAR1000_CHANNEL_2 or ADAR1000_CHANNEL_3 or ADAR1000_CHANNEL_4
		 * @param direction: ADAR1000_DIRECTION_RX or ADAR1000_DIRECTION_TX
		 * @param integralPhase
		 * @param fractionalPhase
		 *
		 * @return 0 if successful
		 */
		int getPhaseFromRegister(Adar1000Channel_t channel, Adar1000Direction_t direction,
				uint16_t &integralPhase, uint16_t &fractionalPhase);

		/**
		 * @fn int getPhaseFromRegister(Adar1000Channel_t, Adar1000Direction_t, uint8_t&, uint8_t&)
		 * @brief
		 *
		 * @param channel: ADAR1000_CHANNEL_1 or ADAR1000_CHANNEL_2 or ADAR1000_CHANNEL_3 or ADAR1000_CHANNEL_4
		 * @param direction: ADAR1000_DIRECTION_RX or ADAR1000_DIRECTION_TX
		 * @param phaseI
		 * @param phaseQ
		 *
		 * @return 0 if successful
		 */
		int getPhaseFromRegister(Adar1000Channel_t channel, Adar1000Direction_t direction,
				uint8_t &phaseI, uint8_t &phaseQ);

		/**
		 * @fn int getPhaseFromRAM(Adar1000Channel_t, Adar1000Direction_t, uint8_t, uint16_t&, uint16_t&)
		 *
		 * @brief get phase from RAM by beam position
		 *
		 * @param channel: ADAR1000_CHANNEL_1 or ADAR1000_CHANNEL_2 or ADAR1000_CHANNEL_3 or ADAR1000_CHANNEL_4
		 * @param direction: ADAR1000_DIRECTION_RX or ADAR1000_DIRECTION_TX
		 * @param beamPosition: beam position in RAM memory (0 ... 120)
		 * @param integralPhase
		 * @param fractionalPhase
		 *
		 * @return 0 if successful
		 */
		int getPhaseFromRAM(Adar1000Channel_t channel, Adar1000Direction_t direction, uint8_t beamPosition,
				uint16_t &integralPhase, uint16_t &fractionalPhase);

		/**
		 * @fn int getPhaseFromRAM(Adar1000Channel_t, Adar1000Direction_t, uint8_t, uint8_t&, uint8_t&)
		 * @brief
		 *
		 * @param channel: ADAR1000_CHANNEL_1 or ADAR1000_CHANNEL_2 or ADAR1000_CHANNEL_3 or ADAR1000_CHANNEL_4
		 * @param direction: ADAR1000_DIRECTION_RX or ADAR1000_DIRECTION_TX
		 * @param beamPosition: beam position in RAM memory (0 ... 120)
		 * @param phaseI
		 * @param phaseQ
		 *
		 * @return 0 if successful
		 */
		int getPhaseFromRAM(Adar1000Channel_t channel, Adar1000Direction_t direction, uint8_t beamPosition,
				uint8_t &phaseI, uint8_t &phaseQ);

		/**
		 * @fn int setGainToRegister(Adar1000Channel_t, Adar1000Direction_t, uint8_t)
		 *
		 * @brief set gain to register
		 *
		 * @param channel: ADAR1000_CHANNEL_1 or ADAR1000_CHANNEL_2 or ADAR1000_CHANNEL_3 or ADAR1000_CHANNEL_4
		 * @param direction: ADAR1000_DIRECTION_RX or ADAR1000_DIRECTION_TX
		 * @param gainControlCode: 8 bit gain control code (0 ... 255) (figure 95 page 35/79)
		 *
		 * @return 0 if successful
		 */
		int setGainToRegister(Adar1000Channel_t channel, Adar1000Direction_t direction, uint8_t gainControlCode);

		/**
		 * @fn int setGainToRAM(Adar1000Channel_t, Adar1000Direction_t, uint8_t, Adar1000ControlState_t, uint8_t)
		 *
		 * @brief set gain to RAM by beam position
		 *
		 * @param channel: ADAR1000_CHANNEL_1 or ADAR1000_CHANNEL_2 or ADAR1000_CHANNEL_3 or ADAR1000_CHANNEL_4
		 * @param direction: ADAR1000_DIRECTION_RX or ADAR1000_DIRECTION_TX
		 * @param beamPosition: beam position in RAM memory (0 ... 120)
		 * @param gainControlCode: 8 bit gain control code (0 ... 255) (figure 95 page 35/79)
		 *
		 * @return 0 if successful
		 */
		int setGainToRAM(Adar1000Channel_t channel, Adar1000Direction_t direction, uint8_t beamPosition, uint8_t gainControlCode);

		/**
		 * @fn int getGainFromRegister(Adar1000Channel_t, Adar1000Direction_t, uint8_t&)
		 *
		 * @brief get gain from register
		 *
		 * @param channel: ADAR1000_CHANNEL_1 or ADAR1000_CHANNEL_2 or ADAR1000_CHANNEL_3 or ADAR1000_CHANNEL_4
		 * @param direction: ADAR1000_DIRECTION_RX or ADAR1000_DIRECTION_TX
		 * @param gainControlCode: 8 bit gain control code (0 ... 255) (figure 95 page 35/79)
		 *
		 * @return 0 if successful
		 */
		int getGainFromRegister(Adar1000Channel_t channel, Adar1000Direction_t direction,
				uint8_t &gainControlCode);

		/**
		 * @fn int getGainFromRAM(Adar1000Channel_t, Adar1000Direction_t, uint8_t, uint8_t&)
		 *
		 * @brief get gain from RAM by position
		 *
		 * @param channel: ADAR1000_CHANNEL_1 or ADAR1000_CHANNEL_2 or ADAR1000_CHANNEL_3 or ADAR1000_CHANNEL_4
		 * @param direction: ADAR1000_DIRECTION_RX or ADAR1000_DIRECTION_TX
		 * @param beamPosition: beam position in RAM memory (0 ... 120)
		 * @param gainControlCode: 8 bit gain control code (0 ... 255) (figure 95 page 35/79)
		 *
		 * @return 0 if successful
		 */
		int getGainFromRAM(Adar1000Channel_t channel, Adar1000Direction_t direction, uint8_t beamPosition,
				uint8_t &gainControlCode);

		/**
		 * @fn int loadGainPhaseFromRegister(Adar1000Direction_t)
		 *
		 * @brief Loads Transmit Working Registers from SPI.
		 * @brief Assert high to update transmit gain and phase settings.
		 * @brief Also can be used when advancing transmit beam position.
		 *
		 * @param direction: ADAR1000_DIRECTION_RX or ADAR1000_DIRECTION_TX
		 *
		 * @return 0 if successful
		 */
		int loadGainPhaseFromRegister(Adar1000Direction_t direction);

		/**
		 * @fn int loadGainPhaseFromRAM(Adar1000Channel_t, Adar1000Direction_t, uint8_t, uint8_t)
		 *
		 * @brief Refer to section Single Memory Fetch (page 32/79)
		 *
		 * @param channel: ADAR1000_CHANNEL_1 or ADAR1000_CHANNEL_2 or ADAR1000_CHANNEL_3 or ADAR1000_CHANNEL_4
		 * @param direction: ADAR1000_DIRECTION_RX or ADAR1000_DIRECTION_TX
		 * @param beamPosition: beam position in RAM memory (0 ... 120)
		 *
		 * @return 0 if successful
		 */
		int loadGainPhaseFromRAM(Adar1000Channel_t channel, Adar1000Direction_t direction, uint8_t beamPosition);

		/**
		 * @fn int updateGainPhaseAllFromRAM(Adar1000Direction_t, uint8_t)
		 *
		 * @brief loading all four transmit/receive channels with the same beam position index
		 * @brief Refer to section Single Memory Fetch (page 32/79)
		 *
		 * @param direction: ADAR1000_DIRECTION_RX or ADAR1000_DIRECTION_TX
		 * @param beamPosition: beam position in RAM memory (0 ... 120)
		 *
		 * @return 0 if successful
		 */
		int loadGainPhaseAllFromRAM(Adar1000Direction_t direction, uint8_t beamPosition);

		/**
		 * @fn int controlTxStateBySpi(Adar1000Channel_t, Adar1000Direction_t, Adar1000ControlState_t)
		 *
		 * @brief Use this function when the device is in SPI control
		 * @brief Enable/disable the Tx subcircuits. The Tx and Rx subcircuits cannot be turned on simultaneously
		 *
		 * @param channel: ADAR1000_CHANNEL_1 or ADAR1000_CHANNEL_2 or ADAR1000_CHANNEL_3 or ADAR1000_CHANNEL_4
		 * @param state: ADAR1000_CONTROL_STATE_OFF, ADAR1000_CONTROL_STATE_ON
		 *
		 * @return 0 if successful
		 */
		int controlTxStateBySpi(Adar1000Channel_t channel, Adar1000ControlState_t state);

		/**
		 * @fn int controlRxStateBySpi(Adar1000Channel_t, Adar1000Direction_t, Adar1000ControlState_t)
		 * @brief Use this function when the device is in SPI control
		 * @brief Enable/disable the Rx subcircuits. The Tx and Rx subcircuits cannot be turned on simultaneously
		 *
		 * @param channel: ADAR1000_CHANNEL_1 or ADAR1000_CHANNEL_2 or ADAR1000_CHANNEL_3 or ADAR1000_CHANNEL_4
		 * @param state: ADAR1000_CONTROL_STATE_OFF, ADAR1000_CONTROL_STATE_ON
		 *
		 * @return 0 if successful
		 */
		int controlRxStateBySpi(Adar1000Channel_t channel, Adar1000ControlState_t state);

		/**
		 * @fn int switchToTxBySpi()
		 *
		 * @brief Use this function when the device is in SPI control
		 * @brief Need to call controlTxState() in advance to config Tx mode to specific channel
		 * @brief Setting TX_EN = 1 causes the PA_BIASx pins to assume the EXT_PAx_BIAS_ON values.
		 * 		  Setting TX_EN = 0 causes the PA_BIASx pins to assume the EXT_PAx_BIAS_OFF values.
		 * 		  It is important to note that the PA_BIASx pins follow the TX_EN bit,
		 * 		  not the TR_SPI bit, while in SPI control
		 *
		 * @return 0 if successful
		 */
		int switchToTxBySpi();

		/**
		 * @fn int switchToRxBySpi()
		 *
		 * @brief Use this function when the device is in SPI control
		 * @brief Need to call controlRxState() in advance to config Rx mode to specific channel
		 * @brief Setting RX_EN = 1 causes the LNA_BIAS pin to assume the EXT_LNAx_BIAS_ON values.
		 * 		  Setting RX_EN = 0 causes the LNA_BIAS pin to assume the EXT_LNAx_BIAS_OFF values.
		 * 		  It is important to note that the LNA_BIAS pin follows the RX_EN bit,
		 * 		  not the TR_SPI bit, while in SPI control.
		 *
		 * @return 0 if successful
		 */
		int switchToRxBySpi();

		/**
		 * @fn int disableAllTrSwOutputs()
		 *
		 * @brief disable current TR_SW control output, lead TR_SW_POS, TR_SW_NEG pin to floating state
		 * @brief use this function in both TR pin and SPI control
		 * @brief Follow Table 14. Controlling TR_SW_POS and TR_SW_NEG Output
		 *
		 * @return 0 if successful
		 */
		int disableAllTrSwOutputs();

		/**
		 * @fn int enableTrSwPosOutput()
		 *
		 * @brief Select TR_SW_POS as Transmit/Receive Output Driver Select
		 * @brief Setting follows Table 14. Controlling TR_SW_POS and TR_SW_NEG Output (page 38)
		 *
		 * @return 0 if successful
		 */
		int enableTrSwPosOutput();

		/**
		 * @fn int enableTrSwNegOutput()
		 *
		 * @brief Select TR_SW_NEG as Transmit/Receive Output Driver Select
		 *
		 * @return 0 if successful
		 */
		int enableTrSwNegOutput();

		/**
		 * @fn int controlBiasDACVaryWithDeviceMode(Adar1000ControlState_t)
		 *
		 * @brief Need to call this API in advance before control state of PA/LNA Bias output
		 *        and check front - end chip's  specification for External Amplifier Bias
		 * @brief config BIAS_EN bit in MISC_ENABLES (0x38) register for External Amplifier Bias Control.
		 *        If 0, DACs assume the ON register values.
		 * 	      If 1, DACs vary with device mode (transmit and receive).
		 *
		 * @param state: ADAR1000_CONTROL_STATE_OFF, ADAR1000_CONTROL_STATE_ON
		 *
		 * @return 0 if successful
		 */
		int controlBiasDACVaryWithDeviceMode(Adar1000ControlState_t state);

		/**
		 * @fn int controlBiasDAC(Adar1000ControlState_t)
		 *
		 * @brief config BIAS_CTRL bit in MISC_ENABLE (0x38) register to enable/disable PA/LNA Bias DACs
		 *
		 * @param state: ADAR1000_CONTROL_STATE_OFF, ADAR1000_CONTROL_STATE_ON
		 *
		 * @return 0 if successful
		 */
		int controlBiasDAC(Adar1000ControlState_t state);

		/**
		 * @fn int setBiasSettingCurrentToRegister(uint8_t, uint8_t, uint8_t, uint8_t)
		 * @brief set bias current setting of each of the active RF subcircuits to trade RF performance for lower dc power
		 *        Follow Table 6. SPI Settings for Nominal and Low Power Modes (30/79)
		 *        set value to registers
		 *
		 * @param currentRx: RX_VM_BIAS/RX_VGA_BIAS bit field in BIAS_CURRENT_RX (0x35) register for Receive Vector Modulator and Receive VGA subcircuit
		 * @param currentRxLna: LNA_BIAS bit field in BIAS_CURRENT_RX_LNA (0x34) register for Receive LNA subcircuit
		 * @param currentTx: TX_VM_BIAS/TX_VGA_BIAS bit field in BIAS_CURRENT_TX (0x36) register for Transmit Vector Modulator/Transmit VGA subcircuits
		 * @param currentTxDrv: TX_DRV_BIAS bit field in BIAS_CURRENT_TX_DRV (0x37) register for Transmit Driver subcircuit
		 *
		 * @return 0 if successful
		 */
		int setBiasSettingCurrentToRegister(uint8_t currentRx, uint8_t currentRxLna,
				uint8_t currentTx, uint8_t currentTxDrv);

		/**
		 * @fn int setBiasSettingCurrentToRAM(uint8_t, uint8_t, uint8_t, uint8_t, uint8_t)
		 * @brief set bias current setting of each of the active RF subcircuits to trade RF performance for lower dc power
		 *        Follow Table 6. SPI Settings for Nominal and Low Power Modes (30/79)
		 *        set values to RAM at setting index
		 *
		 * @param currentRx: RX_VM_BIAS/RX_VGA_BIAS bit field in BIAS_CURRENT_RX for Receive Vector Modulator and Receive VGA subcircuit
		 * @param currentRxLna: LNA_BIAS bit field in BIAS_CURRENT_RX_LNA for Receive LNA subcircuit
		 * @param currentTx: TX_VM_BIAS/TX_VGA_BIAS bit field in BIAS_CURRENT_TX for Transmit Vector Modulator/Transmit VGA subcircuits
		 * @param currentTxDrv: TX_DRV_BIAS bit field in BIAS_CURRENT_TX_DRV for Transmit Driver subcircuit
		 * @param nSetting: setting index in RAM for Bias (1 ... 7)
		 *
		 * @return 0 if successful
		 */
		int setBiasSettingCurrentToRAM(uint8_t currentRx, uint8_t currentRxLna,
				uint8_t currentTx, uint8_t currentTxDrv,
				uint8_t nSetting);

		/**
		 * @fn int getBiasSettingCurrentFromRegister(uint8_t&, uint8_t&, uint8_t&, uint8_t&)
		 * @brief get value from registers
		 *
		 * @param currentRx: RX_VM_BIAS/RX_VGA_BIAS bit field in BIAS_CURRENT_RX (0x35) register for Receive Vector Modulator and Receive VGA subcircuit
		 * @param currentRxLna: LNA_BIAS bit field in BIAS_CURRENT_RX_LNA (0x34) register for Receive LNA subcircuit
		 * @param currentTx: TX_VM_BIAS/TX_VGA_BIAS bit field in BIAS_CURRENT_TX (0x36) register for Transmit Vector Modulator/Transmit VGA subcircuits
		 * @param currentTxDrv: TX_DRV_BIAS bit field in BIAS_CURRENT_TX_DRV (0x37) register for Transmit Driver subcircuit
		 *
		 * @return 0 if successful
		 */
		int getBiasSettingCurrentFromRegister(uint8_t &currentRx, uint8_t &currentRxLna,
				uint8_t &currentTx, uint8_t &currentTxDrv);

		/**
		 * @fn int getBiasSettingCurrentFromRAM(uint8_t&, uint8_t&, uint8_t&, uint8_t&, uint8_t)
		 * @brief get value from RAM at setting index
		 *
		 * @param currentRx: RX_VM_BIAS/RX_VGA_BIAS bit field in BIAS_CURRENT_RX for Receive Vector Modulator and Receive VGA subcircuit
		 * @param currentRxLna: LNA_BIAS bit field in BIAS_CURRENT_RX_LNA for Receive LNA subcircuit
		 * @param currentTx: TX_VM_BIAS/TX_VGA_BIAS bit field in BIAS_CURRENT_TX for Transmit Vector Modulator/Transmit VGA subcircuits
		 * @param currentTxDrv: TX_DRV_BIAS bit field in BIAS_CURRENT_TX_DRV for Transmit Driver subcircuit
		 * @param nSetting: setting index in RAM for Bias (1 ... 7)
		 *
		 * @return 0 if successful
		 */
		int getBiasSettingCurrentFromRAM(uint8_t &currentRx, uint8_t &currentRxLna,
				uint8_t &currentTx, uint8_t &currentTxDrv,
				uint8_t nSetting);

		/**
		 * @fn bool setLnaBiasDACToRegister(uint8_t, uint8_t)
		 *
		 * @brief set on/off LNA bias DAC value to EXT_LNA_BIAS_ON/EXT_LNA_BIAS_OFF register
		 *
		 * @param onValue: follow specification of front - end chip
		 * @param offValue: follow specification of front - end chip
		 *
		 * @return 0 if successful
		 */
		int setLnaBiasDACToRegister(uint8_t onValue, uint8_t offValue);

		/**
		 * @fn bool setLnaBiasDACToRAM(uint8_t, uint8_t, uint8_t)
		 * @brief set on/off LNA Bias DAC value to RAM at setting index
		 *
		 * @param onValue: follow specification of front - end chip
		 * @param offValue: follow specification of front - end chip
		 * @param nSetting: setting index in RAM (1 ... 7)
		 *
		 * @return 0 if successful
		 */
		int setLnaBiasDACToRAM(uint8_t onValue, uint8_t offValue, uint8_t nSetting);

		/**
		 * @fn int getLnaBiasDACFromRegister(uint8_t&, uint8_t&)
		 *
		 * @brief get on/off LNA bias DAC value from EXT_LNA_BIAS_ON/EXT_LNA_BIAS_OFF register
		 *
		 * @param onValue
		 * @param offValue
		 * @return 0 if successful
		 */
		int getLnaBiasDACFromRegister(uint8_t &onValue, uint8_t &offValue);

		/**
		 * @fn int getLnaBiasDACFromRAM(uint8_t,uint8_t,uint8_t)
		 *
		 * @brief get on/off LNA Bias DAC value from RAM at setting index
		 *
		 * @param onValue
		 * @param offValue
		 * @return 0 if successful
		 */
		int getLnaBiasDACFromRAM(uint8_t nSetting, uint8_t &onValue, uint8_t &offValue);

		/**
		 * @fn int setPaBiasDACToRegister(Adar1000Channel_t, uint8_t, uint8_t)
		 *
		 * @brief set on/off PA Bias DAC value to EXT_PAx_BIAS_ON/EXT_PAx_BIAS_OFF register
		 *
		 * @param channel: ADAR1000_CHANNEL_1 or ADAR1000_CHANNEL_2 or ADAR1000_CHANNEL_3 or ADAR1000_CHANNEL_4
		 * @param onValue: follow specification of front - end chip, e.g, ADTR1107 0x39 (-1.1V)
		 * @param offValue: follow specification of front - end chip, e.g, ADTR1107 0x85 (-2.5V)
		 *
		 * @return 0 if successful
		 */
		int setPaBiasDACToRegister(Adar1000Channel_t channel, uint8_t onValue, uint8_t offValue);

		/**
		 * @fn int setPaBiasDACToRAM(Adar1000Channel_t, uint8_t)
		 *
		 * @brief set on/off PA Bias DAC value to RAM at setting index
		 *
		 * @param channel: ADAR1000_CHANNEL_1 or ADAR1000_CHANNEL_2 or ADAR1000_CHANNEL_3 or ADAR1000_CHANNEL_4
		 * @param onValue: follow specification of front - end chip, e.g, ADTR1107 0x39 (-1.1V)
		 * @param offValue: follow specification of front - end chip, e.g, ADTR1107 0x85 (-2.5V)
		 * @param nSetting: setting index in RAM (1 ... 7)
		 *
		 * @return 0 if successful
		 */
		int setPaBiasDACToRAM(Adar1000Channel_t channel, uint8_t onValue, uint8_t offValue, uint8_t nSetting);

		/**
		 * @fn int getPaBiasDACFromRegister(Adar1000Channel_t, uint8_t&, uint8_t&)
		 *
		 * @brief get on/off Pa Bias DAC value from EXT_PAx_BIAS_ON/EXT_PAx_BIAS_OFF register
		 *
		 * @param channel: ADAR1000_CHANNEL_1 or ADAR1000_CHANNEL_2 or ADAR1000_CHANNEL_3 or ADAR1000_CHANNEL_4
		 * @param onValue
		 * @param offValue
		 *
		 * @return 0 if successful
		 */
		int getPaBiasDACFromRegister(Adar1000Channel_t channel, uint8_t &onValue, uint8_t &offValue);

		/**
		 * @fn int getPaBiasDACFromRAM(Adar1000Channel_t, uint8_t, uint8_t&, uint8_t&)
		 *
		 * @brief get on/off Pa Bias DAC value from RAM at setting index
		 *
		 * @param channel: ADAR1000_CHANNEL_1 or ADAR1000_CHANNEL_2 or ADAR1000_CHANNEL_3 or ADAR1000_CHANNEL_4
		 * @param nSetting: setting index in RAM (1 ... 7)
		 * @param onValue
		 * @param offValue
		 *
		 * @return 0 if successful
		 */
		int getPaBiasDACFromRAM(Adar1000Channel_t channel, uint8_t nSetting,
				uint8_t &onValue, uint8_t &offValue);

		/**
		 * @fn int applyBiasSettingFromRegister()
		 *
		 * @brief apply values of PA Bias DAC, LNA Bias DAC, Bias setting current (Rx, Rx LNA, Tx, Tx Drv)
		 * 		  from associated register
		 *
		 * @return 0 if successful
		 */
		int applyBiasSettingFromRegister();

		/**
		 * @fn int applyBiasSettingFromRAM(Adar1000Direction_t, uint8_t)
		 * @brief apply values of PA Bias DAC and Bias setting current (Tx, Tx Drv) in case of Tx direction,
		 * 		  values of LNA Bias DAC and Bias setting current (Rx, Rx LNA) in case of Rx direction
		 * 		  from RAM at setting index
		 *
		 * @param direction: ADAR1000_DIRECTION_RX or ADAR1000_DIRECTION_TX
		 * @param nSetting: setting index in RAM (1 ... 7)
		 *
		 * @return 0 if successful
		 */
		int applyBiasSettingFromRAM(Adar1000Direction_t direction, uint8_t nSetting);

		/**
	     * @fn int controlLnaBiasOutput(Adar1000ControlState_t)
		 * @brief enable/disable LNA Bias output
		 *
		 * @param state: ADAR1000_CONTROL_STATE_OFF, ADAR1000_CONTROL_STATE_ON
		 *
		 * @return 0 if successful
		 */
		int controlLnaBiasOutput(Adar1000ControlState_t state);

		/**
		 * @fn int disableBiasOutput()
		 *
		 * @brief disable all PA and LNA bias outputs
		 *
		 * @return 0 if successful
		 */
		int disableAllBiasOutputs();

		/**
		 * @fn int controlPaOnPin(Adar1000ControlState_t)
		 *
		 * @brief use this function to enable/disable PA Bias outputs when the device is in TR pin control
		 * @brief if direction is Rx, PAx_BIAS voltage is set by EXT_PAx_BIAS_OFF value, PA_ON pin state is ignored
		 * @brief if direction is Tx:
		 * @brief 		PAx_BIAS voltage is set by EXT_PAx_BIAS_ON value if PA_ON pin = 1 (paOnState = ON)
		 * @brief 		PAx_BIAS voltage is set by EXT_PAx_BIAS_OFF value if PA_ON pin = 0 (paOnState = OFF)
		 *
		 * @param state: ADAR1000_CONTROL_STATE_OFF or ADAR1000_CONTROL_STATE_ON
		 *
		 * @return 0 if successful
		 */
		int controlPaOnPin(Adar1000ControlState_t state);
};

#endif /* ADAR1000_H_ */
