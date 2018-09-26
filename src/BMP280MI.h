//Multi interface Bosch Sensortec BMP280  pressure sensor library 
// Copyright (c) 2018 Gregor Christandl <christandlg@yahoo.com>
// home: https://bitbucket.org/christandlg/bmp280mi
//
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.

// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.

// You should have received a copy of the GNU Lesser General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA


#ifndef BMP280MI_H_
#define BMP280MI_H_

#include <Arduino.h>

#include <SPI.h>
#include <Wire.h>

class BMP280MI
{
public:

	enum filter_setting_t : uint8_t
	{
		FILTER_OFF = 0b000,
		FILTER_x02 = 0b001,
		FILTER_x04 = 0b010,
		FILTER_x08 = 0b011,
		FILTER_x16 = 0b100,
	};

	//pressure measurement oversampling register settings
	enum osrs_p : uint8_t
	{
		OSRS_P_x01 = 0b001,
		OSRS_P_x02 = 0b010,
		OSRS_P_x04 = 0b011,
		OSRS_P_x08 = 0b100,
		OSRS_P_x16 = 0b101
	};

	//temperature measurement oversampling register settings
	enum osrs_t : uint8_t
	{
		OSRS_T_x01 = 0b001,		//16 bits
		OSRS_T_x02 = 0b010,		//17 bits
		OSRS_T_x04 = 0b011,		//18 bits
		OSRS_T_x08 = 0b100,		//19 bits
		OSRS_T_x16 = 0b101		//20 bits
	};

	enum power_mode_t : uint8_t
	{
		BMP280_MODE_SLEEP = 0b00,	//device in sleep mode. 
		BMP280_MODE_FORCED = 0b01,	//or 0b10. measurements are performed on demand. 
		BMP280_MODE_NORMAL = 0b11	//measurements are performed periodically. 
	};

	enum standby_time_t : uint8_t
	{
		T_SB_0 = 0b000,		//0.5 ms
		T_SB_1 = 0b001, 	//62.5 ms
		T_SB_2 = 0b010, 	//125 ms
		T_SB_3 = 0b011, 	//250 ms
		T_SB_4 = 0b100, 	//500 ms
		T_SB_5 = 0b101, 	//1000 ms
		T_SB_6 = 0b110, 	//2000 ms
		T_SB_7 = 0b111, 	//4000 ms
	};

	BMP280MI();
	virtual ~BMP280MI();

	virtual bool begin() = 0;

	//starts a measurement. 
	//@return true on success, false otherwise. 
	bool measure();

	//@return true if a measurement was completed, false otherwise. 
	bool hasValue();

	//@return the last measured pressure, in Pa. 
	float getPressure();

	//@return the last measured temperature, in deg C. 
	float getTemperature();

	//triggers a measurement and returns the measured temperature. 
	//@return temperature in deg C or NAN if the measurement failed. 
	float readTemperature();

	//triggers a measurement and returns the measured pressure. 
	//@return pressure in Pa or NAN if the measurement failed. 
	float readPressure();

	//@return the ID of the BMP280. the sensor will always return 0x58, so this function 
	//can be used as a communication check. 
	uint8_t readID();

	/*
	resets all registers to default values. */
	void resetToDefaults();

	//@return pressure oversampling setting
	uint8_t readOversamplingPressure();

	//@param value to set
	//@return true on success, false otherwise. 
	bool writeOversamplingPressure(uint8_t value);

	//@return pressure oversampling setting
	uint8_t readOversamplingTemperature();

	//@param value to set
	//@return true on success, false otherwise. 
	bool writeOversamplingTemperature(uint8_t value);

	//@return filter setting as filter_setting_t.
	uint8_t readFilterSetting();

	//@param filter setting as filter_setting_t.
	//@return true on success, false otherwise. 
	bool writeFilterSetting(uint8_t setting);

	//@return sensors power mode as power_mode_t.
	uint8_t readPowerMode();

	//sets the sensors power mode. 
	//@param power mode as power_mode_t
	//@return true on success, false otherwise. 
	bool writePowerMode(uint8_t mode);

	//@return standby time as standby_time_t. 
	uint8_t readStandbyTime();

	//sets the sensors standby time. only has an effect if measurements are done in 'normal' (automatic) mode. 
	//@param standby time as standby_time_t. 
	//@return true on success, false otherwise. 
	bool writeStandbyTime(uint8_t standby_time);

protected:
	static const uint8_t BMP280_ID = 0x58;

private:
	enum bmp280_register_t : uint8_t
	{
		BMP280_REG_ID = 0xD0,			//contains 0x58 after power on
		BMP280_REG_RESET = 0xE0,		//write 0xB6 to reset
		BMP280_REG_STATUS = 0xF3,		//bit 0: im_update, bit 3: measuring
		BMP280_REG_CTRL_MEAS = 0xF4,	//sets data acquisition options of device	
		BMP280_REG_CONFIG = 0xF5,		//sets the rate, filter and interface options of the device.
		BMP280_REG_PRESS = 0xF7,		//raw pressure measurement output data up[19:0] in registers 0xF7...0xF9
		BMP280_REG_TEMP = 0xFA,			//raw temperature measurement output data ut[19:0] in registers 0xFA...0xFC

		BMP280_REG_DIG_T1 = 0x88,		//dig_T1, unsigned short
		BMP280_REG_DIG_T2 = 0x8A,		//dig_T2, signed short
		BMP280_REG_DIG_T3 = 0x8C,		//dig_T3, signed short

		BMP280_REG_DIG_P1 = 0x8E,		//dig_P1, unsigned short
		BMP280_REG_DIG_P2 = 0x90,		//dig_P2, signed short
		BMP280_REG_DIG_P3 = 0x92,		//dig_P3, signed short
		BMP280_REG_DIG_P4 = 0x94,		//dig_P4, signed short
		BMP280_REG_DIG_P5 = 0x96,		//dig_P5, signed short
		BMP280_REG_DIG_P6 = 0x98,		//dig_P6, signed short
		BMP280_REG_DIG_P7 = 0x9A,		//dig_P7, signed short
		BMP280_REG_DIG_P8 = 0x9C,		//dig_P8, signed short
		BMP280_REG_DIG_P9 = 0x9E,		//dig_P9, signed short
	};

	enum bmp280_mask_t : uint8_t
	{
		BMP280_MASK_ID = 0b11111111,
		BMP280_MSAK_RESET = 0b11111111,

		//register 0xF3
		BMP280_MASK_STATUS_IM_UPDATE = 0b00000001,
		BMP280_MASK_STATUS_MEASURING = 0b00001000,

		//register 0xF4
		BMP280_MASK_MODE = 0b00000011,
		BMP280_MASK_OSRS_P = 0b00011100,
		BMP280_MASK_OSRS_T = 0b11100000,

		//register 0xF5
		//BMP280_MASK_SPI3W_EN = 0b00000001,		//SPI 3 Wire is not supported. 
		BMP280_MASK_FILTER = 0b00011100,
		BMP280_MASK_T_SB = 0b11100000,
	};

	static const uint8_t BMP280_RESET = 0xB6;

	/*
	@param mask
	@return number of bits to shift value so it fits into mask. */
	uint8_t getMaskShift(uint8_t mask);

	/*
	@param register value of register.
	@param mask mask of value in register
	@return value of masked bits. */
	uint8_t getMaskedBits(uint8_t reg, uint8_t mask);

	/*
	@param register value of register
	@param mask mask of value in register
	@param value value to write into masked area
	@param register value with masked bits set to value. */
	uint8_t setMaskedBits(uint8_t reg, uint8_t mask, uint8_t value);

	/*
	reads the masked value from the register.
	@param reg register to read.
	@param mask mask of value.
	@return masked value in register. */
	uint8_t readRegisterValue(uint8_t reg, uint8_t mask);

	/*
	sets values in a register.
	@param reg register to set values in
	@param mask bits of register to set value in
	@param value value to set */
	void writeRegisterValue(uint8_t reg, uint8_t mask, uint8_t value);

	/*
	reads a register from the sensor. must be overwritten by derived classes.
	@param reg register to read.
	@return register content*/
	virtual uint8_t readRegister(uint8_t reg) = 0;

	/*
	reads a series of registers from the sensor. must be overwritten by derived classes.
	@param reg register to read.
	@param length number of bytes to read
	@return register content*/
	virtual uint32_t readRegisters(uint8_t reg, uint8_t length) = 0;

	/*
	writes a register to the sensor. must be overwritten by derived classes.
	this function is also used to send direct commands.
	@param reg register to write to.
	@param value value writeRegister write to register. */
	virtual void writeRegister(uint8_t reg, uint8_t value) = 0;

	bool readRawValues();

	bool readCompensationParameters();

	uint32_t temp_fine_;

	uint32_t raw_pressure_;

	uint32_t raw_temp_;

	//compensation parameters
	struct BMP280CompParams {
		uint16_t dig_T1_;
		int16_t dig_T2_;
		int16_t dig_T3_;

		uint16_t dig_P1_;
		int16_t dig_P2_;
		int16_t dig_P3_;
		int16_t dig_P4_;
		int16_t dig_P5_;
		int16_t dig_P6_;
		int16_t dig_P7_;
		int16_t dig_P8_;
		int16_t dig_P9_;
	};
};

class BMP280I2C : public BMP280MI
{
public:
	BMP280I2C(uint8_t i2c_address);
	virtual ~BMP280I2C();

	virtual bool begin();

private:
	uint8_t readRegister(uint8_t reg);

	uint32_t readRegisters(uint8_t reg, uint8_t length);

	void writeRegister(uint8_t reg, uint8_t value);

	uint8_t i2c_address_;
};

class BMP280SPI : public BMP280MI
{
public:
	BMP280SPI(uint8_t chip_select);
	virtual ~BMP280SPI();

	virtual bool begin();

private:
	uint8_t readRegister(uint8_t reg);

	uint32_t readRegisters(uint8_t reg, uint8_t length);

	void writeRegister(uint8_t reg, uint8_t value);

	uint8_t chip_select_;
};

#endif /* BMP280MI_H_ */ 