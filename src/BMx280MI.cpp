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


#include "BMP280MI.h"

SPISettings BMP280MISPI::spi_settings_ = SPISettings(2000000, MSBFIRST, SPI_MODE1);

BMP280MI::BMP280MI()
{
	//nothing to do here...
}

BMP280MI::~BMP280MI()
{
	//nothing to do here...
}

bool BMP280MI::measure()
{
	//return false if a measurement is already running. 
	if (readRegisterValue(BMP280_REG_STATUS, BMP280_MASK_STATUS_MEASURING))
		return false;

	//start a forced measurement. 
	writeRegisterValue(BMP280_REG_CTRL_MEAS, BMP280_MASK_MODE, BMP280_MODE_FORCED);

	return true;
}

bool BMP280MI::hasValue()
{
	return !static_cast<bool>(readRegisterValue(BMP280_REG_STATUS, BMP280_MASK_STATUS_MEASURING));
}

float BMP280MI::getPressure()
{
	//TODO implement pressure calculation 
	return NAN;
}

float BMP280MI::getTemperature()
{
	//TODO implement temperature calculation 
	return NAN;
}

float BMP280MI::readTemperature()
{
	if (!measure())
		return NAN;

	do
	{
		delay(100);
	} while (!hasValue());

	return getTemperature();
}

float BMP280MI::readPressure()
{
	if (!measure())
		return NAN;

	do
	{
		delay(100);
	} while (!hasValue());

	return getPressure();
}

uint8_t BMP280MI::readID()
{
	return readRegisterValue(BMP280_REG_ID, BMP280_MASK_ID);
}

void BMP280MI::resetToDefaults()
{
	writeRegisterValue(BMP280_REG_RESET, BMP280_MSAK_RESET, BMP280_RESET);
}

uint8_t BMP280MI::readOversamplingPressure()
{
	return readRegisterValue(BMP280_REG_CTRL_MEAS, BMP280_MASK_OSRS_P);
}

bool BMP280MI::writeOversamplingPressure(uint8_t value)
{
	if (value > 0b111)
		return false;

	writeRegisterValue(BMP280_REG_CTRL_MEAS, BMP280_MASK_OSRS_P, value);

	return true;
}

uint8_t BMP280MI::readOversamplingTemperature()
{
	return readRegisterValue(BMP280_REG_CTRL_MEAS, BMP280_MASK_OSRS_T);
}

bool BMP280MI::writeOversamplingTemperature(uint8_t value)
{
	if (value > 0b111)
		return false;

	writeRegisterValue(BMP280_REG_CTRL_MEAS, BMP280_MASK_OSRS_T, value);

	return true;
}

uint8_t BMP280MI::readFilterSetting()
{
	//TODO implement
}

bool BMP280MI::writeFilterSetting(uint8_t setting)
{
	//TODO implement
}

uint8_t BMP280MI::readPowerMode()
{
	return readRegisterValue(BMP280_REG_CTRL_MEAS, BMP280_MASK_MODE);
}

bool BMP280MI::writePowerMode(uint8_t mode)
{
	if (mode > 0x03)
		return false;

	writeRegisterValue(BMP280_REG_CTRL_MEAS, BMP280_MASK_MODE, mode);

	return true;
}

uint8_t BMP280MI::readStandbyTime()
{
	return readRegisterValue(BMP280_REG_CONFIG, BMP280_MASK_T_SB);
}

bool BMP280MI::writeStandbyTime(uint8_t standby_time)
{
	if (standby_time > 0x07)
		return false;

	writeRegisterValue(BMP280_REG_CONFIG, BMP280_MASK_T_SB, standby_time);

	return true;
}

uint8_t BMP280MI::getMaskShift(uint8_t mask)
{
	uint8_t return_value = 0;

	//count how many times the mask must be shifted right until the lowest bit is set
	if (mask != 0)
	{
		while (!(mask & 1))
		{
			return_value++;
			mask >>= 1;
		}
	}

	return return_value;
}

uint8_t BMP280MI::getMaskedBits(uint8_t reg, uint8_t mask)
{
	//extract masked bits
	return ((reg & mask) >> getMaskShift(mask));
}

uint8_t BMP280MI::setMaskedBits(uint8_t reg, uint8_t mask, uint8_t value)
{
	//clear mask bits in register
	reg &= (~mask);

	//set masked bits in register according to value
	return ((value << getMaskShift(mask)) & mask) | reg;
}

uint8_t BMP280MI::readRegisterValue(uint8_t reg, uint8_t mask)
{
	return getMaskedBits(readRegister(reg), mask);
}

void BMP280MI::writeRegisterValue(uint8_t reg, uint8_t mask, uint8_t value)
{
	uint8_t reg_val = readRegister(reg);
	writeRegister(reg, setMaskedBits(reg_val, mask, value));
}

bool BMP280MI::readRawValues()
{
	return false;
}

bool BMP280MI::readCompensationParameters()
{
	return false;
}


//-----------------------------------------------------------------------
//BMP280I2C
BMP280I2C::BMP280I2C(uint8_t i2c_address) :
	i2c_address_(i2c_address)
{
	//nothing to do here...
}

BMP280I2C::~BMP280I2C()
{
	//nothing to do here...
}

bool BMP280I2C::begin()
{
	if (readID() != BMP280_ID)
		return false;

	resetToDefaults();

	return true;
}

uint8_t BMP280I2C::readRegister(uint8_t reg)
{
	#if defined(ARDUINO_SAM_DUE)
		//workaround for Arduino Due. The Due seems not to send a repeated start with the code above, so this 
		//undocumented feature of Wire::requestFrom() is used. can be used on other Arduinos too (tested on Mega2560)
		//see this thread for more info: https://forum.arduino.cc/index.php?topic=385377.0
		Wire.requestFrom(address_, 1, reg, 1, true);
	#else
		Wire.beginTransmission(i2c_address_);
		Wire.write(reg);
		Wire.endTransmission(false);
		Wire.requestFrom(address_, static_cast<uint8_t>(1));
	#endif
	
	return Wire.read();
}

uint32_t BMP280I2C::readRegisterBurst(uint8_t reg, uint8_t length)
{
	return uint32_t();
}

void BMP280I2C::writeRegister(uint8_t reg, uint8_t value)
{
	Wire.beginTransmission(i2c_address_);
	Wire.write(reg);
	Wire.write(value);
	Wire.endTransmission();
}

//-----------------------------------------------------------------------
//BMP280SPI
BMP280SPI::BMP280SPI(uint8_t chip_select) :
	chip_select_(chip_select)
{
	//nothing to do here...
}

BMP280SPI::~BMP280SPI()
{
	//nothing to do here...
}

bool BMP280SPI::begin()
{
	//TODO implement
}

uint8_t BMP280SPI::readRegister(uint8_t reg)
{
	uint8_t return_value = 0;
	
	SPI.beginTransaction(spi_settings_);

	digitalWrite(cs_, LOW);				//select sensor

	SPI.transfer((reg & 0x3F) | 0x40);	//select register and set pin 7 (indicates read)
	
	return_value = SPI.transfer(0);

	digitalWrite(cs_, HIGH);			//deselect sensor
	
	return return_value;
}

uint32_t BMP280SPI::readRegisterBurst(uint8_t reg, uint8_t length)
{
	return uint32_t();
}

void BMP280SPI::writeRegister(uint8_t reg, uint8_t value)
{
	SPI.beginTransaction(spi_settings_);

	digitalWrite(cs_, LOW);				//select sensor

	SPI.transfer((reg & 0x3F));			//select regsiter 
	SPI.transfer(value);

	digitalWrite(cs_, HIGH);			//deselect sensor
}