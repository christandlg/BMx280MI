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

BMP280MI::BMP280MI()
{
}

BMP280MI::~BMP280MI()
{
}

bool BMP280MI::begin()
{
}

//starts a measurement. 
//@return true on success, false otherwise. 
bool BMP280MI::measure()
{
	
}

//@return true if a measurement was completed, false otherwise. 
bool BMP280MI::hasValue()
{
	
}

//@return the last measured pressure, in Pa. 
float BMP280MI::getPressure()
{
	
}

//@return the last measured temperature, in deg C. 
float BMP280MI::getTemperature()
{
	
}

//triggers a measurement and returns the measured temperature. 
//@return temperature in deg C or NAN if the measurement failed. 
float BMP280MI::readTemperature()
{
	
}
	
//triggers a measurement and returns the measured pressure. 
//@return pressure in Pa or NAN if the measurement failed. 
float BMP280MI::readPressure()
{
	
}

//@return pressure oversampling setting
uint8_t BMP280MI::readOversamplingPressure()
{
	
}

//@param value to set
//@return true on success, false otherwise. 
bool BMP280MI::writeOversamplingPressure(uint8_t value)
{
	
}

//@return pressure oversampling setting
uint8_t BMP280MI::readOversamplingTemperature()
{
	
}

//@param value to set
//@return true on success, false otherwise. 
bool BMP280MI::writeOversamplingTemperature(uint8_t value)
{
	
}

//@return filter setting as filter_setting_t.
uint8_t BMP280MI::readFilterSetting()
{
	
}

//@param filter setting as filter_setting_t.
//@return true on success, false otherwise. 
bool BMP280MI::writeFilterSetting(uint8_t setting)
{
	
}

//@return sensors power mode as power_mode_t.
uint8_t BMP280MI::readPowerMode()
{
	
}

//sets the sensors power mode. 
//@param power mode as power_mode_t
//@return true on success, false otherwise. 
bool BMP280MI::writePowerMode(uint8_t mode)
{
	writeRegisterValue(BMP280_REG_CTRL_MEAS, BMP280_MASK_MODE)
}

//@return standby time as standby_time_t. 
uint8_t BMP280MI::readStandbyTime()
{
	readRegisterValue(BMP280_REG_CONFIG, BMP280_MASK_T_SB);
}

//sets the sensors standby time. only has an effect if measurements are done in 'normal' (automatic) mode. 
//@param standby time as standby_time_t. 
//@return true on success, false otherwise. 
bool BMP280MI::writeStandbyTime(uint8_t standby_time)
{
	if (standby_time > 0x07)
		return false;
	
	writeRegisterValue(BMP280_REG_CONFIG, BMP280_MASK_T_SB, standby_time);
	
	return true;
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
	
	//...
	
	return true;
}

uint8_t BMP280I2C::readRegister(uint8_t reg)
{
	
}

void BMP280I2C::writeRegister(uint8_t reg, uint8_t value)
{
	
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
	
}

uint8_t BMP280SPI::readRegister(uint8_t reg)
{
	
}

void BMP280SPI::writeRegister(uint8_t reg, uint8_t value)
{
	
}