// BMx280_otherInterfaces.ino
//
// shows how to use the BMx280 library interfaces other than the native I2C or SPI interfaces. 
// here, the second I2C port of an Arduino Due is used (Wire1)
//
// Copyright (c) 2018 Gregor Christandl
//
// connect the BMx280 to the Arduino Due like this:
//
// Arduino - BMx280
// 3.3V ---- VCC
// GND ----- GND
// SDA1 ---- SDA/SDI
// SCL1 ---- SKC/SCL
// 3.3V ---- CSB		(activates I2C for the BMx280)
// GND ----- SDO		(sets I2C address to 0x76. connecting 3.3V to SDO instead sets the I2C address to 0x77)
// other pins can be left unconnected.

#include <Arduino.h>

#include <Wire.h>

#include <BMx280MI.h>

#define I2C_ADDRESS 0x76

//class derived from BMx280MI that implements communication via an interface other than native I2C or SPI. 
class BMx280Wire1 : public BMx280MI
{
	public:	
		//constructor of the derived class. 
		//@param address i2c address of the sensor.
		BMx280Wire1(uint8_t i2c_address):
		address_(i2c_address)	//initialize the BMx280Wire1 classes private member address_ to the i2c address provided
		{
			//nothing else to do here...
		}
	
	private:
		//this function must be implemented by derived classes. it is used to initialize the interface or check the sensor for example. 
		//@return true on success, false otherwise. 
		bool beginInterface()
		{
			return true;
		}

		//this function must be implemented by derived classes. this function is responsible for reading data from the sensor. 
		//@param reg register to read. 
		//@return read data (1 byte).
		uint8_t readRegister(uint8_t reg)
		{
		#if defined(ARDUINO_SAM_DUE)
			//workaround for Arduino Due. The Due seems not to send a repeated start with the code above, so this 
			//undocumented feature of Wire::requestFrom() is used. can be used on other Arduinos too (tested on Mega2560)
			//see this thread for more info: https://forum.arduino.cc/index.php?topic=385377.0
			Wire1.requestFrom(address_, 1, reg, 1, true);
		#else
			Wire1.beginTransmission(address_);
			Wire1.write(reg);
			Wire1.endTransmission(false);
			Wire1.requestFrom(address_, static_cast<uint8_t>(1));
		#endif
			
			return Wire1.read();
		}
		
		//this function can be implemented by derived classes. implementing this function is optional, but readings may be incorrect if 
		//it is not (see BMP280 / BME280 datasheet). 
		//@param reg register to read. 
		//@param length number of registers to read (max: 4)
		//@return read data. LSB = last register read. 
		uint32_t readRegisterBurst(uint8_t reg, uint8_t length)
		{
			if (length > 4)
				return 0L;

			uint32_t data = 0L;

#if defined(ARDUINO_SAM_DUE)
			//workaround for Arduino Due. The Due seems not to send a repeated start with the code below, so this 
			//undocumented feature of Wire::requestFrom() is used. can be used on other Arduinos too (tested on Mega2560)
			//see this thread for more info: https://forum.arduino.cc/index.php?topic=385377.0
			Wire1.requestFrom(address_, length, data, length, true);
#else
			Wire1.beginTransmission(address_);
			Wire1.write(reg);
			Wire1.endTransmission(false);
			Wire1.requestFrom(address_, static_cast<uint8_t>(length));

			for (uint8_t i = 0; i < length; i++)
			{
				data <<= 8;
				data |= Wire1.read();
			}
#endif

			return data;
		}

		//this function must be implemented by derived classes. this function is responsible for sending data to the sensor. 
		//@param reg register to write to.
		//@param data data to write to register.
		void writeRegister(uint8_t reg, uint8_t data)
		{
			Wire1.beginTransmission(address_);
			Wire1.write(reg);
			Wire1.write(data);
			Wire1.endTransmission();
		}
		
		uint8_t address_;		//i2c address of sensor
};

//create an BMx280 object using the I2C interface, I2C address 0x01 and IRQ pin number 2
BMx280Wire1 bmx280(I2C_ADDRESS);

void setup() {
	// put your setup code here, to run once:
	Serial.begin(9600);

	//wait for serial connection to open (only necessary on some boards)
	while (!Serial);

	Wire1.begin();

	//begin() checks the Interface and I2C Address passed to the constructor and resets the BMx280 to 
	//default values.
	if (!bmx280.begin())
	{
		Serial.println("begin() failed. check your BMx280 Interface and I2C Address.");
		while (1);
	}


        //reset sensor to default parameters.
        bmx280.resetToDefaults();

        //if sensor is a BME280, set highest oversampling setting for humidity measurements.
        if (sensor_.isBME280())
                sensor_.writeOversamplingHumidity(BMx280MI::OSRS_H_x16);

        //set highest oversampling setting for pressure and temperature measurements.
        sensor_.writeOversamplingPressure(BMx280MI::OSRS_P_x16);
        sensor_.writeOversamplingTemperature(BMx280MI::OSRS_T_x16);

	//...
}

void loop() {
  // put your main code here, to run repeatedly:

	delay(1000);

	//start a measurement
	if (!bmx280.measure())
	{
		Serial.println("could not start measurement, is a measurement already running?");
		return;
	}

	//wait for the measurement to finish
	do
	{
		delay(100);
	} while (!bmx280.hasValue());

	Serial.print("Pressure: "); Serial.println(bmx280.getPressure());
	Serial.print("Temperature: "); Serial.println(bmx280.getTemperature());
}
