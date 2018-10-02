// BMx280_LightningDetector_SPI.ino
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
// D2 ------ IRQ		must be a pin supporting external interrupts, e.g. D2 or D3 on an Arduino Uno.
// SDA1 ---- MOSI
// SCL1 ---- SCL
// 5V ------ SI		(activates I2C for the BMx280)
// 5V ------ A0		(sets the BMx280' I2C address to 0x01)
// GND ----- A1		(sets the BMx280' I2C address to 0x01)
// other pins can be left unconnected.

#include <Arduino.h>

#include <Wire.h>

#include <BMx280MI.h>

//class derived from BMx280MI that implements communication via an interface other than native I2C or SPI. 
class BMx280Wire1 : public BMx280MI
{
	public:	
		//constructor of the derived class. 
		//@param address i2c address of the sensor.
		BMx280Wire1(uint8_t i2c_address) 
		i2c_address_(i2c_address)	//initialize the BMx280Wire1 classes private member i2c_address_ to the i2c address provided
		{
			//nothing else to do here...
		}
		
		//this function must be implemented by derived classes. it is used to initialize the interface or check the sensor for example. 
		//@return true on success, false otherwise. 
		bool begin()
		{
			if (readID() != BMx280_ID)
				return false;

			resetToDefaults();

			return true;
		}
	
	private:
		//this function must be implemented by derived classes. this function is responsible for reading data from the sensor. 
		//@param reg register to read. 
		//@return read data (1 byte).
		uint8_t readRegister(uint8_t reg)
		{
		#if defined(ARDUINO_SAM_DUE)
			//workaround for Arduino Due. The Due seems not to send a repeated start with the code above, so this 
			//undocumented feature of Wire::requestFrom() is used. can be used on other Arduinos too (tested on Mega2560)
			//see this thread for more info: https://forum.arduino.cc/index.php?topic=385377.0
			Wire1.requestFrom(i2c_address_, 1, reg, 1, true);
		#else
			Wire1.beginTransmission(i2c_address_);
			Wire1.write(reg);
			Wire1.endTransmission(false);
			Wire1.requestFrom(i2c_address_, static_cast<uint8_t>(1));
		#endif
			
			return Wire1.read();
		}

		//this function must be implemented by derived classes. this function is responsible for sending data to the sensor. 
		//@param reg register to write to.
		//@param data data to write to register.
		void writeRegister(uint8_t reg, uint8_t data)
		{
			Wire1.beginTransmission(i2c_address_);
			Wire1.write(reg);
			Wire1.write(data);
			Wire1.endTransmission();
		}
		
		uint8_t i2c_address_;		//i2c address of sensor
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

    if (bmx280.isBME280())
      bmx280.writeOversamplingHumidity(BMx280MI::OSRS_H_x16);

    bmx280.writeOversamplingPressure(BMx280MI::OSRS_P_x16);
    bmx280.writeOversamplingTemperature(BMx280MI::OSRS_T_x16);

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
