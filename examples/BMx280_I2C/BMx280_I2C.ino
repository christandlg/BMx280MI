// AS3935MI_LightningDetector_I2C.ino
//
// shows how to use the AS3935MI library with the lightning sensor connected using I2C.
//
// Copyright (c) 2018 Gregor Christandl
//
// connect the AS3935 to the Arduino like this:
//
// Arduino - AS3935
// 5V ------ VCC
// GND ----- GND
// D2 ------ IRQ		must be a pin supporting external interrupts, e.g. D2 or D3 on an Arduino Uno.
// SDA ----- MOSI
// SCL ----- SCL
// 5V ------ SI		(activates I2C for the AS3935)
// 5V ------ A0		(sets the AS3935' I2C address to 0x01)
// GND ----- A1		(sets the AS3935' I2C address to 0x01)
// 5V ------ EN_VREG !IMPORTANT when using 5V Arduinos (Uno, Mega2560, ...)
// other pins can be left unconnected.

#include <Arduino.h>
#include <Wire.h>

#include <BMP280MI.h>

#define I2C_ADDRESS 0x76

//create an AS3935 object using the I2C interface, I2C address 0x01 and IRQ pin number 2
BMP280I2C bmp280(I2C_ADDRESS);

void setup() {
  // put your setup code here, to run once:
	Serial.begin(9600);

	//wait for serial connection to open (only necessary on some boards)
	while (!Serial);

	Wire.begin();

	//begin() checks the Interface and I2C Address passed to the constructor and resets the AS3935 to 
	//default values.
	if (!bmp280.begin())
	{
		Serial.println("begin() failed. check your BMP280 Interface and I2C Address.");
		while (1);
	}

	//...
}

void loop() {
  // put your main code here, to run repeatedly:

	delay(1000);

	//start a measurement
	if (!bmp280.measure())
	{
		Serial.println("could not start measurement, is a measurement already running?");
		return;
	}

	//wait for the measurement to finish
	do
	{
		delay(100);
	} while (!bmp280.hasValue());

	Serial.print("Pressure: "); Serial.println(bmp280.getPressure());
	Serial.print("Temperature: "); Serial.println(bmp280.getTemperature());
}