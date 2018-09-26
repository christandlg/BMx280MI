// AS3935_LightningDetector_SPI.ino
//
// shows how to use the AS3935 library with the lightning sensor connected using SPI.
//
// Copyright (c) 2018 Gregor Christandl
//
// connect the AS3935 to the Arduino like this:
//
// Arduino - AS3935
// 5V ------ VCC
// GND ----- GND
// D2 ------ IRQ		must be a pin supporting external interrupts, e.g. D2 or D3 on an Arduino Uno.
// MOSI ---- MOSI
// MISO ---- MISO
// SCK ----- SCK
// GND ----- SI		(activates SPI for the AS3935)
// D3 ------ CS		chip select pin for AS3935
// 5V ------ EN_VREG !IMPORTANT when using 5V Arduinos (Uno, Mega2560, ...)
// other pins can be left unconnected.

#include <Arduino.h>
#include <SPI.h>

#include <BMP280MI.h>

#define PIN_CS 4

//create an AS3935 object using the I2C interface, I2C address 0x01 and IRQ pin number 2
BMP280SPI bmp280(PIN_CS);

void setup() {
	// put your setup code here, to run once:
	Serial.begin(9600);

	//wait for serial connection to open (only necessary on some boards)
	while (!Serial);

	SPI.begin();

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