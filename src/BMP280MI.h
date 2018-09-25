#ifndef BMP280MI_H_
#define BMP280MI_H_

class BMP280MI 
{
	public:
	
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
	
	BMP280MI();
	virtual ~BMP280MI ();

	virtual bool begin() = 0;
	
	bool readRawValues();
	
	uint8_t getOversamplingSetting();
	
	bool setOversamplingSetting(uint8_t setting);
	
	private:
		bool readCompensationParameters();
	
		uint32_t temp_fine_;
		
		uint32_t read_last_;
		
		uint32_t raw_pressure_;
		uint32_t raw_temp_;
		
		
		//compensation parameters
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

class BMP280I2C : public BMP280MI
{
	public:
	BMP280I2C(uint8_t i2c_address);
	virtual ~BMP280I2C();
	
	virtual bool begin();
	
	private:
		uint8_t i2c_address_;		
};

class BMP280SPI : public BMP280MI
{
	public:
	BMP280I2C(uint8_t chip_select);
	virtual ~BMP280I2C();
	
	virtual bool begin();
	
	private:
		uint8_t chip_select_;		
};

#endif /* BMP280MI_H_ */ 