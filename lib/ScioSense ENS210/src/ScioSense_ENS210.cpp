/*
  ScioSense_ENS210.h - Library for the ENS210 relative humidity and temperature sensor with I2C interface from ScioSense
  2020 Apr 06	v3	Christoph Friese	Changed nomenclature to ScioSense as product shifted from ams
  2018 Aug 28	v2	Christoph Friese	Adjusted I2C communication 
  2017 Aug 01	v1	Maarten Pennings	Created
*/

#include "ScioSense_ENS210.h"
#include <assert.h>

// Compute the CRC-7 of 'val' (should only have 17 bits)
// https://en.wikipedia.org/wiki/Cyclic_redundancy_check#Computation
static uint32_t crc7( uint32_t val ) 
{
	// Setup polynomial
	uint32_t pol= CRC7POLY;
	// Align polynomial with data
	pol = pol << (DATA7WIDTH-CRC7WIDTH-1);
	// Loop variable (indicates which bit to test, start with highest)
	uint32_t bit = DATA7MSB;
	// Make room for CRC value
	val = val << CRC7WIDTH;
	bit = bit << CRC7WIDTH;
	pol = pol << CRC7WIDTH;
	// Insert initial vector
	val |= CRC7IVEC;
	// Apply division until all bits done
	while( bit & (DATA7MASK<<CRC7WIDTH) ) {
		if( bit & val ) val ^= pol;
		bit >>= 1;
		pol >>= 1;
	}
	return val;
}

// Init I2C communication, resets ENS210 and checks its PART_ID. Returns false on I2C problems or wrong PART_ID.
// Stores solder correction.
bool ScioSense_ENS210::begin(bool debug) 
{
	bool result;

	debugENS210 = debug;
	
	//init I2C
	_i2c_init();
	if (debugENS210) {
		Serial.println("ens210 debug - I2C init done");
	}
  
	// Record solder correction
	this->_soldercorrection= 0;
	this->_available = false;
	this->_singleMode = true;
	this->_t_data = -1;
	this->_h_data = -1;	
	this->lowpower(false);
	
	result = getversion();
	
	if(this->_partID == ENS210_PARTID ) {
		this->_available = true;
	}
	return result;
}

// Init I2C communication, resets ENS210 and checks its PART_ID. Returns false on I2C problems or wrong PART_ID.
// Stores solder correction.
bool ScioSense_ENS210::begin(TwoWire *wire, bool userWire, bool debug) 
{
	bool result;

	debugENS210 = debug;
	
	//init I2C
	if (!userWire) {
		_i2c_init();
	} else {
		_wire=wire;
	}
	if (debugENS210) {
		Serial.println("ens210 debug - I2C init done");
	}
  
	// Record solder correction
	this->_soldercorrection= 0;
	this->_available = false;
	this->_singleMode = true;
	this->_t_data = -1;
	this->_h_data = -1;	
	this->lowpower(false);
	
	result = getversion();
	
	if(this->_partID == ENS210_PARTID ) {
		this->_available = true;
	}
	return result;
}

// Sends a reset to the ENS210. Returns false on I2C problems.
bool ScioSense_ENS210::reset(void) 
{
	uint8_t result = this->write8(_slaveaddress, ENS210_REG_SYS_CTRL, 0x80);
 	if (debugENS210) {
		Serial.print("ens210 debug - reset: 0x");
		Serial.println(result,HEX);
	}
	delay(ENS210_BOOTING);                   // Wait to boot after reset
	return result==0;
}


// Sets ENS210 to low (true) or high (false) power. Returns false on I2C problems.
bool ScioSense_ENS210::lowpower(bool enable) 
{
	uint8_t power = enable ? 0x01: 0x00;
	uint8_t result = this->write8(this->_slaveaddress, ENS210_REG_SYS_CTRL, power);
	if (debugENS210) {
		Serial.print("ens210 debug - lowpower: 0x");
		Serial.println(result,HEX);
	}
 	delay(ENS210_BOOTING);                   // Wait boot-time after power switch
	return result==0;
}


// Reads PART_ID and UID of ENS210. Returns false on I2C problems.
bool ScioSense_ENS210::getversion()//(uint16_t partid, uint64_t uid) 
{
	bool ok;
	uint8_t i2cbuf[8];
	uint8_t result;

	// Must disable low power to read PART_ID or UID
	ok= lowpower(false); if(!ok) goto errorexit;

	// Read the PART_ID
	result = this->read(this->_slaveaddress, ENS210_REG_PART_ID, i2cbuf, 2);
	if (debugENS210) {
		Serial.print("ens210 debug - PART_ID I2C result: 0x");
		Serial.println(result, HEX);
	}
	//this->_partID = (uint16_t)(i2cbuf[1]*256U + i2cbuf[0]*1U);
	this->_partID = (uint16_t)(((uint16_t)i2cbuf[1] << 8) | ((uint16_t)i2cbuf[0]));
	if (debugENS210) {
		Serial.print("PART_ID: 0x");
		Serial.println(this->_partID,HEX);
	}

	// Read the UID
	result = this->read(_slaveaddress, ENS210_REG_UID,i2cbuf,8); 
	if (debugENS210) {
		Serial.print("ens210 debug - UID 0x");
		Serial.println(result,HEX);
	}
	this->_uID = (uint64_t)((uint64_t)i2cbuf[7]<<56 | (uint64_t)i2cbuf[6]<<48 | (uint64_t)i2cbuf[5]<<40 | (uint64_t)i2cbuf[4]<<32 | (uint64_t)i2cbuf[3]<<24 | (uint64_t)i2cbuf[2]<<16 | (uint64_t)i2cbuf[1]<<8 | (uint64_t)i2cbuf[0]);
	
	//for( int i=0; i<8; i++) ((uint8_t*)this->_uID)[i]=i2cbuf[i]; //((uint8_t*)this->_uID)[i]=i2cbuf[i];
	this->_uIDhi = (uint32_t)(this->_uID >> 32);
	this->_uIDlo = (uint32_t)(this->_uID & 0xFFFFFFFF);
	
	if (debugENS210) {
		Serial.print("UID hi 0x");
		Serial.print(this->_uIDhi,HEX);
		Serial.print(" - 0x");
		Serial.println(this->_uIDlo,HEX);
	}
	
	// Go back to default power mode (low power enabled)
	ok= lowpower(true); if(!ok) goto errorexit;

	// Success
	return true;

	errorexit:
	// Try to go back to default mode (low power enabled)
	ok= lowpower(true);

	// Hopefully enabling low power was successful; but there was an error before that anyhow
	return false;
}

// Configures ENS210 measurement mode
// false for continuous mode / true for  single shot measurement. Returns false on I2C problems.
bool ScioSense_ENS210::setSingleMode(bool enable)
{
	this->_singleMode = enable;
	uint8_t mode = enable ? 0x00: 0x03;
	uint8_t result = this->write8(_slaveaddress, ENS210_REG_SENS_RUN, mode);
	if (debugENS210) {
		Serial.print("ens210 debug - start mode 0x");
		Serial.print(mode,HEX);
		Serial.print(" - result 0x");
		Serial.println(result,HEX);
	}
	return result==0;
}

// Performs one single shot temperature and relative humidity measurement.
void ScioSense_ENS210::measure() //int * t_data, int * t_status, int * h_data, int * h_status ) 
{
	bool ok;
	// Set default status for early bail out
	if (debugENS210) Serial.println("Start measurement");
	this->_t_status = ENS210_STATUS_I2CERROR;
	this->_h_status = ENS210_STATUS_I2CERROR;

	// Start a single shot measurement
	if (this->_singleMode) 
	{
		uint8_t result = this->write8(_slaveaddress, ENS210_REG_SENS_RUN, 0x00);
		if (debugENS210) {
			Serial.print("ens210 debug - start single 0x");
			Serial.println(result,HEX);
		}
	}
	
	//Trigger measurement
	uint8_t result = this->write8(_slaveaddress, ENS210_REG_SENS_START, 0x03);
	if (debugENS210) {
		Serial.print("ens210 debug - trigger measurement 0x");
		Serial.println(result,HEX);
	}
	
	// Wait for measurement to complete
	if (this->_singleMode) delay(ENS210_THCONV_SINGLE_MS);
	else delay(ENS210_THCONV_CONT_MS);
	
	// Get the measurement data
	if (debugENS210) Serial.println("Start measurement");
	ok = readValue(); if(!ok) return;
	if (debugENS210) Serial.println("Measurement ok");
}

// Reads measurement data from the ENS210. Returns false on I2C problems.
bool ScioSense_ENS210::readValue()	//uint32_t *t_val, uint32_t *h_val) 
{
	uint8_t i2cbuf[6];
	uint8_t valid;
	uint32_t crc;
	uint32_t payload;
	uint8_t crc_ok;
	
	// Read T_VAL and H_VAL
	uint8_t result = this->read(_slaveaddress, ENS210_REG_T_VAL, i2cbuf, 6);
	if (debugENS210) {
		Serial.print("ens210 debug - readValue:");
		Serial.println(result);
		Serial.print(i2cbuf[0],HEX); Serial.print("\t");
		Serial.print(i2cbuf[1],HEX); Serial.print("\t");
		Serial.print(i2cbuf[2],HEX); Serial.print("\t");
		Serial.print(i2cbuf[3],HEX); Serial.print("\t");
		Serial.print(i2cbuf[4],HEX); Serial.print("\t");
		Serial.print(i2cbuf[5],HEX); Serial.println("\t");
	}
	// Retrieve and pack bytes into t_val and h_val
	uint32_t t_val= (uint32_t)((uint32_t)i2cbuf[2]<<16 | (uint32_t)i2cbuf[1]<<8 | (uint32_t)i2cbuf[0]);
	this->_t_data = (t_val>>0) & 0xffff;
	valid   = (t_val>>16) & 0x1;
	crc     = (t_val>>17) & 0x7f;
	payload = (t_val>>0 ) & 0x1ffff;
	crc_ok = crc7(payload)==crc;
	if( !crc_ok ) this->_t_status = ENS210_STATUS_CRCERROR;
	else if( !valid ) this->_t_status = ENS210_STATUS_INVALID;
	else this->_t_status = ENS210_STATUS_OK;
	if (debugENS210) {
		Serial.print("_t_data 0x");
		Serial.print(t_val,HEX);
		Serial.print(" - 0x");
		Serial.println(this->_t_data,HEX);
		Serial.print("Valid: ");
		Serial.print(valid);
		Serial.print(" Status: ");
		Serial.println(this->_t_status);
	}	
	
	uint32_t h_val= (uint32_t)((uint32_t)i2cbuf[5]<<16 | (uint32_t)i2cbuf[4]<<8 | (uint32_t)i2cbuf[3]);
	this->_h_data = (h_val>>0) & 0xffff;

	valid   = (h_val>>16) & 0x1;
	crc     = (h_val>>17) & 0x7f;
	payload = (h_val>>0 ) & 0x1ffff;
	crc_ok= crc7(payload)==crc;
	if( !crc_ok ) this->_h_status= ENS210_STATUS_CRCERROR;
	else if( !valid ) this->_h_status= ENS210_STATUS_INVALID;
	else this->_h_status= ENS210_STATUS_OK;
	if (debugENS210) {
		Serial.print("_h_data 0x");
		Serial.print(h_val,HEX);
		Serial.print(" - 0x");
		Serial.println(this->_h_data,HEX);
		Serial.print("Valid: ");
		Serial.print(valid);
		Serial.print(" Status: ");
		Serial.println(this->_h_status);
	}	
	
	// Success
	return true;
}


// Converts a status (ENS210_STATUS_XXX) to a human readable string.
const char * ScioSense_ENS210::status_str( int status ) 
{
	switch( status ) {
		case ENS210_STATUS_I2CERROR : return "i2c-error";
		case ENS210_STATUS_CRCERROR : return "crc-error";
		case ENS210_STATUS_INVALID  : return "data-invalid";
		case ENS210_STATUS_OK       : return "ok";
		default                     : return "unknown-status";
	}
}


// Convert raw `t_data` temperature to Kelvin (also applies the solder correction).
// The output value is in Kelvin multiplied by parameter `multiplier`.
float ScioSense_ENS210::getTempKelvin() 
{
	// Force 32 bits
	float t = this->_t_data & 0xFFFF;
	// Compensate for soldering effect
	t-= _soldercorrection;
	// Return m*K. This equals m*(t/64) = (m*t)/64
	// Note m is the multiplier, K is temperature in Kelvin, t is raw t_data value.
	// Uses K=t/64.
	return t/64; //IDIV(t,64);
}

// Convert raw `t_data` temperature to Celsius (also applies the solder correction).
// The output value is in Celsius multiplied by parameter `multiplier`.
float ScioSense_ENS210::getTempCelsius() 
{
	//assert( (1<=multiplier) && (multiplier<=1024) );
	// Force 32 bits
	float t= this->_t_data & 0xFFFF;
	// Compensate for soldering effect
	t-= _soldercorrection;
	// Return m*C. This equals m*(K-273.15) = m*K - 27315*m/100 = m*t/64 - 27315*m/100
	// Note m is the multiplier, C is temperature in Celsius, K is temperature in Kelvin, t is raw t_data value.
	// Uses C=K-273.15 and K=t/64.
	return t/64 - 27315L/100; //IDIV(t,64) - IDIV(27315L,100);
}


// Convert raw `t_data` temperature to Fahrenheit (also applies the solder correction).
float ScioSense_ENS210::getTempFahrenheit() 
{
	float t= this->_t_data & 0xFFFF;
	// Compensate for soldering effect
	t-= _soldercorrection;
	// Return m*F. This equals m*(1.8*(K-273.15)+32) = m*(1.8*K-273.15*1.8+32) = 1.8*m*K-459.67*m = 9*m*K/5 - 45967*m/100 = 9*m*t/320 - 45967*m/100
	// Note m is the multiplier, F is temperature in Fahrenheit, K is temperature in Kelvin, t is raw t_data value.
	// Uses F=1.8*(K-273.15)+32 and K=t/64.
	return 9*t/320 - 45967/100; //IDIV(9*t,320) - IDIV(45967L,100);
	// The first multiplication stays below 32 bits (t:16, multiplier:11, 9:4)
	// The second multiplication stays below 32 bits (multiplier:10, 45967:16)
}


// Convert raw `h_data` relative humidity to %RH.
float ScioSense_ENS210::getHumidityPercent() 
{
	float h= this->_h_data & 0xFFFF;
	// Return m*H. This equals m*(h/512) = (m*h)/512
	// Note m is the multiplier, H is the relative humidity in %RH, h is raw h_data value.
	// Uses H=h/512.
	return h/512; //IDIV(h, 512);
}


// Convert raw `h_data` absolute humidity to %RH.
#define MOLAR_MASS_OF_WATER     18.01534
#define UNIVERSAL_GAS_CONSTANT  8.21447215

float ScioSense_ENS210::getAbsoluteHumidityPercent() 
{
	//taken from https://carnotcycle.wordpress.com/2012/08/04/how-to-convert-relative-humidity-to-absolute-humidity/
	//precision is about 0.1°C in range -30 to 35°C
	//August-Roche-Magnus   6.1094 exp(17.625 x T)/(T + 243.04)
	//Buck (1981)     6.1121 exp(17.502 x T)/(T + 240.97)
	//reference https://www.eas.ualberta.ca/jdwilson/EAS372_13/Vomel_CIRES_satvpformulae.html    // Use Buck (1981)

	return (6.1121 * pow(2.718281828,(17.67* this->getTempCelsius())/(this->getTempCelsius() + 243.5))* this->getHumidityPercent() *MOLAR_MASS_OF_WATER)/((273.15+ this->getTempCelsius() )*UNIVERSAL_GAS_CONSTANT);
}


// Sets the solder correction (default is 50mK) - only used by the `toXxx` functions.
void ScioSense_ENS210::correction_set(int correction) 
{
	assert( -1*64<correction && correction<+1*64 ); // A correction of more than 1 Kelvin does not make sense (but the 1K is arbitrary)
	this->_soldercorrection = correction;
}


/****************************************************************************/
/*	General functions														*/
/****************************************************************************/

void ScioSense_ENS210::_i2c_init()
{
	Wire.begin();
	_wire = &Wire;
}
/**************************************************************************/

uint8_t ScioSense_ENS210::read8(uint8_t addr, byte reg)
{
	uint8_t ret;
	this->read(addr, reg, &ret, 1);
	
	return ret;
}

uint8_t ScioSense_ENS210::read(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t num)
{
	uint8_t pos = 0;
	uint8_t result = 0;
	
	if (debugENS210) {
		Serial.print("I2C read address: 0x");
		Serial.print(addr, HEX);
		Serial.print(" - register: 0x");
		Serial.println(reg, HEX);
	}
	
	//on arduino we need to read in 32 byte chunks
	while(pos < num){
		
		uint8_t read_now = 32;//min((uint8_t)32, (uint8_t)(num - pos));
		_wire->beginTransmission((uint8_t)addr);
		
		_wire->write((uint8_t)reg + pos);
		result = _wire->endTransmission();
		_wire->requestFrom((uint8_t)addr, read_now);
		
		//for(int i=0; i<read_now; i++){
		for(int i=0; i<num; i++){
			buf[pos] = _wire->read();
			if (debugENS210) {
				Serial.print("Pos: ");
				Serial.print(pos);
				Serial.print(" - Value: 0x");
				Serial.println(buf[pos], HEX);
			}
			pos++;
		}
	}
	return result;
}

/**************************************************************************/

uint8_t ScioSense_ENS210::write8(uint8_t addr, byte reg, byte value)
{
	uint8_t result = this->write(addr, reg, &value, 1);
	return result;
}

uint8_t ScioSense_ENS210::write(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t num)
{
	if (debugENS210) {
		Serial.print("I2C write address: 0x");
		Serial.print(addr, HEX);
		Serial.print(" - register: 0x");
		Serial.print(reg, HEX);
		Serial.print(" -  value: 0x");
		for (int i = 0; i<num; i++) {
			Serial.print(" 0x");
			Serial.print(buf[i], HEX);
		}
		Serial.println();
	}
	
	_wire->beginTransmission((uint8_t)addr);
	_wire->write((uint8_t)reg);
	_wire->write((uint8_t *)buf, num);
	uint8_t result = _wire->endTransmission();
	return result;
}

