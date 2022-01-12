#pragma once
#include <Arduino.h>
#include <SckBase.h>
#include <Sensors.h>

// Include the necessary libraries for the auxiliary sensor to be used
#include <Wire.h>
#include <FlashStorage.h>

// INA219 libs
#include <Adafruit_INA219.h>

// Gases Board libs
#include <GasesBoard.h>

// Urban board library
#include <SckUrban.h>

// Groove_OLED libs
#include <U8g2lib.h>
// Icons for screen
#include <Icons.h>

// DS2482 library (I2C-1Wire bridge)
#include <DS2482.h>

// I2C Moisture Sensor (chirp)
#include <I2CSoilMoistureSensor.h>

// Libraries for DallasTemp
#include <OneWire.h>
#include <DallasTemperature.h>

// Sparkfun VL6180x time of flight range finder
#include <SparkFun_VL6180X.h>

// Adafruit BME608 library
#include <Adafruit_BME680.h>

// Library for GPS data parsing
#include "TinyGPS++.h"

// Librtary for XA1110 Sparkfun i2c GPS
#include <SparkFun_I2C_GPS_Arduino_Library.h>

// Library for SparkFun u-Blox NEO-M8U GPS
#ifdef ID
#undef ID 	// Fix conflict with define on SPIMemory.h
#endif
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>

// Adafruit library for ADS1x15 12/16 bits ADC
#include <Adafruit_ADS1015.h>

#include "TCA9548A.h"

// ADS Tester (Only supported in version 2.0 and greater)
// This function is only for internal purposes
// #define adsTest 	// Uncomment for enabling tester board support (remember selecting board lookup table in GasesBoardTester.h)

#ifdef adsTest
#include "GasesBoardTester.h"
#endif

// Sparkfun library for SCD30 CO2 sensor
#include <SparkFun_SCD30_Arduino_Library.h>

// Sparkfun library for SCD4x (SCD41) CO2 sensor
//#include <SparkFun_SCD4x_Arduino_Library.h>

// Seeed Studios TCA9549A 8 channel I2C Mux (Switch)
// deployed in SCS V3 when needed to help balance the I2C bus when one device blocks others from using the bus.
// its not deployed to deal with duplicate I2C addresses (a more normal way to use such a Mux)
#include "TCA9548A.h"

extern TwoWire auxWire;


class SckBase;


struct Calibration {
	bool  moistureCalDataValid = false;
	int32_t dryPoint;
	int32_t wetPoint;
};

struct EepromAuxData {
	bool valid = false;
	Calibration calibration;
};

bool I2Cdetect(byte address);

typedef struct I2C_MuxChannel {						// struct to hold addr/chan/type/xclusive infor connectd to a channel
	byte addr;
	byte chan;
	SensorType type;
	bool exclusive;
} muxChannel;

class AuxBoards
{

	public:

		// List for storing Auxiliary sensors I2C address (SENSOR_COUNT - (BASE AND URBAN SENSORS))
		// TODO: store this in epprom, load it on boot, make a function to change the addresses by console command
		// TODO2: check sensor count for SCD41
		byte devAddress[SENSOR_COUNT - 24] {
			0x55,			// SENSOR_GASESBOARD_SLOT_1A,
			0x55,			// SENSOR_GASESBOARD_SLOT_1W,
			0x56,			// SENSOR_GASESBOARD_SLOT_2A,
			0x56,			// SENSOR_GASESBOARD_SLOT_2W,
			0x54,			// SENSOR_GASESBOARD_SLOT_3A,
			0x54,			// SENSOR_GASESBOARD_SLOT_3W,
			0x44,			// SENSOR_GASESBOARD_TEMPERATURE,
			0x44,			// SENSOR_GASESBOARD_HUMIDITY,

			0x59,			// SENSOR_GROOVE_I2C_ADC,

			0x40,			// SENSOR_INA219_BUSVOLT,
			0x40,			// SENSOR_INA219_SHUNT,
			0x40,			// SENSOR_INA219_CURRENT,
			0x40,			// SENSOR_INA219_LOADVOLT,

			0x18, 			// SENSOR_WATER_TEMP_DS18B20,
			0x66,			// SENSOR_ATLAS_TEMPERATURE,
			0x63,			// SENSOR_ATLAS_PH,
			0x64,			// SENSOR_ATLAS_EC,
			0x64,			// SENSOR_ATLAS_EC_SG,
			0x61,			// SENSOR_ATLAS_DO,		--> Conflict with SENSOR_SCD30_CO2 (et al)
			0x61,			// SENSOR_ATLAS_DO_SAT,	--> Conflict with SENSOR_SCD30_CO2

			0x61, 			// SENSOR_SCD30_CO2, 	--> Conflict with SENSOR_ATLAS_DO
			0x61, 			// SENSOR_SCD30_TEMP, 	--> Conflict with SENSOR_ATLAS_DO
			0x61, 			// SENSOR_SCD30_HUM, 	--> Conflict with SENSOR_ATLAS_DO

			0x20,			// SENSOR_CHIRP_MOISTURE_RAW,
			0x20,			// SENSOR_CHIRP_MOISTURE,
			0x20,			// SENSOR_CHIRP_TEMPERATURE,
			0x20,			// SENSOR_CHIRP_LIGHT,

			0x02,			// SENSOR_EXT_PM_1,
			0x02,			// SENSOR_EXT_PM_25,
			0x02,			// SENSOR_EXT_PM_10,
			0x02,			// SENSOR_EXT_PN_03,
			0x02,			// SENSOR_EXT_PN_05,
			0x02,			// SENSOR_EXT_PN_1,
			0x02,			// SENSOR_EXT_PN_25,
			0x02,			// SENSOR_EXT_PN_5,
			0x02,			// SENSOR_EXT_PN_10,
			0x02,			// SENSOR_EXT_A_PM_1,
			0x02,			// SENSOR_EXT_A_PM_25,
			0x02,			// SENSOR_EXT_A_PM_10,
			0x02,			// SENSOR_EXT_A_PN_03,
			0x02,			// SENSOR_EXT_A_PN_05,
			0x02,			// SENSOR_EXT_A_PN_1,
			0x02,			// SENSOR_EXT_A_PN_25,
			0x02,			// SENSOR_EXT_A_PN_5,
			0x02,			// SENSOR_EXT_A_PN_10,
			0x02,			// SENSOR_EXT_B_PM_1,
			0x02,			// SENSOR_EXT_B_PM_25,
			0x02,			// SENSOR_EXT_B_PM_10,
			0x02,			// SENSOR_EXT_B_PN_03,
			0x02,			// SENSOR_EXT_B_PN_05,
			0x02,			// SENSOR_EXT_B_PN_1,
			0x02,			// SENSOR_EXT_B_PN_25,
			0x02,			// SENSOR_EXT_B_PN_5,
			0x02,			// SENSOR_EXT_B_PN_10,

			0x02,			// SENSOR_PM_DALLAS_TEMP,
			0x00,			// SENSOR_DALLAS_TEMP, 		-- 2 wire (no I2C address)

			0x44,			// SENSOR_SHT31_TEMP,
			0x44,			// SENSOR_SHT31_HUM,
			0x45,			// SENSOR_SHT35_TEMP,
			0x45,			// SENSOR_SHT35_HUM,

			0x29,			// SENSOR_RANGE_LIGHT,
			0x29,			// SENSOR_RANGE_DISTANCE,

			0x77,			// SENSOR_BME680_TEMPERATURE,   -- conflict with 7th possible address of TCA9548A if there are up to 8 x 8 chan mux units !
			0x77,			// SENSOR_BME680_HUMIDITY,
			0x77,			// SENSOR_BME680_PRESSURE,
			0x77,			// SENSOR_BME680_VOCS,

			0x02, 			// SENSOR_GPS_* Grove Gps (on PM board)
			0x10, 			// SENSOR_GPS_* XA111 Gps
			0x42, 			// SENSOR_GPS_* NEO-M8U Gps

			0x3c,			// SENSOR_GROOVE_OLED,

			0x48, 			// SENSOR_ADS1X15_XX_X
			0x49, 			// SENSOR_ADS1X15_XX_X
			0x4a, 			// SENSOR_ADS1X15_XX_X
			0x4b, 			// SENSOR_ADS1X15_XX_X

			//0x3d			// SENSOR_GROOVE_OLED2
			//0x70			// TCA9548A I2C MUX  (not certain this is neded here)
			0x62, 			// SENSOR_SCD4x_CO2, 	 
			0x62, 			// SENSOR_SCD4x_TEMP, 	 
			0x62, 			// SENSOR_SCD4x_HUM, 	 
			
			//(0x12			// PM Board (2)
			0x12,			// SENSOR_WIND_DIR
			0x12,			// SENSOR_WIND_SPEED
			0x12,			// SENSOR_RAIN_ACC
			0x12,			// SENSOR_RAIN_EVENTACC
			0x12,			// SENSOR_RAIN_TOTALACC
			0x12			// SENSOR_RAIN_INTERVAL
			
		};

		bool start(SckBase* base, SensorType wichSensor);
		bool stop(SckBase* base, SensorType wichSensor);

		void getReading(SckBase* base, OneSensor *wichSensor);

		bool getBusyState(SckBase* base,SensorType wichSensor);
		String control(SckBase* base,SensorType wichSensor, String command);
		void print(SckBase* base, SensorType wichSensor,char *payload);
		void updateDisplay(SckBase* base,SensorType wichSensor, bool force=false);
		void plot(SckBase* base,SensorType wichSensor,String value, const char *title=NULL, const char *unit=NULL);
		bool updateGPS(SckBase* base,SensorType wichSensor);

		uint8_t restartTCAMux();

		EepromAuxData data;
		bool dataLoaded = false;

		// 8 Channel I2C MUX
		bool TCAMUXMODE=false;

		// uint8_t findDevicePort(TwoWire *_Wire, byte address);	// find the port that a device is connected to.
													// there are two versions: one uses Aux Bus to find addresses; the other performs 
													// a lookup in the Vector Array
		uint8_t findDeviceChan(SckBase* base,byte address, SensorType wichSensor,bool openChan,bool exclusive=false);	// find the port that a device is connected to.
													// there are two versions: one uses Aux Bus to find addresses; the other performs 
													// a lookup in the Vector Array
		uint8_t tcaDiscoverAMux(SckBase* base,TwoWire *_Wire);		// Discover if a TCA9548A Mux is connected to AUX Bus

		uint8_t AUXI2C_MUXCONNECTED_DEVICES=0;

		bool buildMuxChannelMap(SckBase* base,TwoWire *_Wire);  	// function to populate channel map
		bool checkMuxMapEntryUnique(uint8_t address);
		uint8_t listMuxChanMap(SckBase* base);
		bool testI2C(SckBase* base,uint8_t address, uint8_t channel,SensorType wichSensor,bool exclusive);

		uint8_t countTCAOpenChannels(SckBase* base);
	private:
		static const uint8_t MUX_CHANNELS=8+1;		// there are 9 pysical I2C connections on the MUX (including the driver connection)
		byte channelAry[MUX_CHANNELS] = {
			0x0,							// defines AUX bus itself; not conn thru MUX
			TCA_CHANNEL_0,
			TCA_CHANNEL_1,
			TCA_CHANNEL_2,
			TCA_CHANNEL_3,
			TCA_CHANNEL_4,
			TCA_CHANNEL_5,
			TCA_CHANNEL_6,
			TCA_CHANNEL_7
		};
		
		const static uint8_t I2CMUX_MAX_SENSORS = 15;	// an arbitrary number: the maximum number of I2C devices on the south side of the MUX.
		struct I2C_MuxChannel v_I2CMuxChannels[I2CMUX_MAX_SENSORS];	// One entry in the Map array for each I2C address; 2 Bytes per entry.
		muxChannel getMuxChannel(uint8_t address);
		bool openmuxchanExclusive=false;
};

class GrooveI2C_ADC
{
	public:

		bool start(SckBase* base,AuxBoards* auxBoard, SensorType wichSensor);
		bool stop();
		float getReading(SckBase* base,AuxBoards* auxBoard, SensorType wichSensor);

		const byte deviceAddress 	= 0x59;
		const float V_REF 		= 3.30;
		const byte REG_ADDR_RESULT	= 0x00;
		const byte REG_ADDR_ALERT	= 0x01;
		const byte REG_ADDR_CONFIG	= 0x02;
		const byte REG_ADDR_LIMITL	= 0x03;
		const byte REG_ADDR_LIMITH	= 0x04;
		const byte REG_ADDR_HYST	= 0x05;
		const byte REG_ADDR_CONVL	= 0x06;
		const byte REG_ADDR_CONVH	= 0x07;

	private:
		bool tcaMuxMode=false;
		byte localPortNum= 0x00; // I2c mux port number (0 = no mux)

};

class INA219
{
	public:

		const byte deviceAddress = 0x40;

		enum typeOfReading {BUS_VOLT, SHUNT_VOLT, CURRENT, LOAD_VOLT};

		Adafruit_INA219 ada_ina219 = Adafruit_INA219(deviceAddress);
		bool start(SckBase* base,AuxBoards* auxBoard, SensorType wichSensor);
		bool stop();
		float getReading(SckBase* base,AuxBoards* auxBoard, SensorType wichSensor,typeOfReading wichReading=CURRENT);

	private:
		bool tcaMuxMode=false;
		byte localPortNum= 0x00; // I2c mux port number (0 = no mux)

};

// This implementation works with a 128x128 pixel Oled screen with SH1107 controler
class Groove_OLED
{
	public:
		const byte deviceAddress = 0x3c;
		const uint32_t showTime = 3;

		U8G2_SH1107_SEEED_128X128_F_2ND_HW_I2C u8g2_oled = U8G2_SH1107_SEEED_128X128_F_2ND_HW_I2C(U8G2_R0, U8X8_PIN_NONE);

		bool start(SckBase* base,AuxBoards* auxBoard, SensorType wichSensor);
		bool stop();
		void print(SckBase* base,AuxBoards* auxBoard, SensorType wichSensor, char *payload);
		void update(SckBase* base,AuxBoards* auxBoard, SensorType wichSensor, bool force);
		void displayReading(SckBase* base,AuxBoards* auxBoard, SensorType wichSensor);
		void plot(SckBase* base,AuxBoards* auxBoard, SensorType wichSensor,String value, const char *title=NULL, const char *unit=NULL);

	private:
		const char *_unit;
		const char *_title;

		uint32_t lastUpdate;
		const uint32_t refreshRate = 50;

		void drawBar(SckBase* base,AuxBoards* auxBoard, SensorType wichSensor);
		void drawError(SckBase* base,AuxBoards* auxBoard, SensorType wichSensor,errorType wichError);
		errorType lastError;
		void drawSetup(SckBase* base,AuxBoards* auxBoard, SensorType wichSensor);

		// Plot
		void remap(SckBase* base,AuxBoards* auxBoard, SensorType wichSensor,float newMaxY);
		const float screenMax = 100;
		const float screenMin = 0;
		LinkedList<int8_t> plotData;
		float maxY = 100;
		float minY = 0;

		// For debug log view
		void printLine(SckBase* base,AuxBoards* auxBoard,SensorType wichSensor,char *payload, uint8_t size);
		const uint8_t *font = u8g2_font_6x10_mr;	// if you change font update the next two variables
		const uint8_t font_width = 6;
		const uint8_t font_height = 10;
		const uint8_t columns = 21;
		const uint8_t lines = 12;
		uint8_t currentLine = 1;

		SensorType lastShown;
		uint32_t showStartTime = 0;

		bool tcaMuxMode=false;
		byte localPortNum= 0x00; // I2c mux port number (0 = no mux)

};

/*! @class DS2482_100
 *  @brief class for handling the DS18B20 temperature sensor connected to the I2C port
 *   through the DS2482-100 board. This was based on an example made by
 *   <a href="https://github.com/paeaetech/paeae.git">paeae</a>
 */
class WaterTemp_DS18B20
{

	public:

		byte deviceAddress = 0x18;

		DS2482 DS_bridge = DS2482(0);

		byte data[9];
		byte addr[8];

		uint8_t conf =0x05;

		bool detected = false;

		/* Start the transmission of data for the DS18B20 trough the DS2482_100 bridge */
		bool start(SckBase* base,AuxBoards* auxBoard, SensorType wichSensor);
		bool stop();
		/* Read the temperature of the DS18B20 through the DS2482_100 bridge */
		/* @return Temperature */
		float getReading(SckBase* base,AuxBoards* auxBoard, SensorType wichSensor);
	private:
		bool tcaMuxMode=false;
		byte localPortNum= 0x00; // I2c mux port number (0 = no mux)

};

class Atlas
{
	public:

		byte deviceAddress;
		SensorType atlasType;
		bool PH = false;
		bool EC = false;
		bool DO = false;
		bool TEMP = false;
		float newReading[4];
		String atlasResponse;
		uint32_t lastCommandSent = 0;
		uint32_t lastUpdate = 0;
		enum State {
			REST,
			TEMP_COMP_SENT,
			ASKED_READING,
		};
		State state = REST;

		// Constructor varies by sensor type
		Atlas(SensorType wichSensor) {
			atlasType = wichSensor;

			switch(atlasType) {
				case SENSOR_ATLAS_PH: {

						deviceAddress = 0x63;
						PH = true;
						break;

					} case SENSOR_ATLAS_EC:
					case SENSOR_ATLAS_EC_TDS:
					case SENSOR_ATLAS_EC_SAL:
					case SENSOR_ATLAS_EC_SG: {

						deviceAddress = 0x64;
						EC = true;
						break;

					} case SENSOR_ATLAS_DO:
					case SENSOR_ATLAS_DO_SAT: {

						deviceAddress = 0x61;
						DO = true;
						break;

					} case SENSOR_ATLAS_TEMPERATURE: {

						deviceAddress = 0x66;
						TEMP = true;
						break;

					} default: break;
			}

		}

		bool start(SckBase* base,AuxBoards* auxBoard, SensorType wichSensor);
		bool beginDone = false;
		bool stop();
		bool getReading(SckBase* base,AuxBoards* auxBoard, SensorType wichSensor);
		bool getBusyState(SckBase* base,AuxBoards* auxBoard, SensorType wichSensor);

		void goToSleep(SckBase* base,AuxBoards* auxBoard, SensorType wichSensor);
		bool sendCommand(SckBase* base,char* command,AuxBoards* auxBoard, SensorType wichSensor);
		bool tempCompensation(SckBase *Base,AuxBoards* auxBoard, SensorType wichSensor);
		uint8_t getResponse(SckBase* base,AuxBoards* auxBoard, SensorType wichSensor);

		uint16_t longWait = 910; //ms
		uint16_t mediumWait = 610; //ms
		uint16_t shortWait = 310; //ms

		bool detected = false;

	private:
		bool tcaMuxMode=false;
		byte localPortNum= 0x00; // I2c mux port number (0 = no mux)

};

class Moisture
{
	private:
		byte deviceAddress = 0x20;
		//bool tcaMuxMode=false;
		I2CSoilMoistureSensor chirp = I2CSoilMoistureSensor(deviceAddress);
		bool alreadyStarted = false;

	public:
		byte localPortNum= 0x00; // I2c mux port number (0 = no mux) (must be public for this sensor)
		bool detected = false;
		bool calibrated = false;
		int32_t dryPoint;
		int32_t wetPoint;
		
		bool start(SckBase* base,AuxBoards* auxBoard, SensorType wichSensor);
		bool stop();
		bool getReading(SckBase* base,SensorType wichSensor,AuxBoards* auxBoard);
		uint8_t getVersion(SckBase* base,AuxBoards* auxBoard, SensorType wichSensor);
		bool resetAddress(SckBase* base,int currentAddress,AuxBoards* auxBoard, SensorType wichSensor);

		uint32_t raw;
		float moisture;
		int16_t temperature;
		int32_t light;

		// TODO
		// * Measure sensor consumption
		// * Send sensor to sleep between readings (needs FIX, it hangs)
		void sleep(SckBase* base,AuxBoards* auxBoard, SensorType wichSensor);

};

enum PMslot {SLOT_A, SLOT_B, SLOT_AVG};
enum PMcommands
{
	START_PMA, 	  	// Start PM in slot A
	START_PMB,      	// Start PM in slot B
	GET_PMA, 		// Get values for PM in slot A
	GET_PMB, 		// Get values for PM in slot B
	STOP_PMA, 		// Stop PM in slot A
	STOP_PMB, 		// Stop PM in slot B
	DALLASTEMP_START,
	DALLASTEMP_STOP,
	GET_DALLASTEMP,
	GROVEGPS_START,
	GROVEGPS_STOP,
	GROVEGPS_GET
 };

class PMsensor
{
	public:
		// Constructor varies by sensor type
		PMsensor(PMslot wichSlot) {
			_slot = wichSlot;
		}

		const byte deviceAddress = 0x02;

		uint16_t pm1;
		uint16_t pm25;
		uint16_t pm10;
		uint16_t pn03;
		uint16_t pn05;
		uint16_t pn1;
		uint16_t pn25;
		uint16_t pn5;
		uint16_t pn10;

		bool start(SckBase* base,AuxBoards* auxBoard, SensorType wichSensor);
		bool stop(SckBase* base,AuxBoards* auxBoard, SensorType wichSensor);
		bool update(SckBase* base,AuxBoards* auxBoard, SensorType wichSensor);
	private:
		bool started = false;
		bool failed = false;

		static const uint8_t valuesSize = 18;
		uint8_t values[valuesSize];

		// 24 bytes:
		// 0:1->pm1, 2:3->pm25, 4:5->pm10,
		// Number of particles with diameter beyond X um in 0.1 L of air.
		// 6:7 -> 0.3 um
		// 8:9 -> 0.5 um
		// 10:11 -> 1.0 um
		// 12:13 -> 2.5 um
		// 14:15 -> 5.0 um
		// 16:17 -> 10.0 um
		//
		uint32_t lastReading = 0;
		PMslot _slot;
		bool tcaMuxMode=false;
		byte localPortNum= 0x00; // I2c mux port number (0 = no mux)

};

typedef enum PM2slot {WIND, RAIN} pm2slot;

typedef enum PM2commands {
	none=0,
	COMM_START_WIND,
	COMM_STOP_WIND,
	COMM_GET_WIND_DIR,
	COMM_GET_WIND_SPEED,
	COMM_START_RAIN,
	COMM_STOP_RAIN,
	COMM_GET_RAIN_ACC,
	COMM_GET_RAIN_EVENTACC,
	COMM_GET_RAIN_TOTALACC,
	COMM_GET_RAIN_INTVACC,
	NUM_COMMANDS
} PM2sensorCommand;

struct ReadingSize {
	PM2sensorCommand sensor;
	byte bytes;
	uint32_t lastReadingTime;	
};

union windandrainrdg {
	float f;
	byte  b[4];
};

class PM2sensor
{
	public:
		PM2sensor(PM2slot wichSlot) {
			_slot = wichSlot;
		}
		const byte deviceAddress = 0x12;

		bool start(SckBase* base,AuxBoards* auxBoard, SensorType wichSensor);
		bool stop(SckBase* base,AuxBoards* auxBoard, SensorType wichSensor);
		bool update(SckBase* base,AuxBoards* auxBoard, SensorType wichSensor);
		String windDir;
		String windSpeed;
		String rainAcc;
		String rainEventAcc;
		String rainTotalAcc;
		String rainIntAcc;
	private:
		bool started = false;
		bool failed = false;

		PM2slot _slot;
		ReadingSize getReadingSize(SensorType wichSensor);
		void setLastReading(SensorType wichSensor,PM2sensorCommand wichCommand);
		ReadingSize v_readingFetchSizes[NUM_COMMANDS] {
			ReadingSize{COMM_START_WIND,0,0},
			ReadingSize{COMM_STOP_WIND,0,0},
			ReadingSize{COMM_GET_WIND_DIR,4,0},
			ReadingSize{COMM_GET_WIND_SPEED,4,0},
			ReadingSize{COMM_START_RAIN,0,0},
			ReadingSize{COMM_STOP_RAIN,0,0},
			ReadingSize{COMM_GET_RAIN_ACC,4,0},
			ReadingSize{COMM_GET_RAIN_EVENTACC,4,0},
			ReadingSize{COMM_GET_RAIN_TOTALACC,4,0},
			ReadingSize{COMM_GET_RAIN_INTVACC,4,0}
		};
		// uint32_t lastReading = 0;
		byte localPortNum= 0x00; // I2c mux port number (0 = no mux)
		windandrainrdg myreading;

};

class PM_DallasTemp
{
	public:
		const byte deviceAddress = 0x02;
		bool start(SckBase* base,AuxBoards* auxBoard, SensorType wichSensor);
		bool stop(SckBase* base,AuxBoards* auxBoard, SensorType wichSensor);
		float getReading(SckBase* base,AuxBoards* auxBoard, SensorType wichSensor);
	private:
		union u_reading {
			byte b[4];
			float fval;
		} uRead;

		float reading;
		bool tcaMuxMode=false;
		byte localPortNum= 0x00; // I2c mux port number (0 = no mux)

};

struct GpsReadings
{
	// Data (40 bytes)
	// Fix Quality -> uint8 - 1
	// 	0 = Invalid
	// 	1 = GPS fix (SPS)
	// 	2 = DGPS fix
	// 	3 = PPS fix
	// 	4 = Real Time Kinematic
	// 	5 = Float RTK
	// 	6 = estimated (dead reckoning) (2.3 feature)
	// 	7 = Manual input mode
	// 	8 = Simulation mode
	// locationValid -> bool - 1
	// Latitude DDD.DDDDDD (negative is south) -> double - 8
	// Longitude DDD.DDDDDD (negative is west) -> double - 8
	// altitudeValid -> bool - 1
	// Altitude in meters -> float - 4
	// timeValid -> bool - 1
	// Time (epoch) -> uint32 - 4
	// speedValid -> bool - 1
	// Speed (meters per second) -> float - 4
	// hdopValid -> bool - 1
	// Horizontal dilution of position -> float - 4
	// satellitesValid -> bool - 1
	// Number of Satellites being traked -> uint8 - 1

	uint8_t fixQuality = 0;
	bool locationValid = false;
	double latitude;
	double longitude;
	bool altitudeValid = false;
	float altitude;
	bool speedValid = false;
	float speed;
	bool hdopValid = false;
	float hdop;
	bool satellitesValid = false;
	uint8_t satellites;
	bool timeValid = false;
	uint32_t epochTime = 0;
};

class GPS_Source
{
	public:
		virtual bool stop(SckBase* base,AuxBoards* auxBoard, SensorType wichSensor);
		virtual bool getReading(SckBase* base,AuxBoards* auxBoard,SensorType wichSensor, GpsReadings &r);
		virtual bool update(SckBase* base,AuxBoards* auxBoard, SensorType wichSensor);
		
};

class Sck_GPS
{
	private:
		bool started = false;
		uint8_t fixCounter = 0;
		GPS_Source *gps_source;
		bool tcaMuxMode=false;
		byte localPortNum= 0x00; // I2c mux port number (0 = no mux)
	public:
		GpsReadings r;

		bool start(SckBase* base,AuxBoards* auxBoard, SensorType wichSensor);
		bool stop(SckBase* base,AuxBoards* auxBoard, SensorType wichSensor);
		bool getReading(SckBase* base,AuxBoards* auxBoard, SensorType wichSensor);
		bool update(SckBase* base,AuxBoards* auxBoard, SensorType wichSensor);
		

};

class PM_Grove_GPS: public GPS_Source
{
	public:
		const byte deviceAddress = 0x02;

		bool start(SckBase* base,AuxBoards* auxBoard, SensorType wichSensor);
		virtual bool stop(SckBase* base,AuxBoards* auxBoard, SensorType wichSensor);
		virtual bool getReading(SckBase* base,AuxBoards* auxBoard,SensorType wichSensor, GpsReadings &r);
		virtual bool update(SckBase* base,AuxBoards* auxBoard, SensorType wichSensor);

	private:
		static const uint8_t DATA_LEN = 40;
		byte data[DATA_LEN];
		uint32_t lastReading = 0;
		bool tcaMuxMode=false;
		byte localPortNum= 0x00; // I2c mux port number (0 = no mux)
};

class XA111GPS: public GPS_Source
{
	public:
		const byte deviceAddress = 0x10;

		bool start(SckBase* base,AuxBoards* auxBoard, SensorType wichSensor);
		virtual bool stop(SckBase* base,AuxBoards* auxBoard, SensorType wichSensor);
		virtual bool getReading(SckBase* base,AuxBoards* auxBoard,SensorType wichSensor, GpsReadings &r);
		virtual bool update(SckBase* base,AuxBoards* auxBoard, SensorType wichSensor);

	private:
		I2CGPS i2cGps;
		uint32_t lastReading = 0;
		bool tcaMuxMode=false;
		byte localPortNum= 0x00; // I2c mux port number (0 = no mux)

};

class NEOM8UGPS: public GPS_Source
{
	public:
		const byte deviceAddress = 0x42;

		bool start(SckBase* base,AuxBoards* auxBoard, SensorType wichSensor);
		virtual bool stop(SckBase* base,AuxBoards* auxBoard,SensorType wichSensor);
		virtual bool getReading(SckBase* base,AuxBoards* auxBoard,SensorType wichSensor, GpsReadings &r);
		virtual bool update(SckBase* base,AuxBoards* auxBoard,SensorType wichSensor);

	private:
		SFE_UBLOX_GNSS ubloxGps;
		uint32_t lastReading = 0;
		bool tcaMuxMode=false;
		byte localPortNum= 0x00; // I2c mux port number (0 = no mux)

};

class Sck_DallasTemp
{
	// This is for a Dallas temperature sensor connected to the plugged in (direct, not via I2C) Aux groove connector using pin pinAUX_WIRE_SCL (13 - PA17)
	public:
		bool start();
		bool stop();
		bool getReading();

		float reading;
	private:
		uint8_t _oneWireAddress[8];
		
};

class Sck_Range
{
	public:
		const byte deviceAddress = 0x29;
		bool start(SckBase* base,AuxBoards* auxBoard,SensorType wichSensor);
		bool stop();
		bool getReading(SckBase* base,AuxBoards* auxBoard,SensorType wichSensor);

		float readingLight;
		float readingDistance;
	private:

		bool alreadyStarted = false;
		VL6180x vl6180x = VL6180x(deviceAddress);
		bool tcaMuxMode=false;
		byte localPortNum= 0x00; // I2c mux port number (0 = no mux)
};

class Sck_BME680
{
	public:
		const byte deviceAddress = 0x77;
		bool start(SckBase* base,AuxBoards* auxBoard,SensorType wichSensor);
		bool stop();
		bool getReading(SckBase* base,AuxBoards* auxBoard,SensorType wichSensor);

		float temperature;
		float humidity;
		float pressure;
		float VOCgas;
	private:
		uint32_t lastTime = 0;
		uint32_t minTime = 1000; 	// Avoid taking readings more often than this value (ms)
		bool alreadyStarted = false;
		Adafruit_BME680 bme;
		bool tcaMuxMode=false;
		byte localPortNum= 0x00; // I2c mux port number (0 = no mux)

};

class Sck_ADS1X15
{
	public:
		const byte deviceAddress = 0x48;
		bool start(SckBase* base,uint8_t address,AuxBoards* auxBoard,SensorType wichSensor);
		bool stop();
		bool getReading(SckBase* base,AuxBoards* auxBoard,uint8_t wichChannel, SensorType wichSensor);
		float reading;

		#ifdef adsTest
		uint8_t adsChannelW = 0; // Default channel for WE
		uint8_t adsChannelA = 1; // Default channel for AE
		void setTesterCurrent(int16_t wichCurrent, uint8_t wichChannel);
		// double preVoltA = -99;
		// double preVoltW = -99;
		// double threshold = 0.05;
		// uint8_t maxErrorsA = 5;
		// uint8_t maxErrorsW = 5;
		void runTester(uint8_t wichChannel);
		testerGasesBoard tester;
		#endif

	private:
		float VOLTAGE = 3.3;
		bool started = false;
		Adafruit_ADS1115 ads = Adafruit_ADS1115(&auxWire);
		bool tcaMuxMode=false;
		byte localPortNum= 0x00; // I2c mux port number (0 = no mux)

	// TODO
	// Test ADS1015
};

class Sck_SCD30
{
	public:
		const byte deviceAddress = 0x61;
		bool start(SckBase* base, SensorType wichSensor,AuxBoards* auxBoard);
		bool stop(SckBase* base,SensorType wichSensor,AuxBoards* auxBoard);
		bool getReading(SckBase* base,SensorType wichSensor,AuxBoards* auxBoard);
		uint16_t interval(SckBase* base,AuxBoards* auxBoard, SensorType wichSensor,uint16_t newInterval=0);
		bool autoSelfCal(SckBase* base,AuxBoards* auxBoard, SensorType wichSensor,int8_t value=-1);
		uint16_t forcedRecalFactor(SckBase* base,AuxBoards* auxBoard, SensorType wichSensor,uint16_t newFactor=0);
		float tempOffset(SckBase* base,AuxBoards* auxBoard, SensorType wichSensor,float userTemp=0.00001, bool off=false);

		uint16_t co2 = 0;
		float temperature = 0;
		float humidity = 0;

		bool pressureCompensated = false;

	private:
		uint8_t enabled[3][2] = { {SENSOR_SCD30_CO2, 0}, {SENSOR_SCD30_TEMP, 0}, {SENSOR_SCD30_HUM, 0} };
		bool _debug = false;
		bool started = false;
		uint16_t measInterval = 2; 	// "2-1800 seconds"
		SCD30 sparkfun_scd30;
		bool tcaMuxMode=false;
		byte localPortNum= 0x00; // I2c mux port number (0 = no mux)

};
#include "SparkFun_SCD4x_Arduino_Library.h"

class Sck_SCD4x
{
	public:
		const byte deviceAddress = 0x62;
		bool start(SckBase* base, SensorType wichSensor,AuxBoards* auxBoard);  // base is needed to allow pressure sensor to be read;
		bool stop(SckBase* base, SensorType wichSensor,AuxBoards* auxBoard);
		bool getReading(SckBase* base,SensorType wichSensor,AuxBoards* auxBoard); // base is needed to allow current port number to be written
		bool update(SckBase* base,SensorType wichSensor,AuxBoards* auxBoard);		
		uint16_t interval(SckBase* base,AuxBoards* auxBoard, SensorType wichSensor,uint16_t newInterval=0);
		bool autoSelfCal(SckBase* base,AuxBoards* auxBoard,SensorType wichSensor,int8_t value=-1);
		uint16_t forcedRecalFactor(SckBase* base,AuxBoards* auxBoard,SensorType wichSensor, uint16_t newFactor=0);
		float tempOffset(SckBase* base,AuxBoards* auxBoard,SensorType wichSensor,float userTemp=0.000001, bool off=false); // base is needed to allow main temp sensor to be read
		bool setPressureComp(SckBase* base, bool value=true);
		uint16_t co2 = 0;
		float temperature = 0;
		float humidity = 0;
		float offsetPct = 0;

		bool pressureCompensated = false;

	private:
		uint8_t enabled[3][2] = { {SENSOR_SCD4x_CO2, 0}, {SENSOR_SCD4x_TEMP, 0}, {SENSOR_SCD4x_HUM, 0} };

		uint32_t SCD4X_LAST_SINGLESHOT=0;
		const uint32_t SCD4X_SINGLESHOT_WAIT_INTERVAL=5001;
		uint8_t readcount=0;
		bool _debug = false;
		bool started = false;
		uint16_t measInterval = 5; 	// "2-1800 seconds" (5 is the minimum for single shot readings)
		
		SCD4x sparkfun_SCD4x;
		
		bool tcaMuxMode=false;
		byte localPortNum= 0x00; // I2c mux port number (0 = no mux)

};

void writeI2C(byte deviceAddress, byte instruction, byte data);
byte readI2C(byte deviceAddress, byte instruction);
