
#include <Arduino.h>
#include <String.h>
#include "SckAux.h"

#ifndef GASBOARD_DISABLE
GasesBoard		gasBoard;
#endif
GrooveI2C_ADC		grooveI2C_ADC;
INA219			ina219;

#ifdef TWO_OLED
Groove_OLED		groove_OLED(0x3c);
Groove_OLED		groove_OLED2(0x3d);
#endif
#ifdef ONE_OLED
//Groove_OLED		groove_OLED(0x3d);
Groove_OLED		groove_OLED(0x3c);
#endif

#ifndef MISC_DISABLE
WaterTemp_DS18B20 	waterTemp_DS18B20;
#endif
#ifndef ATLAS_DISABLE
Atlas			atlasPH = Atlas(SENSOR_ATLAS_PH);
Atlas			atlasEC = Atlas(SENSOR_ATLAS_EC);
Atlas			atlasDO = Atlas(SENSOR_ATLAS_DO);
Atlas 			atlasTEMP = Atlas(SENSOR_ATLAS_TEMPERATURE);
#endif
#ifndef CHIRP_DISABLE
Moisture 		moistureChirp;
#endif
PMsensor		pmSensorA = PMsensor(SLOT_A);
PMsensor		pmSensorB = PMsensor(SLOT_B);
PM2sensor		windandrain;

PM_DallasTemp 		pmDallasTemp;
#ifndef MISC_DISABLE
Sck_DallasTemp 		dallasTemp;
#endif
Sck_SHT31 		sht31 = Sck_SHT31(&auxWire);
Sck_SHT31 		sht35 = Sck_SHT31(&auxWire, 0x45);
#ifndef MISC_DISABLE
Sck_Range 		range;
Sck_BME680 		bme680;
#endif
Sck_GPS 		gps;
PM_Grove_GPS 		pmGroveGps;
XA111GPS 		xa1110gps;
NEOM8UGPS 		neoM8uGps;
Sck_ADS1X15 		ads48;
Sck_ADS1X15 		ads49;
Sck_ADS1X15 		ads4A;
Sck_ADS1X15 		ads4B;
#ifndef SCD30_DISABLE
Sck_SCD30 		scd30;
#endif
Sck_SCD4x		scd4x;		// the default is SCD40 - not what we have here
TCA9548A<TwoWire> TCA;

// ScioSense ENS160 Eval Board
Sck_ENS160		ens160;
Sck_ENS210		ens210;



// Eeprom flash emulation to store I2C address
FlashStorage(eepromAuxData, EepromAuxData);

bool AuxBoards::start(SckBase* base, SensorType wichSensor)
{
	//base->sckOut("AuxBoards::start: called",PRIO_MED,true);
	
	byte error =0;
	if (!dataLoaded) {	
		base->sckOut("AUX loading eepromdata",PRIO_LOW,true);
		data = eepromAuxData.read();
		dataLoaded = true;
		base->sckOut("AUX eepromdata data loaded",PRIO_MED,true);

		#ifndef CHIRP_DISABLE
		if (data.calibration.moistureCalDataValid) {
			moistureChirp.dryPoint = data.calibration.dryPoint;
			moistureChirp.wetPoint = data.calibration.wetPoint;
			moistureChirp.calibrated = true;
		}
		base->sckOut("AUX moisture calibration data loaded",PRIO_LOW,true);
		#endif
		

		// TCA9548A I2C Mux initialisation and auxWire channel discovery
		// (only needs to be loaded once)
		//long timer=micros();							

		uint16_t ctr=0;
		muxAddress.i=tcaDiscoverAMux(base,&auxWire);		// discover the I2C Mux address if any is installed.
		sprintf(base->outBuff,"A TCA9458A Mux has been found on Aux Bus 0x%03x", muxAddress.b);
		//sprintf(base->outBuff,"TCA Mux address found: 0x%03x", muxAddress.b);
		base->sckOut(PRIO_MED,true);
		uint32_t tcatimer=micros();
		while ((micros()-tcatimer <100)) {
			//base->sckOut("in while loop...",PRIO_MED,true);
			TCA.begin(auxWire,muxAddress.i);				// begin the instance of TCA 9458A
			delayMicroseconds(10);							// allow a short delay for instantiation;
			auxWire.beginTransmission(muxAddress.b);		// check that the Mux is talking
			error = auxWire.endTransmission();
			ctr++;
			if (error == 0) {
				TCAMUXMODE=true;
				//base->sckOut("AUX TCA building Channel Map",PRIO_MED,true);
				buildMuxChannelMap(base,&auxWire);
				//base->sckOut("AUX TCA Channel Map built",PRIO_MED,true);
			} else {
				//base->sckOut("There was an error connecting to TCA Mux",PRIO_MED,true);
				TCAMUXMODE=false;
			}
		}
		//sprintf(base->outBuff,"TCA begin took %i attempts",ctr);
		//base->sckOut(PRIO_LOW,true);
		
		//int timetaken=(micros()- timer )/1000;	// milliseconds
		//sprintf(base->outBuff, "I2C MUX discovery took %i seconds\r\n", timetaken/1000);
		//base->sckOut(PRIO_LOW,true);
	}


	//sprintf(base->outBuff,"AUX is trying to start up: %s",base->sensors[wichSensor].title);
	//base->sckOut(PRIO_LOW,true);

	switch (wichSensor) {
		#ifndef GASBOARD_DISABLE
		case SENSOR_GASESBOARD_SLOT_1A:
		case SENSOR_GASESBOARD_SLOT_1W:
		case SENSOR_GASESBOARD_SLOT_2A:
		case SENSOR_GASESBOARD_SLOT_2W:
		case SENSOR_GASESBOARD_SLOT_3A:
		case SENSOR_GASESBOARD_SLOT_3W:
		case SENSOR_GASESBOARD_HUMIDITY:
		case SENSOR_GASESBOARD_TEMPERATURE: 	return gasBoard.start(); break;
		#endif
		case SENSOR_GROOVE_I2C_ADC: 		return grooveI2C_ADC.start(base,this,wichSensor); break;
		case SENSOR_INA219_BUSVOLT:
		case SENSOR_INA219_SHUNT:
		case SENSOR_INA219_CURRENT:
		case SENSOR_INA219_LOADVOLT: 		return ina219.start(base,this,wichSensor); break;
		#ifndef MISC_DISABLE
		case SENSOR_WATER_TEMP_DS18B20:		return waterTemp_DS18B20.start(base,this,wichSensor); break;
		#endif
		#ifndef ATLAS_DISABLE
		case SENSOR_ATLAS_TEMPERATURE: 		return atlasTEMP.start(base,this,wichSensor); break;
		case SENSOR_ATLAS_PH:			return atlasPH.start(base,this,wichSensor);
		case SENSOR_ATLAS_EC:
		case SENSOR_ATLAS_EC_TDS:
		case SENSOR_ATLAS_EC_SAL:
		case SENSOR_ATLAS_EC_SG: 		return atlasEC.start(base,this,wichSensor); break;
		case SENSOR_ATLAS_DO:
		case SENSOR_ATLAS_DO_SAT: 		return atlasDO.start(base,this,wichSensor); break;
		#endif
		#ifndef CHIRP_DISABLE
		case SENSOR_CHIRP_MOISTURE_RAW:
		case SENSOR_CHIRP_MOISTURE:
		case SENSOR_CHIRP_TEMPERATURE:
		case SENSOR_CHIRP_LIGHT:		return moistureChirp.start(base,this,wichSensor); break;
		#endif
		case SENSOR_EXT_A_PM_1:
		case SENSOR_EXT_A_PM_25:
		case SENSOR_EXT_A_PM_10:
		case SENSOR_EXT_A_PN_03:
		case SENSOR_EXT_A_PN_05:
		case SENSOR_EXT_A_PN_1:
		case SENSOR_EXT_A_PN_25:
		case SENSOR_EXT_A_PN_5:
		case SENSOR_EXT_A_PN_10: 		return pmSensorA.start(base,this,wichSensor); break;
		case SENSOR_EXT_B_PM_1:
		case SENSOR_EXT_B_PM_25:
		case SENSOR_EXT_B_PM_10:
		case SENSOR_EXT_B_PN_03:
		case SENSOR_EXT_B_PN_05:
		case SENSOR_EXT_B_PN_1:
		case SENSOR_EXT_B_PN_25:
		case SENSOR_EXT_B_PN_5:
		case SENSOR_EXT_B_PN_10: 		return pmSensorB.start(base,this,wichSensor); break;
		case SENSOR_PM_DALLAS_TEMP: 	return pmDallasTemp.start(base,this,wichSensor); break;
		#ifndef MISC_DISABLE
		case SENSOR_DALLAS_TEMP: 		return dallasTemp.start(); break;
		#endif
		case SENSOR_SHT31_TEMP:
		case SENSOR_SHT31_HUM: {
			#ifndef GASBOARD_DISABLE
			if (sht31.start() && !gasBoard.start()) return true;
			#else
			if (sht31.start()) return true;
			#endif
			else return false;
			break;
			
		}
		case SENSOR_SHT35_TEMP:
		case SENSOR_SHT35_HUM: 			return sht35.start(); break;
		#ifndef MISC_DISABLE
		case SENSOR_RANGE_DISTANCE: 		return range.start(base,this,wichSensor); break;
		case SENSOR_RANGE_LIGHT: 		return range.start(base,this,wichSensor); break;
		case SENSOR_BME680_TEMPERATURE:		return bme680.start(base,this,wichSensor); break;
		case SENSOR_BME680_HUMIDITY:		return bme680.start(base,this,wichSensor); break;
		case SENSOR_BME680_PRESSURE:		return bme680.start(base,this,wichSensor); break;
		case SENSOR_BME680_VOCS:		return bme680.start(base,this,wichSensor); break;
		#endif
		case SENSOR_GPS_FIX_QUALITY:
		case SENSOR_GPS_LATITUDE:
		case SENSOR_GPS_LONGITUDE:
		case SENSOR_GPS_ALTITUDE:
		case SENSOR_GPS_SPEED:
		case SENSOR_GPS_HDOP:
		case SENSOR_GPS_SATNUM:			return gps.start(base,this,wichSensor); break;
		case SENSOR_ADS1X15_48_0:
		case SENSOR_ADS1X15_48_1:
		case SENSOR_ADS1X15_48_2:
		case SENSOR_ADS1X15_48_3: 		return ads48.start(base,0x48,this,wichSensor); break;
		case SENSOR_ADS1X15_49_0:
		case SENSOR_ADS1X15_49_1:
		case SENSOR_ADS1X15_49_2:
		case SENSOR_ADS1X15_49_3: 		return ads49.start(base,0x49,this,wichSensor); break;
		case SENSOR_ADS1X15_4A_0:
		case SENSOR_ADS1X15_4A_1:
		case SENSOR_ADS1X15_4A_2:
		case SENSOR_ADS1X15_4A_3: 		return ads4A.start(base,0x4A,this,wichSensor); break;
		case SENSOR_ADS1X15_4B_0:
		case SENSOR_ADS1X15_4B_1:
		case SENSOR_ADS1X15_4B_2:
		case SENSOR_ADS1X15_4B_3: 		return ads4B.start(base,0x4B,this,wichSensor); break;
		#ifndef SCD30_DISABLE
		case SENSOR_SCD30_CO2: 			return scd30.start(base, SENSOR_SCD30_CO2,this); break;
		case SENSOR_SCD30_TEMP: 		return scd30.start(base, SENSOR_SCD30_TEMP,this); break;
		case SENSOR_SCD30_HUM: 			return scd30.start(base, SENSOR_SCD30_HUM,this); break;
		#endif
		case SENSOR_SCD4x_CO2: 			return scd4x.start(base, SENSOR_SCD4x_CO2,this); break;
		case SENSOR_SCD4x_TEMP: 		return scd4x.start(base, SENSOR_SCD4x_TEMP,this); break;
		case SENSOR_SCD4x_HUM: 			return scd4x.start(base, SENSOR_SCD4x_HUM,this); break;
		#ifdef ONE_OLED 
		case SENSOR_GROVE_OLED: 		return groove_OLED.start(base,this,wichSensor); break;
		#endif
		#ifdef TWO_OLED
		case SENSOR_GROVE_OLED: 		return groove_OLED.start(base,this,wichSensor); break;
		case SENSOR_GROVE_OLED2: 		return groove_OLED2.start(base,this,wichSensor); break;
		#endif
		case SENSOR_WIND_DIR:			return windandrain.start(base,this,SENSOR_WIND_DIR); break;
		case SENSOR_WIND_SPEED:			return windandrain.start(base,this,SENSOR_WIND_SPEED); break;
		case SENSOR_RAIN_ACC:			return windandrain.start(base,this,SENSOR_RAIN_ACC); break;
		case SENSOR_RAIN_EVENTACC:		return windandrain.start(base,this,SENSOR_RAIN_EVENTACC); break;
		case SENSOR_RAIN_TOTALACC:		return windandrain.start(base,this,SENSOR_RAIN_TOTALACC); break;
		case SENSOR_RAIN_INTERVAL:		return windandrain.start(base,this,SENSOR_RAIN_INTERVAL); break;
		case SENSOR_ENS160_ECO2:		return ens160.start(base,this,SENSOR_ENS160_ECO2); break;
		case SENSOR_ENS160_TVOC:		return ens160.start(base,this,SENSOR_ENS160_TVOC); break;
		case SENSOR_ENS160_AQI:		return ens160.start(base,this,SENSOR_ENS160_AQI); break;
		case SENSOR_ENS210_TEMP:		return ens210.start(base,this,SENSOR_ENS210_TEMP); break;
		case SENSOR_ENS210_ABSTEMP:		return ens210.start(base,this,SENSOR_ENS210_ABSTEMP); break;
		case SENSOR_ENS210_HUM:		return ens210.start(base,this,SENSOR_ENS210_HUM); break;

		default: break;
	}
	//sprintf(base->outBuff,"AUX did not start: %s",base->sensors[wichSensor].title);
	//base->sckOut(PRIO_LOW,true);
	return false;
}

bool AuxBoards::stop(SckBase* base, SensorType wichSensor)
{
	if (TCAMUXMODE) resetI2CMux();

	switch (wichSensor) {
		#ifndef GASBOARD_DISABLE
		case SENSOR_GASESBOARD_SLOT_1A:
		case SENSOR_GASESBOARD_SLOT_1W:
		case SENSOR_GASESBOARD_SLOT_2A:
		case SENSOR_GASESBOARD_SLOT_2W:
		case SENSOR_GASESBOARD_SLOT_3A:
		case SENSOR_GASESBOARD_SLOT_3W:
		case SENSOR_GASESBOARD_HUMIDITY:
		case SENSOR_GASESBOARD_TEMPERATURE: 	return gasBoard.stop(); break;
		#endif
		case SENSOR_GROOVE_I2C_ADC: 		return grooveI2C_ADC.stop(); break;
		case SENSOR_INA219_BUSVOLT:
		case SENSOR_INA219_SHUNT:
		case SENSOR_INA219_CURRENT:
		case SENSOR_INA219_LOADVOLT: 		return ina219.stop(); break;
		#ifndef MISC_DISABLE
		case SENSOR_WATER_TEMP_DS18B20:		return waterTemp_DS18B20.stop(); break;
		#endif
		#ifndef ATLAS_DISABLE
		case SENSOR_ATLAS_TEMPERATURE: 		return atlasTEMP.stop(); break;
		case SENSOR_ATLAS_PH:			return atlasPH.stop();
		case SENSOR_ATLAS_EC:
		case SENSOR_ATLAS_EC_TDS:
		case SENSOR_ATLAS_EC_SAL:
		case SENSOR_ATLAS_EC_SG: 		return atlasEC.stop(); break;
		case SENSOR_ATLAS_DO:
		case SENSOR_ATLAS_DO_SAT: 		return atlasDO.stop(); break;
		#endif
		#ifndef CHIRP_DISABLE
		case SENSOR_CHIRP_TEMPERATURE:
		case SENSOR_CHIRP_MOISTURE:		return moistureChirp.stop(); break;
		#endif
		case SENSOR_EXT_A_PM_1:
		case SENSOR_EXT_A_PM_25:
		case SENSOR_EXT_A_PM_10:
		case SENSOR_EXT_A_PN_03:
		case SENSOR_EXT_A_PN_05:
		case SENSOR_EXT_A_PN_1:
		case SENSOR_EXT_A_PN_25:
		case SENSOR_EXT_A_PN_5:
		case SENSOR_EXT_A_PN_10: 		return pmSensorA.stop(base,this,wichSensor); break;
		case SENSOR_EXT_B_PM_1:
		case SENSOR_EXT_B_PM_25:
		case SENSOR_EXT_B_PM_10:
		case SENSOR_EXT_B_PN_03:
		case SENSOR_EXT_B_PN_05:
		case SENSOR_EXT_B_PN_1:
		case SENSOR_EXT_B_PN_25:
		case SENSOR_EXT_B_PN_5:
		case SENSOR_EXT_B_PN_10: 		return pmSensorB.stop(base,this,wichSensor); break;
		case SENSOR_PM_DALLAS_TEMP: 		return pmDallasTemp.stop(base,this,wichSensor); break;
		#ifndef MISC_DISABLE
		case SENSOR_DALLAS_TEMP: 		return dallasTemp.stop(); break;
		#endif
		case SENSOR_SHT31_TEMP:
		case SENSOR_SHT31_HUM: 			return sht31.stop(); break;
		case SENSOR_SHT35_TEMP:
		case SENSOR_SHT35_HUM: 			return sht35.stop(); break;
		#ifndef MISC_DISABLE
		case SENSOR_RANGE_DISTANCE: 		return range.stop(); break;
		case SENSOR_RANGE_LIGHT: 		return range.stop(); break;
		case SENSOR_BME680_TEMPERATURE:		return bme680.stop(); break;
		case SENSOR_BME680_HUMIDITY:		return bme680.stop(); break;
		case SENSOR_BME680_PRESSURE:		return bme680.stop(); break;
		case SENSOR_BME680_VOCS:		return bme680.stop(); break;
		#endif
		case SENSOR_GPS_FIX_QUALITY:
		case SENSOR_GPS_LATITUDE:
		case SENSOR_GPS_LONGITUDE:
		case SENSOR_GPS_ALTITUDE:
		case SENSOR_GPS_SPEED:
		case SENSOR_GPS_HDOP:
		case SENSOR_GPS_SATNUM:			return gps.stop(base,this,wichSensor); break;
		case SENSOR_ADS1X15_48_0:
		case SENSOR_ADS1X15_48_1:
		case SENSOR_ADS1X15_48_2:
		case SENSOR_ADS1X15_48_3: 		return ads48.stop(); break;
		case SENSOR_ADS1X15_49_0:
		case SENSOR_ADS1X15_49_1:
		case SENSOR_ADS1X15_49_2:
		case SENSOR_ADS1X15_49_3: 		return ads49.stop(); break;
		case SENSOR_ADS1X15_4A_0:
		case SENSOR_ADS1X15_4A_1:
		case SENSOR_ADS1X15_4A_2:
		case SENSOR_ADS1X15_4A_3: 		return ads4A.stop(); break;
		case SENSOR_ADS1X15_4B_0:
		case SENSOR_ADS1X15_4B_1:
		case SENSOR_ADS1X15_4B_2:
		case SENSOR_ADS1X15_4B_3: 		return ads4B.stop(); break;
		#ifndef SCD30_DISABLE
		case SENSOR_SCD30_CO2: 			return scd30.stop(base,SENSOR_SCD30_CO2,this); break;
		case SENSOR_SCD30_TEMP: 		return scd30.stop(base,SENSOR_SCD30_TEMP,this); break;
		case SENSOR_SCD30_HUM: 			return scd30.stop(base,SENSOR_SCD30_HUM,this); break;
		#endif
		case SENSOR_SCD4x_CO2: 			return scd4x.stop(base,SENSOR_SCD4x_CO2,this); break;
		case SENSOR_SCD4x_TEMP: 		return scd4x.stop(base,SENSOR_SCD4x_TEMP,this); break;
		case SENSOR_SCD4x_HUM: 			return scd4x.stop(base,SENSOR_SCD4x_HUM,this); break;
		case SENSOR_WIND_DIR:			return windandrain.stop(base,this,SENSOR_WIND_DIR); break;
		case SENSOR_WIND_SPEED:			return windandrain.stop(base,this,SENSOR_WIND_SPEED); break;
		case SENSOR_RAIN_ACC:			return windandrain.stop(base,this,SENSOR_RAIN_ACC); break;
		case SENSOR_RAIN_EVENTACC:		return windandrain.stop(base,this,SENSOR_RAIN_EVENTACC); break;
		case SENSOR_RAIN_TOTALACC:		return windandrain.stop(base,this,SENSOR_RAIN_TOTALACC); break;
		case SENSOR_RAIN_INTERVAL:		return windandrain.stop(base,this,SENSOR_RAIN_INTERVAL); break;
		#ifdef ONE_OLED 
		case SENSOR_GROVE_OLED: 		return groove_OLED.stop(); break;
		#endif
		#ifdef TWO_OLED
		case SENSOR_GROVE_OLED: 		return groove_OLED.stop(); break;
		case SENSOR_GROVE_OLED2: 		return groove_OLED2.stop(); break;
		#endif
		case SENSOR_ENS160_ECO2:		return ens160.stop(); break;
		case SENSOR_ENS160_TVOC:		return ens160.stop(); break;
		case SENSOR_ENS160_AQI:		return ens160.stop(); break;
		case SENSOR_ENS210_TEMP:		return ens210.stop(); break;
		case SENSOR_ENS210_ABSTEMP:		return ens210.stop(); break;
		case SENSOR_ENS210_HUM:		return ens210.stop(); break;
		default: break;
	}
	
	return false;
}

void AuxBoards::getReading(SckBase* base, OneSensor *wichSensor)
{
	//sprintf(base->outBuff,"AUX is reading: %s",base->sensors[wichSensor->type].title);
	//base->sckOut(PRIO_MED,true);

	if (TCAMUXMODE) resetI2CMux();
	
	wichSensor->state = 0;
	switch (wichSensor->type) {
		#ifndef GASBOARD_DISABLE
		case SENSOR_GASESBOARD_SLOT_1A:	 	wichSensor->reading = String(gasBoard.getElectrode(gasBoard.Slot1.electrode_A)); return;
		case SENSOR_GASESBOARD_SLOT_1W: 	wichSensor->reading = String(gasBoard.getElectrode(gasBoard.Slot1.electrode_W)); return;
		case SENSOR_GASESBOARD_SLOT_2A: 	wichSensor->reading = String(gasBoard.getElectrode(gasBoard.Slot2.electrode_A)); return;
		case SENSOR_GASESBOARD_SLOT_2W: 	wichSensor->reading = String(gasBoard.getElectrode(gasBoard.Slot2.electrode_W)); return;
		case SENSOR_GASESBOARD_SLOT_3A: 	wichSensor->reading = String(gasBoard.getElectrode(gasBoard.Slot3.electrode_A)); return;
		case SENSOR_GASESBOARD_SLOT_3W: 	wichSensor->reading = String(gasBoard.getElectrode(gasBoard.Slot3.electrode_W)); return;
		case SENSOR_GASESBOARD_HUMIDITY: 	wichSensor->reading = String(gasBoard.getHumidity()); return;
		case SENSOR_GASESBOARD_TEMPERATURE: 	wichSensor->reading = String(gasBoard.getTemperature()); return;
		#endif
		case SENSOR_GROOVE_I2C_ADC: 		wichSensor->reading = String(grooveI2C_ADC.getReading(base,this,wichSensor->type)); return;
		case SENSOR_INA219_BUSVOLT: 		wichSensor->reading = String(ina219.getReading(base,this,wichSensor->type,ina219.BUS_VOLT)); return;
		case SENSOR_INA219_SHUNT: 		wichSensor->reading = String(ina219.getReading(base,this,wichSensor->type,ina219.SHUNT_VOLT)); return;
		case SENSOR_INA219_CURRENT: 		wichSensor->reading = String(ina219.getReading(base,this,wichSensor->type,ina219.CURRENT)); return;
		case SENSOR_INA219_LOADVOLT: 		wichSensor->reading = String(ina219.getReading(base,this,wichSensor->type,ina219.LOAD_VOLT)); return;
		#ifndef MISC_DISABLE
		case SENSOR_WATER_TEMP_DS18B20:		wichSensor->reading = String(waterTemp_DS18B20.getReading(base,this,wichSensor->type)); return;
		#endif
		#ifndef ATLAS_DISABLE
		case SENSOR_ATLAS_TEMPERATURE: 		if (atlasTEMP.getReading(base,this,wichSensor->type)) 	{ wichSensor->reading = String(atlasTEMP.newReading[0]); return; } break;
		case SENSOR_ATLAS_PH:			if (atlasPH.getReading(base,this,wichSensor->type)) 	{ wichSensor->reading = String(atlasPH.newReading[0]); return; } break;
		case SENSOR_ATLAS_EC:			if (atlasEC.getReading(base,this,wichSensor->type)) 	{ wichSensor->reading = String(atlasEC.newReading[0]); return; } break;
		case SENSOR_ATLAS_EC_TDS:		if (atlasEC.getReading(base,this,wichSensor->type)) 	{ wichSensor->reading = String(atlasEC.newReading[1]); return; } break;
		case SENSOR_ATLAS_EC_SAL:		if (atlasEC.getReading(base,this,wichSensor->type)) 	{ wichSensor->reading = String(atlasEC.newReading[2]); return; } break;
		case SENSOR_ATLAS_EC_SG:		if (atlasEC.getReading(base,this,wichSensor->type)) 	{ wichSensor->reading = String(atlasEC.newReading[3]); return; } break;
		case SENSOR_ATLAS_DO:			if (atlasDO.getReading(base,this,wichSensor->type)) 	{ wichSensor->reading = String(atlasDO.newReading[0]); return; } break;
		case SENSOR_ATLAS_DO_SAT:		if (atlasDO.getReading(base,this,wichSensor->type)) 	{ wichSensor->reading = String(atlasDO.newReading[1]); return; } break;
		#endif
		#ifndef CHIRP_DISABLE
		case SENSOR_CHIRP_MOISTURE_RAW:		if (moistureChirp.getReading(base,SENSOR_CHIRP_MOISTURE_RAW,this)) { wichSensor->reading = String(moistureChirp.raw); return; } break;
		case SENSOR_CHIRP_MOISTURE:		if (moistureChirp.getReading(base,SENSOR_CHIRP_MOISTURE,this)) { wichSensor->reading = String(moistureChirp.moisture); return; } break;
		case SENSOR_CHIRP_TEMPERATURE:		if (moistureChirp.getReading(base,SENSOR_CHIRP_TEMPERATURE,this)) { wichSensor->reading = String(moistureChirp.temperature); return; } break;
		case SENSOR_CHIRP_LIGHT:		if (moistureChirp.getReading(base,SENSOR_CHIRP_LIGHT,this)) { wichSensor->reading = String(moistureChirp.light); return; } break;
		#endif
		case SENSOR_EXT_A_PM_1: 		if (pmSensorA.update(base,this,wichSensor->type)) { wichSensor->reading = String(pmSensorA.pm1); return; } break;
		case SENSOR_EXT_A_PM_25: 		if (pmSensorA.update(base,this,wichSensor->type)) { wichSensor->reading = String(pmSensorA.pm25); return; } break;
		case SENSOR_EXT_A_PM_10: 		if (pmSensorA.update(base,this,wichSensor->type)) { wichSensor->reading = String(pmSensorA.pm10); return; } break;
		case SENSOR_EXT_A_PN_03: 		if (pmSensorA.update(base,this,wichSensor->type)) { wichSensor->reading = String(pmSensorA.pn03); return; } break;
		case SENSOR_EXT_A_PN_05: 		if (pmSensorA.update(base,this,wichSensor->type)) { wichSensor->reading = String(pmSensorA.pn05); return; } break;
		case SENSOR_EXT_A_PN_1: 		if (pmSensorA.update(base,this,wichSensor->type)) { wichSensor->reading = String(pmSensorA.pn1); return; } break;
		case SENSOR_EXT_A_PN_25: 		if (pmSensorA.update(base,this,wichSensor->type)) { wichSensor->reading = String(pmSensorA.pn25); return; } break;
		case SENSOR_EXT_A_PN_5: 		if (pmSensorA.update(base,this,wichSensor->type)) { wichSensor->reading = String(pmSensorA.pn5); return; } break;
		case SENSOR_EXT_A_PN_10:		if (pmSensorA.update(base,this,wichSensor->type)) { wichSensor->reading = String(pmSensorA.pn10); return; } break;
		case SENSOR_EXT_B_PM_1: 		if (pmSensorB.update(base,this,wichSensor->type)) { wichSensor->reading = String(pmSensorB.pm1); return; } break;
		case SENSOR_EXT_B_PM_25:                if (pmSensorB.update(base,this,wichSensor->type)) { wichSensor->reading = String(pmSensorB.pm25); return; } break;
		case SENSOR_EXT_B_PM_10:                if (pmSensorB.update(base,this,wichSensor->type)) { wichSensor->reading = String(pmSensorB.pm10); return; } break;
		case SENSOR_EXT_B_PN_03:                if (pmSensorB.update(base,this,wichSensor->type)) { wichSensor->reading = String(pmSensorB.pn03); return; } break;
		case SENSOR_EXT_B_PN_05:                if (pmSensorB.update(base,this,wichSensor->type)) { wichSensor->reading = String(pmSensorB.pn05); return; } break;
		case SENSOR_EXT_B_PN_1:                 if (pmSensorB.update(base,this,wichSensor->type)) { wichSensor->reading = String(pmSensorB.pn1); return; } break;
		case SENSOR_EXT_B_PN_25:                if (pmSensorB.update(base,this,wichSensor->type)) { wichSensor->reading = String(pmSensorB.pn25); return; } break;
		case SENSOR_EXT_B_PN_5:                 if (pmSensorB.update(base,this,wichSensor->type)) { wichSensor->reading = String(pmSensorB.pn5); return; } break;
		case SENSOR_EXT_B_PN_10: 		if (pmSensorB.update(base,this,wichSensor->type)) { wichSensor->reading = String(pmSensorB.pn10); return; } break;
		case SENSOR_PM_DALLAS_TEMP: 		wichSensor->reading = String(pmDallasTemp.getReading(base,this,wichSensor->type)); return;
		#ifndef MISC_DISABLE
		case SENSOR_DALLAS_TEMP: 		if (dallasTemp.getReading()) 			{ wichSensor->reading = String(dallasTemp.reading); return; } break;
		#endif
		case SENSOR_SHT31_TEMP: 		if (sht31.getReading()) 				{ wichSensor->reading = String(sht31.temperature); return; } break;
		case SENSOR_SHT31_HUM: 			if (sht31.getReading()) 				{ wichSensor->reading = String(sht31.humidity); return; } break;
		case SENSOR_SHT35_TEMP: 		if (sht35.getReading()) 				{ wichSensor->reading = String(sht35.temperature); return; } break;
		case SENSOR_SHT35_HUM: 			if (sht35.getReading()) 				{ wichSensor->reading = String(sht35.humidity); return; } break;
		#ifndef MISC_DISABLE
		case SENSOR_RANGE_DISTANCE: 		if (range.getReading(base,this,SENSOR_RANGE_DISTANCE)) 	{ wichSensor->reading = String(range.readingDistance); return; } break;
		case SENSOR_RANGE_LIGHT: 		if (range.getReading(base,this,SENSOR_RANGE_LIGHT)) 	{ wichSensor->reading = String(range.readingLight); return; } break;
		case SENSOR_BME680_TEMPERATURE:		if (bme680.getReading(base,this,wichSensor->type)) 			{ wichSensor->reading = String(bme680.temperature); return; } break;
		case SENSOR_BME680_HUMIDITY:		if (bme680.getReading(base,this,wichSensor->type)) 			{ wichSensor->reading = String(bme680.humidity); return; } break;
		case SENSOR_BME680_PRESSURE:		if (bme680.getReading(base,this,wichSensor->type)) 			{ wichSensor->reading = String(bme680.pressure); return; } break;
		case SENSOR_BME680_VOCS:		if (bme680.getReading(base,this,wichSensor->type)) 			{ wichSensor->reading = String(bme680.VOCgas); return; } break;
		#endif
		case SENSOR_GPS_FIX_QUALITY: 		if (gps.getReading(base,this, SENSOR_GPS_FIX_QUALITY)) 	{ wichSensor->reading = String(gps.r.fixQuality); return; } break;
		case SENSOR_GPS_LATITUDE: 		if (gps.getReading(base, this,SENSOR_GPS_LATITUDE)) 		{ wichSensor->reading = String(gps.r.latitude, 6); return; } break;
		case SENSOR_GPS_LONGITUDE: 		if (gps.getReading(base, this,SENSOR_GPS_LONGITUDE)) 	{ wichSensor->reading = String(gps.r.longitude, 6); return; } break;
		case SENSOR_GPS_ALTITUDE: 		if (gps.getReading(base, this,SENSOR_GPS_ALTITUDE)) 		{ wichSensor->reading = String(gps.r.altitude, 2); return; } break;
		case SENSOR_GPS_SPEED: 			if (gps.getReading(base, this,SENSOR_GPS_SPEED)) 		{ wichSensor->reading = String(gps.r.speed, 2); return; } break;
		case SENSOR_GPS_HDOP: 			if (gps.getReading(base, this,SENSOR_GPS_HDOP) )		{ wichSensor->reading = String(gps.r.hdop, 2); return; } break;
		case SENSOR_GPS_SATNUM:			if (gps.getReading(base, this,SENSOR_GPS_SATNUM)) 		{ wichSensor->reading = String(gps.r.satellites); return; } break;
		case SENSOR_ADS1X15_48_0: 		if (ads48.getReading(base,this,0,wichSensor->type)) 			{ wichSensor->reading = String(ads48.reading, 6); return;} break;
		case SENSOR_ADS1X15_48_1: 		if (ads48.getReading(base,this,1,wichSensor->type)) 			{ wichSensor->reading = String(ads48.reading, 6); return;} break;
		case SENSOR_ADS1X15_48_2: 		if (ads48.getReading(base,this,2,wichSensor->type)) 			{ wichSensor->reading = String(ads48.reading, 6); return;} break;
		case SENSOR_ADS1X15_48_3: 		if (ads48.getReading(base,this,3,wichSensor->type)) 			{ wichSensor->reading = String(ads48.reading, 6); return;} break;
		case SENSOR_ADS1X15_49_0: 		if (ads49.getReading(base,this,0,wichSensor->type)) 			{ wichSensor->reading = String(ads49.reading, 6); return;} break;
		case SENSOR_ADS1X15_49_1: 		if (ads49.getReading(base,this,1,wichSensor->type)) 			{ wichSensor->reading = String(ads49.reading, 6); return;} break;
		case SENSOR_ADS1X15_49_2: 		if (ads49.getReading(base,this,2,wichSensor->type)) 			{ wichSensor->reading = String(ads49.reading, 6); return;} break;
		case SENSOR_ADS1X15_49_3: 		if (ads49.getReading(base,this,3,wichSensor->type)) 			{ wichSensor->reading = String(ads49.reading, 6); return;} break;
		case SENSOR_ADS1X15_4A_0: 		if (ads4A.getReading(base,this,0,wichSensor->type)) 			{ wichSensor->reading = String(ads4A.reading, 6); return;} break;
		case SENSOR_ADS1X15_4A_1: 		if (ads4A.getReading(base,this,1,wichSensor->type)) 			{ wichSensor->reading = String(ads4A.reading, 6); return;} break;
		case SENSOR_ADS1X15_4A_2: 		if (ads4A.getReading(base,this,2,wichSensor->type)) 			{ wichSensor->reading = String(ads4A.reading, 6); return;} break;
		case SENSOR_ADS1X15_4A_3: 		if (ads4A.getReading(base,this,3,wichSensor->type)) 			{ wichSensor->reading = String(ads4A.reading, 6); return;} break;
		case SENSOR_ADS1X15_4B_0: 		if (ads4B.getReading(base,this,0,wichSensor->type)) 			{ wichSensor->reading = String(ads4B.reading, 6); return;} break;
		case SENSOR_ADS1X15_4B_1: 		if (ads4B.getReading(base,this,1,wichSensor->type)) 			{ wichSensor->reading = String(ads4B.reading, 6); return;} break;
		case SENSOR_ADS1X15_4B_2: 		if (ads4B.getReading(base,this,2,wichSensor->type)) 			{ wichSensor->reading = String(ads4B.reading, 6); return;} break;
		case SENSOR_ADS1X15_4B_3: 		if (ads4B.getReading(base,this,3,wichSensor->type)) 			{ wichSensor->reading = String(ads4B.reading, 6); return;} break;
		#ifndef SCD30_DISABLE		
		case SENSOR_SCD30_CO2: 			if (scd30.getReading(base,SENSOR_SCD30_CO2,this)) 				{ wichSensor->reading = String(scd30.co2); return; } break;
		case SENSOR_SCD30_TEMP: 		if (scd30.getReading(base,SENSOR_SCD30_TEMP,this)) 				{ wichSensor->reading = String(scd30.temperature); return; } break;
		case SENSOR_SCD30_HUM: 			if (scd30.getReading(base,SENSOR_SCD30_HUM,this)) 				{ wichSensor->reading = String(scd30.humidity); return; } break;
		#endif
		case SENSOR_SCD4x_CO2: 			if ( scd4x.getReading(base,SENSOR_SCD4x_CO2,this)) 				{ wichSensor->reading = String(scd4x.co2); return; } break;
		case SENSOR_SCD4x_HUM: 			if ( scd4x.getReading(base,SENSOR_SCD4x_HUM,this)) 				{ wichSensor->reading = String(scd4x.humidity); return; } break;
		case SENSOR_SCD4x_TEMP: 		if ( scd4x.getReading(base,SENSOR_SCD4x_TEMP,this)) 			{ wichSensor->reading = String(scd4x.temperature); return; } break;
		case SENSOR_WIND_DIR:			if ( windandrain.update(base,this,SENSOR_WIND_DIR))				{ wichSensor->reading = windandrain.windDir;return; } break;
		case SENSOR_WIND_SPEED:			if ( windandrain.update(base,this,SENSOR_WIND_SPEED))			{ wichSensor->reading = windandrain.windSpeed; return; } break;
		case SENSOR_RAIN_ACC:			if ( windandrain.update(base,this,SENSOR_RAIN_ACC))				{ wichSensor->reading = windandrain.rainAcc; return; } break;
		case SENSOR_RAIN_EVENTACC:		if ( windandrain.update(base,this,SENSOR_RAIN_EVENTACC))		{ wichSensor->reading = windandrain.rainEventAcc; return; } break;
		case SENSOR_RAIN_TOTALACC:		if ( windandrain.update(base,this,SENSOR_RAIN_TOTALACC))		{ wichSensor->reading = windandrain.rainTotalAcc; return; } break;
		case SENSOR_RAIN_INTERVAL:		if ( windandrain.update(base,this,SENSOR_RAIN_INTERVAL))		{ wichSensor->reading = windandrain.rainIntAcc; return; } break;
		case SENSOR_ENS160_ECO2:		if ( ens160.getReading(base,this,SENSOR_ENS160_ECO2)) 			{ wichSensor->reading = String(ens160.eCO2); return; } break;
		case SENSOR_ENS160_TVOC:		if ( ens160.getReading(base,this,SENSOR_ENS160_TVOC)) 			{ wichSensor->reading = String(ens160.tVOC); return; } break;
		case SENSOR_ENS160_AQI:			if ( ens160.getReading(base,this,SENSOR_ENS160_AQI)) 			{ wichSensor->reading = String(ens160.AQI); return; } break;
		case SENSOR_ENS210_TEMP:		if ( ens210.getReading(base,this,SENSOR_ENS210_TEMP)) 			{ wichSensor->reading = String(ens210.temp); return; } break;
		case SENSOR_ENS210_ABSTEMP:		if ( ens210.getReading(base,this,SENSOR_ENS210_ABSTEMP)) 		{ wichSensor->reading = String(ens210.abstemp); return; } break;
		case SENSOR_ENS210_HUM:			if ( ens210.getReading(base,this,SENSOR_ENS210_HUM)) 			{ wichSensor->reading = String(ens210.hum); return; } break;

		default: break;
	}
	//sprintf(base->outBuff,"AUX did not read from: %s",base->sensors[wichSensor->type].title);
	//base->sckOut(PRIO_MED,true);
	wichSensor->reading = "null";
	wichSensor->state = -1;
}

bool AuxBoards::getBusyState(SckBase* base,SensorType wichSensor)
{

	switch(wichSensor) {
		#ifndef ATLAS_DISABLE
		case SENSOR_ATLAS_TEMPERATURE:  return atlasTEMP.getBusyState(base, this,wichSensor); break;
		case SENSOR_ATLAS_PH: 		return atlasPH.getBusyState(base,this,wichSensor); break;
		case SENSOR_ATLAS_EC:
		case SENSOR_ATLAS_EC_TDS:
		case SENSOR_ATLAS_EC_SAL:
		case SENSOR_ATLAS_EC_SG: 	return atlasEC.getBusyState(base,this,wichSensor); break;
		case SENSOR_ATLAS_DO:
		case SENSOR_ATLAS_DO_SAT: 	return atlasDO.getBusyState(base,this,wichSensor); break;
		#endif
		default: return false; break;
	}
}

String AuxBoards::control(SckBase* base,SensorType wichSensor, String command)
{
	switch(wichSensor) {
		#ifndef GASBOARD_DISABLE
		case SENSOR_GASESBOARD_SLOT_1A:
		case SENSOR_GASESBOARD_SLOT_1W:
		case SENSOR_GASESBOARD_SLOT_2A:
		case SENSOR_GASESBOARD_SLOT_2W:
		case SENSOR_GASESBOARD_SLOT_3A:
		case SENSOR_GASESBOARD_SLOT_3W: {

			if (command.startsWith("set pot")) {

				Electrode wichElectrode;

				switch(wichSensor) {
					case SENSOR_GASESBOARD_SLOT_1A: wichElectrode = gasBoard.Slot1.electrode_A;
					case SENSOR_GASESBOARD_SLOT_1W: wichElectrode = gasBoard.Slot1.electrode_W;
					case SENSOR_GASESBOARD_SLOT_2A: wichElectrode = gasBoard.Slot2.electrode_A;
					case SENSOR_GASESBOARD_SLOT_2W: wichElectrode = gasBoard.Slot2.electrode_W;
					case SENSOR_GASESBOARD_SLOT_3A: wichElectrode = gasBoard.Slot3.electrode_A;
					case SENSOR_GASESBOARD_SLOT_3W: wichElectrode = gasBoard.Slot3.electrode_W;
					default: break;
				}

				command.replace("set pot", "");
				command.trim();
				int wichValue = command.toInt();
				gasBoard.setPot(wichElectrode, wichValue);
				return String F("Setting pot to: ") + String(wichValue) + F(" Ohms\n\rActual value: ") + String(gasBoard.getPot(wichElectrode)) + F(" Ohms");

			#ifdef gasesBoardTest
			} else if (command.startsWith("test")) {
				command.replace("test", "");
				command.trim();

				// Get slot
				String slotSTR = String(command.charAt(0));
				uint8_t wichSlot = slotSTR.toInt();

				command.remove(0,1);
				command.trim();

				if (command.startsWith("set")) {

					command.replace("set", "");
					command.trim();

					// Get value
					int wichValue = command.toInt();
					gasBoard.setTesterCurrent(wichValue, wichSlot);

				} else if (command.startsWith("full")) {

					gasBoard.runTester(wichSlot);

				} else {
					return F("Unrecognized test command!!\r\nOptions:\r\ntest slot set value (slot: 1-3, value:-1400/+1400 nA)\r\ntest slot full (test the full cycle on slot (1-3))");
				}

				return F("\nTesting finished!");

			} else if (command.startsWith("autotest")) {

				return String(gasBoard.autoTest());
			#endif

			} else if (command.startsWith("help")) {
				return F("Available commands for this sensor:\n\r* set pot ");

			} else {
				return F("Unrecognized command!! please try again...");
			}

			break;

		} 
		#endif
		#ifndef ATLAS_DISABLE
		case SENSOR_ATLAS_PH:
		case SENSOR_ATLAS_EC:
		case SENSOR_ATLAS_EC_TDS:
		case SENSOR_ATLAS_EC_SAL:
		case SENSOR_ATLAS_EC_SG:
		case SENSOR_ATLAS_DO:
		case SENSOR_ATLAS_DO_SAT: {

			Atlas *thisAtlas = &atlasPH;
			if (wichSensor == SENSOR_ATLAS_EC || wichSensor == SENSOR_ATLAS_EC_SG) thisAtlas = &atlasEC;
			else if (wichSensor == SENSOR_ATLAS_DO || wichSensor == SENSOR_ATLAS_DO_SAT) thisAtlas = &atlasDO;

			// 	 Calibration command options:
			// 		Atlas PH: (https://www.atlas-scientific.com/files/pH_EZO_Datasheet.pdf) page 52
			// 			* com cal,mid,7
			// 			* com cal,low,4
			// 			* com cal,high,10
			// 			* com cal,clear
			// 			* com cal,?
			// 		Atlas EC: (https://www.atlas-scientific.com/_files/_datasheets/_circuit/EC_EZO_Datasheet.pdf) page 55
			// 			* com cal,dry
			// 			* com cal,low,12880
			// 			* com cal,high,80000
			// 			* com cal,clear
			// 			* com cal,?
			// 		Atlas DO: (https://www.atlas-scientific.com/_files/_datasheets/_circuit/DO_EZO_Datasheet.pdf) page 52
			// 			* com cal
			// 			* com cal,0
			// 			* com cal,clear
			// 			* com cal,?
			if (command.startsWith("com")) {

				command.replace("com", "");
				command.trim();
				thisAtlas->sendCommand(base,(char*)command.c_str(),this,wichSensor);

				uint8_t responseCode = thisAtlas->getResponse(base,this,wichSensor);
				if (responseCode == 254) {
					delay(1000);
					responseCode = thisAtlas->getResponse(base,this,wichSensor);
				}
				if (responseCode == 1) return thisAtlas->atlasResponse;
				else return String(responseCode);

			}
			break;

		}
		#endif 
		#ifndef CHIRP_DISABLE
		case SENSOR_CHIRP_MOISTURE_RAW:
		case SENSOR_CHIRP_MOISTURE:
		case SENSOR_CHIRP_TEMPERATURE:
		case SENSOR_CHIRP_LIGHT: {
			if (command.startsWith("get ver")) {

				return String(moistureChirp.getVersion(base,this,wichSensor));

			} else if (command.startsWith("reset")) {

				for(uint8_t address = 1; address < 127; address++ ) {
					if (openChannel,base,address,moistureChirp.localPortNum,true)) {
						auxWire.beginTransmission(address);

						if (auxWire.endTransmission() == 0) {
							if (moistureChirp.resetAddress(base,address,this,wichSensor)) return F("Changed chirp address to default 0x20");
						}
					}
				}
				return F("Can't find any chirp sensor...");

			} else if (command.startsWith("cal")) {

				command.replace("cal", "");
				command.trim();

				int space_index = command.indexOf(" ");

				if (space_index > 0) {

					String preDry = command.substring(0, space_index);
					String preWet = command.substring(space_index + 1);

					int32_t dryInt = preDry.toInt();
					int32_t wetInt = preWet.toInt();

					if ((dryInt == 0 && preDry != "0") || (wetInt == 0 && preWet != "0")) return F("Error reading values, please try again!");

					moistureChirp.dryPoint = data.calibration.dryPoint = dryInt;
					moistureChirp.wetPoint = data.calibration.wetPoint = wetInt;
					data.calibration.moistureCalDataValid = true;
					moistureChirp.calibrated = true;

					data.valid = true;
					eepromAuxData.write(data);
				}

				String response;
				if (moistureChirp.calibrated) {
					response += "Dry point: " + String(moistureChirp.dryPoint) + "\r\nWet point: " + String(moistureChirp.wetPoint);
				} else {
					response = F("Moisture sensor is NOT calibrated");
				}
				return response;


			} else if (command.startsWith("help") || command.length() == 0) return F("Available commands:\r\n* get ver\r\n* reset (connect only the chirp to aux)\r\n* cal dryPoint wetPoint");
			else return F("Unrecognized command!! please try again...");
			break;

		}
		#endif 
		case SENSOR_ADS1X15_48_0:
		case SENSOR_ADS1X15_48_1:
		case SENSOR_ADS1X15_48_2:
		case SENSOR_ADS1X15_48_3: {
#ifdef adsTest
			if (command.startsWith("test")) {
				command.replace("test", "");
				command.trim();

				// Get channels
				String channelSTR = String(command.charAt(0));
				uint8_t wichChannel = channelSTR.toInt();

				command.remove(0,1);
				command.trim();

				if (command.startsWith("set")) {

					command.replace("set", "");
					command.trim();

					// Get value
					int wichValue = command.toInt();
					ads48.setTesterCurrent(wichValue, wichChannel);

				} else if (command.startsWith("full")) {

					ads48.runTester(wichChannel);

				} else {

					return F("Unrecognized test command!!\r\nOptions:\r\ntest set value (-1400/+1400 nA)\r\ntest slot full (test the full cycle)");
				}

				return F("\nCurrent set!");
			}
#endif
			break;
		} case SENSOR_ADS1X15_49_0:
		case SENSOR_ADS1X15_49_1:
		case SENSOR_ADS1X15_49_2:
		case SENSOR_ADS1X15_49_3: {
#ifdef adsTest
			if (command.startsWith("test")) {
				command.replace("test", "");
				command.trim();

				// Get channels
				String channelSTR = String(command.charAt(0));
				uint8_t wichChannel = channelSTR.toInt();

				command.remove(0,1);
				command.trim();

				if (command.startsWith("set")) {

					command.replace("set", "");
					command.trim();

					// Get value
					int wichValue = command.toInt();
					ads49.setTesterCurrent(wichValue, wichChannel);

				} else if (command.startsWith("full")) {

					ads49.runTester(wichChannel);

				} else {

					return F("Unrecognized test command!!\r\nOptions:\r\ntest set value (-1400/+1400 nA)\r\ntest slot full (test the full cycle)");
				}

				return F("\nCurrent set!");
			}
#endif
			break;
		} case SENSOR_ADS1X15_4A_0:
		case SENSOR_ADS1X15_4A_1:
		case SENSOR_ADS1X15_4A_2:
		case SENSOR_ADS1X15_4A_3: {
#ifdef adsTest
			if (command.startsWith("test")) {
				command.replace("test", "");
				command.trim();

				// Get channels
				String channelSTR = String(command.charAt(0));
				uint8_t wichChannel = channelSTR.toInt();

				command.remove(0,1);
				command.trim();

				if (command.startsWith("set")) {

					command.replace("set", "");
					command.trim();

					// Get value
					int wichValue = command.toInt();
					ads4A.setTesterCurrent(wichValue, wichChannel);

				} else if (command.startsWith("full")) {

					ads4A.runTester(wichChannel);

				} else {

					return F("Unrecognized test command!!\r\nOptions:\r\ntest set value (-1400/+1400 nA)\r\ntest slot full (test the full cycle)");
				}

				return F("\nCurrent set!");
			}
#endif
			break;
		} case SENSOR_ADS1X15_4B_0:
		case SENSOR_ADS1X15_4B_1:
		case SENSOR_ADS1X15_4B_2:
		case SENSOR_ADS1X15_4B_3: {
#ifdef adsTest
			if (command.startsWith("test")) {
				command.replace("test", "");
				command.trim();

				// Get channels
				String channelSTR = String(command.charAt(0));
				uint8_t wichChannel = channelSTR.toInt();

				command.remove(0,1);
				command.trim();

				if (command.startsWith("set")) {

					command.replace("set", "");
					command.trim();

					// Get value
					int wichValue = command.toInt();
					ads4B.setTesterCurrent(wichValue, wichChannel);

				} else if (command.startsWith("full")) {

					ads4B.runTester(wichChannel);

				} else {

					return F("Unrecognized test command!!\r\nOptions:\r\ntest set value (-1400/+1400 nA)\r\ntest slot full (test the full cycle)");
				}

				return F("\nCurrent set!");
			}
#endif
			break;
		}
		#ifndef SCD30_DISABLE
		case SENSOR_SCD30_CO2:
		case SENSOR_SCD30_TEMP:
		case SENSOR_SCD30_HUM: {

			if (command.startsWith("interval")) {

				command.replace("interval", "");
				command.trim();

				uint16_t newInterval = command.toInt();
				scd30.interval(base,this,wichSensor,newInterval);

				return String F("Measuring Interval: ") + String(scd30.interval(base,this,wichSensor));

			} else if (command.startsWith("autocal")) {

				command.replace("autocal", "");
				command.trim();

				if (command.startsWith("on")) scd30.autoSelfCal(base,this,wichSensor,1);
				else if (command.startsWith("off")) scd30.autoSelfCal(base,this,wichSensor,0);

				return String F("Auto Self Calibration: ") + String(scd30.autoSelfCal(base,this,wichSensor) ? "on" : "off");

			} else if (command.startsWith("calfactor")) {

				command.replace("calfactor", "");
				command.trim();

				uint16_t newFactor = command.toInt();

				return String F("Forced Recalibration Factor: ") + String(scd30.forcedRecalFactor(base,this,wichSensor,newFactor));

			} else if (command.startsWith("caltemp")) {

				command.replace("caltemp", "");
				command.trim();

				float userTemp = 0.000001;
				bool off = false;

				if (command.startsWith("off")) off = true;
				else {

					if (command.length() > 0) userTemp = command.toFloat();
				}

				scd30.getReading(base,SENSOR_SCD30_TEMP,this);

				return String F("Current temperature: ") + String(scd30.temperature) + F(" C") + F("\r\nTemperature offset: ") + String(scd30.tempOffset(base,this,wichSensor,userTemp, off)) + F(" C");
			} else if (command.startsWith("pressure")) {

				return String F("Pressure compensation on last boot: ") + String(scd30.pressureCompensated ? "True" : "False");

			} else {
				return F("Wrong command!!\r\nOptions:\r\ninterval [2-1000 (seconds)]\r\nautocal [on/off]\r\ncalfactor [400-2000 (ppm)]\r\ncaltemp [newTemp/off]\r\npressure");
			}


		} 
		#endif
		case SENSOR_SCD4x_CO2:
		case SENSOR_SCD4x_TEMP:
		case SENSOR_SCD4x_HUM: {
			if (command.startsWith("interval")) {
			
				

				command.replace("interval", "");
				command.trim();

				//return F("Interval Command is not supported by this sensor");

				uint16_t newInterval = command.toInt();
				scd4x.interval( base,this,wichSensor, newInterval);

				return String F("Measuring Interval: ") + String(scd4x.interval(base,this,wichSensor,newInterval));
				
				
			} else if (command.startsWith("autocal")) {

				command.replace("autocal", "");
				command.trim();

				if (command.startsWith("on")) scd4x.autoSelfCal(base,this,wichSensor,1);
				else if (command.startsWith("off")) scd4x.autoSelfCal(base,this,wichSensor,0);

				return String F("Auto Self Calibration: ") + String(scd4x.autoSelfCal(base,this,wichSensor) ? "on" : "off");

			} else if (command.startsWith("calfactor")) {

				command.replace("calfactor", "");
				command.trim();

				uint16_t newFactor = command.toInt();

				return String F("Forced Recalibration Factor: ") + String(scd4x.forcedRecalFactor(base,this,wichSensor,newFactor));

			} else if (command.startsWith("caltemp")) {

				command.replace("caltemp", "");
				command.trim();

				float userTemp = 0.000001;
				bool off = false;

				if (command.startsWith("off")) off = true;
				else {

					if (command.length() > 0) userTemp = command.toFloat();
				}

				scd4x.getReading(base, SENSOR_SCD4x_TEMP,this);

				return String F("Current temperature: ") + String(scd4x.temperature) + F(" C") + F("\r\nTemperature offset: ") + String(scd4x.tempOffset(base,this,wichSensor, userTemp, off)) + F(" C");

			} else if (command.startsWith("pressure")) {
				
				command.replace("pressure", "");
				command.trim();

				bool off = false;

				if (command.startsWith("off")) off = true;  // the command string: pressure off / pressure on; 
				
				scd4x.pressureCompensated=scd4x.setPressureComp(base,off);  // boolean turns pressure comp on or off (we ignore the return boolean)

				return String F("Pressure compensation : ") + String(scd4x.pressureCompensated ? "True" : "False");

			} else {
				return F("Wrong command!!\r\nOptions:\r\ninterval [5,30-1000 (seconds)]\r\nautocal [on/off]\r\ncalfactor [400-2000 (ppm)]\r\ncaltemp [newTemp/off]\r\npressure [on/off] ");
			}


		} default: return "Unrecognized sensor!!!"; break;
	}
	return "Unknown error on control command!!!";
}

void AuxBoards::print(SckBase* base,SensorType wichSensor,char *payload)
{

	groove_OLED.print(base,this,wichSensor,payload);
}

void AuxBoards::updateDisplay(SckBase* base,SensorType wichSensor,bool force)
{
	#ifdef ONE_OLED
	if (base->sensors[SENSOR_GROVE_OLED].enabled) {
		groove_OLED.update(base,this,wichSensor,force);
	}
	#endif
	#ifdef TWO_OLED
	if (base->sensors[SENSOR_GROVE_OLED2].enabled) {
		switch (currentDisplay) {
			case 1: {
				
				groove_OLED.update(base,this,SENSOR_GROVE_OLED,force);
				if (base->sensors[SENSOR_GROVE_OLED2].enabled) currentDisplay=2;
				break;
			}
			case 2: {
				
				groove_OLED2.update(base,this,SENSOR_GROVE_OLED2,force);
				if (base->sensors[SENSOR_GROVE_OLED].enabled) currentDisplay=1;
				
				break;
			}
			default: {
				break;
			}
		}
	} else {
		groove_OLED.update(base,this,wichSensor,force);
	}
	#endif
	
}
bool AuxBoards::updateGPS(SckBase* base,SensorType wichSensor)
{
	return gps.update(base,this,wichSensor);
}

void AuxBoards::plot(SckBase* base,SensorType wichSensor,String value, const char *title, const char *unit)
{
	if (title != NULL && unit != NULL) groove_OLED.plot(base,this,wichSensor,value, title, unit);
	else groove_OLED.plot(base,this,wichSensor,value);
}

uint8_t AuxBoards::restartTCAMux() {

	/*
	NOTES: There may be issues with the TCA Mux; if it happens that the I2C bus is busy and the 
	channel switching request does not get heard;
	*/
	TCA.begin(auxWire,muxAddress.b);
	delayMicroseconds(20);
	TCA.closeAll();
	delayMicroseconds(20);
	return TCA.readStatus();		// should return 0 = all channels are closed

}
// this  performs a lookup of the  address in the v_I2CMuxChannels array; 
// this info is discovered only once; (Faster) and low overhead.
// although there is an argument that it ought to be refreshed when i2c command is run from the UI Command line
// NOTE: this function assumes that a specified address is found on only one channel.
// if there are multiple instances it will return the first one found
uint8_t AuxBoards::findDeviceChan(SckBase* base,byte address, SensorType wichSensor,bool openChan,bool exclusive) {
	uint32_t iteration=0;
	uint8_t i;
	uint8_t currchan;
	//sprintf(base->outBuff,"Finding a Channel for %s device",base->sensors[wichSensor].title);
	//base->sckOut(PRIO_LOW,true);
	if (address != 0x00) {
		for (i = 0; i < I2CMUX_MAX_SENSORS; i++) {	// i is a pointer to vector members
			if (iteration >= 15) break;					// for some reason we have seen this loop exceeding the iteration upper bound. Memory corruption ?
			iteration++;
			I2C_MuxChannel mc=v_I2CMuxChannels[i];
			if (mc.addr != 0x00) {
				if (mc.addr == address) {
					v_I2CMuxChannels[i].type=wichSensor;
					currchan=TCA.readStatus();		// bcd hex address of channel
					delay(1);
					if (openChan) {
						if (currchan != mc.chan) {
							if (exclusive) {
								TCA.closeAll();
								delayMicroseconds(100);
								TCA.openChannel(mc.chan);
							} else {
								TCA.openChannel(mc.chan);
							}
							delay(1);
						}
					}
					byte fred=mc.chan;
					return fred;
				}
			}
		}
	}
	return (TCA_CHANNEL_7+1);
}


uint8_t AuxBoards::tcaDiscoverAMux(SckBase* base,TwoWire *_Wire) {
	uint8_t address;
	for( address = 0x70; address < 0x78; address++ ) {	
		if (!I2Cdetect(_Wire, address)) {
			sprintf(base->outBuff, "An I2C Mux was NOT discovered at address (0x%02x) ", address);
			base->sckOut(PRIO_MED,true);
		} else {
			sprintf(base->outBuff, "An I2C Mux was discovered at address (0x%02x) ", address);
			base->sckOut(PRIO_MED,true);
			return address;
		}
	}
	//base->sckOut("An I2C Mux was NOT discovered",PRIO_LOW,true);
	// if nothing found
	return 0;
}
bool AuxBoards::buildMuxChannelMap(SckBase* base,TwoWire *_Wire) {
	/*
	Build a map of I2c addresses that are discovered connected to I2c Mux ports/channels.
	This consists of a 4 x N array; one entry for each I2C address that was found during the scan.
	Steps:
	1. Discover Address of the MUX (only one allowed on the Aux Bus, but it could be one of 8 possible addresses)
	*/
	
	uint8_t address;
	uint8_t chan;
	bool muxMode=false;
	//uint8_t status=0;
	byte muxchan;
	AUXI2C_MUXCONNECTED_DEVICES=0;
	// the mux is discovered first; it must be the very first entry in the map; so it is more easily found if needed.
	if (muxAddress.i != 0) {
		//base->sckOut("Building the map", PRIO_MED,true);
		// the MUX address is recorded as the first entry in the map, and it is connected direct to the AuxBux (channel 0).
		v_I2CMuxChannels[0].addr=muxAddress.b;
		v_I2CMuxChannels[0].chan=0x00;
		AUXI2C_MUXCONNECTED_DEVICES++;
		muxMode=true;	

		// 2. loop through each port and scan for I2C addresses connected to that port
		// add each address and channel as a pair (using I2C_MuxChannel struct ) to the vector array
		//base->sckOut("looping thru the channels", PRIO_LOW,true);
		
		// when this command is issued normally after a power cycle.  
		// I do not know how to differentiate between (a) power cycle startup 
		// and (b) reset button// (c) reset UI command startup
		// If it is after a power cycle; then Some devices need a delay under such conditions
		// to finish setup() and enable the wire port (normally found at the end of setup())
		// a 1 second delay is not a huge price to pay to allow all the sensors to be brought on line.
		delay(1000);
		uint8_t i=1;	// v_I2CMuxChannels array index selector
		for (chan=1; chan < 9; chan++) {		//  NOT checking first entry in the channel array (this is the raw AuxBus)
			muxchan=channelAry[chan];
			TCA.closeAll();
			delayMicroseconds(10);
			TCA.openChannel(muxchan);		// open the channel
			delayMicroseconds(10);
			//sprintf(base->outBuff,"Checking Port: 0x%02x status: 0x%02x",muxchan,TCA.readStatus());
			//base->sckOut(PRIO_LOW,true);
			for (address=0x01; address<=0x7f; address++) {
				if (address != muxAddress.i) {
					if (I2Cdetect(_Wire, address) && checkMuxMapEntryUnique(address)) {
						//sprintf(base->outBuff,"Adding addr 0x%02x to chan 0x%02x",address,chan);
						//base->sckOut(PRIO_MED,true);
						v_I2CMuxChannels[i].addr=address;
						v_I2CMuxChannels[i].chan=muxchan;
						AUXI2C_MUXCONNECTED_DEVICES++;
						i++;
					}
				}
			}
		}
		sprintf(base->outBuff, "The I2C Mux channel map has (%u) entries", AUXI2C_MUXCONNECTED_DEVICES);
		base->sckOut(PRIO_MED,true);
	}
	return muxMode;
}
bool AuxBoards::checkMuxMapEntryUnique(uint8_t address){
	for (uint8_t j=0;j< I2CMUX_MAX_SENSORS; j++){
		if (v_I2CMuxChannels[j].addr == address) {
			return false;
		}
	}
	return true;
}
uint8_t AuxBoards::listMuxChanMap(SckBase* base) {
	
	base->sckOut("::List of Device Addresses found on the AuxBus I2c MUX::\r\n",PRIO_MED,true);

	testMuxChanMap(base);
	
	I2C_MuxChannel mc;
	uint8_t ctr=0; 
	for (uint8_t i = 0; i < I2CMUX_MAX_SENSORS; i++) {	// i is a index to array members
		mc=v_I2CMuxChannels[i];
		if (mc.addr !=0x00) {
			ctr++;
			//sprintf(base->outBuff, "address: 0x%02x found on MUX Port: 0x%02x",mc.addr,mc.chan );
			//base->sckOut(PRIO_MED,true);
		}
	}
	return ctr;
}

void AuxBoards::testMuxChanMap(SckBase* base) {
	
	base->sckOut("::testing connectivity to each Address found on the AuxBus I2c MUX::\r\n",PRIO_MED,true);
	I2C_MuxChannel mc;
	//byte chan=0x00;
	byte status=TCA.readStatus();
	auxWire.endTransmission();
	auxWire.setClock(100000);

	uint8_t ctr=0; 
	uint8_t i = 0;
	for ( i = 0; i < I2CMUX_MAX_SENSORS; i++) {	// i is a index to array members
		mc=v_I2CMuxChannels[i];
		if (mc.addr !=0x00) {
			ctr++;
			sprintf(base->outBuff, "Testing address: 0x%02x on MUX Port: 0x%02x",mc.addr,mc.chan );
			base->sckOut(PRIO_MED,false);
			status=TCA.readStatus();
			if (status != mc.chan) {
				TCA.closeAll();
			}
			TCA.openChannel(mc.chan);
			auxWire.beginTransmission(mc.addr);
			byte error = auxWire.endTransmission();
			if (error == 0) {
				base->sckOut(":: OK",PRIO_MED,true);
			} else {
				base->sckOut(":: NOT OK",PRIO_MED,true);
			}
		}
	}
	return;
}
void AuxBoards::resetI2CMux() {
	
	auxWire.endTransmission();
	auxWire.setClock(100000);
	TCA.closeAll();

	return;

}
uint8_t AuxBoards::countTCAOpenChannels(SckBase* base) {
	// count the number of mux channels that are open
	byte status=0x0f;
	uint8_t chanidx=0;
	byte muxchan=0x00;
	byte test=0x00;
	uint8_t openchancount=0;

	status=TCA.readStatus();
	if (status != 0x0f) {
		for (chanidx=1; chanidx < 9; chanidx++) {	
			muxchan=channelAry[chanidx];
			test= status & muxchan;
			if (test > 0x00) {			// that channel is open
				//sprintf(base->outBuff,"Channel 0x%02x is open",muxchan);
				//base->sckOut(PRIO_MED,true);
				openchancount++;
			}
		}
	} else {
		openchancount=8;
	}
	//sprintf(base->outBuff,"There are %i open channels",openchancount);
	//base->sckOut(PRIO_MED,true);
	return (openchancount);

}
I2C_MuxChannel AuxBoards::getMuxChannel(uint8_t address) {
	muxChannel mc=v_I2CMuxChannels[I2CMUX_MAX_SENSORS-1];
	for (uint8_t i = 0; i < I2CMUX_MAX_SENSORS; i++) {	// i is a pointer to vector members
		mc=v_I2CMuxChannels[i];
		if (mc.addr == address) {
			return mc;
		}
	}
	return mc;
}
bool AuxBoards::openChannel(SckBase* base,uint8_t address, uint8_t channel,bool exclusive) {
	// 1/. Check to see if the requested channel is already open; if so return true
	uint8_t status=0x0f;
	uint8_t openchannelcount=0;

	status = TCA.readStatus();
	if (((status & channel) > 0 ) && !exclusive) {
		//sprintf(base->outBuff,"Channel 0x%02x is already open",channel);
		//base->sckOut(PRIO_LOW,true);
		return true;
	}

	// so, requested channel is not already open.
	// 2/. if exclusive is requested, then close open channels and open the requested channel
	if (exclusive) {
		TCA.closeAll();
		delayMicroseconds(10);
		TCA.openChannel(channel);
		// check to see if it was opened:
		status=TCA.readStatus();
		delayMicroseconds(10);
		if ((status & channel) > 0) {
			//sprintf(base->outBuff,"Channel 0x%02x is now open exclusively",channel);
			//base->sckOut(PRIO_MED,true);
			return true;
		}
	}

	// exclusive is not requested.  But maximum 3 can be open at one time.<-- local rulez is still rulez
	// this rule has been established by testing.  It has been found that Too many open channels will likely
	// destabilise the bus.

	// 3/. count how many channels are open.  If N = 3 then close all and open the one requested
	// otherwise just open the one requested
	openchannelcount=countTCAOpenChannels(base);
	if (openchannelcount >= 2) {
		TCA.closeAll();
		delayMicroseconds(10);
	}
	TCA.openChannel(channel);
	delayMicroseconds(10);
	status=TCA.readStatus();
	if ((status & channel) > 0) {
		//sprintf(base->outBuff,"Channel 0x%02x is now also open",channel);
		//base->sckOut(PRIO_MED,true);
		return true;
	} else {
		return false;
	}

	// NOTE: it is not the job of this function to confirm that the requesting device can actually communicate thru the mux;
	// that must be verified by the device itself

}
/*
bool AuxBoards::testI2C(SckBase* base,uint8_t address, uint8_t channel,SensorType wichSensor,bool exclusive) {
	

	auxWire.beginTransmission(address);
	byte error = auxWire.endTransmission();
	bool status=0x00;
	//uint8_t ctr=0;
	
	uint8_t openchannelcount=0;
	if (error == 0) {
		//base->sckOut("I2C connection verified (no need to switch mux)");
		return true;
	} else if (channel != 0x00 && channel <= TCA_CHANNEL_7) {
		//base->sckOut("we need to switch mux");
		status = TCA.readStatus();
		//sprintf(base->outBuff,"1 Mux status: 0x%02x",status);
		//base->sckOut();
		if (status != channel) {
			// find out how many open channels there are right now
			if (status == 0xff) {		// this defines a bus fault which we attempt to clear
				TCA.closeAll();
				status=TCA.readStatus();
				if (status == 0xff) {	// bus fault persists; now try a re-instatiation
					delay(1000);		// maybe the bus fault will clear?
					status=restartTCAMux();
					TCA.closeAll();
					status=TCA.readStatus();					
				}
				if (status == 0xff) {
					return false;	// at this point we are giving up but I would like the ability to 
									// set the reset wire on the TCA; but at the moment lack the means to do so
									// we need a spare digital pin on the SAMD21G to do it; 
									// plus a physical wire connected to the reset pad on the mux board.
				} else {
					openchannelcount=0;
				}
			} else {
				openchannelcount=countTCAOpenChannels(base);
			}
			if (openchannelcount >= 3 || exclusive ) {
				TCA.closeAll();
			}
			if (exclusive) {
				openmuxchanExclusive=true;
			}
			TCA.openChannel(channel);		// open the requested channel 
			delay(1);
		}
	}
	// verify I2C connection to device again after the channel was switched 
	delay(10);
	auxWire.beginTransmission(address);
	error = auxWire.endTransmission();
	if (error == 0) {
		//if (ctr > 0) base->sckOut("final:I2C connection verified");
		return true;
	} else if (error == 4) {
		base->sckOut("I2C connection UNKNOWN ERROR",PRIO_LOW,true);
	//} else {
	//	base->sckOut("I2C connection failed: bailing out",PRIO_LOW,true);
	}
	return false;	
}
*/
bool GrooveI2C_ADC::start(SckBase* base,AuxBoards* auxBoard,SensorType wichSensor)
{
	
	if (auxBoard->TCAMUXMODE && deviceAddress != 0x00) {
		//localPortNum=base->findDevicePort(&auxWire, deviceAddress);  // find and switch to Mux port on which the address is found
		localPortNum=auxBoard->findDeviceChan(base,deviceAddress,wichSensor, true,false);
		if (localPortNum > TCA_CHANNEL_7) {
			
			localPortNum=0x00;
			return false;	// the device was not found 
		} else {			// even if we find a port its no good if we cannt communicate with the device
			if (!I2Cdetect(&auxWire, deviceAddress)) return false;
		}
	} else if (!I2Cdetect(&auxWire, deviceAddress)) return false;

	auxWire.beginTransmission(deviceAddress);		// transmit to device
	auxWire.write(REG_ADDR_CONFIG);				// Configuration Register
	auxWire.write(0x20);
	auxWire.endTransmission();
	return true;
}
bool GrooveI2C_ADC::stop()
{

	return true;
}
float GrooveI2C_ADC::getReading(SckBase* base,AuxBoards* auxBoard,SensorType wichSensor)
{
	if (auxBoard->TCAMUXMODE) {
		if ( !auxBoard->openChannel(base,deviceAddress,localPortNum,false)) {
			return 0;
		}
	} else if (!I2Cdetect(&auxWire, deviceAddress)) return 0;
	
	uint32_t data = 0;

	auxWire.beginTransmission(deviceAddress);		// transmit to device
	auxWire.write(REG_ADDR_RESULT);				// get result
	auxWire.endTransmission();

	auxWire.requestFrom(deviceAddress, 2);			// request 2byte from device
	delay(1);

	if (auxWire.available()<=2) {
		data = (auxWire.read()&0x0f)<<8;
		data |= auxWire.read();
	}

	return data * V_REF * 2.0 / 4096.0;
}
// TODO: need to investigate why INA219 has never returned any readings : could it be the wiring ?
bool INA219::start(SckBase* base,AuxBoards* auxBoard,SensorType wichSensor)
{
	// If we are using a I2C port MUX; then the Port must be enabled prior to calling this library
	//base->sckOut("INA219 start request");

	if (auxBoard->TCAMUXMODE) {
		localPortNum=auxBoard->findDeviceChan(base,deviceAddress,wichSensor, true,true);
		//sprintf(base->outBuff,"INA219 at Address 0x%02x has channel 0x%02x",deviceAddress,localPortNum);
		//base->sckOut(PRIO_MED,true);
		if (localPortNum > TCA_CHANNEL_7) {
			
			localPortNum=0x00;
			//base->sckOut("The device was not found connected to the Mux");
			return false;	// the device was not found 
		} else {			// even if we find a port its no good if we cannt communicate with the device
			if (!I2Cdetect(&auxWire, deviceAddress)) return false;
		}
	} else if (!I2Cdetect(&auxWire, deviceAddress)) return false;
	
	//base->sckOut("INA219 was found; instantiating/starting the device");

	ada_ina219.begin(&auxWire);

	//base->sckOut("INA219 was started");

	// By default the initialization will use the largest range (32V, 2A).
	//ada_ina219.setCalibration_32V_1A();
	ada_ina219.setCalibration_16V_400mA();
	//base->sckOut("INA219 was calibrated");
	return true;
}
bool INA219::stop()
{
	// TODO check if there is a way to minimize power consumption:Nope the library provides no access 
	// although it looks like there might be a register command to do it
	return true;
}
float INA219::getReading(SckBase* base,AuxBoards* auxBoard,SensorType wichSensor,typeOfReading wichReading)
{
	//base->sckOut("INA219 getreading request");
	if (auxBoard->TCAMUXMODE) {
		if ( !auxBoard->openChannel(base,deviceAddress,localPortNum,true)) {
			return 0;
		}
	} else if (!I2Cdetect(&auxWire, deviceAddress)) return 0;
	
	//base->sckOut("conducting a reading");
	switch(wichReading) {
		case BUS_VOLT: {

			return ada_ina219.getBusVoltage_V();
			break;

		} case SHUNT_VOLT: {

			return ada_ina219.getShuntVoltage_mV();
			break;

		} case CURRENT: {

			return ada_ina219.getCurrent_mA();
			break;

		} case LOAD_VOLT: {

			float busvoltage 	= ada_ina219.getBusVoltage_V();
			float shuntvoltage 	= ada_ina219.getShuntVoltage_mV();

			return busvoltage + (shuntvoltage / 1000);
			break;

		}
	}

	return 0;
}
bool Groove_OLED::start(SckBase* base,AuxBoards* auxBoard,SensorType wichSensor)
{
	// If we are using a I2C port MUX; then the Port must be enabled prior to calling this library
	if (auxBoard->TCAMUXMODE && deviceAddress != 0x00) {
		localPortNum=auxBoard->findDeviceChan(base,deviceAddress,wichSensor, true,false);
		if (localPortNum > TCA_CHANNEL_7) {
			
			localPortNum=0x00;
			return false;	// the device was not found 
		} else {			// even if we find a port its no good if we cannt communicate with the device
			if (!I2Cdetect(&auxWire, deviceAddress)) return false;
		}
	} else if (!I2Cdetect(&auxWire, deviceAddress)) return false;

	sprintf(base->outBuff, "Disply %s started at address 0x%02x",base->sensors[wichSensor].title,deviceAddress);
	base->sckOut(PRIO_MED,true);
	
	u8g2_oled.setBusClock(1000000); 	// 1000000 -> 68 ms for a full buffer redraw
	u8g2_oled.begin();
	u8g2_oled.drawXBM( 16, 16, 96, 96, scLogo);
	u8g2_oled.sendBuffer();
	currentLine = 1;
	delay(1000);
	u8g2_oled.clearDisplay();

	return true;;
}

bool Groove_OLED::stop()
{

	return true;
}

void Groove_OLED::print(SckBase* base,AuxBoards* auxBoard,SensorType wichSensor,char *payload)
{
	if (auxBoard->TCAMUXMODE) {
		if ( !auxBoard->openChannel(base,deviceAddress,localPortNum,false)) {
			return ;
		}
	} else if (!I2Cdetect(&auxWire, deviceAddress)) return ;
	
	// checks that I2C Address is accessible on currently open channel
	// switches I2C Mux channel if necessary and available

	u8g2_oled.setFont(font);

	uint8_t thisLineChar = 0;
	uint8_t lineStart = 0;
	for (uint8_t i=0; i<strlen(payload); i++) {

		// If there is a newLine char
		if (payload[i] == 0xA || i == (strlen(payload) - 1)) {

			printLine(base,auxBoard,wichSensor,&payload[lineStart], thisLineChar + 1);
			lineStart += (thisLineChar + 1);
			thisLineChar = 0;

		// If line is full
		} else if (thisLineChar == (columns - 1)) {

			printLine(base,auxBoard,wichSensor, &payload[lineStart], thisLineChar + 1);
			lineStart += (thisLineChar + 1);
			thisLineChar = 0;

		// No new line yet
		} else thisLineChar ++;
	}

	u8g2_oled.sendBuffer();
}
void Groove_OLED::printLine(SckBase* base,AuxBoards* auxBoard,SensorType wichSensor,char *payload, uint8_t size)
{
	if (auxBoard->TCAMUXMODE) {
		if ( !auxBoard->openChannel(base,deviceAddress,localPortNum,false)) {
			return ;
		}
	} else if (!I2Cdetect(&auxWire, deviceAddress)) return ;

	// Reject empty lines
	if (size < 1) return;
	if (payload[0] == 0xA || payload[0] == 0xD) return;

	// Clear screen if we are on the top
	if (currentLine == 1) {
		u8g2_oled.clearDisplay();
		u8g2_oled.home();

	// Slide screen one line up when bottom is reached
	} else if (currentLine > lines) {
		uint8_t *buffStart = u8g2_oled.getBufferPtr();
		memcpy(&buffStart[0], &buffStart[128], 1920);
		memset(&buffStart[1920], 0, 128);
		currentLine = lines;
	}

	// Print line
	char toPrint[size+1];
	snprintf(toPrint, sizeof(toPrint), payload);

	u8g2_oled.drawStr(0, currentLine * font_height, toPrint);
	u8g2_oled.drawStr(columns * font_width, currentLine * font_height, " "); 	// Clear incomplete chars at the end
	currentLine++;
}
void Groove_OLED::update(SckBase* base,AuxBoards* auxBoard,SensorType wichSensor, bool force)
{
	sprintf(base->outBuff, "%s update: Addr: 0x%02x Chan: 0x%02x",base->sensors[wichSensor].title,deviceAddress,localPortNum);
	base->sckOut(PRIO_MED,true);

	if (auxBoard->TCAMUXMODE) {
		if ( !auxBoard->openChannel(base,deviceAddress,localPortNum,false)) {
			return ;
		}
	} else if (!I2Cdetect(&auxWire, deviceAddress)) return ;

	if (millis() - lastUpdate < refreshRate && !force) return;
	lastUpdate = millis();

	if (base->config.debug.oled) return;

	// Info bar
	drawBar(base, auxBoard,wichSensor);

	if (base->st.error == ERROR_NONE) {

		// Setup mode screen
		if (base->st.onSetup) drawSetup(base,auxBoard,wichSensor);
		else displayReading(base,auxBoard,wichSensor);

	} else {

		// Error popup
		drawError(base,auxBoard,wichSensor, base->st.error);

	}

}
void Groove_OLED::drawBar(SckBase* base,AuxBoards* auxBoard,SensorType wichSensor)
{
	if (auxBoard->TCAMUXMODE) {
		if ( !auxBoard->openChannel(base,deviceAddress,localPortNum,false)) {
			return ;
		}
	} else if (!I2Cdetect(&auxWire, deviceAddress)) return ;

	u8g2_oled.setFont(u8g2_font_nine_by_five_nbp_tr);

	// Clear the buffer area of the bar
	uint8_t *buffStart = u8g2_oled.getBufferPtr();
	memset(&buffStart[0], 0, 256);

	uint8_t font_h = u8g2_oled.getMaxCharHeight();

	// Print current mode on the left
	if (base->st.onSetup) u8g2_oled.drawStr(0, font_h, "SETUP");
	else if (base->st.onShell) u8g2_oled.drawStr(0, font_h, "SHELL");
	else if (base->st.mode == MODE_NET) u8g2_oled.drawStr(0, font_h, "WIFI");
	else if (base->st.mode == MODE_SD) u8g2_oled.drawStr(0, font_h, "SD");

	// Print "icon tray" from right to left
	uint8_t tray_x = 128;
	uint8_t sep = 4;

	if (base->battery.present) {

		// Battery percent
		char percent[5];
		snprintf(percent, sizeof(percent), "%u%%", base->battery.last_percent);
		tray_x -= u8g2_oled.getStrWidth(percent);
		u8g2_oled.drawStr(tray_x, font_h, percent);

		// Battery Icon
		tray_x -= (batt_charge_width + sep);
		if (base->led.chargeStatus == base->led.CHARGE_CHARGING) u8g2_oled.drawXBM(tray_x, (16 - batt_charge_height) / 2, batt_charge_width, batt_charge_height, batt_charge_bits);
		else if (base->battery.last_percent > 75) u8g2_oled.drawXBM(tray_x, (16 - batt_full_height) / 2, batt_full_width, batt_full_height, batt_full_bits);
		else if (base->battery.last_percent > 25) u8g2_oled.drawXBM(tray_x, (16 - batt_half_height) / 2, batt_half_width, batt_half_height, batt_half_bits);
		else u8g2_oled.drawXBM(tray_x, (16 - batt_empty_height) / 2, batt_empty_width, batt_empty_height, batt_empty_bits);

	}

	// AC icon
	if (base->charger.onUSB) {
		tray_x -= (AC_width + sep);
		u8g2_oled.drawXBM(tray_x, (16 - AC_height) / 2, AC_width, AC_height, AC_bits);
	}

	// Time sync icon
	if (base->st.timeStat.ok) {
		tray_x -= (clock_width + sep);
		u8g2_oled.drawXBM(tray_x, (16 - clock_height) / 2, clock_width, clock_height, clock_bits);
	}

	// Sdcard icon
	if (base->st.cardPresent) {
		tray_x -= (sdcard_width + sep);
		u8g2_oled.drawXBM(tray_x, (16 - sdcard_height) / 2, sdcard_width, sdcard_height, sdcard_bits);
	}

	// Wifi icon
	if (base->st.wifiStat.ok && base->st.espON) {
		tray_x -= (wifi_width + sep);
		u8g2_oled.drawXBM(tray_x, (16 - wifi_height) / 2, wifi_width, wifi_height, wifi_bits);
	}

	u8g2_oled.drawHLine(0, 15, 128);
	u8g2_oled.updateDisplayArea(0, 0, 16, 2);
}
void Groove_OLED::drawError(SckBase* base,AuxBoards* auxBoard,SensorType wichSensor,errorType wichError)
{
	if (auxBoard->TCAMUXMODE) {
		if ( !auxBoard->openChannel(base,deviceAddress,localPortNum,false)) {
			return ;
		}
	} else if (!I2Cdetect(&auxWire, deviceAddress)) return ;
	
	if (lastError == wichError) return;

	// Clear error buffer area
	uint8_t *buffStart = u8g2_oled.getBufferPtr();
	memset(&buffStart[1792], 0, 256);

	u8g2_oled.setFont(u8g2_font_7x13B_mr);

	// Print a frame with an alert icon on the left
	u8g2_oled.drawFrame(0, 112, 128, 16);
	u8g2_oled.drawBox(0, 112, 16, 16);
	u8g2_oled.drawXBM(2, 114, error_width, error_height, error_bits);

	// Set message
	char errorMsg[18];
	switch(wichError) {
		case ERROR_SD:
			snprintf(errorMsg, sizeof(errorMsg), "NO SDCARD FOUND");
			break;
		case ERROR_SD_PUBLISH:
			snprintf(errorMsg, sizeof(errorMsg), "SDCARD ERROR");
			break;
		case ERROR_TIME:
			snprintf(errorMsg, sizeof(errorMsg), "TIME NOT SYNCED");
			break;
		case ERROR_NO_WIFI_CONFIG:
			snprintf(errorMsg, sizeof(errorMsg), "NO WIFI SET");
			break;
		case ERROR_AP:
			snprintf(errorMsg, sizeof(errorMsg), "WRONG WIFI SSID");
			break;
		case ERROR_PASS:
			snprintf(errorMsg, sizeof(errorMsg), "WRONG WIFI PASS");
			break;
		case ERROR_WIFI_UNKNOWN:
			snprintf(errorMsg, sizeof(errorMsg), "WIFI ERROR");
			break;
		case ERROR_MQTT:
			snprintf(errorMsg, sizeof(errorMsg), "MQTT ERROR");
			break;
		case ERROR_NO_TOKEN_CONFIG:
			snprintf(errorMsg, sizeof(errorMsg), "NO TOKEN SET");
			break;
		case ERROR_BATT:
			snprintf(errorMsg, sizeof(errorMsg), "LOW BATTERY");
			break;
		default:
			break;
	}

	// Print message
	u8g2_oled.drawStr(19, 125, errorMsg);

	lastError = wichError;

	// Update display
	u8g2_oled.updateDisplayArea(0, 14, 16, 2);
}
void Groove_OLED::drawSetup(SckBase* base,AuxBoards* auxBoard,SensorType wichSensor)
{
	if (auxBoard->TCAMUXMODE) {
		if ( !auxBoard->openChannel(base,deviceAddress,localPortNum,false)) {
			return ;
		}
	} else if (!I2Cdetect(&auxWire, deviceAddress)) return ;
	
	// Clear buffer (except info bar)
	uint8_t *buffStart = u8g2_oled.getBufferPtr();
	memset(&buffStart[256], 0, 1792);

	u8g2_oled.setFont(u8g2_font_nine_by_five_nbp_tr);
	uint8_t font_h = u8g2_oled.getMaxCharHeight();

	char conn[] = "Connect to the Wi-Fi:";
	u8g2_oled.drawStr((128 - u8g2_oled.getStrWidth(conn)) / 2, font_h + 30, conn);

	u8g2_oled.setFont(u8g2_font_t0_16b_tf);
	u8g2_oled.drawStr((128 - u8g2_oled.getStrWidth(base->hostname)) / 2, font_h + 55, base->hostname);

	u8g2_oled.setFont(u8g2_font_nine_by_five_nbp_tr);
	char conn2[] = "If no window opens,";
	u8g2_oled.drawStr((128 - u8g2_oled.getStrWidth(conn2)) / 2, font_h + 80, conn2);
	char conn3[] = "with your browser go to";
	u8g2_oled.drawStr((128 - u8g2_oled.getStrWidth(conn3)) / 2, font_h + 92, conn3);
	char conn4[] = "http://sck.me";
	u8g2_oled.drawStr((128 - u8g2_oled.getStrWidth(conn4)) / 2, font_h + 104, conn4);
	char conn5[] = "or 192.168.1.1";
	u8g2_oled.drawStr((128 - u8g2_oled.getStrWidth(conn5)) / 2, font_h + 116, conn5);

	u8g2_oled.updateDisplayArea(0, 2, 16, 14);
}
void Groove_OLED::displayReading(SckBase* base,AuxBoards* auxBoard,SensorType wichSensor)
{
	if (auxBoard->TCAMUXMODE) {
		if ( !auxBoard->openChannel(base,deviceAddress,localPortNum,false)) {
			return ;
		}
	} else if (!I2Cdetect(&auxWire, deviceAddress)) return ;

	if (base->rtc.getEpoch() - showStartTime <= showTime) return;

	SensorType sensorToShow = SENSOR_COUNT;
	uint8_t cycles = 0;

	// Find next sensor to show
	for (uint8_t i=lastShown+1; i<SENSOR_COUNT; i++) {

		SensorType thisSensor = static_cast<SensorType>(i);

		if (base->sensors[thisSensor].enabled &&
				base->sensors[thisSensor].oled_display &&
				base->sensors[thisSensor].type != SENSOR_GROVE_OLED && 		//Oled screen has nothing to show
				// base->sensors[thisSensor].type != SENSOR_GROVE_OLED2 && 		//Oled screen has nothing to show
				base->sensors[thisSensor].type != SENSOR_BATT_PERCENT) { 	// Battery is already shown on oled info-bar

			sensorToShow = thisSensor;
			break;
		}
		if (i == SENSOR_COUNT - 1) {
			i = 0;
			cycles++;
			if (cycles > 1) break; 	// Avoid getting stuck here if no sensor is enabled
		}
	}

	// Clear buffer (except info bar)
	uint8_t *buffStart = u8g2_oled.getBufferPtr();
	memset(&buffStart[256], 0, 1792);

	// Draw Title
	u8g2_oled.setFont(u8g2_font_t0_16b_tf);

	// Split in two lines if needed
	const char *sensorTitle = base->sensors[sensorToShow].title;
	uint8_t baseLine = 55 - u8g2_oled.getMaxCharHeight();
	if (u8g2_oled.getStrWidth(sensorTitle) > 128) {

		baseLine = 55;

		// Try splitting on first space
		char line1[20];
		char blank[] = " ";
		uint8_t splitPoint = strcspn(sensorTitle, blank);
		memcpy(line1, sensorTitle, splitPoint);
		line1[splitPoint + 1] = '\0';

		char *line2 = strchr(sensorTitle, ' ') + 1;

		// If some of the lines is to big split in half
		if (u8g2_oled.getStrWidth(line2) > 128 ||
			u8g2_oled.getStrWidth(line1) > 128) {

			// Split in half
			splitPoint = strlen(sensorTitle) / 2;
			memcpy(line1, sensorTitle, splitPoint);
			line1[splitPoint + 1] = '\0';
			u8g2_oled.drawStr((128 - u8g2_oled.getStrWidth(line1)) / 2, (baseLine - u8g2_oled.getMaxCharHeight()) - 2, line1);
			u8g2_oled.drawStr((128 - u8g2_oled.getStrWidth(&sensorTitle[splitPoint])) / 2, baseLine + u8g2_oled.getDescent(), &sensorTitle[splitPoint]);

		} else {

			// Split on space
			u8g2_oled.drawStr((128 - u8g2_oled.getStrWidth(line1)) / 2, (baseLine - u8g2_oled.getMaxCharHeight()) - 2, line1);
			u8g2_oled.drawStr((128 - u8g2_oled.getStrWidth(line2)) / 2, baseLine + u8g2_oled.getDescent(), line2);
		}

	} else u8g2_oled.drawStr((128 - u8g2_oled.getStrWidth(sensorTitle)) / 2, baseLine + u8g2_oled.getDescent(), sensorTitle);

	// Draw unit
	const char *sensorUnit = base->sensors[sensorToShow].unit;
	u8g2_oled.drawStr((128 - u8g2_oled.getStrWidth(sensorUnit)) / 2, 128 + u8g2_oled.getDescent(), sensorUnit);

	// Draw Value
	uint8_t vCenter = baseLine + ((128 - u8g2_oled.getMaxCharHeight() - baseLine) / 2);
	u8g2_oled.setFont(u8g2_font_fub30_tn);
	String value = base->sensors[sensorToShow].reading;
	if (base->sensors[sensorToShow].state != 0) value = "--";
	if (u8g2_oled.getStrWidth(value.c_str()) > 128) {
		u8g2_oled.setFont(u8g2_font_fub20_tn);
		u8g2_oled.drawStr((128 - u8g2_oled.getStrWidth(value.c_str())) / 2, vCenter + (u8g2_oled.getMaxCharHeight() / 2), value.c_str());
	} else {
		u8g2_oled.drawStr((128 - u8g2_oled.getStrWidth(value.c_str())) / 2, vCenter + (u8g2_oled.getMaxCharHeight() / 2), value.c_str());
	}


	u8g2_oled.updateDisplayArea(0, 2, 16, 14);

	lastShown = sensorToShow;
	showStartTime = base->rtc.getEpoch();
}
void Groove_OLED::plot(SckBase* base,AuxBoards* auxBoard,SensorType wichSensor,String value, const char *title, const char *unit)
{
	if (auxBoard->TCAMUXMODE) {
		if ( !auxBoard->openChannel(base,deviceAddress,localPortNum,false)) {
			return ;
		}
	} else if (!I2Cdetect(&auxWire, deviceAddress)) return ;


	// TODO support negative values
	// TODO Auto adjust lower limit of Y scale

	// Fresh start
	if (title != NULL) {
		_unit = unit;
		_title = title;
		minY = 0;
		maxY = 100;
		u8g2_oled.clearDisplay();
		plotData.clear();
	}

	u8g2_oled.clearBuffer();

	// Title and unit
	u8g2_oled.setFont(u8g2_font_nine_by_five_nbp_tr);
	u8g2_oled.drawStr(0, u8g2_oled.getMaxCharHeight() + u8g2_oled.getDescent(), _title);
	u8g2_oled.drawStr(0, (u8g2_oled.getMaxCharHeight() * 2) - 2, value.c_str());
	u8g2_oled.drawStr(u8g2_oled.getStrWidth(value.c_str()) + 10, (u8g2_oled.getMaxCharHeight() * 2) - 2, _unit);


	// Get the new value
	float Fvalue = value.toFloat();

	// If this value is grater than our limit adjust scale
	if (Fvalue > maxY) remap(base,auxBoard,wichSensor,Fvalue);

	// Store the new value scaled to screen size
	int8_t Ivalue = map(Fvalue, minY, maxY, screenMin, screenMax);
	plotData.add(Ivalue);
	if (plotData.size() > 128) plotData.remove(0); // Keep the size of the array limited to the screen size

	// Check if we need to reajust scale (Happens when big values get out of scope)
	float currentMaxY = 0;
	for (uint8_t i=0; i<plotData.size(); i++) {
		if (plotData.get(i) > currentMaxY) currentMaxY = plotData.get(i);
	}
	float bigCurrentMaxY = map(currentMaxY, screenMin, screenMax, minY, maxY);
	if (bigCurrentMaxY < (maxY - (maxY / 10)) && bigCurrentMaxY > 0) remap(base,auxBoard,wichSensor,bigCurrentMaxY);


	// Plot the data on the display
	uint8_t ii = plotData.size() - 1;
	for (uint8_t i=127; i>0; i--) {
		if (ii == 0) break;
		u8g2_oled.drawLine(i, 128 - plotData.get(ii), i - 1, 128 - plotData.get(ii - 1));
		ii--;
	}

	// Print Y top value
	u8g2_oled.setFont(u8g2_font_nine_by_five_nbp_tr);
	char buff[8];
	snprintf(buff, sizeof(buff), "%.0f", maxY);
	u8g2_oled.drawStr(128 - u8g2_oled.getStrWidth(buff), 16 + u8g2_oled.getMaxCharHeight(), buff);
	u8g2_oled.drawHLine(118, 18 + u8g2_oled.getMaxCharHeight(), 10);

	// Print Y bottom value
	snprintf(buff, sizeof(buff), "%.0f", minY);
	u8g2_oled.drawStr(128 - u8g2_oled.getStrWidth(buff), 126, buff);
	u8g2_oled.drawHLine(118, 127, 10);

	u8g2_oled.sendBuffer();
}
void Groove_OLED::remap(SckBase* base,AuxBoards* auxBoard,SensorType wichSensor,float newMaxY)
{
	if (auxBoard->TCAMUXMODE) {
		if ( !auxBoard->openChannel(base,deviceAddress,localPortNum,false)) {
			return ;
		}
	} else if (!I2Cdetect(&auxWire, deviceAddress)) return ;

	newMaxY += (newMaxY / 10);

	float fa = maxY / newMaxY;

	for (uint8_t i=0; i<plotData.size(); i++) {

		float remaped = plotData.get(i) * fa;

		// round it properly
		if (remaped > 0) remaped += 0.5;
		else remaped -= 0.5;

		plotData.set(i, (int)remaped);
	}

	maxY = newMaxY;
}
#ifndef MISC_DISABLE
bool WaterTemp_DS18B20::start(SckBase* base,AuxBoards* auxBoard,SensorType wichSensor)
{
	// If we are using a I2C port MUX; then the Port must be enabled prior to calling this library
	if (auxBoard->TCAMUXMODE && deviceAddress != 0x00) {
		localPortNum=auxBoard->findDeviceChan(base,deviceAddress,wichSensor, true,false);
		if (localPortNum > TCA_CHANNEL_7) {
			
			localPortNum=0x00;
			return false;	// the device was not found 
		} else {			// even if we find a port its no good if we cannt communicate with the device
			if (!I2Cdetect(&auxWire, deviceAddress)) return false;
		}		// else fall thru
	} else if (!I2Cdetect(&auxWire, deviceAddress)) return false;

	auxWire.begin();

	DS_bridge.reset();
	DS_bridge.wireReset();
	DS_bridge.wireSkip();
	DS_bridge.wireWriteByte(0x44);

	detected = true;

	return true;
}
bool WaterTemp_DS18B20::stop()
{
	return true;
}
float WaterTemp_DS18B20::getReading(SckBase* base,AuxBoards* auxBoard,SensorType wichSensor)
{
	if (auxBoard->TCAMUXMODE) {
		if ( !auxBoard->openChannel(base,deviceAddress,localPortNum,false)) {
			return 0;
		}
	} else if (!I2Cdetect(&auxWire, deviceAddress)) return 0;

	
	while (!DS_bridge.wireSearch(addr)) {

		DS_bridge.wireResetSearch();
		DS_bridge.wireReset();
		DS_bridge.selectChannel(0); 			// After reset need to set channel 0 because we are using the version with single channel (DS2482_100)
		DS_bridge.configure(conf);
		DS_bridge.wireSkip();
		DS_bridge.configure(conf); 				// Set bus on strong pull-up after next write, not only LSB nibble is required
		DS_bridge.wireWriteByte(0x44); 			// Convert temperature on all devices
		DS_bridge.configure(0x01);
	}

	//	Test if device is in reality the DS18B20 Water Temperature
	if (addr[0]==0x28) {

		// Read temperature data.
		DS_bridge.wireReset(); 				//DS_bridge.reset();
		DS_bridge.selectChannel(0); 		//necessary on -800
		DS_bridge.wireSelect(addr);
		DS_bridge.wireWriteByte(0xbe);      // Read Scratchpad command

		// We need to read 9 bytes
		for (int i=0; i<9; i++) data[i] = DS_bridge.wireReadByte();

		// Convert to decimal temperature
		int LowByte = data[0];
		int HighByte = data[1];
		int TReading = (HighByte << 8) + LowByte;
		int SignBit = TReading & 0x8000;

		// If Temperature is negative
		if (SignBit) TReading = (TReading ^ 0xffff) + 1;

		int Tc_100 = (double)TReading * 0.0625 * 10;

		// If the reading is negative make it efective
		if (SignBit) Tc_100 = 0 - Tc_100;

		return ((float)(Tc_100) / 10) + 1;
	}

	return 0;
}
#endif
#ifndef ATLAS_DISABLE
bool Atlas::start(SckBase* base,AuxBoards* auxBoard,SensorType wichSensor)
{

	// If we are using a I2C port MUX; then the Port must be enabled prior to calling this library
	if (auxBoard->TCAMUXMODE && deviceAddress != 0x00) {
		localPortNum=auxBoard->findDeviceChan(base,deviceAddress,wichSensor, true,true);
		if (localPortNum > TCA_CHANNEL_7) {
			
			localPortNum=0x00;
			return false;	// the device was not found 
		} else {			// even if we find a port its no good if we cannt communicate with the device
			if (!I2Cdetect(&auxWire, deviceAddress)) return false;
		}
	} else if (!I2Cdetect(&auxWire, deviceAddress)) return false;

	if (beginDone) return true;

	// Protocol lock
	if (!sendCommand(base,(char*)"Plock,1",auxBoard,wichSensor)) return false;
	delay(shortWait);

	// This actions are only for conductivity (EC) sensor
	if (EC) {

		// ----  Set parameters
		if (sendCommand(base,(char*)"O,?",auxBoard,wichSensor)) { 	// Ask info about enabled parameters
			delay(shortWait);
			getResponse(base,auxBoard,wichSensor);
			if (!atlasResponse.equals("?O,EC,TDS,S,SG")) {
				SerialUSB.println("Enabling all metrics for EC Atlas");
				const char *parameters[4] = PROGMEM {"O,EC,1", "O,TDS,1", "O,S,1", "O,SG,1"};
				for (int i = 0; i<4; ++i) {
					if (!sendCommand(base,(char*)parameters[i],auxBoard,wichSensor)) return false;
					delay(longWait);
				}
			}
		} else return false;

	} else if (DO) {

		// ---- Check if this is really a Atlas DO sensor (allows sharing I2C addres 0x61 with SCD30 CO2 sensor)
		if (sendCommand(base,(char*)"i",auxBoard,wichSensor)) {
			delay(shortWait);
			getResponse(base,auxBoard,wichSensor);
			if (!atlasResponse.startsWith("?I,DO")) return false;
		} else return false;

		// ---- Set parameters
		if (sendCommand(base,(char*)"O,?",auxBoard,wichSensor)) {
			delay(shortWait);
			getResponse(base,auxBoard,wichSensor);
			if (!atlasResponse.equals((char*)"?O,%,mg")) {
				if (!sendCommand(base,(char*)"O,%,1",auxBoard,wichSensor)) return false;
				delay(shortWait);
				if (!sendCommand(base,(char*)"O,mg,1",auxBoard,wichSensor)) return false;
				delay(shortWait);
			}
		} else return false;
	}

	beginDone = true;
	detected = true;

	goToSleep(base,auxBoard,wichSensor);

	return true;
}
bool Atlas::stop()
{
	return true;
}
bool Atlas::getReading(SckBase* base, AuxBoards* auxBoard,SensorType wichSensor)
{
	if (auxBoard->TCAMUXMODE) {
		if ( !auxBoard->openChannel(base,deviceAddress,localPortNum,true)) {
			return false;
		}
	} else if (!I2Cdetect(&auxWire, deviceAddress)) return false;

	if (millis() - lastUpdate < 2000) return true;
	uint32_t started = millis();
	while (getBusyState(base, auxBoard,wichSensor)) {
		if (millis() - started > 5000) return false; 	// Timeout
		delay(2);
	}
	return true;
}
bool Atlas::getBusyState(SckBase* base,AuxBoards* auxBoard,SensorType wichSensor)
{
	if (auxBoard->TCAMUXMODE) {
		if ( !auxBoard->openChannel(base,deviceAddress,localPortNum,true)) {
			return false;
		}
	}
	if (!I2Cdetect(&auxWire, deviceAddress)) return false;

	if (millis() - lastUpdate < 2) return true;
	switch (state) {

		case REST: {

			if (TEMP) {
				state = TEMP_COMP_SENT;
				break;
			}

			if (tempCompensation(base,auxBoard,wichSensor)) state = TEMP_COMP_SENT;
			break;

		} case TEMP_COMP_SENT: {

			if (millis() - lastCommandSent >= shortWait) {
				if (sendCommand(base,(char*)"r",auxBoard,wichSensor)) state = ASKED_READING;
			}
			break;

		} case ASKED_READING: {

			uint16_t customWait = longWait;
			if (TEMP) customWait = mediumWait;

			if (millis() - lastCommandSent >= customWait) {

				uint8_t code = getResponse(base,auxBoard,wichSensor);

				if (code == 254) {
					// Still working (wait a little more)
					lastCommandSent = lastCommandSent + 200;
					break;

				} else if (code == 1) {

					// Reading OK
					state = REST;

					if (PH || TEMP)	newReading[0] = atlasResponse.toFloat();
					if (EC || DO) {

						uint8_t readingNum = 2;
						if (EC) readingNum = 4;

						for (uint8_t i=0; i<readingNum; i++) {

							uint8_t endIdx = atlasResponse.indexOf(",");

							String newReadingStr;
							if (endIdx > 0) {
								newReadingStr = atlasResponse.substring(0, endIdx);
								atlasResponse.remove(0, endIdx+1);
							} else {
								newReadingStr = atlasResponse.substring(0);
							}

							newReading[i] = newReadingStr.toFloat();
						}
					}
					goToSleep(base,auxBoard,wichSensor);
					return false;
					break;

				} else {

					// Error
					state = REST;
					newReading[0] = 0;
					goToSleep(base,auxBoard,wichSensor);
					return false;
					break;
				}
			}
			break;
		}
	}

	lastUpdate = millis();
	return true;
}
void Atlas::goToSleep(SckBase* base,AuxBoards* auxBoard,SensorType wichSensor)
{
	if (auxBoard->TCAMUXMODE) {
		if ( !auxBoard->openChannel(base,deviceAddress,localPortNum,true)) {
			return ;
		}
	} else if (!I2Cdetect(&auxWire, deviceAddress)) return ;

	auxWire.beginTransmission(deviceAddress);
	auxWire.write("Sleep");
	auxWire.endTransmission();
}
bool Atlas::sendCommand(SckBase* base,char* command,AuxBoards* auxBoard,SensorType wichSensor)
{
	if (auxBoard->TCAMUXMODE) {
		if ( !auxBoard->openChannel(base,deviceAddress,localPortNum,true)) {
			return false;
		}
	} else if (!I2Cdetect(&auxWire, deviceAddress)) return false;

	uint8_t retrys = 5;

	for (uint8_t i=0; i<retrys; ++i) {

		auxWire.beginTransmission(deviceAddress);
		auxWire.write(command);

		auxWire.requestFrom(deviceAddress, 1, true);
		uint8_t confirmed = auxWire.read();
		auxWire.endTransmission();

		if (confirmed == 1) {
			lastCommandSent = millis();
			return true;
		}

		delay(300);
	}
	return false;
}
bool Atlas::tempCompensation(SckBase* base,AuxBoards* auxBoard,SensorType wichSensor)
{
	if (auxBoard->TCAMUXMODE) {
		if ( !auxBoard->openChannel(base,deviceAddress,localPortNum,true)) {
			return false;
		}
	} else if (!I2Cdetect(&auxWire, deviceAddress)) return false;

	// Temperature comepnsation for PH, EC, and DO
	float temperature;
	if (waterTemp_DS18B20.detected) temperature = waterTemp_DS18B20.getReading(base,auxBoard,wichSensor);
	else if (atlasTEMP.detected) {

		if (millis() - atlasTEMP.lastUpdate > 10000) {
			while (atlasTEMP.getBusyState(base,auxBoard,wichSensor)) delay(2);
		}

		char data[10];
		temperature = atlasTEMP.newReading[0];
		sprintf(data,"T,%.2f",temperature);

		if (!sendCommand(base,data,auxBoard,wichSensor)) return false;
	}

	// Salinity compensation only for DO
	if (DO && atlasEC.detected) {

		if (millis() - atlasEC.lastUpdate > 10000) {
			while (atlasEC.getBusyState(base,auxBoard,wichSensor)) delay(2);
		}

		char salData[20];
		float salinity = atlasEC.newReading[2];
		sprintf(salData,"S,%.2f,ppt",salinity);

		if (!sendCommand(base,salData,auxBoard,wichSensor)) return false;
	}

	return true;
}

uint8_t Atlas::getResponse(SckBase* base,AuxBoards* auxBoard,SensorType wichSensor)
{
	if (auxBoard->TCAMUXMODE) {
		if ( !auxBoard->openChannel(base,deviceAddress,localPortNum,true)) {
			return false;
		}
	} else if (!I2Cdetect(&auxWire, deviceAddress)) return false;
	
	uint8_t code;

	auxWire.requestFrom(deviceAddress, 20, 1);
	uint32_t time = millis();
	while (!auxWire.available()) if ((millis() - time)>500) return 0x00;
	code = auxWire.read();

	atlasResponse = "";

	switch (code) {
		case 0: 		// Undefined
		case 2:			// Error
		case 255:		// No data sent
		case 254: {		// Still procesing, not ready

			return code;
			break;

		} default : {

			while (auxWire.available()) {
				char buff = auxWire.read();
				atlasResponse += buff;
			}
			auxWire.endTransmission();

			if (atlasResponse.length() > 0) {
				return 1;
			}

			return 2;
		}
	}
}
#endif
#ifndef CHIRP_DISABLE
bool Moisture::start(SckBase* base,AuxBoards* auxBoard,SensorType wichSensor)
{
	// If we are using a I2C port MUX; then the Port must be enabled prior to calling this library
	if (auxBoard->TCAMUXMODE && deviceAddress != 0x00) {
		localPortNum=auxBoard->findDeviceChan(base,deviceAddress,wichSensor, true,true);
		if (localPortNum > TCA_CHANNEL_7) {
			
			localPortNum=0x00;
			detected=false;
			return false;	// the device was not found 
		} else {			// even if we find a port its no good if we cannt communicate with the device
			if (!I2Cdetect(&auxWire, deviceAddress)) return false;
		}
	} else if (!I2Cdetect(&auxWire, deviceAddress)) return false;

	if (alreadyStarted) return true;

	chirp.begin();

	alreadyStarted = true;

	detected = true;

	return true;
}
bool Moisture::stop()
{
	return true;
}
bool Moisture::getReading(SckBase* base,SensorType wichSensor,AuxBoards* auxBoard)
{
	if (auxBoard->TCAMUXMODE) {
		if ( !auxBoard->openChannel(base,deviceAddress,localPortNum,true)) {
			return false;
		}
	} else if (!I2Cdetect(&auxWire, deviceAddress)) return false;
	
	uint32_t started = millis();
	while (chirp.isBusy()) {
		if (millis() - started > 5000) return 0; 	// Timeout
		delay(1);
	}

	switch(wichSensor) {
		case SENSOR_CHIRP_MOISTURE_RAW: {

			raw = chirp.getCapacitance();
			return true;

		} case SENSOR_CHIRP_MOISTURE: {

			if (!calibrated) return false;
			int32_t thisValue = chirp.getCapacitance();
			moisture = map(thisValue, dryPoint, wetPoint, 0.0, 100.0);
			return true;

		} case SENSOR_CHIRP_TEMPERATURE: {

			temperature = chirp.getTemperature() / 10.0;
			return true;

		} case SENSOR_CHIRP_LIGHT: {

			// From Arduino library documentation
			// The measurement gives 65535 in a dark room away form desk lamp - so more light, lower reading.

			light = chirp.getLight(false); 		// We are sending the reading from previous request
			chirp.startMeasureLight(); 		// Ask for next reading
			return true;

		} default: break;
	}

	return false;
}
uint8_t Moisture::getVersion(SckBase* base,AuxBoards* auxBoard,SensorType wichSensor)
{
	if (auxBoard->TCAMUXMODE) {
		if ( !auxBoard->openChannel(base,deviceAddress,localPortNum,true)) {
			return 0;
		}
	} else if (!I2Cdetect(&auxWire, deviceAddress)) return 0;

	return chirp.getVersion();
}
void Moisture::sleep(SckBase* base,AuxBoards* auxBoard,SensorType wichSensor)
{
	if (auxBoard->TCAMUXMODE) {
		if ( !auxBoard->openChannel(base,deviceAddress,localPortNum,true)) {
			return ;
		}
	} else if (!I2Cdetect(&auxWire, deviceAddress)) return ;

	chirp.sleep();
}
bool Moisture::resetAddress(SckBase* base,int currentAddress,AuxBoards* auxBoard,SensorType wichSensor)
{
	if ( !auxBoard->openChannel(base,deviceAddress,localPortNum,false)) {
		sprintf(base->outBuff, "(Moisture::resetAddress) device inaccessible at Address: 0x%02x",currentAddress);
		base->sckOut();
		
	} else if ( !auxBoard->openChannel(base,0x20,localPortNum,false)) {
		sprintf(base->outBuff, "(Moisture::resetAddress) device inaccessible at Address: 0x%02x",0x20);
		base->sckOut();
		return false;
	}
	
	chirp.changeSensor(currentAddress, true);
	return chirp.setAddress(0x20, true);
}
#endif

// PMSensor is Class used to communicate with PM Board #1 to read and control 2 x PM sensors AND DALLAS TEMP AND (SCKGPS) connected to it.
bool PMsensor::start(SckBase* base,AuxBoards* auxBoard,SensorType wichSensor)
{
	if (started) return true;
	// If we are using a I2C port MUX; then the Port must be enabled prior to calling this library
	if (auxBoard->TCAMUXMODE && deviceAddress != 0x00) {
		localPortNum=auxBoard->findDeviceChan(base,deviceAddress,wichSensor, true,false);
		if (localPortNum > TCA_CHANNEL_7) {
			
			localPortNum=0x00;
			return false;	// the device was not found 
		} else {			// even if we find a port its no good if we cannt communicate with the device
			if (!I2Cdetect(&auxWire, deviceAddress)) return false;
		}
	} else {
		if (!I2Cdetect(&auxWire, deviceAddress) || failed) return false;
	}

	auxWire.beginTransmission(deviceAddress);
	if (_slot == SLOT_A) auxWire.write(START_PMA);
	else if (_slot == SLOT_B) auxWire.write(START_PMB);
	auxWire.endTransmission();
	auxWire.requestFrom(deviceAddress, 1);

	bool result = auxWire.read();

	if (result == 0) failed = true;
	else started = true;
	
	return result;
}
bool PMsensor::stop(SckBase* base,AuxBoards* auxBoard,SensorType wichSensor)
{
	if (auxBoard->TCAMUXMODE) {
		if ( !auxBoard->openChannel(base,deviceAddress,localPortNum,false)) {
			return false;
		}
	} else if (!I2Cdetect(&auxWire, deviceAddress)) return false;

	auxWire.beginTransmission(deviceAddress);
	if (_slot == SLOT_A) auxWire.write(STOP_PMA);
	else if (_slot == SLOT_B) auxWire.write(STOP_PMB);
	auxWire.endTransmission();

	started = false;
	return true;
}
bool PMsensor::update(SckBase* base,AuxBoards* auxBoard,SensorType wichSensor)
{
	if (auxBoard->TCAMUXMODE) {
		if ( !auxBoard->openChannel(base,deviceAddress,localPortNum,false)) {
			return false;
		}
	} else if (!I2Cdetect(&auxWire, deviceAddress)) return false;
	// Only update if more than one second has past
	if (millis() - lastReading > 1000) {

		// Ask for readings
		auxWire.beginTransmission(deviceAddress);
		if (_slot == SLOT_A) auxWire.write(GET_PMA);
		else if (_slot == SLOT_B) auxWire.write(GET_PMB);
		auxWire.endTransmission();
		delay(2);

		// Get the readings
		auxWire.requestFrom(deviceAddress, valuesSize);
		uint32_t time = millis();
		while (!auxWire.available()) if ((millis() - time) > 500) return false;

		// Check for errors
		bool isError = true;
		for (uint8_t i=0; i<valuesSize; i++) {
			values[i] = auxWire.read();
			if (values[i] != 255) isError = false;
		}
		if (isError) return false;

		pm1 = (values[0]<<8) + values[1];
		pm25 = (values[2]<<8) + values[3];
		pm10 = (values[4]<<8) + values[5];
		pn03 = (values[6]<<8) + values[7];
		pn05 = (values[8]<<8) + values[9];
		pn1 = (values[10]<<8) + values[11];
		pn25 = (values[12]<<8) + values[13];
		pn5 = (values[14]<<8) + values[15];
		pn10 = (values[16]<<8) + values[17];

		lastReading = millis();
	}

	return true;
}
// =======
// PM2Sensor is Class used to communicate with PM Board #2 to read and control ANEMOMETER AND RAIN Gauge connected to it via UART ports.
bool PM2sensor::start(SckBase* base,AuxBoards* auxBoard,SensorType wichSensor)
{
	//sprintf(base->outBuff,"PM2sensor start requested for measurement: %s", base->sensors[wichSensor].title);
	//base->sckOut(PRIO_MED,true);
	/* 
	
	BIG NOTE:  The device takes around 950 mS to go through the setup routine.

	If this ::start function is executed soon after power up of the system as a whole; then
	not enough time may have elapsed for it to complete setup.
	And the consequence is that the first one or two commands (WIND_START etc)
	may fail.

	Thus, first time around; it is necessary for us to include a delay right at the start; please do not remove it.

	*/
	if (started) {
		return true;
	} else {
		delay(1200); 		// if start command is requested immediately after a power cycle then
							// we must wait for PM2 device to send startup commands to each serially connected device.
							// otherwise the start command will always fail
	}

	// If we are using a I2C port MUX; then the Port must be enabled prior to I2C communication
	if (auxBoard->TCAMUXMODE && deviceAddress != 0x00) {
		localPortNum=auxBoard->findDeviceChan(base,deviceAddress,wichSensor, true,false);
		if (localPortNum > TCA_CHANNEL_7) {
			localPortNum=0x00;
			//base->sckOut("I2C Port out of range");
			return false;	// the device was not found 
		} else if (!I2Cdetect(&auxWire, deviceAddress)) {
			//base->sckOut("Device not detected on the bus");
			return false;
		//} else {
			//base->sckOut("A Mux port was identified and verified", PRIO_MED,true);
		}
	} else if (!I2Cdetect(&auxWire, deviceAddress)) {
		return false;
	//} else {
		//base->sckOut("the device is connected to Aux Bus and working", PRIO_MED,true);
	}

	//sprintf(base->outBuff,"start requested for #: %i", wichSensor);
	//base->sckOut(PRIO_MED,false);
	//sprintf(base->outBuff," aka: %s", base->sensors[wichSensor].title);
	//base->sckOut(PRIO_MED,true);

	//ReadingSize readingsize = getReadingSize(wichSensor);

	
	auxWire.beginTransmission(deviceAddress);
	switch (wichSensor) {
		// Calypso ULP Anemometer
		case SENSOR_WIND_DIR: 
		case SENSOR_WIND_SPEED: {
			auxWire.write(COMM_START_WIND);
			break;
		}

		// Radeon ULP IR Rain Gauge
		case SENSOR_RAIN_ACC: 
		case SENSOR_RAIN_EVENTACC: 
		case SENSOR_RAIN_TOTALACC: 
		case SENSOR_RAIN_INTERVAL: {
			auxWire.write(COMM_START_RAIN);
			break;
		}
		// unrecognised sensor
		default: {
			break;
		}
	}
	bool error = auxWire.endTransmission();
	uint32_t timer=micros();
	uint32_t timeout=1000;// (1mS)
	bool error2=false;
	uint8_t x=0;
	if (error == 0) {
		//base->sckOut("I2C transaction: command was successfully sent: requesting response",PRIO_MED,true);
		auxWire.requestFrom(deviceAddress,0x01,true);	// request 1 Byte (for start command) acknowledgement and then stop
		while (!auxWire.available()) {
			delayMicroseconds(1);
			if (micros()-timer > timeout){
				//base->sckOut("timeout occured waiting for a response to request",PRIO_MED,true);
				error2=true;
				break;
			}

		}
		// Testing indicates that inevitably the very first request will result in a timeout.
		
		if (error2) {		// therefore we retry immediately
			auxWire.requestFrom(deviceAddress,1,true);	// request acknowledgement and then stop
			while (!auxWire.available()) {
				delayMicroseconds(1);
				if (micros()-timer > timeout){
					//base->sckOut("timeout occured on retry waiting for a response to request",PRIO_LOW,true);
					error2=true;
					break;
				}
			}
		} else {
			while (auxWire.available()) {
				auxWire.read();		// just emptying the read buffer at this point : we do not need the data
				x++;
			}
		}
	} else {
		//base->sckOut("error sending start command",PRIO_MED,true);
	}

	if (x==0  || error2 || error) {
		failed = true;
		return false;
	} else {
		//sprintf(base->outBuff,"PM2sensor has started %s", base->sensors[wichSensor].title);
		//base->sckOut(PRIO_MED,true);
		for (uint8_t i=0; i<6; i++) if (enabled[i][0] == wichSensor) enabled[i][1] = 1;
		started = true;
	}

	return started;
}
bool PM2sensor::stop(SckBase* base,AuxBoards* auxBoard,SensorType wichSensor)
{
	if (auxBoard->TCAMUXMODE) {
		if ( !auxBoard->openChannel(base,deviceAddress,localPortNum,false)) {
			return false;
		}
	} else if (!I2Cdetect(&auxWire, deviceAddress)) return false;

	auxWire.beginTransmission(deviceAddress);
	switch (wichSensor) {
		// Calypso ULP Anemometer
		case SENSOR_WIND_DIR: 
		case SENSOR_WIND_SPEED: {
			auxWire.write(COMM_STOP_WIND);
			
			break;
		}

		// Radeon ULP IR Rain Gauge
		case SENSOR_RAIN_ACC: 
		case SENSOR_RAIN_EVENTACC: 
		case SENSOR_RAIN_TOTALACC: 
		case SENSOR_RAIN_INTERVAL: {
			auxWire.write(COMM_STOP_RAIN);
			
			break;
		}
		// unrecognised sensor
		default: {
			break;
		}
	}
	auxWire.endTransmission();

	started = false;
	return true;
}

ReadingSize PM2sensor::getReadingSize(SensorType wichSensor) {
	for (uint8_t i=0;i<NUM_COMMANDS;i++) {
		if (v_readingFetchSizes[i].command == wichSensor) return v_readingFetchSizes[i];
	}
	// else
	return v_readingFetchSizes[0];
}

void PM2sensor::setLastReading(SensorType wichSensor,PM2sensorCommand wichCommand) {
	
	//ReadingSize readingsize = getReadingSize( wichSensor);
	uint8_t i = 0;
	for ( i=0;i<NUM_COMMANDS;i++) {
		if (v_readingFetchSizes[i].command == wichSensor) {
			v_readingFetchSizes[i].lastReadingTime=millis();
			break;
		}
	}
	return ;

}

bool PM2sensor::update(SckBase* base,AuxBoards* auxBoard,SensorType wichSensor)
{
	//sprintf(base->outBuff,"PM2 sensor reading %s",base->sensors[wichSensor].title);
	//base->sckOut(PRIO_MED,true);
	
	if (auxBoard->TCAMUXMODE) {
		if ( !auxBoard->openChannel(base,deviceAddress,localPortNum,false)) {
			//base->sckOut("could not open a channel");
			return false;
		}
	} else if (!I2Cdetect(&auxWire, deviceAddress)) {
		return false;
	}
	
	
	//base->sckOut("PM2Sensor update connected and executing",PRIO_MED,true);
	/*sprintf(base->outBuff,"Update requested for #: %i", wichSensor);
	base->sckOut(PRIO_MED,false);
	sprintf(base->outBuff," aka: %s", base->sensors[wichSensor].title);
	base->sckOut(PRIO_MED,true);
	*/
	ReadingSize readingsize = getReadingSize(wichSensor);
	/*
	sprintf(base->outBuff,"Reading command: %i",readingsize.command);
	base->sckOut(PRIO_MED,false);
	sprintf(base->outBuff,": sensor: %i",readingsize.sensor);
	base->sckOut(PRIO_MED,false);
	sprintf(base->outBuff,": size: %i",readingsize.bytes);
	base->sckOut(PRIO_MED,true);
	*/	
	// Only update if more than one second has past
	if (millis() - readingsize.lastReadingTime > 1000) {
		//base->sckOut("PM2Sensor commanding that a reading be taken");
		// Ask for readings steps:
		// open the connection: 				: auxWire.beginTransmission(deviceAddress);
		// send a command 						: auxWire.write(<commandbyte>>); (tell slave to prepare the data for sending)
		// close connection 					: auxWire.endTransmission();
		// request the prepared data be sent 	: auxWire.requestFrom(deviceAddress, <no of bytes>>);
		
		PM2sensorCommand command=readingsize.sensor;
		auxWire.beginTransmission(deviceAddress);
		auxWire.write(readingsize.sensor);						// the sensor command
		auxWire.endTransmission();								// should be no need to insert extra delays
		auxWire.requestFrom(deviceAddress,readingsize.bytes,true);	// ask device to send N bytes 
		
		// wait up to 0.5seconds for a response (a blocking call !)
		uint32_t time = millis();
		while (!auxWire.available()) {
			if ((micros() - time) > 20) {
				//base->sckOut("Timeout waiting for a response to requestFrom()",PRIO_MED,true);
				return false;
			}
		}

		// Check for errors
		uint8_t i=0;
		while (auxWire.available()) {
			myreading.b[i] = auxWire.read();	// reading (4) bytes one by one comprising a float (but could be more or less data)
			i++;
		}
		if (i != (readingsize.bytes)  ) {
			//sprintf(base->outBuff,"ERROR: Incorrect number of Bytes received: %i (expected %i)",i,readingsize.bytes);
			//base->sckOut(PRIO_MED,true);
			return false;
		}
		//base->sckOut("Reading OK; decoding it:",PRIO_MED,false);
		//sprintf(base->outBuff,"Reading value %f", myreading.f);
		////base->sckOut(PRIO_MED,true);
		// record the reading time :
		setLastReading( wichSensor, command);
		if (!isnan(myreading.f)) {		// check to ensure all 4 bytes have been reeived
			switch (wichSensor) {
				// Calypso ULP Anemometer
				case SENSOR_WIND_DIR: {
					windDir=String(myreading.f);		// convert char array into a String using overloaded = oerator
					break;
				}
				case SENSOR_WIND_SPEED: {
					windSpeed=String(myreading.f);
					break;
				}
				// Radeon ULP IR Rain Gauge
				case SENSOR_RAIN_ACC: {
					rainAcc=String(myreading.f);	
					break;
				}
				case SENSOR_RAIN_EVENTACC: {
					rainEventAcc=String(myreading.f);	
					break;
				}
				case SENSOR_RAIN_TOTALACC: {
					rainTotalAcc=String(myreading.f);	
					break;
				}
				case SENSOR_RAIN_INTERVAL: {
					rainIntAcc=String(myreading.f);	
					break;
				}
				// unrecognised sensor
				default: {
					//base->sckOut("ERROR: incorrect sensor request",PRIO_MED,true);
					break;
				}
			}
		} else {
			//base->sckOut("ERROR: reading value returned was not a float (isNan)",PRIO_MED,true);
			return false;
		}
		
	//} else {
		//base->sckOut("It is not yet time for the next reading",PRIO_MED,true);
	}
	//base->sckOut("reading value was OK",PRIO_MED,true);
	return true;
}


// PM_DallasTemp is a temperature probe connected to a GPIO port on PM Board #1
// communication is via I2C -> PM Board -> Sensor
bool PM_DallasTemp::start(SckBase* base,AuxBoards* auxBoard,SensorType wichSensor)
{
	//base->sckOut("Dallas:: I have been asked to start", PRIO_LOW,true);
	// If we are using a I2C port MUX; then the Port must be enabled prior to calling this library
	if (auxBoard->TCAMUXMODE && deviceAddress != 0x00) {
		localPortNum=auxBoard->findDeviceChan(base,deviceAddress,wichSensor, true,false);
		if (localPortNum > TCA_CHANNEL_7) {
			localPortNum=0x00;
			//base->sckOut("Dallas:: sorry, but I could not get a good channel", PRIO_LOW,true);
			return false;	// the device was not found 
		} else {			// even if we find a port its no good if we cannt communicate with the device
			if (!I2Cdetect(&auxWire, deviceAddress)) return false;
			//base->sckOut("Dallas:: sorry, but I am not listening", PRIO_LOW,true);
		}
	} else if (!I2Cdetect(&auxWire, deviceAddress)) {	
		//base->sckOut("Dallas:: sorry, but I am not listening", PRIO_LOW,true);
		return false;
	}
	//base->sckOut("It seems we can talk to the Dallas Probe via PMB1", PRIO_LOW,true);

	auxWire.beginTransmission(deviceAddress);
	auxWire.write(DALLASTEMP_START);			// starting a conversation with PMB1
	auxWire.endTransmission();
	auxWire.requestFrom(deviceAddress, 1,true);		// requesting a confirmation result and a stop signal so Bus is not hung up

	bool result = auxWire.read();
	//if (result){
		//base->sckOut("Dallas Probe via PMB1 is started", PRIO_LOW,true);
	//} else {
	//	base->sckOut("Dallas Probe via PMB1 is NOT started", PRIO_LOW,true);
	//}
	return result;
}
bool PM_DallasTemp::stop(SckBase* base,AuxBoards* auxBoard,SensorType wichSensor)
{
	if (auxBoard->TCAMUXMODE) {
		if ( !auxBoard->openChannel(base,deviceAddress,localPortNum,false)) {
			return false;
		}
	} else if (!I2Cdetect(&auxWire, deviceAddress)) return false;

	auxWire.beginTransmission(deviceAddress);
	auxWire.write(DALLASTEMP_STOP);
	auxWire.endTransmission();
	auxWire.requestFrom(deviceAddress, 1);

	bool result = auxWire.read();
	return result;
}
float PM_DallasTemp::getReading(SckBase* base,AuxBoards* auxBoard,SensorType wichSensor)
{
	if (auxBoard->TCAMUXMODE) {
		if ( !auxBoard->openChannel(base,deviceAddress,localPortNum,false)) {
			return false;
		}
	} else if (!I2Cdetect(&auxWire, deviceAddress)) return false;

	auxWire.beginTransmission(deviceAddress);
	auxWire.write(GET_DALLASTEMP);
	auxWire.endTransmission();

	// Get the reading
	auxWire.requestFrom(deviceAddress, 4);
	uint32_t start = millis();
	while (!auxWire.available()) if ((millis() - start)>500) return -9999;
	for (uint8_t i=0; i<4; i++) uRead.b[i] = auxWire.read();
	return uRead.fval;
}
// tucked away here in Global space:
TinyGPSPlus tinyGps;
TinyGPSCustom fixQuality(tinyGps, "GPGGA", 6);
TinyGPSCustom nfixQuality(tinyGps, "GNGGA", 6);

//Sck_GPS is a consolidation class for thre several possible GPS sensors; combining them into one for reading purpose
bool Sck_GPS::start(SckBase* base,AuxBoards* auxBoard,SensorType wichSensor)
{
	if (started) return true;

	if (neoM8uGps.start(base,auxBoard,wichSensor)) {
		gps_source = &neoM8uGps;
		started = true;
		return true;
	}

	if (xa1110gps.start(base,auxBoard,wichSensor)) {
		gps_source = &xa1110gps;
		started = true;
		return true;
	}

	if (pmGroveGps.start(base,auxBoard,wichSensor)) {
		gps_source = &pmGroveGps;
		started = true;
		return true;
	}

	return false;
}
bool Sck_GPS::stop(SckBase* base,AuxBoards* auxBoard,SensorType wichSensor)
{
	if (!started) return true;

	gps_source->stop(base,auxBoard,wichSensor);
	started = false;

	return true;
}
bool Sck_GPS::getReading(SckBase* base,AuxBoards* auxBoard, SensorType wichSensor)
{
	// Use time from gps to set RTC if time is not set or older than 1 hour
	if (((millis() - base->lastTimeSync) > 3600000 || base->lastTimeSync == 0)) {

		if (gps_source->getReading(base,auxBoard,SENSOR_GPS_FIX_QUALITY, r)) {
			if (r.fixQuality > 0 && r.timeValid) {
				// Wait for some GPS readings after sync to be sure time is accurate
				if (fixCounter > 5) base->setTime(String(r.epochTime));
				else fixCounter++;
			}
		}

	} else {
		fixCounter = 0;
	}

	if (!gps_source->getReading(base,auxBoard,wichSensor, r)) return false;

	return true;
}
bool Sck_GPS::update(SckBase* base,AuxBoards* auxBoard,SensorType wichSensor)
{
	return gps_source->update(base,auxBoard,wichSensor);
}
bool PM_Grove_GPS::start(SckBase* base,AuxBoards* auxBoard,SensorType wichSensor)
{
	// If we are using a I2C port MUX; then the Port must be enabled prior to calling this library
	if (auxBoard->TCAMUXMODE && deviceAddress != 0x00) {
		localPortNum=auxBoard->findDeviceChan(base,deviceAddress,wichSensor, true,false);
		if (localPortNum > TCA_CHANNEL_7) {
			
			localPortNum=0x00;
			return false;	// the device was not found 
		} else {			// even if we find a port its no good if we cannt communicate with the device
			if (!I2Cdetect(&auxWire, deviceAddress)) return false;
		}
	} else if (!I2Cdetect(&auxWire, deviceAddress)) return false;

	auxWire.beginTransmission(deviceAddress);
	auxWire.write(GROVEGPS_START);
	auxWire.endTransmission();
	auxWire.requestFrom(deviceAddress, 1);

	bool result = auxWire.read();

	return result;
}
bool PM_Grove_GPS::stop(SckBase* base,AuxBoards* auxBoard,SensorType wichSensor)
{
	if (auxBoard->TCAMUXMODE) {
		if ( !auxBoard->openChannel(base,deviceAddress,localPortNum,false)) {
			return false;
		}
	} else if (!I2Cdetect(&auxWire, deviceAddress)) return false;

	auxWire.beginTransmission(deviceAddress);
	auxWire.write(GROVEGPS_STOP);
	auxWire.endTransmission();

	return true;
}
bool PM_Grove_GPS::getReading(SckBase* base,AuxBoards* auxBoard,SensorType wichSensor, GpsReadings &r)
{
	if (auxBoard->TCAMUXMODE) {
		if ( !auxBoard->openChannel(base,deviceAddress,localPortNum,false)) {
			return false;
		}
	} else if (!I2Cdetect(&auxWire, deviceAddress)) return false;

	//  Only ask for readings if last one is older than
	if (millis() - lastReading < 500 && r.fixQuality > 0) return true;

	// Ask for reading
	auxWire.beginTransmission(deviceAddress);
	auxWire.write(GROVEGPS_GET);
	auxWire.endTransmission();

	// Get the reading
	auxWire.requestFrom(deviceAddress, DATA_LEN);
	uint32_t time = millis();
	while (!auxWire.available()) if ((millis() - time)>500) return false;

	for (uint8_t i=0; i<DATA_LEN; i++) data[i] = auxWire.read();

	// Fix quality
	memcpy(&r.fixQuality, &data[0], 1);

	// Time
	memcpy(&r.timeValid, &data[23], 1);
	if (r.timeValid) memcpy(&r.epochTime, &data[24], 4);
	// With this GPS wrong time is reported as Valid when no GPS fix
	// So if no fix we mark time as invalid
	if (r.fixQuality == 0) r.timeValid = false;

	// Location
	memcpy(&r.locationValid, &data[1], 1);
	if (r.locationValid) {

		// Latitude
		memcpy(&r.latitude, &data[2], 8);

		// Longitude
		memcpy(&r.longitude, &data[10], 8);

	} else if (wichSensor == SENSOR_GPS_LATITUDE ||	wichSensor == SENSOR_GPS_LONGITUDE) return false;

	// Altitude
	memcpy(&r.altitudeValid, &data[18], 1);
	if (r.altitudeValid) memcpy(&r.altitude, &data[19], 4);
	else if (wichSensor == SENSOR_GPS_ALTITUDE) return false;

	// Speed
	memcpy(&r.speedValid, &data[28], 1);
	if (r.speedValid) memcpy(&r.speed, &data[29], 4);
	else if (wichSensor == SENSOR_GPS_SPEED) return false;

	// Horizontal dilution of position
	memcpy(&r.hdopValid, &data[33], 1);
	if (r.hdopValid) memcpy(&r.hdop, &data[34], 4);
	else if (wichSensor == SENSOR_GPS_HDOP) return false;

	// Satellites
	memcpy(&r.satellitesValid, &data[38], 1);
	if (r.satellitesValid) memcpy(&r.satellites, &data[39], 1);
	else if (wichSensor == SENSOR_GPS_SATNUM) return false;

	lastReading = millis();

	return true;
}
bool PM_Grove_GPS::update(SckBase* base,AuxBoards* auxBoard,SensorType wichSensor)
{
	return true;
}
bool XA111GPS::start(SckBase* base,AuxBoards* auxBoard,SensorType wichSensor)
{
	// If we are using a I2C port MUX; then the Port must be enabled prior to calling this library
	if (auxBoard->TCAMUXMODE && deviceAddress != 0x00) {
		localPortNum=auxBoard->findDeviceChan(base,deviceAddress,wichSensor, true,false);
		if (localPortNum > TCA_CHANNEL_7) {
			
			localPortNum=0x00;
			return false;	// the device was not found 
		} else {			// even if we find a port its no good if we cannt communicate with the device
			if (!I2Cdetect(&auxWire, deviceAddress)) return false;
		}
	} else if (!I2Cdetect(&auxWire, deviceAddress)) return false;

	if (!i2cGps.begin(auxWire)) return false;

	return true;
}
bool XA111GPS::stop(SckBase* base,AuxBoards* auxBoard,SensorType wichSensor)
{
	return true;
}
bool XA111GPS::getReading(SckBase* base,AuxBoards* auxBoard,SensorType wichSensor, GpsReadings &r)
{
	if (auxBoard->TCAMUXMODE ) {
		if ( !auxBoard->openChannel(base,deviceAddress,localPortNum,false)) {
			return false;
		}
	} else if (!I2Cdetect(&auxWire, deviceAddress)) return false;

	//  Only ask for readings if last one is older than
	if (millis() - lastReading < 500) return true;

	// TODO
	// this was moved to update funtion, check if it works OK
	/* while (i2cGps.available()) tinyGps.encode(i2cGps.read()); */

	// Time
	r.timeValid = tinyGps.time.isValid();
	if (r.timeValid) {
		// Time (epoch) -> uint32 - 4
		struct tm tm; 				// http://www.nongnu.org/avr-libc/user-manual/structtm.html
		tm.tm_isdst = -1; 			// -1 means no data available
		tm.tm_yday = 0;
		tm.tm_wday = 0;
		tm.tm_year = tinyGps.date.year() - 1900; 	// tm struct expects years since 1900
		tm.tm_mon = tinyGps.date.month() - 1; 	// tm struct uses 0-11 months
		tm.tm_mday = tinyGps.date.day();
		tm.tm_hour = tinyGps.time.hour();
		tm.tm_min = tinyGps.time.minute();
		tm.tm_sec = tinyGps.time.second();
		r.epochTime = mktime(&tm);
	}

	// Fix Quality
	String fixQual = fixQuality.value();
	r.fixQuality = fixQual.toInt();
	if (r.fixQuality == 0) {
		fixQual = nfixQuality.value();
		r.fixQuality = fixQual.toInt();
	}

	// Location
	r.locationValid = tinyGps.location.isValid();
	if (r.locationValid) {

		// Latitude
		r.latitude = tinyGps.location.lat();

		// Longitude
		r.longitude = tinyGps.location.lng();

	} else if (wichSensor == SENSOR_GPS_LATITUDE ||	wichSensor == SENSOR_GPS_LONGITUDE) return false;

	// Altitude
	r.altitudeValid = tinyGps.altitude.isValid();
	if (r.altitudeValid) r.altitude = tinyGps.altitude.meters();
	else if (wichSensor == SENSOR_GPS_ALTITUDE) return false;

	// Speed
	r.speedValid = tinyGps.speed.isValid();
	if (r.speedValid) r.speed = tinyGps.speed.mps();
	else if (wichSensor == SENSOR_GPS_SPEED) return false;

	// Horizontal dilution of position
	r.hdopValid = tinyGps.hdop.isValid();
	if (r.hdopValid) r.hdop = tinyGps.hdop.value();
	else if (wichSensor == SENSOR_GPS_HDOP) return false;

	// Satellites
	r.satellitesValid = tinyGps.satellites.isValid();
	if (r.satellitesValid) r.satellites = tinyGps.satellites.value();
	else if (wichSensor == SENSOR_GPS_SATNUM) return false;

	lastReading = millis();

	// TODO use power save mode between readings if posible

	return true;
}
bool XA111GPS::update(SckBase* base,AuxBoards* auxBoard,SensorType wichSensor)
{
	if (auxBoard->TCAMUXMODE ) {
		if ( !auxBoard->openChannel(base,deviceAddress,localPortNum,false)) {
			return false;
		}
	} else if (!I2Cdetect(&auxWire, deviceAddress)) return false;
	// Test with the GPS if this make sense here
	while (i2cGps.available()) tinyGps.encode(i2cGps.read());

	return true;
}
bool NEOM8UGPS::start(SckBase* base,AuxBoards* auxBoard,SensorType wichSensor)
{
	// If we are using a I2C port MUX; then the Port must be enabled prior to calling this library
	if (auxBoard->TCAMUXMODE ) {
		localPortNum=auxBoard->findDeviceChan(base,deviceAddress,wichSensor, true,false);
		if (localPortNum > TCA_CHANNEL_7) {
			localPortNum=0x00;
			return false;	// the device was not found 
		} else {			// even if we find a port its no good if we cannt communicate with the device
			if (!I2Cdetect(&auxWire, deviceAddress)) return false;
		}
	} else if (!I2Cdetect(&auxWire, deviceAddress)) return false;

	if (!ubloxGps.begin(auxWire)) return false;

	// Lowpower mode Off
	ubloxGps.powerSaveMode(false, 1);	// turn off power save mode
	ubloxGps.setUART1Output(0); 		// Disable the UART1 port output
	ubloxGps.setUART2Output(0); 		// Disable Set the UART2 port output
	ubloxGps.setI2COutput(COM_TYPE_UBX); 	// Set the I2C port to output UBX only (turn off NMEA noise)
	ubloxGps.setNavigationFrequency(4);
	ubloxGps.setAutoPVT(true); 		// Tell the GPS to "send" each solution
	ubloxGps.saveConfiguration(); 		// Save the current settings to flash and BBR

	auxWire.endTransmission();

	return true;
}
bool NEOM8UGPS::stop(SckBase* base,AuxBoards* auxBoard,SensorType wichSensor)
{
	if (auxBoard->TCAMUXMODE ) {
		if ( !auxBoard->openChannel(base,deviceAddress,localPortNum,false)) {
			return false;
		}
	}
	if (!I2Cdetect(&auxWire, deviceAddress)) return false;
	
	// TODO (done ?)
	// Turn ON Lowpower mode
	bool result=ubloxGps.powerSaveMode(true, 1);
	auxWire.endTransmission();

	return result;
}
bool NEOM8UGPS::getReading(SckBase* base,AuxBoards* auxBoard,SensorType wichSensor, GpsReadings &r)
{
	if (auxBoard->TCAMUXMODE ) {
		if ( !auxBoard->openChannel(base,deviceAddress,localPortNum,false)) {
			return false;
		}
	} else if (!I2Cdetect(&auxWire, deviceAddress)) return false;

	//base->sckOut("NEO GPS connectivity OK",PRIO_MED,true);

	switch(wichSensor) {

		case SENSOR_GPS_FIX_QUALITY:
		{

			// Time
			if (ubloxGps.getDateValid() && ubloxGps.getTimeValid()) {
				// Time (epoch) -> uint32 - 4
				struct tm tm; 					// http://www.nongnu.org/avr-libc/user-manual/structtm.html
				tm.tm_isdst = -1; 				// -1 means no data available
				tm.tm_yday = 0;
				tm.tm_wday = 0;
				tm.tm_year = ubloxGps.getYear() - 1900; 	// tm struct expects years since 1900
				tm.tm_mon = ubloxGps.getMonth() - 1; 		// tm struct uses 0-11 months
				tm.tm_mday = ubloxGps.getDay();
				tm.tm_hour = ubloxGps.getHour();
				tm.tm_min = ubloxGps.getMinute();
				tm.tm_sec = ubloxGps.getSecond();
				r.timeValid = true;
				r.epochTime = mktime(&tm);
			} else {
				r.timeValid = false;
			}

			uint8_t fixQual = ubloxGps.getFixType(); 		// Type of fix: 0=no, 3=3D, 4=GNSS+Deadreckoning */
			// TODO
			// Translate fix quality to NMEA standard
			r.fixQuality = fixQual;
			break;

		}
		case SENSOR_GPS_LATITUDE:
		case SENSOR_GPS_LONGITUDE:
		{
			// Location
			r.locationValid = true;
			// Latitude
			r.latitude = (float)ubloxGps.getLatitude() / 10000000.0;
			// Longitude
			r.longitude = (float)ubloxGps.getLongitude() / 10000000.0;
			break;
		}
		case  SENSOR_GPS_ALTITUDE:
		{
			// Altitude
			// TODO check if mean sea level option (getAltitudeMSL()) is better for us
			r.altitudeValid = true;
			r.altitude = (float)ubloxGps.getAltitude() / 1000.0;
			break;
		}
		case SENSOR_GPS_SPEED:
		{
			// Speed
			r.speedValid = true;
			r.speed = (float)ubloxGps.getGroundSpeed() / 1000.0;
			break;
		}
		case SENSOR_GPS_HDOP:
		{
			// Horizontal dilution of position
			r.hdopValid = true;
			r.hdop = ubloxGps.getPDOP();
			break;
		}
		case SENSOR_GPS_SATNUM:
		{
			// Satellites
			r.satellitesValid = true;
			r.satellites = ubloxGps.getSIV();
			break;
		}
		default:
			break;
	}

	lastReading = millis();
	// conjecture: sometimes on exit; the uBlox library is forgetting to end the transmission
	// having left it  (holding the bus open).
	auxWire.endTransmission();

	// TODO use power save mode between readings if posible

	return true;
}
bool NEOM8UGPS::update(SckBase* base,AuxBoards* auxBoard,SensorType wichSensor)
{
	if (auxBoard->TCAMUXMODE ) {
		if ( !auxBoard->openChannel(base,deviceAddress,localPortNum,false)) {
			return false;
		}
	} else if (!I2Cdetect(&auxWire, deviceAddress)) return false;
	
	ubloxGps.checkUblox();
	delay(1);
	/*
	if (ubloxGps.getPVT()) {
		base->sckOut("Update finished; returning data");
		return true;
	} else {
		base->sckOut("Update finished; ubloxGps returned no data");
		return false;
	}*/
	bool response=ubloxGps.getPVT();;
	auxWire.endTransmission();
	return response;
}
#ifndef MISC_DISABLE
bool Sck_DallasTemp::start()
{
	
	pinPeripheral(pinAUX_WIRE_SCL, PIO_DIGITAL);
	OneWire oneWire = OneWire(pinAUX_WIRE_SCL);
	DallasTemperature _dallasTemp = DallasTemperature(&oneWire);

	_dallasTemp.begin();

	// If no device is found return false
	_dallasTemp.getAddress(_oneWireAddress, 0);

	_dallasTemp.setResolution(12);
	_dallasTemp.setWaitForConversion(true);

	if (!getReading()) return false;

	pinPeripheral(pinAUX_WIRE_SCL, PIO_SERCOM);

	return true;
}
bool Sck_DallasTemp::stop()
{

	return true;
}
bool Sck_DallasTemp::getReading()
{
	pinPeripheral(pinAUX_WIRE_SCL, PIO_DIGITAL);
	OneWire oneWire = OneWire(pinAUX_WIRE_SCL);
	DallasTemperature _dallasTemp = DallasTemperature(&oneWire);

	_dallasTemp.requestTemperatures();
	reading = _dallasTemp.getTempC(_oneWireAddress);
	if (reading <= DEVICE_DISCONNECTED_C) return false;

	pinPeripheral(pinAUX_WIRE_SCL, PIO_SERCOM);

	return true;
}
bool Sck_Range::start(SckBase* base,AuxBoards* auxBoard,SensorType wichSensor)
{
	if (alreadyStarted) return true;
	// If we are using a I2C port MUX; then the Port must be enabled prior to calling this library
	if (auxBoard->TCAMUXMODE ) {
		localPortNum=auxBoard->findDeviceChan(base,deviceAddress,wichSensor, true,true);
		if (localPortNum > TCA_CHANNEL_7) {
			localPortNum=0x00;
			return false;	// the device was not found 
		} else {			// even if we find a port its no good if we cannt communicate with the device
			if (!I2Cdetect(&auxWire, deviceAddress)) return false;
		}
	} else if (!I2Cdetect(&auxWire, deviceAddress)) return false;
	
	
	if(vl6180x.VL6180xInit() != 0) return false;

	vl6180x.VL6180xDefautSettings();

	alreadyStarted = true;

	return true;
}
bool Sck_Range::stop()
{
	alreadyStarted = false;
	return true;
}
bool Sck_Range::getReading(SckBase* base,AuxBoards* auxBoard,SensorType wichSensor)
{
	if (auxBoard->TCAMUXMODE ) {
		if ( !auxBoard->openChannel(base,deviceAddress,localPortNum,true)) {
			return false;
		}
	} else if (!I2Cdetect(&auxWire, deviceAddress)) return false;

	switch(wichSensor)
	{
		case SENSOR_RANGE_DISTANCE:
			readingDistance = vl6180x.getDistance();
			break;
		case SENSOR_RANGE_LIGHT:
			readingLight = vl6180x.getAmbientLight(GAIN_1);
			break;
		default:
			return false;
	}
	return true;
}

bool Sck_BME680::start(SckBase* base,AuxBoards* auxBoard,SensorType wichSensor)
{
	if (alreadyStarted) return true;

	if (auxBoard->TCAMUXMODE ) {
		localPortNum=auxBoard->findDeviceChan(base,deviceAddress,wichSensor, true,false);
		if (localPortNum > TCA_CHANNEL_7) {
			localPortNum=0x00;
			return false;	// the device was not found 
		} else {			// even if we find a port its no good if we cannt communicate with the device
			if (!I2Cdetect(&auxWire, deviceAddress)) return false;
		}
	} else if (!I2Cdetect(&auxWire, deviceAddress)) return false;	

	if (!bme.begin(deviceAddress)) return false;

	alreadyStarted = true;
	return true;
}

bool Sck_BME680::stop()
{
	alreadyStarted = false;
	return true;
}

bool Sck_BME680::getReading(SckBase* base,AuxBoards* auxBoard,SensorType wichSensor)
{
	if (auxBoard->TCAMUXMODE ) {
		if ( !auxBoard->openChannel(base,deviceAddress,localPortNum,false)) {
			return false;
		}
	} else if (!I2Cdetect(&auxWire, deviceAddress)) return false;

	if (millis() - lastTime > minTime) {
		if (!bme.performReading()) return false;
		lastTime = millis();
	}

	temperature = bme.temperature;
	humidity = bme.humidity;
	pressure = bme.pressure / 1000;  // Converted to kPa
	VOCgas = bme.gas_resistance;

	return true;
}
#endif
bool Sck_ADS1X15::start(SckBase* base,uint8_t address,AuxBoards* auxBoard,SensorType wichSensor)
{
	// If we are using a I2C port MUX; then the Port must be enabled prior to calling this library
	if (auxBoard->TCAMUXMODE ) {
		localPortNum=auxBoard->findDeviceChan(base,deviceAddress,wichSensor, true,true);
		if (localPortNum > TCA_CHANNEL_7) {
			localPortNum=0x00;
			return false;	// the device was not found 
		} else {			// even if we find a port its no good if we cannt communicate with the device
			if (!I2Cdetect(&auxWire, address)) return false;
		}
	} else if (!I2Cdetect(&auxWire, address)) return false;

	if (started) return true;

	ads.begin(address);
	started = true;
	return true;
}

bool Sck_ADS1X15::stop()
{
	started = false;
	return true;
}

bool Sck_ADS1X15::getReading(SckBase* base,AuxBoards* auxBoard,uint8_t wichChannel,SensorType wichSensor)
{
	if (auxBoard->TCAMUXMODE ) {
		if ( !auxBoard->openChannel(base,deviceAddress,localPortNum,true)) {
			return false;
		}
	} else if (!I2Cdetect(&auxWire, deviceAddress)) return false;

	// Reset gain
	ads.setGain(GAIN_TWOTHIRDS);
	double voltage_range = 6.144;

	// Get value with full range
	uint16_t value = ads.readADC_SingleEnded(wichChannel);

	// If value is under 4.096v increase the gain depending on voltage
	if (value < 21845) {
		if (value > 10922) {

			// 1x gain, 4.096V
			ads.setGain(GAIN_ONE);
			voltage_range = 4.096;

		} else if (value > 5461) {

			// 2x gain, 2.048V
			ads.setGain(GAIN_TWO);
			voltage_range = 2.048;

		} else if (value > 2730) {

			// 4x gain, 1.024V
			ads.setGain(GAIN_FOUR);
			voltage_range = 1.024;

		} else if (value > 1365) {

			// 8x gain, 0.25V
			ads.setGain(GAIN_EIGHT);
			voltage_range = 0.512;

		} else {

			// 16x gain, 0.125V
			ads.setGain(GAIN_SIXTEEN);
			voltage_range = 0.256;
		}

		// Get the value again
		value = ads.readADC_SingleEnded(wichChannel);
	}

	reading = (float)value / 32768 * voltage_range;

	return true;
}

#ifdef adsTest
void Sck_ADS1X15::setTesterCurrent(int16_t wichCurrent, uint8_t wichChannel)
{
	// Support both combinations of ADC channels:
	// wichChannel = 0 (default) -> WE in ADS_Ch0 and AE in ADS_Ch1
	// wichChannel = 1 			 -> WE in ADS_Ch2 and AE in ADS_Ch3
	if (wichChannel > 0) {
		adsChannelW = 2;
		adsChannelA = 3;
	}

	SerialUSB.print("Setting test current to: ");
	SerialUSB.println(wichCurrent);

	tester.setCurrent(tester.electrode_W, wichCurrent);
	tester.setCurrent(tester.electrode_A, wichCurrent);

	SerialUSB.print("Tester Electrode W: ");
	SerialUSB.println(tester.getCurrent(tester.electrode_W));
	SerialUSB.print("ISB W:");
	this->getReading(adsChannelW);
	SerialUSB.println(this->reading);

	SerialUSB.print("Tester Electrode A: ");
	SerialUSB.println(tester.getCurrent(tester.electrode_A));
	SerialUSB.print("ISB A: ");
	this->getReading(adsChannelA);
	SerialUSB.println(this->reading);

}

void Sck_ADS1X15::runTester(uint8_t wichChannel)
{
	// Support both combinations of ADC channels:
	// wichChannel = 0 (default) -> WE in ADS_Ch0 and AE in ADS_Ch1
	// wichChannel = 1 			 -> WE in ADS_Ch2 and AE in ADS_Ch3
	if (wichChannel > 0) {
		adsChannelW = 2;
		adsChannelA = 3;
	}

	// Print headers
	SerialUSB.println("testW,readW,testA,readA");

	// Output from -1400 to +1400 nA
	for (int16_t i=-1400; i<1400; i++) {
		tester.setCurrent(tester.electrode_W, i);
		double currVoltW = -10;

		if (this->getReading(adsChannelW))  currVoltW = this->reading;
		else SerialUSB.println("Error in Working electrode");

		// if (preVoltW != -99) if ((currVoltW - preVoltW) < threshold) maxErrorsW--;
		// preVoltW = currVoltW;
		// if (maxErrorsW == 0) SerialUSB.println("Working electrode fail !!!");

		tester.setCurrent(tester.electrode_A, i);
		double currVoltA = -10;

		if (this->getReading(adsChannelA)) currVoltA = this->reading;
		else SerialUSB.println("Error in Working electrode");

		// if (preVoltA != -99) if ((currVoltA - preVoltA) < threshold) maxErrorsA--;
		// preVoltA = currVoltA;
		// if (maxErrorsA == 0) SerialUSB.println("Auxiliary electrode fail !!!");

		SerialUSB.print(tester.getCurrent(tester.electrode_W));
		SerialUSB.print(",");
		SerialUSB.print(currVoltW, 8);
		SerialUSB.print(",");
		SerialUSB.print(tester.getCurrent(tester.electrode_A));
		SerialUSB.print(",");
		SerialUSB.println(currVoltA, 8);
	}
	SerialUSB.println("Run test finished!");
}
#endif
#ifndef SCD30_DISABLE
bool Sck_SCD30::start(SckBase* base, SensorType wichSensor,AuxBoards* auxBoard)
{
	// If we are using a I2C port MUX; then the Port must be enabled prior to calling this library
	if (auxBoard->TCAMUXMODE ) {
		localPortNum=auxBoard->findDeviceChan(base,deviceAddress,wichSensor, true,false);
		if (localPortNum > TCA_CHANNEL_7) {
			localPortNum=0x00;
			return false;	// the device was not found 
		} else {			// even if we find a port its no good if we cannt communicate with the device
			if (!I2Cdetect(&auxWire, deviceAddress)) return false;
		}
	} else if (!I2Cdetect(&auxWire, deviceAddress)) return false;

	if (started) {
		// Mark this specific metric as enabled
		for (uint8_t i=0; i<3; i++) if (enabled[i][0] == wichSensor) enabled[i][1] = 1;
		return true;
	}

	if (_debug) sparkfun_scd30.enableDebugging(SerialUSB);

	// Without this delay sensor init fails sometimes
	delay(500);

	// Unset measbegin option to avoid begin() function to set measuring interval to default value of 2 seconds.
	if (!sparkfun_scd30.begin(auxWire, false, false)) return false;

	// Ambient pressure compensation
	OneSensor *pressureSensor = &base->sensors[SENSOR_PRESSURE];

	if (pressureSensor->enabled && base->getReading(pressureSensor)) {
		float pressureReading = pressureSensor->reading.toFloat();
		uint16_t pressureInMillibar = pressureReading * 10;  // converting KPa to mbar

		if (pressureInMillibar > 700 && pressureInMillibar < 1200) {
			if (sparkfun_scd30.setAmbientPressure(pressureInMillibar)) {
				pressureCompensated = true;
			}
		}
	}

	// Start measuring with this function respects the saved interval
	if (!sparkfun_scd30.beginMeasuring()) return false;

	// Mark this specific metric as enabled
	for (uint8_t i=0; i<3; i++) if (enabled[i][0] == wichSensor) enabled[i][1] = 1;

	started = true;
	return true;
}

bool Sck_SCD30::stop(SckBase* base,SensorType wichSensor,AuxBoards* auxBoard)
{
	// Mark this specific metric as disabled
	for (uint8_t i=0; i<3; i++) if (enabled[i][0] == wichSensor) enabled[i][1] = 0;

	// Turn sensor off only if all 3 metrics are disabled
	for (uint8_t i=0; i<3; i++) {
		if (enabled[i][1] == 1) return false;
	}
	if (auxBoard->TCAMUXMODE ) {
		if ( !auxBoard->openChannel(base,deviceAddress,localPortNum,false)) {
			return false;
		}
	} else if (!I2Cdetect(&auxWire, deviceAddress)) return false;

	sparkfun_scd30.StopMeasurement();
	started = false;
	return true;
}

bool Sck_SCD30::getReading(SckBase* base,SensorType wichSensor,AuxBoards* auxBoard)
{
	if (auxBoard->TCAMUXMODE ) {
		if ( !auxBoard->openChannel(base,deviceAddress,localPortNum,false)) {
			return false;
		}
	} else if (!I2Cdetect(&auxWire, deviceAddress)) return false;

	switch (wichSensor)
	{
		case SENSOR_SCD30_CO2:
			co2 = sparkfun_scd30.getCO2();
			break;

		case SENSOR_SCD30_TEMP:
			temperature = sparkfun_scd30.getTemperature();
			break;

		case SENSOR_SCD30_HUM:
			humidity = sparkfun_scd30.getHumidity();
			break;

		default:
			return false;
	}

	return true;
}

uint16_t Sck_SCD30::interval(SckBase* base,AuxBoards* auxBoard,SensorType wichSensor,uint16_t newInterval)
{
	if (auxBoard->TCAMUXMODE ) {
		if ( !auxBoard->openChannel(base,deviceAddress,localPortNum,false)) {
			return false;
		}
	} else if (!I2Cdetect(&auxWire, deviceAddress)) return false;

	// Even if the sensor responds OK it doesn't seems to accept any value grater than 1000
	if (newInterval >= 2 && newInterval <= 1800) sparkfun_scd30.setMeasurementInterval(newInterval);

	uint16_t currentInterval;
	sparkfun_scd30.getMeasurementInterval(&currentInterval);

	// Restart measuring so we don't need to wait the current interval to finish (useful when you come from very long intervals)
	sparkfun_scd30.StopMeasurement();
	sparkfun_scd30.beginMeasuring();

	return currentInterval;
}

bool Sck_SCD30::autoSelfCal(SckBase* base,AuxBoards* auxBoard,SensorType wichSensor,int8_t value)
{
	// Value: 0 -> disable, 1 -> enable, any other -> get current setting
	if (auxBoard->TCAMUXMODE ) {
		if ( !auxBoard->openChannel(base,deviceAddress,localPortNum,false)) {
			return false;
		}
	} else if (!I2Cdetect(&auxWire, deviceAddress)) return false;

	if (value == 1)	sparkfun_scd30.setAutoSelfCalibration(true);
	else if (value == 0) sparkfun_scd30.setAutoSelfCalibration(false);

	return sparkfun_scd30.getAutoSelfCalibration();
}

uint16_t Sck_SCD30::forcedRecalFactor(SckBase* base,AuxBoards* auxBoard,SensorType wichSensor,uint16_t newFactor)
{
	if (auxBoard->TCAMUXMODE)  {
		if ( !auxBoard->openChannel(base,deviceAddress,localPortNum,false)) {
			return false;
		}
	}
	if (!I2Cdetect(&auxWire, deviceAddress)) return false;

	if (newFactor >= 400 && newFactor <= 2000) {
		// Maybe not needed, but done for safety
		sparkfun_scd30.setAutoSelfCalibration(false);
		// Send command to SCD30
		sparkfun_scd30.setForcedRecalibrationFactor(newFactor);
	}
	uint16_t saved_value = 0;
	// Check saved value
	sparkfun_scd30.getForcedRecalibration(&saved_value);
	return saved_value;
}

float Sck_SCD30::tempOffset(SckBase* base,AuxBoards* auxBoard,SensorType wichSensor,float userTemp, bool off)
{
	// We expect from user the REAL temperature measured during calibration
	// We calculate the difference against the sensor measured temperature to set the correct offset. Please wait for sensor to stabilize temperatures before aplying an offset.
	// Temperature offset should always be positive (the sensor is generating heat)
	if (auxBoard->TCAMUXMODE ) {
		if ( !auxBoard->openChannel(base,deviceAddress,localPortNum,false)) {
			return false;
		}
	}
	if (!I2Cdetect(&auxWire, deviceAddress)) return false;

	uint16_t currentOffsetTemp;
	sparkfun_scd30.getTemperatureOffset(&currentOffsetTemp);

	getReading(base,SENSOR_SCD30_TEMP,auxBoard);

	if (abs(userTemp) >0.0001 && temperature > userTemp) sparkfun_scd30.setTemperatureOffset(temperature - userTemp);
	else if (off) sparkfun_scd30.setTemperatureOffset(0);

	sparkfun_scd30.getTemperatureOffset(&currentOffsetTemp);

	return currentOffsetTemp / 100.0;
}
#endif
// Added by Bryn Parrott for SCS V3
// bryn.parrott@gmail.com; 20211219 @ Taichung, Taiwan ROC
/*
	SCD4x sensors collect CO2; Humidity; Temperature Readings.
	The SCD41 version provides single shot mode (but we are not using it)
	If The sensor receives regular updated of Barometric Pressure; Readings can be more accurate.

	The Sensor can automatically initiate readings every 5 seconds or every 30 seconds.
	It can also take One-Shot Readings.

	we are using automatic interval readings; not single shot (it's simpler to implent)
*/

bool Sck_SCD4x::start(SckBase* base, SensorType wichSensor,AuxBoards* auxBoard)
{
	// If we are using a I2C port MUX; then the Port must be enabled prior to calling this library
	if (auxBoard->TCAMUXMODE ) {
		localPortNum=auxBoard->findDeviceChan(base,deviceAddress,wichSensor, true,true);
		if (localPortNum > TCA_CHANNEL_7) {
			localPortNum=0x00;
			return false;	// the device was not found 
		} else {			// even if we find a port its no good if we cannt communicate with the device
			if (!I2Cdetect(&auxWire, deviceAddress)) return false;
		}
	} else if (!I2Cdetect(&auxWire, deviceAddress)) {
		return false;
	}

	if (started) {
		
		// Mark this metric as enabled (and not again conduct the other setup tasks, unnecessary)
		// this device is used to read 3 different measurements; each one needs to be enabled.
		for (uint8_t i=0; i<3; i++) if (enabled[i][0] == wichSensor) enabled[i][1] = 1;
		return true;
	}

	// *** In SCK we use automatic periodical measurements.  
	// The Interval setting allows swap between interval of 5 seconds and 30 seconds

	// (re) instantiate the library

	if (_debug) sparkfun_SCD4x.enableDebugging(SerialUSB);

	// start the device:
	if (!sparkfun_SCD4x.begin(auxWire, false, false, false)) {
					//measBegin_________/     |     |
					//autoCalibrate__________/      |
					//skipStopPeriodicMeasurements_/  
		//base->sckOut("SCD41 library NOT initiated", PRIO_MED, true);
		return false;
	//} else  {
		// base->sckOut("SCD41 library initiated", PRIO_MED, true);
	}



	// set up Ambient pressure compensation
	base->sckOut("SCD41 starting pressure compensation", PRIO_MED, true);
	pressureCompensated=setPressureComp(base,true);

	// check for automatic self Calibration;
	
	if (!sparkfun_SCD4x.getAutomaticSelfCalibrationEnabled()) {

		sparkfun_SCD4x.setAutomaticSelfCalibrationEnabled();
		delay(1);
		base->sckOut("SCD4x Automatic Self Calibration is enabled");
	};
	
	
	// Get the first set of measurements under way 
	base->sckOut("SCD41 starting periodic measurements (5 sec)", PRIO_MED, true);
	sparkfun_SCD4x.startPeriodicMeasurement();  
	// see begin (above) readings will be available after 5 seconds have elapsed
	

	// Mark this metric (x3) as enabled or all measurements from the sensor
	//base->sckOut("SCD41 enabling related sensors", PRIO_MED, true);
	for (uint8_t i=0; i<3; i++) if (enabled[i][0] == wichSensor) enabled[i][1] = 1;

	started = true;
	
	return true;
}

bool Sck_SCD4x::stop(SckBase* base,SensorType wichSensor,AuxBoards* auxBoard)
{
	
	// Mark this specific metric as disabled for all measurements from the sensor
	for (uint8_t i=0; i<3; i++) if (enabled[i][0] == wichSensor) enabled[i][1] = 0;

	// Turn sensor off only if all 3 metrics are disabled
	for (uint8_t i=0; i<3; i++) {
		if (enabled[i][1] == 1) return false;
	}

	sparkfun_SCD4x.stopPeriodicMeasurement();
	started = false;
	return true;
}

bool Sck_SCD4x::getReading(SckBase* base,SensorType wichSensor,AuxBoards* auxBoard)
{
	
	if (auxBoard->TCAMUXMODE ) {
		if ( !auxBoard->openChannel(base,deviceAddress,localPortNum,true)) {
			return false;
		}
	} else if (!I2Cdetect(&auxWire, deviceAddress)) {
		return false;
	}
	// if there is a reading waiting to be read: read it
	// NOTE : The Library will automatically trigger a new readMeasurement() if the current CO2 reading has previously been read
	// 
	/*
	if (readcount > 2) {
		// apply pressure compensation and temp offset every 3 times we come here
		pressureCompensated=setPressureComp(base,true);
		float changePct=tempOffset(base,auxBoard, wichSensor,0, true);
		if (changePct > 1.0) {
			base->sckOut("Temperature compensation adjusted");
		}
		readcount=0;
	}
	*/
	switch (wichSensor) {
		case SENSOR_SCD4x_CO2: {
			co2 = sparkfun_SCD4x.getCO2();
			//sprintf(base->outBuff,"CO2 reading (1) (float) %i",co2);
			//base->sckOut(PRIO_MED,true);
			if (co2==0) {
				sparkfun_SCD4x.readMeasurement();
				co2 = sparkfun_SCD4x.getCO2();
				//sprintf(base->outBuff,"CO2 reading (1a) (float) %i",co2);
				//base->sckOut(PRIO_MED,true);
			}
			readcount++;		
			break;
		}
		case SENSOR_SCD4x_TEMP: {
			temperature = sparkfun_SCD4x.getTemperature();	// The Library will trigger a new readMeasurement() if the current temp reading has previously been read
			readcount++;
			break;
		}
		case SENSOR_SCD4x_HUM: {
			humidity = sparkfun_SCD4x.getHumidity();	// The Library will trigger a new readMeasurement() if the current Hum reading has previously been read
			readcount++;
			break;
		}
		default: {
			//base->sckOut("SCD4x: Unrecognised request");
			return false;
			break;
		}
	}
	return true;
}
uint16_t Sck_SCD4x::interval(SckBase* base,AuxBoards* auxBoard,SensorType wichSensor,uint16_t newInterval)
{
	if (auxBoard->TCAMUXMODE ) {
		if ( !auxBoard->openChannel(base,deviceAddress,localPortNum,true)) {
			return false;
		}
	} else if (!I2Cdetect(&auxWire, deviceAddress)) return false;

	// Even if the sensor responds OK it doesn't seems to accept any value grater than 1000
	uint16_t currentInterval=0;
	switch (newInterval){
		case 0 ... 4: {
			base->sckOut("SCD41 device supports fixed measurement intervals of 5 or 30 seconds",PRIO_MED,true);
			return false;
			break;
		}
		case 5: {
			if (!sparkfun_SCD4x.startPeriodicMeasurement()) {
				return false;
			} else {
				currentInterval=newInterval;
			}
			break;
		}
		case 6 ... 29: {
			base->sckOut("SCD41 device supports fixed measurement intervals of 5 or 30 seconds",PRIO_MED,true);
			return false;
			break;
		}
		case 30 ... 1000: {
			if (!sparkfun_SCD4x.startLowPowerPeriodicMeasurement()) {
				return false;
			} else {
				currentInterval=newInterval;
			}
			break;
		}
		default: {
			base->sckOut("SCD41 device supports fixed measurement intervals of 5 or 30 seconds",PRIO_MED,true);
			return false;
		}

	}
	return currentInterval;
}
bool Sck_SCD4x::setPressureComp(SckBase* base,bool value) {
	// Define the ambient pressure in Pascals, so RH and CO2 are compensated for atmospheric pressure
  	// setAmbientPressure overrides setSensorAltitude
	// we set Ambient pressure at the beginning and after every reading
	// this should help attain highest accuracy possible
	// set up Ambient pressure compensation
	bool comp=false;
	if (!value) {  // value is used to turn pressure compensation or off:  true means turn it off false means turn it on.
		OneSensor *pressureSensor = &base->sensors[SENSOR_PRESSURE];
		if (pressureSensor->enabled && base->getReading(pressureSensor)) {
			float pressureReading = pressureSensor->reading.toFloat();
			float pressureInPascals = pressureReading * 1000; // converting kPa to Pa (The device uses hPa; but the library divides Press by 100 so Pa must be supplied)
			if (pressureReading > 70 && pressureReading < 150) {
				if (sparkfun_SCD4x.setAmbientPressure(pressureInPascals,0)) {		// 0 = no delay.  We do not wait for command to finis execution (1 mS)
					sprintf(base->outBuff,"SCD4x Pressure Compensation updated : %03.02f kPa",pressureReading);
					base->sckOut(PRIO_MED,true);
					comp = true;
				//} else {
					//sprintf(base->outBuff,"Pressure Compensation NOT updated (out of limits) : %i ",pressureInMillibar);
					//base->sckOut(PRIO_LOW,true);

				}
			}
		}
	} else {
		sparkfun_SCD4x.setAmbientPressure(0,0);  // 0,0 = zero pressure; no delay.  We do not wait for command to finish execution (1 mS)
		comp = false;
	}
	return comp;
}
bool Sck_SCD4x::autoSelfCal(SckBase* base,AuxBoards* auxBoard,SensorType wichSensor, int8_t value)
{
	// Value: 0 -> disable, 1 -> enable, any other -> get current setting
	if (auxBoard->TCAMUXMODE ) {
		if ( !auxBoard->openChannel(base,deviceAddress,localPortNum,true)) {
			return false;
		}
	} else if (!I2Cdetect(&auxWire, deviceAddress)) return false;

	if (value == 1)	sparkfun_SCD4x.setAutomaticSelfCalibrationEnabled(true);
	else if (value == 0) sparkfun_SCD4x.setAutomaticSelfCalibrationEnabled(false);

	return sparkfun_SCD4x.getAutomaticSelfCalibrationEnabled();
}

uint16_t Sck_SCD4x::forcedRecalFactor(SckBase* base,AuxBoards* auxBoard,SensorType wichSensor, uint16_t newFactor)
{
	float FRCCorrection=0.0;
	if (auxBoard->TCAMUXMODE ) {
		if ( !auxBoard->openChannel(base,deviceAddress,localPortNum,true)) {
			return 0;
		}
	} else if (!I2Cdetect(&auxWire, deviceAddress)) return 0;
	// The global level of CO2 is just over 400 ppm. 
	if (newFactor >= 400 && newFactor <= 2000) {
		// Maybe not needed, but done for safety
		sparkfun_SCD4x.setAutomaticSelfCalibrationEnabled(false);
		delay(1);	// mandatory delay (see library)
		// force a recalibration: Send command to SCD4x
		FRCCorrection=sparkfun_SCD4x.performForcedRecalibration(newFactor);
		sprintf(base->outBuff,"SCD41 forced recalibration activated.  FRCCorrection factor : %f",FRCCorrection);
		base->sckOut(PRIO_MED,true);
	}
	
	return FRCCorrection;
}

float Sck_SCD4x::tempOffset(SckBase* base,AuxBoards* auxBoard,SensorType wichSensor,float userTemp, bool off)
{
	float temperatureReading=0.00;
	uint16_t currentOffsetTemp=0.00;
	float lastTemperatureReading=0.00;
	// We expect from user the REAL temperature measured during calibration
	// the temperature readings from this device may vary due to internal heating of the device (CO2 sensor)
	// We calculate the difference against the trusted sensor measured temperature to set the correct offset. 
	// if the user does not supply a temperature then we check current system temperature 
	// and use that instead.
	// Please wait for sensor to stabilize temperatures before aplying an offset.
	// Temperature offset should always (assumption) be positive (the sensor is generating heat)
	if (auxBoard->TCAMUXMODE ) {
		if ( !auxBoard->openChannel(base,deviceAddress,localPortNum,true)) {
			return 0;
		}
	} else if (!I2Cdetect(&auxWire, deviceAddress)) return 0;

	currentOffsetTemp = sparkfun_SCD4x.getTemperatureOffset(); // The SCD4x library works differently than SCD30

	// Ambient temperature correction using either PN Board Dallas Temp probe (outside air) 
	// or Urban Board Temperature sensor  (Dallas is more accurate if it is installed)
	// we set Ambient temperature offset at the beginning and after every reading
	// this should help attain highest accuracy possible
	
	OneSensor *temperatureSensor1 = &base->sensors[SENSOR_TEMPERATURE];
	OneSensor *temperatureSensor2 = &base->sensors[SENSOR_PM_DALLAS_TEMP];
	if (temperatureSensor2->enabled && base->getReading(temperatureSensor2)) {
		temperatureReading = temperatureSensor2->reading.toFloat();
	} else if (temperatureSensor1->enabled && base->getReading(temperatureSensor1)) {
		temperatureReading = temperatureSensor1->reading.toFloat();
	}
	// get temperature measured by this sensor
	// this is NOT OK : it creates an infinite loop (hangs):  getReading(base, SENSOR_SCD4x_TEMP,auxBoard); 
	// alternative: get the last reading 
	OneSensor *thisTemperatureSensor = &base->sensors[SENSOR_SCD4x_TEMP];
	lastTemperatureReading=thisTemperatureSensor->reading.toFloat();
	

	if (abs(userTemp) > 0.001 && lastTemperatureReading != userTemp) {
		sparkfun_SCD4x.setTemperatureOffset(lastTemperatureReading - userTemp);
	} else if (userTemp == 0 && lastTemperatureReading != temperatureReading) {
		 sparkfun_SCD4x.setTemperatureOffset(lastTemperatureReading - temperatureReading);
	}

	currentOffsetTemp=sparkfun_SCD4x.getTemperatureOffset();
	sprintf(base->outBuff,"SCD41 Temperature Offset updated : %i %% ",currentOffsetTemp/100);
	base->sckOut(PRIO_MED,true);

	return currentOffsetTemp / 100.0; // this would be the percentage error or percentage change
}

/*
	This version of the ENS160 class presupposes the use of an evaluation board from ScioSense
	which has both ENS160 VOC sensor and ENS210 temp/humidity sensor on the same board.
	The T/RH readings from ENS210 are used to continuously calibrate the ENS160.
*/
bool Sck_ENS160::start(SckBase* base,AuxBoards* auxBoard,SensorType wichSensor)
{
	if (alreadyStarted) return true;
	sprintf(base->outBuff,"Starting %s",base->sensors[wichSensor].title);
	base->sckOut(PRIO_LOW,true);
	if (auxBoard->TCAMUXMODE ) {
		localPortNum=auxBoard->findDeviceChan(base,deviceAddress,wichSensor, true,false);
		if (localPortNum > TCA_CHANNEL_7) {
			localPortNum=0x00;
			return false;	// the device was not found 
		} else {			// even if we find a port its no good if we cannt communicate with the device
			if (!I2Cdetect(&auxWire, deviceAddress)) return false;
		}
	} else if (!I2Cdetect(&auxWire, deviceAddress)) return false;	

	sprintf(base->outBuff,"Device %s is accessible",base->sensors[wichSensor].title);
	base->sckOut(PRIO_LOW,true);

	if (!ss_ens160.begin(&auxWire,false,false,true)) {
		return false;
	} else {
		if (ss_ens210.begin(&auxWire,true,false)) {
			// read lastest temp and Hum from attached device on the eval board
			if (ss_ens210.available()) {
				ss_ens210.setSingleMode(true);
				ens210Available=true;
			}
		}
	}
	if (ss_ens160.available()) {
		// Print ENS160 versions
		sprintf(base->outBuff,"\tScioSense ENS160 firmware Rev: %i ", ss_ens160.getMajorRev() );
		base->sckOut(PRIO_MED,false);
		sprintf(base->outBuff,".%i", ss_ens160.getMinorRev());
		base->sckOut(PRIO_MED,false);
		sprintf(base->outBuff,".%i", ss_ens160.getBuild());
		base->sckOut(PRIO_MED,true);

		// set operation mode to standard
		ss_ens160.setMode(ENS160_OPMODE_STD);
	}

	alreadyStarted = true;
	return true;
}

bool Sck_ENS160::stop()
{
	ss_ens160.setMode(ENS160_OPMODE_DEP_SLEEP); // set deep sleep  mode
	alreadyStarted = false;
	return true;
}

bool Sck_ENS160::getReading(SckBase* base,AuxBoards* auxBoard,SensorType wichSensor)
{
	sprintf(base->outBuff,"Reading %s",base->sensors[wichSensor].title);
	base->sckOut(PRIO_LOW,true);
	if (auxBoard->TCAMUXMODE ) {
		if ( !auxBoard->openChannel(base,deviceAddress,localPortNum,false)) {
			return false;
		}
	} else if (!I2Cdetect(&auxWire, deviceAddress)) return false;

	sprintf(base->outBuff,"Reading %s is accessible",base->sensors[wichSensor].title);
	base->sckOut(PRIO_LOW,true);
	if (millis() - lastTime > minTime) {
		if (ens210Available && ss_ens210.available()) {
			ss_ens210.measure();
			ss_ens160.set_envdata210(ss_ens210.getDataT(),ss_ens210.getDataH());
			base->sckOut("ENS210 calibration applied to ENS160",PRIO_LOW,true);
		} else {
			base->sckOut("ENS210 not available for calibration",PRIO_LOW,true);
		}
		if (ss_ens160.available()) {
    		ss_ens160.measure(0);
			lastTime = millis();
			base->sckOut("ENS160 measurement requested",PRIO_LOW,true);
		} else {
			base->sckOut("ENS160 is not available",PRIO_LOW,true);
			return false;
		}
	}
	switch (wichSensor) {
		case SENSOR_ENS160_ECO2: {
			eCO2 = ss_ens160.geteCO2();
			break;
		}
		case SENSOR_ENS160_TVOC: {
			tVOC = ss_ens160.getTVOC();
			break;
		}
		case SENSOR_ENS160_AQI: {
			AQI = ss_ens160.getAQI();
			break;
		}
		default: {
			return false;
			break;

		}
	}
	return true;
}
bool Sck_ENS210::start(SckBase* base,AuxBoards* auxBoard,SensorType wichSensor)
{
	if (alreadyStarted) return true;
	sprintf(base->outBuff,"Starting %s",base->sensors[wichSensor].title);
	base->sckOut(PRIO_LOW,true);
	if (auxBoard->TCAMUXMODE ) {
		localPortNum=auxBoard->findDeviceChan(base,deviceAddress,wichSensor, true,false);
		if (localPortNum > TCA_CHANNEL_7) {
			localPortNum=0x00;
			return false;	// the device was not found 
		} else {			// even if we find a port its no good if we cannt communicate with the device
			if (!I2Cdetect(&auxWire, deviceAddress)) return false;
		}
	} else if (!I2Cdetect(&auxWire, deviceAddress)) return false;	

	sprintf(base->outBuff,"Device %s is accessible",base->sensors[wichSensor].title);
	base->sckOut(PRIO_LOW,true);

	if (!ss_ens210.begin(&auxWire,true,false)) {
		return false;
	} else {
		if (ss_ens210.available()) {
			ss_ens210.setSingleMode(false);	// continuous mode
		}
	
	}

	alreadyStarted = true;
	return true;
}

bool Sck_ENS210::stop()
{
	ss_ens210.setSingleMode(true);		// turn OFF continuous mode
	alreadyStarted = false;
	return true;
}

bool Sck_ENS210::getReading(SckBase* base,AuxBoards* auxBoard,SensorType wichSensor)
{
	sprintf(base->outBuff,"Reading %s",base->sensors[wichSensor].title);
	base->sckOut(PRIO_LOW,true);
	if (auxBoard->TCAMUXMODE ) {
		if ( !auxBoard->openChannel(base,deviceAddress,localPortNum,false)) {
			return false;
		}
	} else if (!I2Cdetect(&auxWire, deviceAddress)) return false;

	sprintf(base->outBuff,"Reading %s is accessible",base->sensors[wichSensor].title);
	base->sckOut(PRIO_LOW,true);
	if (millis() - lastTime > minTime) {
		if (ss_ens210.available()) {
			ss_ens210.measure();
			base->sckOut("ENS210 measurement requested",PRIO_LOW,true);
		} else {
			base->sckOut("ENS210 not available for measurement",PRIO_LOW,true);
			return false;
		}
		
	}

	switch (wichSensor) {
		case SENSOR_ENS210_TEMP: {
			temp = ss_ens210.getTempCelsius();
			break;
		}
		case SENSOR_ENS210_ABSTEMP: {
			abstemp = ss_ens210.getTempKelvin();
			break;
		}
		case SENSOR_ENS210_HUM: {
			hum = ss_ens210.getHumidityPercent();
			break;
		}
		/*
		case SENSOR_ENS210_ABSHUM: {
			hum = ens210.getAbsoluteHumidityPercent();
			break;
		}
		*/
		default: {
			return false;
			break;

		}
	}
	return true;
}

// BP: I cannot find a reference (call) in this file to this function in this file - is it redundant ?
void writeI2C(byte deviceaddress, byte instruction, byte data )
{
	auxWire.beginTransmission(deviceaddress);
	auxWire.write(instruction);
	auxWire.write(data);
	auxWire.endTransmission();
}

byte readI2C(byte deviceaddress, byte instruction)
{
	byte  data = 0x0000;
	auxWire.beginTransmission(deviceaddress);
	auxWire.write(instruction);
	auxWire.endTransmission();
	auxWire.requestFrom(deviceaddress,1);
	unsigned long time = millis();
	while (!auxWire.available()) if ((millis() - time)>500) return 0x00;
	data = auxWire.read();
	return data;
}
