#include "SckList.h"
#include "SckBase.h"

// General flash utilities
int8_t SckList::_flashStart()
{

	if (debug) base->sckOut("F: Starting memory");

	digitalWrite(pinCS_SDCARD, HIGH);	// disables SDcard
	digitalWrite(pinCS_FLASH, LOW);
	if (!flash.begin()) return -1;
	flash.setClock(133000);

	_scanSectors();

	// If no current sector found that means we never used the flash before or some really bad problem has ocurred, so we format the flash and start from scratch
	if (_currSector == -1) {

		dumpSector(0);

		// Print error
		base->sckOut("Flash memory is not formated or damaged!!", PRIO_HIGH);


		// TODO recover data in case it exists and find a way to update current sector
		// maybe we can avoid format by just erasing some sectors to recover sanity

		_flashFormat();
		return 1;
	}

	if (debug) base->sckOut("F: Started OK");
	return 0;
}
bool SckList::_flashFormat()
{
	base->sckOut("Formating... be patient, don't turn off your kit!", PRIO_HIGH);

	if (!flash.eraseChip()) {
		return false;
		base->sckOut("ERROR while formating flash memory!!!", PRIO_HIGH);
	}

	_currSector = 0;
	_addr = 3;

	base->sckOut("Flash memory formated OK, please power cycle your kit. (not just reset)", PRIO_HIGH);
	return true;
}

// Read/Write functions
bool SckList::_append(char value)
{
	if (!flash.writeByte(_addr, value)) {
		sprintf(base->outBuff, "F: Error writing on address %lu", _addr);
		base->sckOut();
		return false;
	}

	_addr++;
	return true;
}

// Group functions
bool SckList::_getGrpAddr(GroupIndex* wichGroup)
{
	uint32_t startAddr = _getSectAddr(wichGroup->sector);
	uint32_t endAddr = startAddr + SECTOR_SIZE;
	uint32_t address = startAddr + 3; 	// First two bytes used for sector state and flags
	int16_t groupCount = 0;

	// Read from the byte 2 until we found 0xFFFF
	while (address < endAddr) {
		if (groupCount == wichGroup->group) {
			wichGroup->address = address;
			return true;
		}
		uint16_t groupSize = flash.readWord(address);

		address += groupSize;
		groupCount++;
	}

	return false;
}
int8_t SckList::_setGrpPublished(GroupIndex wichGroup, PubFlags wichFlag)
{
	// Choose byte position depending on requesed flag
	uint8_t position = GROUP_NET;
	if (wichFlag == PUB_SD) position = GROUP_SD;

	if (debug) {
		sprintf(base->outBuff, "F: Marking group %i in sector %u as %s", wichGroup.group, wichGroup.sector, wichFlag == PUB_NET ? "network published" : "sdcard saved");
		base->sckOut();
	}

	uint32_t flagsAddr = wichGroup.address + position;

	// Sanity check
	if (flagsAddr == 0) return -1;

	// If this is the last group report no more data is available
	if (_dataAvailableSect[wichFlag] == _currSector && wichGroup.group == _lastGroup.group) availableReadings[wichFlag] = false;

	// And write flags byte back
	return flash.writeByte(flagsAddr, PUBLISHED);
}
int8_t SckList::_isGrpPublished(GroupIndex wichGroup, PubFlags wichFlag)
{
	// Choose byte position depending on requesed flag
	uint8_t position = GROUP_NET;
	if (wichFlag == PUB_SD) position = GROUP_SD;

	_getGrpAddr(&wichGroup);
	wichGroup.address += position;

	// Sanity check
	if (wichGroup.address == 0) return -1;

	byte byteFlags = flash.readByte(wichGroup.address);

	if (byteFlags == PUBLISHED) return 1;

	return 0;
}
uint8_t SckList::_countReadings(GroupIndex wichGroup)
{
	// Sanity check
	if (wichGroup.address == 0) {
		if (debug) {
			sprintf(base->outBuff, "F: Wrong address (0) for group %i on sector %u", wichGroup.group, wichGroup.sector);
			base->sckOut();
		}
		return 0;
	}

	uint32_t finalGrpAddr = wichGroup.address + flash.readWord(wichGroup.address);
	uint32_t readingAddr = wichGroup.address + GROUP_READINGS;
	uint8_t readingCounter = 0;

	while (readingAddr < finalGrpAddr) {
		readingAddr += flash.readByte(readingAddr);
		readingCounter++;
	}

	return readingCounter;
}

// Sector functions
uint32_t SckList::_getSectAddr(uint16_t wichSector)
{
	// Sanity check
	if (wichSector > SCKLIST_SECTOR_NUM) return 0xFFFFFFFF;

	return (uint32_t)wichSector * SECTOR_SIZE;
}
int16_t SckList::_getSectFreeSpace(uint16_t wichSector)
{
	if (debug) {
		sprintf(base->outBuff, "F: Calculating free space on sector %u", wichSector);
		base->sckOut();
	}

	uint32_t startAddr = _getSectAddr(wichSector);

	// Sanity check
	if (startAddr > (uint32_t)(SCKLIST_SECTOR_NUM * SECTOR_SIZE)) return -1;

	uint32_t endAddr = startAddr + SECTOR_SIZE;
	uint32_t address = startAddr + 3; 	// First tree bytes used for sector state and flags

	// Read from here until we found 0xFFFF
	while (address < endAddr) {

		uint16_t groupSize = flash.readWord(address);
		if (groupSize == 0xFFFF) break;
		else if (groupSize == 0) return -1;
		else if (groupSize > SECTOR_SIZE - 3) return -1; 	// One group shouldn't be bigger than the sector
		address += groupSize;
	}

	uint16_t freeSpace = endAddr - address;

	if (debug) {
		sprintf(base->outBuff, "F: Sector %u has %u bytes free", wichSector, freeSpace);
		base->sckOut();
	}

	return freeSpace;
}
uint8_t SckList::_getSectState(uint16_t wichSector)
{
	uint32_t startAddr = _getSectAddr(wichSector);
	return flash.readByte(startAddr);
}
int8_t SckList::_setSectPublished(uint16_t wichSector, PubFlags wichFlag)
{

	// Choose byte position depending on requesed flag
	uint8_t position = SECTOR_NET;
	if (wichFlag == PUB_SD) position = SECTOR_SD;

	// Get sector flags Index
	uint32_t flagsAddr = _getSectAddr(wichSector) + position;

	// Sanity check
	if (flagsAddr > (uint32_t)(SCKLIST_SECTOR_NUM * SECTOR_SIZE)) return -1;

	// If sector is not in SECTOR_FULL state we dont accept the flag
	if (_getSectState(wichSector) != SECTOR_USED) return -1;

	// Only set flag if ALL groups on the sector are marked as published
	if (_countSectGroups(wichSector, wichFlag, NOT_PUBLISHED) > 0) return -1;

	// And write flags byte
	if (!flash.writeByte(flagsAddr, PUBLISHED)) return -1;


	if (debug) {
		sprintf(base->outBuff, "F: Marked sector %u %s", wichSector, wichFlag == PUB_NET ? " as network published" : " as sdcard saved");
		base->sckOut();
	}

	// Scan to find missing readings on used sectors, starting from this one
	_searchUnpubSect(wichFlag, wichSector);

	return 1;
}
int8_t SckList::_closeSector(uint16_t wichSector)
{
	// Mark sector as full
	flash.writeByte(_getSectAddr(wichSector), SECTOR_USED);


	// Erase next sector and start using it
	_currSector ++;
	if (_currSector >= SCKLIST_SECTOR_NUM) _currSector = 0;
	flash.eraseSector(_getSectAddr(_currSector));
	_addr = _getSectAddr(_currSector) + 3;

	if (debug) {
		sprintf(base->outBuff, "F: Closed previous sector, now using %u", _currSector);
		base->sckOut();
	}

	// If no readings left, mark sector as published (_setSectPublished() will check if all readings are published)
	_setSectPublished(_currSector - 1, PUB_NET);
	_setSectPublished(_currSector - 1, PUB_SD);

	return 1;
}
int8_t SckList::_isSectPublished(uint16_t wichSector, PubFlags wichFlag)
{
	// Choose byte position depending on requesed flag
	uint8_t position = SECTOR_NET;
	if (wichFlag == PUB_SD) position = SECTOR_SD;

	// Get sector flags Index
	uint32_t flagsAddr = _getSectAddr(wichSector) + position;

	// Sanity check
	if (flagsAddr > (uint32_t)(SCKLIST_SECTOR_NUM * SECTOR_SIZE)) return -1;

	byte byteFlags = flash.readByte(flagsAddr);
	if (byteFlags == PUBLISHED) return 1;
	else if (byteFlags == NOT_PUBLISHED) return 0;

	return -1;
}
SckList::GroupIndex SckList::_getUnpubGrpIdx(uint16_t wichSector, PubFlags wichFlag)
{
	GroupIndex thisGroup = {(int16_t)wichSector, -1, 0};
	bool founded = false;

	uint32_t startAddr = _getSectAddr(wichSector);

	// Sanity check
	if (startAddr > uint32_t(SCKLIST_SECTOR_NUM * SECTOR_SIZE)) return {-1,-1,0};

	uint32_t endAddr = startAddr + SECTOR_SIZE;
	thisGroup.address = startAddr + 3; 		// First tree bytes used for sector state and flags

	thisGroup.group = 0;

	// Get right addr depending on requested flag
	uint8_t addPositionFlag = GROUP_NET;
	if (wichFlag == PUB_SD) addPositionFlag = GROUP_SD;

	// If a potencial next group was stored before let's check it first
	if (potencialNextGroup.group > 0) {

		// Find out groupSize
		uint16_t groupSize = flash.readWord(potencialNextGroup.address);
		
		if (groupSize != 0xFFFF && (potencialNextGroup.address + groupSize) <= endAddr) {

			// Check if group is NOT published in the requested flag
			byte byteFlags = flash.readByte(potencialNextGroup.address + addPositionFlag);

			if (byteFlags == NOT_PUBLISHED) {

				// Store the group and add the next potencial
				thisGroup.group = potencialNextGroup.group;
				thisGroup.address = potencialNextGroup.address;
				potencialNextGroup = {(int16_t)wichSector, potencialNextGroup.group + 1, potencialNextGroup.address + groupSize};
				founded = true;
			}
		}
	}

	// Scan the rest of the sector
	if (!founded) {
		// Read from the byte 2 until we found 0xFFFF or an unpublished group
		while (thisGroup.address < endAddr) {

			// Find out groupSize
			uint16_t groupSize = flash.readWord(thisGroup.address);

			if (groupSize == 0xFFFF || (thisGroup.address + groupSize) > endAddr) {

				// If this sector is already marked as USED and it is not marked as fully published markt it!
				if (_getSectState(wichSector) == SECTOR_USED && !_isSectPublished(wichSector, wichFlag)) _setSectPublished(wichSector, wichFlag);

				// If this group is the last one there is no next potencial group in this sector
				potencialNextGroup = {-1,-1,0};

				return {-1,-1,0}; 	// If GroupSize is not yet written that means no valid group is present
			}

			// Check if group is NOT published in the requested flag
			byte byteFlags = flash.readByte(thisGroup.address + addPositionFlag);
			if (byteFlags == NOT_PUBLISHED) {
				// Store position of the next potencial group
				potencialNextGroup = {(int16_t)wichSector, thisGroup.group + 1, thisGroup.address + groupSize};
				break;
			}

			thisGroup.address += groupSize;
			thisGroup.group++;
		}
	}
	
	return thisGroup;
}
bool SckList::_searchUnpubSect(PubFlags wichFlag, uint16_t startSector)
{
	if (debug) {
		sprintf(base->outBuff, "F: Scanning sectors from %i ->", startSector);
		base->sckOut(PRIO_MED, false);
	}

	// Scan sector by sector
	uint16_t thisSector = startSector;
	while (thisSector < SCKLIST_SECTOR_NUM) {

		uint8_t thisState = _getSectState(thisSector);

		// Only report used sectors
		if (thisState == SECTOR_USED) {

			// Report unpublished sectors
			if (_isSectPublished(thisSector, wichFlag) == 0) {
				_dataAvailableSect[wichFlag] = thisSector;
				availableReadings[wichFlag] = true;

				if (debug) {
					sprintf(base->outBuff, " data found on sector %i", thisSector);
					base->sckOut();
				}

				return true;
			}
		} else if (thisState == SECTOR_EMPTY) break;  		// If sector is empty no case on scanning the rest

		thisSector++;
		if (thisSector >= SCKLIST_SECTOR_NUM) thisSector = 0; 	// If we reach the end start from 0
		if (thisSector == startSector) break; 			// Exit after one loop
	}

	if (debug) base->sckOut(" no data found");

	_dataAvailableSect[wichFlag] = _currSector;
	return false;
}
void SckList::_scanSectors()
{

	_currSector = _dataAvailableSect[PUB_NET] = _dataAvailableSect[PUB_SD] = -1;

	if (debug) base->sckOut("F: Scanning sectors (used/empty):  ", PRIO_MED, false);

	for (uint16_t i=0; i<SCKLIST_SECTOR_NUM; i++) {

		uint8_t thisState = _getSectState(i);

		switch(thisState) {

			case SECTOR_USED:
			{
				// If we still don't have any sector with available data, we check if this one has any
				if (_dataAvailableSect[PUB_NET] < 0) {
					// If sector is not marked as published, search for available groups
					if (!_isSectPublished(i, PUB_NET)) {
						GroupIndex netGroup = _getUnpubGrpIdx(i, PUB_NET);
						if (netGroup.group >= 0) {
							if (debug) {
								sprintf(base->outBuff, "F: Network unpublished data founded on sector: %u", i);
								base->sckOut();
							}
							_dataAvailableSect[PUB_NET] = i;
							availableReadings[PUB_NET] = true;
						// If there is no unpublished data, mark sector as published.
						} else _setSectPublished(i, PUB_NET);
					}
				}
				// The same but for sdcard
				if (_dataAvailableSect[PUB_SD] < 0) {
					if (!_isSectPublished(i, PUB_SD)) {
						GroupIndex sdGroup = _getUnpubGrpIdx(i, PUB_SD);
						if (sdGroup.group >= 0) {
							if (debug) {
								sprintf(base->outBuff, "F: Data not saved to sdcard founded on sector: %u", i);
								base->sckOut();
							}
							_dataAvailableSect[PUB_SD] = i;
							availableReadings[PUB_SD] = true;
						} else _setSectPublished(i, PUB_SD);
					}
				}
				break;
			}
			case SECTOR_EMPTY:
			{
				if (debug) {
					sprintf(base->outBuff, "F: current sector found: %u", i);
					base->sckOut();
				}

				_currSector = i;

				// Calculate current sector freespace
				int16_t freeSpace = _getSectFreeSpace(_currSector);

				// Check for errors while reading current sector
				if (freeSpace == -1) {
					if (debug) {
						sprintf(base->outBuff, "F: <ERROR (%u)", i);
						base->sckOut();
					}

					// Reuse sector
					flash.eraseSector(_getSectAddr(_currSector));
					freeSpace = SECTOR_SIZE - 3;
					return;
				}

				// Find starting point to continue writing
				_addr = _getSectAddr(_currSector) + SECTOR_SIZE - freeSpace;

				// If no previous sector with unpublished data has been found, mark current as the one to publish
				if (_dataAvailableSect[PUB_NET] < 0) _dataAvailableSect[PUB_NET] = _currSector;
				if (_dataAvailableSect[PUB_SD] < 0) _dataAvailableSect[PUB_SD] = _currSector;

				return;
			}
			default:
			{
				// This sector has an error, lets reuse it
				if (debug) {
					sprintf(base->outBuff, "F: <ERROR (%u)", i);
					base->sckOut();
				}

				_currSector = i;
				flash.eraseSector(_getSectAddr(_currSector));
				_addr = _getSectAddr(_currSector) + 3;
				return;
			}
		}
	}

	// If we didn't find a empty sector that means the flash memory is full, then we start from first sector
	_currSector = 0;
	flash.eraseSector(_getSectAddr(_currSector));
	_addr = _getSectAddr(_currSector) + 3;

}
bool SckList::_countSectGroups(uint16_t wichSector, SectorInfo* info)
{
	uint32_t startAddr = _getSectAddr(wichSector);

	// Sanity check
	if (startAddr > uint32_t(SCKLIST_SECTOR_NUM * SECTOR_SIZE)) return false;;

	uint8_t minimalGroupSize = 11; 		// size (2) + flags (2) + timestamp (4) + readingSize (1) + sensorType (1) + reading (at least 1)
	uint32_t endAddr = startAddr + SECTOR_SIZE - minimalGroupSize;
	uint32_t address = startAddr + 3; 	// First tree bytes used for sector state and flags

	// Read from the byte 2 until we found 0xFFFF or an unpublished group
	while (address < endAddr) {

		// Find out groupSize
		uint16_t groupSize = flash.readWord(address);
		if (groupSize == 0xFFFF) break; 	// If GroupSize is not yet written that means no valid group is present

		// Store group state
		// get NET flag
		byte byteNetFlag = flash.readByte(address + GROUP_NET);
		byteNetFlag == PUBLISHED ? info->grpPubNet++ : info->grpUnPubNet++;
		// get SD flag
		byte byteSdFlag = flash.readByte(address + GROUP_SD);
		byteSdFlag == PUBLISHED ? info->grpPubSd++ : info->grpUnPubSd++;

		info->grpTotal++;

		address += groupSize;
	}

	return true;
}
int16_t SckList::_countSectGroups(uint16_t wichSector, PubFlags wichFlag, byte publishedState, bool getAll)
{
	uint32_t startAddr = _getSectAddr(wichSector);

	// Sanity check
	if (startAddr > uint32_t(SCKLIST_SECTOR_NUM * SECTOR_SIZE)) return -1;
	
	uint8_t minimalGroupSize = 11; 		// size (2) + flags (2) + timestamp (4) + readingSize (1) + sensorType (1) + reading (at least 1)
	uint32_t endAddr = startAddr + SECTOR_SIZE - minimalGroupSize;
	uint32_t address = startAddr + 3; 	// First tree bytes used for sector state and flags

	// Get right addrs depending on requested flag
	uint32_t addPositionFlag = GROUP_NET;
	if (wichFlag == PUB_SD) addPositionFlag = GROUP_SD;

	int16_t groupTotal = 0;

	// Read from the byte 2 until we found 0xFFFF or an unpublished group
	while (address < endAddr) {

		// Find out groupSize
		uint16_t groupSize = flash.readWord(address);
		if (groupSize == 0xFFFF) break; 	// If GroupSize is not yet written that means no valid group is present

		// Check if group is NOT published
		byte byteFlags = flash.readByte(address + addPositionFlag);
		if (byteFlags == publishedState || getAll) groupTotal++;

		address += groupSize;
	}

	return groupTotal;
}


uint8_t SckList::_formatSD(GroupIndex wichGroup, char* buffer)
{
	if (debug) base->sckOut("F: Preparing group data for sdcard saving");

	uint8_t readingNum = _countReadings(wichGroup);

	uint32_t thisTime = flash.readULong(wichGroup.address + GROUP_TIME);
	base->epoch2iso(thisTime, buffer); 		// print time stamp to buffer
	base->epoch2iso(thisTime, base->ISOtimeBuff); 	// Update base time buffer for console message.

	if (debug) {
		sprintf(base->outBuff, "F: (%s) -> ", base->ISOtimeBuff);
		base->sckOut(PRIO_MED, false);
	}

	for (uint8_t i=0; i<SENSOR_COUNT; i++) {

		SensorType wichSensorType = base->sensors.sensorsPriorized(i);

		if (base->sensors[wichSensorType].enabled) {

			bool found = false;
			uint32_t pos = wichGroup.address + GROUP_READINGS;
			for (uint8_t ii=0; ii<readingNum; ii++) {

				uint8_t readSize = flash.readByte(pos); // Get reading size
				SensorType thisType = static_cast<SensorType>(flash.readByte(pos + 1)); 	// Get sensorType

				if (thisType == wichSensorType) {

					// Get the reading value
					String thisReading;
					for (uint32_t r=pos+2; r<pos+readSize; r++) thisReading.concat((char)flash.readByte(r));

					if (debug) {
						sprintf(base->outBuff, "%s %s %s, ", base->sensors[thisType].title, thisReading.c_str(), base->sensors[thisType].unit);
						base->sckOut(PRIO_MED, false);
					}

					sprintf(buffer + strlen(buffer), ",%s", thisReading.c_str());
					found = true;
					break;
				}
				pos += readSize;
			}
			if (!found) sprintf(buffer + strlen(buffer), ",null");
		}
	}
	sprintf(buffer + strlen(buffer), "\r\n"); 	// print newline to buffer

	if (debug) base->sckOut("<-");

	return readingNum;
}
uint8_t SckList::_formatNET(GroupIndex wichGroup, char* buffer)
{
	if (debug) base->sckOut("F: Preparing group data for network publishing");

	// /* Example
	// {	t:2017-03-24T13:35:14Z,
	// 		29:48.45,
	// 		13:66,
	// 		12:28,
	// 		10:4.45
	// }

	uint8_t readingNum = _countReadings(wichGroup);

	// Prepare buffer for ESP format
	sprintf(buffer, "%c", ESPMES_MQTT_PUBLISH); 	// Write command to buffer

	// Write time
	base->epoch2iso(flash.readULong(wichGroup.address + GROUP_TIME), base->ISOtimeBuff);
	sprintf(buffer + strlen(buffer), "{t:%s", base->ISOtimeBuff);

	if (debug) {
		sprintf(base->outBuff, "F: (%s) -> ", base->ISOtimeBuff);
		base->sckOut(PRIO_MED, false);
	}

	wichGroup.address += GROUP_READINGS;  // Jump to first reading
	// Write sensor readings
	for (uint8_t i=0; i<readingNum; i++) {

		uint8_t readSize = flash.readByte(wichGroup.address); // Get reading size (full size - size and flags)
		SensorType thisType = static_cast<SensorType>(flash.readByte(wichGroup.address+1)); 	// Get sensorType

		// Get the reading value
		String thisReading;
		for (uint32_t r=wichGroup.address+2; r<wichGroup.address+readSize; r++) thisReading.concat((char)flash.readByte(r));

		if (base->sensors[thisType].id > 0 && !thisReading.startsWith("null")) {

			if (debug) {
				sprintf(base->outBuff, "%s %s %s, ", base->sensors[thisType].title, thisReading.c_str(), base->sensors[thisType].unit);
				base->sckOut(PRIO_MED, false);
			}

			sprintf(buffer + strlen(buffer), ",%u:%s", base->sensors[thisType].id, thisReading.c_str());
		}

		wichGroup.address += readSize;
	}
	sprintf(buffer + strlen(buffer), "}");

	if (debug) base->sckOut("<-");

	return readingNum;
}

// Public fnctions
int8_t SckList::setup()
{
	debug = base->config.debug.flash;
	return _flashStart();
}
bool SckList::flashFormat()
{
	return _flashFormat();
}
void SckList::flashUpdate()
{
	_scanSectors();
}
SckList::GroupIndex SckList::saveGroup()
{

	// First prepare the group to be saved (on ram buffer)

	// Init tags in NOT_PUBLISHED for all
	byte finit = 0xFF;
	memcpy(&flashBuff[GROUP_NET], &finit, 1);
	memcpy(&flashBuff[GROUP_SD], &finit, 1);

	// Store timeStamp of current group
	memcpy(&flashBuff[GROUP_TIME], &base->lastSensorUpdate, 4);  	// Write timeStamp (4 bytes)

	base->epoch2iso(base->lastSensorUpdate, base->ISOtimeBuff);
	if (debug) {
		sprintf(base->outBuff, "F: *** Saving group (%s) -> ", base->ISOtimeBuff);
		base->sckOut(PRIO_MED, false);
	}

	// Store sensor readings
	uint16_t pos = GROUP_READINGS; 	// Variable to store the buffer index position
	uint8_t enabledSensors = 0;
	for (uint8_t i=0; i<SENSOR_COUNT; i++) {

		SensorType stype = static_cast<SensorType>(i);

		if (base->sensors[stype].enabled && base->sensors[stype].lastReadingTime == base->lastSensorUpdate) { 			// If sensor is enabled and a reading has been taken in last loop

			String value = base->sensors[stype].reading;

			if (!value.startsWith("null")) {

				uint8_t vsize = value.length() + 1 + 1; 				// Value.length + Sensortype + size byte
				memcpy(&flashBuff[pos], &vsize, 1); pos+=1;				// Size (1 byte)
				memcpy(&flashBuff[pos], &stype, 1); pos+=1;				// SensorType (1 byte)

				if (debug) {
					sprintf(base->outBuff, "%s %s %s, ", base->sensors[stype].title, value.c_str(), base->sensors[stype].unit);
					base->sckOut(PRIO_MED, false);
				}

				for (uint8_t c=0; c<value.length(); c++) {
					char thischar = value.charAt(c);
					memcpy(&flashBuff[pos], &thischar, 1);			// Reading value char by char
					pos+=1;
				}
				enabledSensors++;
			}
		}
		if (enabledSensors == 0) return {-1,-1,0};
	}
	if (debug) base->sckOut("<-");

	// Save group size at the begining of the group
	memcpy(&flashBuff[GROUP_SIZE], &pos, 2);

	// Check if new group fits in current sector
	int16_t _currentFreeSpace = _getSectFreeSpace(_currSector);
	bool sectorIsFull = false;
	if (_currentFreeSpace < pos) {

		if (debug) {
			sprintf(base->outBuff, "F: Sector %u doesn't have enough free space, will search for a new one.", _currSector);
			base->sckOut();
		}

		// Close this one and find next usable sector
		_closeSector(_currSector);

		// If selected sector doesn't have enough space, close it and keep scanning until we found a sector that fits the data
		while (_getSectFreeSpace(_currSector) < pos) _closeSector(_currSector);

		if (debug) {
			sprintf(base->outBuff, "F: Using sector %u in address %lu", _currSector, _addr);
			base->sckOut();
		}

	// Check if after saving the group the sector is completely full
	} else if (_currentFreeSpace == pos) sectorIsFull = true;

	uint32_t startAddress = _addr;

	// Copy buffer to flash memory
	for (uint16_t i=0; i<pos; i++) _append(flashBuff[i]);

	availableReadings[PUB_NET] = true;
	availableReadings[PUB_SD] = true;

	int16_t thisGroup = _countSectGroups(_currSector, PUB_NET, NOT_PUBLISHED, true) - 1;
	_lastGroup = {_currSector, thisGroup, startAddress};

	if (debug) {
		sprintf(base->outBuff, "F: Saved %u readings on group %u in sector %u", enabledSensors, thisGroup, _currSector);
		base->sckOut();
	}

	if (sectorIsFull) {
		if (debug) {
			sprintf(base->outBuff, "F: We just filled sector %u.", _currSector);
			base->sckOut();
		}
		// Close full sector
		_closeSector(_currSector);

		if (debug) {
			sprintf(base->outBuff, "F: Now using sector %u in address %lu", _currSector, _addr);
			base->sckOut();
		}
	}

	return _lastGroup;
}
SckList::GroupIndex SckList::readGroup(PubFlags wichFlag, GroupIndex forceIndex)
{
	GroupIndex thisGroup = {-1, -1, 0};

	if (forceIndex.group < 0) {

		if (debug) {
			sprintf(base->outBuff, "F: --- Searching for group not %s", wichFlag == PUB_NET ? "published to network" : "saved to sdcard");
			base->sckOut();
		}

		uint32_t started = millis();
		while ((_dataAvailableSect[wichFlag] >= 0 && _dataAvailableSect[wichFlag] != _currSector) && millis() - started < 1000) {

			if (debug) {
				sprintf(base->outBuff, "F: Sector %i seems to contain unpublished data, checking it...", _dataAvailableSect[wichFlag]);
				base->sckOut(PRIO_MED, false);
			}

			// Check if that sector still has some unpublished data, if not it will be marked as published to avoid future false positives
			// When a sector is marked as published a scan is performed to look for available data (this will change the value of _dataAvailableSect)
			thisGroup = _getUnpubGrpIdx(_dataAvailableSect[wichFlag], wichFlag);

			// If no group found search again for new sector..
			if (thisGroup.group < 0) {
				if (debug) {
					sprintf(base->outBuff, " no data found");
					base->sckOut();
				}
			} else {
				if (debug) {
					sprintf(base->outBuff, " data available in group %i", thisGroup.group);
					base->sckOut();
				}
				break;
			}
		}
		
		// If no data is pending to be published lets check the current sector
		if (thisGroup.group < 0) {

			if (debug) base->sckOut("F: Searching inside current sector...", PRIO_MED, false);

			thisGroup.sector = _currSector;

			// try to find unpublished group inside sector
			thisGroup = _getUnpubGrpIdx(thisGroup.sector, wichFlag);

			// If there is no group available
			if (thisGroup.group < 0) {
				if (debug) base->sckOut(" no data found");
				availableReadings[wichFlag] = false;
				return {-1,-1,0};
			}

			if (debug) {
				sprintf(base->outBuff, " data available in group %i", thisGroup.group);
				base->sckOut();
			}
		}


	} else {

		thisGroup = forceIndex;
		_getGrpAddr(&thisGroup);
	}

	uint8_t readingNum = 0;
	if (wichFlag == PUB_SD) readingNum = _formatSD(thisGroup, flashBuff);
	else if (wichFlag == PUB_NET) readingNum = _formatNET(thisGroup, base->netBuff);

	if (readingNum > 0) return thisGroup;

	if (debug) base->sckOut("F: No readings available for this group!!");

	return {-1,-1,0};
}
uint8_t SckList::setPublished(GroupIndex wichGroup, PubFlags wichFlag)
{
	// Sanity check
	if (wichGroup.group < 0) return -1;

	_setGrpPublished(wichGroup, wichFlag);

	// Try to mark this sector as published (flag will be rejected if not all groups are published)
	_setSectPublished(wichGroup.sector, wichFlag);

	return _countReadings(wichGroup);
}
uint32_t SckList::countGroups(PubFlags wichFlag)
{
	uint16_t groupTotal = 0;

	for (uint16_t i=0; i<SCKLIST_SECTOR_NUM; i++) {

		uint8_t thisState = _getSectState(i);

		if (!_isSectPublished(i, wichFlag)) {
			int16_t thisSectGroups = _countSectGroups(i, wichFlag, NOT_PUBLISHED);
			if (thisSectGroups > 0)	groupTotal += thisSectGroups;
		}

		if (thisState == SECTOR_EMPTY) break;
	}
	return groupTotal;
}

uint16_t SckList::recover(uint16_t wichSector, PubFlags wichFlag)
{
	if (debug) {
		sprintf(base->outBuff, "F: Recovering groups on sector %u and %s", wichSector, wichFlag == PUB_NET ? "publishing them to the network" : "saving them to sdcard");
		base->sckOut();
	}

	uint16_t groupNum = _countSectGroups(wichSector, PUB_NET, PUBLISHED, true);  // Gets all groups with in the sector (getAll = true)
	uint16_t totalRecovered = 0;

	for (int16_t i=0; i<groupNum; i++) {


		// prepare the flash group
		GroupIndex thisGroup = {(int16_t)wichSector, i};
		GroupIndex tryingGroup = readGroup(wichFlag, thisGroup);

		if (wichFlag == PUB_SD) {

			// Save to sdcard
			if (!base->sdPublish()) {
				sprintf(base->outBuff, "Group %u of sector %u saving to sd-card ERROR!!", i, wichSector);
				base->sckOut();
			} else {
				_setGrpPublished(thisGroup, wichFlag);
				totalRecovered++;
				base->epoch2iso(flash.readULong(tryingGroup.address + GROUP_TIME), base->ISOtimeBuff); 	// print time stamp to buffer
				sprintf(base->outBuff, "(%s) - Group %u of sector %u saved to sd-card OK!", base->ISOtimeBuff, i, wichSector);
				base->sckOut();
			}
		} else {
			// Make sure ESP is ready
			if (!base->st.espON) {
				base->ESPcontrol(base->ESP_ON);
				while (base->st.espBooting) base->ESPbusUpdate();
			}

			// Send MQTT and wait for response or timeout (10 seconds)
			uint32_t timeout = millis();
			while (millis() - timeout < 10000) {

				if (base->st.publishStat.retry()) base->sendMessage();

				base->ESPbusUpdate();
				if (base->st.publishStat.ok) {
					_setGrpPublished(thisGroup, wichFlag);
					totalRecovered++;
					base->st.publishStat.reset();
					base->epoch2iso(flash.readULong(tryingGroup.address + GROUP_TIME), base->ISOtimeBuff); 	// print time stamp to buffer
					sprintf(base->outBuff, "(%s) - Group %u of sector %u published OK!", base->ISOtimeBuff, i, wichSector);
					base->sckOut();
					break;
				}
				if (base->st.publishStat.error) {
					sprintf(base->outBuff, "Group %u of sector %u publish ERROR!!", i, wichSector);
					base->sckOut();
					base->st.publishStat.reset();
					break;
				}
			}
		}
	}
	return totalRecovered;
}
SckList::SectorInfo SckList::sectorInfo(uint16_t wichSector)
{
	if (debug) {
		sprintf(base->outBuff, "F: Scanning sector %u", wichSector);
		base->sckOut();
	}

	SectorInfo info;

	if (wichSector > SCKLIST_SECTOR_NUM) return info;

	info.used = _getSectState(wichSector) == SECTOR_USED ? true : false;
	info.current = wichSector == _currSector ? true : false;
	info.pubNet = _isSectPublished(wichSector, PUB_NET);
	info.pubSd = _isSectPublished(wichSector, PUB_SD);
	_countSectGroups(wichSector, &info);
	info.freeSpace = _getSectFreeSpace(wichSector);
	info.addr = _getSectAddr(wichSector);

	if (info.grpTotal > 0) {

		GroupIndex firstGroup = {(int16_t)wichSector, 0, 0};
		_getGrpAddr(&firstGroup);
		info.firstTime = flash.readULong(firstGroup.address + GROUP_TIME);

		GroupIndex lastGroup = {(int16_t)wichSector, (int16_t)((int16_t)info.grpTotal - 1), 0};
		_getGrpAddr(&lastGroup);
		info.lastTime = flash.readULong(lastGroup.address + GROUP_TIME);
	}

	return info;
}
void SckList::dumpSector(uint16_t wichSector, uint16_t howMany) // listo
{
	SerialUSB.print("F: HEX dump of sector ");
	SerialUSB.print(wichSector);
	SerialUSB.print(" starting on address ");
	SerialUSB.print(_getSectAddr(wichSector));
	SerialUSB.println(": ");

	for (uint32_t i=_getSectAddr(wichSector); i<(_getSectAddr(wichSector)+howMany); i++) {
		byte readed = flash.readByte(i);
		SerialUSB.print(readed, HEX);
		if (readed < 16) SerialUSB.print(" ");
		SerialUSB.print(" ");
	}
	SerialUSB.println("");
}
void SckList::flashInfo(FlashInfo* info)
{
	if (debug) base->sckOut("F: Scanning flash memory sectors");

	bool firstEmpty = true;
	uint8_t sectPerLine = 32;
	uint8_t sectPrinted = 0;

	for (uint16_t i=0; i<SCKLIST_SECTOR_NUM; i++) {

		byte state = _getSectState(i);

		if (sectPrinted == 0) sprintf(base->outBuff, "\r\n%u > |", i);

		if (state == SECTOR_USED || (state == SECTOR_EMPTY && firstEmpty)) {

			if (state == SECTOR_EMPTY) {

				// Print uppercase U to indicate this is the current sector
				firstEmpty = false;
				info->sectFree++;
				snprintf(base->outBuff + strlen(base->outBuff), sizeof(base->outBuff), "U");

			} else {
				// Print lowercase u to indicate this sector is used
				info->sectUsed++;
				snprintf(base->outBuff + strlen(base->outBuff), sizeof(base->outBuff), "u");
			}

			SectorInfo sectInfo;
			_countSectGroups(i, &sectInfo);
			info->grpTotal += sectInfo.grpTotal;
			info->grpUnPubNet += sectInfo.grpUnPubNet;
			info->grpUnPubSd += sectInfo.grpUnPubSd;

			// Print the total of groups in the sector
			snprintf(base->outBuff + strlen(base->outBuff), sizeof(base->outBuff), "%u", sectInfo.grpTotal);

			// Print the number of unpublished readings for net/sd
			if (sectInfo.grpUnPubNet > 0) snprintf(base->outBuff + strlen(base->outBuff), sizeof(base->outBuff), "(%u/", sectInfo.grpUnPubNet);
			else snprintf(base->outBuff + strlen(base->outBuff), sizeof(base->outBuff), "(_/");
			if (sectInfo.grpUnPubSd > 0) snprintf(base->outBuff + strlen(base->outBuff), sizeof(base->outBuff), "%u)", sectInfo.grpUnPubSd);
			else snprintf(base->outBuff + strlen(base->outBuff), sizeof(base->outBuff), "_)");

		} else if (state == SECTOR_EMPTY)  {
			info->sectFree++;
			// Print an e to indicate an empty sector
			snprintf(base->outBuff + strlen(base->outBuff), sizeof(base->outBuff), "e");
		}

		snprintf(base->outBuff + strlen(base->outBuff), sizeof(base->outBuff), "|");
		
		if (strlen(base->outBuff) > 200) {
			base->sckOut(PRIO_HIGH, false);
			sprintf(base->outBuff, "");
		}
		
		sectPrinted++;
		if (sectPrinted == sectPerLine) {
			base->sckOut(PRIO_HIGH, false);
			sectPrinted = 0;
		}
	}
	base->sckOut("\r\nSector state:\r\nu -> Used sector\r\nU -> In use sector\r\ne -> empty sector");
	base->sckOut("\r\nReadings are shown after sector state in the form of: total(net-pending/sd-pending) (ej. |u78(65/45)|)");

	info->currSector = _currSector;
}
uint32_t SckList::getFlashCapacity()
{
	return flash.getCapacity();
}
bool SckList::testFlash()
{
	String writeSRT = "testing the flash!";

	// Inside first sector
	uint32_t fAddress = 1000;
	flash.writeStr(fAddress, writeSRT);

	String readSTR;
	flash.readStr(fAddress, readSTR);

	// Erase first sector to leave it clean
	flash.eraseSector(_getSectAddr(0));

	if (!readSTR.equals(writeSRT)) return false;
	return true;
}
