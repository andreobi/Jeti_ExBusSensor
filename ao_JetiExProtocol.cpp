
#include "ao_JetiExProtocol.h"

#include"ao_SystemConf.h"

extern const char *const JBM_SysCon_table[];

// JetiSensor
JetiSensor::JetiSensor(uint8_t arrIdx, JetiExProtocol * pProtocol)
  : m_active(false)
  //m_textLen(0), m_unitLen(0), m_dataType(0), m_precision(0), m_bufLen(0)
{
	// sensor enabled
	m_enabled = (pProtocol->m_enabledSensors[arrIdx >> 3] & (1 << (arrIdx & 7))) ? true : false;
	if (!m_enabled) return;
  
	// has sensor fresh data
	m_active = (pProtocol->m_activeSensors[arrIdx >> 3] & (1 << (arrIdx & 7))) ? true : false;

	// copy constant data 
	JetiSensorConst constData;
	memcpy_P(&constData, &pProtocol->m_pSensorsConst[arrIdx], sizeof(JetiSensorConst));

char lab[20];
strcpy_P(lab,(char *)pgm_read_word(&(JBM_SysCon_table[arrIdx+16])));

	m_dataType = constData.dataType; 
//	m_id       = constData.id;
//	m_id       = arrIdx+1;
// value
	m_value = pProtocol->m_pValues[arrIdx].m_value;

	// copy to combined sensor/value buffer
//	copyLabel((const uint8_t*)constData.text, (const uint8_t*)constData.unit, m_label, sizeof(m_label), &m_textLen, &m_unitLen);
	copyLabel((const uint8_t*)lab, (const uint8_t*)constData.unit, m_label, sizeof(m_label), &m_textLen, &m_unitLen);

	// 0...3 decimal places
	m_precision=(constData.precision<<5)&0x60;
	
  // set needed space in EX frame buffer
	switch(m_dataType) {
	case TYPE_6b:  m_bufLen = 2; break;  //  1 byte id and data type + 1 byte value (incl. sign and prec)
	case TYPE_14b: m_bufLen = 3; break;  //  1 byte id and data type + 2 byte value (incl. sign and prec)
	case TYPE_22b: m_bufLen = 4; break;  //  1 byte id and data type + 3 byte value (incl. sign and prec)
	case TYPE_DT:  m_bufLen = 4; break;  //  1 byte id and data type + 3 byte value
	case TYPE_30b: m_bufLen = 5; break;  //  1 byte id and data type + 4 byte value
	case TYPE_GPS: m_bufLen = 5; break;  //  1 byte id and data type + 4 byte value
	}
}

// JetiExProtocol
JetiExProtocol::JetiExProtocol() :
  m_nameLen(0), m_pSensorsConst(0), m_pValues(0), m_sensorIdx(0), m_dictIdx(0), 
  m_devIdLow(DEVICE_ID_LOW), m_devIdHi(DEVICE_ID_HI), m_FrameCnt(0)
{
	m_name[0] = '\0';
	memset(m_enabledSensors, 0, sizeof(m_enabledSensors));	// default: all sensors off
	memset(m_activeSensors, 0, sizeof(m_activeSensors));	// default: all sensors old data
}

void JetiExProtocol::init( const char *name, JETISENSOR_CONST *pSensorArray){
	if (m_pValues) return;								// call it once only !

	m_nameLen = (strlen(name)<=20 ? strlen(name):20);
	memcpy(m_name, name, m_nameLen);					// sensor name
	m_pSensorsConst = pSensorArray;
	m_pValues = new JetiValue[MAX_SENSORS];				// init sensor value array
}

void JetiExProtocol::setMessage(const char *message, uint8_t messageClass){
	uint8_t cSREG= SREG;
	cli();
	m_messageLength= (strlen(message)<=20 ? strlen(message):20);
	memcpy(m_message, message, m_messageLength);						
	m_messageClass=messageClass&0x07;
	m_messageNew=true;		
	SREG = cSREG;			// restore SREG value (I-bit)
}

void JetiExProtocol::setSensorValue(uint8_t id, int32_t value){
	uint8_t cSREG= SREG;
	cli();
	if (m_pValues && id>0 && id <= MAX_SENSORS){
		m_pValues[--id].m_value = value;
		m_activeSensors[id>>3] |= (1<<(id&0x07));
	}
	SREG = cSREG;			// restore SREG value (I-bit)
}

void JetiExProtocol::setSensorValueGPS(uint8_t id, bool bLongitude, float value){
// Jeti doc: If the lowest bit of a decimal point (Bit 5) equals log. 1, the data represents longitude. According to the highest bit (30) of a decimal point it is either West (1) or East (0).
// Jeti doc: If the lowest bit of a decimal point (Bit 5) equals log. 0, the data represents latitude. According to the highest bit (30) of a decimal point it is either South (1) or North (0).
// Byte 0: lo of minute, Byte 1: hi of minute, Byte 2: lo von degree, Byte 3: hi of degree 
	union {
		int32_t vInt;
		char    vBytes[4];
	} gps;

// E 11° 33' 22.176" --> 11.55616 --> 11° 33.369' see  http://www.gpscoordinates.eu/convert-gps-coordinates.php
// N 48° 14' 44.520" --> 48.24570 --> 48° 14.742'
	float deg, frac = modff( value, &deg );
	uint16_t deg16 = (uint16_t)fabs( deg );
	uint16_t min16 = (uint16_t)fabs( frac * 0.6f * 100000 );
	gps.vInt = 0;
	gps.vBytes[0]  = min16 & 0xFF;
	gps.vBytes[1]  = ( min16 >> 8 ) & 0xFF;
	gps.vBytes[2]  = deg16 & 0xFF;                      // degrees 0..255
	gps.vBytes[3]  = ( deg16 >> 8 ) & 0x01;             // degrees 256..359
	gps.vBytes[3] |= bLongitude  ? 0x20 : 0;
	gps.vBytes[3] |= (value < 0) ? 0x40 : 0;
  
	setSensorValue( id, gps.vInt );
}

void JetiExProtocol::setSensorValueGPS(uint8_t id, uint32_t value){
	setSensorValue( id, value);
}

void JetiExProtocol::setSensorValueDate( uint8_t id, uint8_t day, uint8_t month, uint16_t year )
{
// Jeti doc: If the lowest bit of a decimal point equals log. 1, the data represents date
// Jeti doc: (decimal representation: b0-7 day, b8-15 month, b16-20 year - 2 decimals, number 2000 to be added).
// doc seems to be wrong, this is working: b0-b7 year, b16-b20: day
	union {
		int32_t vInt;
		char    vBytes[4];
	} date;
	if( year >= 2000 ) year -= 2000;
	date.vInt = 0;
	date.vBytes[0]  = year;
	date.vBytes[1]  = month;
	date.vBytes[2]  = day & 0x1F;
	date.vBytes[2] |= 0x20;
	setSensorValue(id, date.vInt);
}

void JetiExProtocol::setSensorValueTime(uint8_t id, uint8_t hour, uint8_t minute, uint8_t second) {
// If the lowest bit of a decimal point equals log. 0, the data represents time
// (decimal representation: b0-7 seconds, b8-15 minutes, b16-20 hours).
	union {
		int32_t vInt;
		char    vBytes[4];
	} date;
	date.vInt = 0;
	date.vBytes[0]  = second;
	date.vBytes[1]  = minute;
	date.vBytes[2]  = hour & 0x1F;
	setSensorValue( id, date.vInt );
}

void JetiExProtocol::setSensorEnable(uint8_t id, bool bEnable) {
	if (0<id && id<=MAX_SENSORS){
		id--;
		if (bEnable)
			m_enabledSensors[id >>3] |=  (1 << (id & 7));
		else
			m_enabledSensors[id >>3] &= ~(1 << (id & 7));
	}
}

bool JetiExProtocol::getSensorEnable(uint8_t id) {
	if (0<id && id<=MAX_SENSORS){
		id--;
		return m_enabledSensors[id>>3] &(1<<(id&7));
	}
	return false;
}


uint8_t JetiExProtocol::setupExFrame(uint8_t * exBuffer) {
	uint8_t n= 0;																// sensor name in frame 0
	if (m_messageNew) {															// sensor name
		exBuffer[1] = 0x80;  													// 2Bit packet type(0-3) 0x80= Message, 0x40=Data, 0x00=Text 
		exBuffer[7] = 0x00;														// 8Bit id 
		exBuffer[8] = m_messageClass<<5| m_messageLength;						// class + length(max 0x1f)
		memcpy(&exBuffer[9],m_message, m_messageLength);
		n = m_messageLength + 9;
		m_messageNew=false;		
	} else {
		m_FrameCnt++;
		m_FrameCnt&=0x03ff;
		if (m_FrameCnt == 0) {													// sensor name
			exBuffer[1] = 0x00;  												// 2Bit packet type(0-3) 0x80= Message, 0x40=Data, 0x00=Text 
			exBuffer[7] = 0x00;													// 8Bit id 
			exBuffer[8] = m_nameLen << 3;										// 5Bit description, 3Bit unit length (use one space character)
			memcpy(&exBuffer[9], m_name, m_nameLen);							// copy label plus unit to ex buffer starting from pos 9
			n = m_nameLen + 9;
		} else if (((m_FrameCnt /16) <= MAX_SENSORS && (m_FrameCnt %16) == 0)) {// use every 16th frame to transfer sensor dictionary
			for (uint8_t nDict = 0; nDict < MAX_SENSORS; nDict++) {
				uint8_t m_id= m_dictIdx;										// first parameter has id 1
				JetiSensor sensor(m_id, this);
				if (++m_dictIdx >= MAX_SENSORS) m_dictIdx = 0;
				if (sensor.m_enabled) {
					exBuffer[1] = 0x00;											// 2Bit packet type(0-3) 0x40=Data, 0x00=Text
					exBuffer[7] = m_id+1;										// 8Bit id
					exBuffer[8] = (sensor.m_textLen << 3) | sensor.m_unitLen;	// 5Bit description, 3Bit unit length 
					n= 9+ sensor.jetiCopyLabel(&exBuffer[9]);					// copy label, unit to ex buffer[9]
					break;
				}
			}
		} else {																// send EX values in all other frames
			uint8_t bufLen;
			uint8_t nVal = 0;
			exBuffer[1] = 0x40;													// 2Bit Type(0-3) 0x40=Data, 0x00=Text
			n = 7;																// start at 8th byte in buffer
			do {
				bufLen = 0;														// last value buffer length    
				JetiSensor sensor(m_sensorIdx, this);
				m_activeSensors[m_sensorIdx >>3] &= (~(1<<(m_sensorIdx&0x07)));	// delete Data freshness
				m_sensorIdx++;
				if (sensor.m_active && sensor.m_enabled) {						// is sensor data fresh and enabled
					if (m_sensorIdx > 15) {
						exBuffer[n++] = 0x00 | (sensor.m_dataType & 0x0F);		// sensor id > 15 --> put id to next byte
						exBuffer[n++] = m_sensorIdx;
					} else
						exBuffer[n++] = (m_sensorIdx << 4) | (sensor.m_dataType & 0x0F);  // 4Bit id, 4 bit data type (i.e. int14_t)

					bufLen = sensor.m_bufLen;
					n += sensor.jetiEncodeValue(&exBuffer[n]);
				}
				if (m_sensorIdx >= MAX_SENSORS) m_sensorIdx = 0;				// wrap index when array is at the end
				if (++nVal >= MAX_SENSORS) break;								// dont send twice in a frame
			} while (n < (26 - bufLen));										// jeti spec says max 29 Bytes per buffer
		}
	}

	exBuffer[0] = 0x2F;															// start of packet          
	exBuffer[1] |= n-1;															// frame length to Byte 2, packet length omitting 0x7e and crc8
	exBuffer[2] = MANUFACTURER_ID_LOW; exBuffer[3] = MANUFACTURER_ID_HI;		// sensor ID
	exBuffer[4] = m_devIdLow;          exBuffer[5] = m_devIdHi;
	exBuffer[6] = 0x00;															// reserved (key for encryption)
	uint8_t crc = 0;															// calculate crc
	for (uint8_t c = 1; c < n; c++)
		crc = update_crc(exBuffer[c], crc);
	exBuffer[n] = crc;
	return n + 1;																// return length
}

// **************************************
// Helpers
// **************************************
// merge name and unit and terminate with '\0'
void JetiSensor::copyLabel(const uint8_t * name, const uint8_t * unit,  uint8_t * dest, uint8_t dest_size, uint8_t * nameLen, uint8_t * unitLen) {
	uint8_t maxchar = dest_size -1 ;
	uint8_t i =0, j =0;
	while( name[i] != '\0' && j < maxchar )
		dest[j++] = name[i++];
	*nameLen = i;

	i = 0;
	while( unit[i] != '\0' && j < maxchar )
		dest[j++] = unit[i++];
	*unitLen = i;
	dest[j] = '\0';
}

// encode sensor value to jeti ex format and copy to buffer
uint8_t JetiSensor::jetiEncodeValue( uint8_t * ex_buf) {
  switch (m_dataType) {
  case TYPE_6b:
    ex_buf[0]  = ( m_value & 0x1F) | ((m_value < 0) ? 0x80 :0x00 );               // 5 bit value and sign 
    ex_buf[0] |= m_precision;                                                     // precision in bit 5/6 (0, 20, 40)
    return 1;
  case TYPE_14b:
    ex_buf[0]  = m_value & 0xFF;                                                  // lo byte
    ex_buf[1]  = ( (m_value >> 8) & 0x1F) | ((m_value < 0) ? 0x80 :0x00 );        // 5 bit hi byte and sign 
    ex_buf[1] |= m_precision;                                                     // precision in bit 5/6 (0, 20, 40)
    return 2;
  case TYPE_22b:
    ex_buf[0]  = m_value & 0xFF;                                                  // lo byte
    ex_buf[1]  = (m_value >> 8 ) & 0xFF;                                          // mid byte
    ex_buf[2]  = ((m_value >> 16) & 0x1F) | ((m_value < 0) ? 0x80 :0x00 );		  // 5 bit hi byte and sign 
    ex_buf[2] |= m_precision;                                                     // precision in bit 5/6 (0, 20, 40)
    return 3;
  case TYPE_DT:
    ex_buf[0]  = m_value & 0xFF;                                                  // value has been prepared by SetSensorValueDate/Time 
    ex_buf[1]  = (m_value >> 8 ) & 0xFF;
    ex_buf[2]  = ( (m_value >> 16) & 0xFF) | ((m_value < 0) ? 0x80 :0x00 );
    return 3;
  case TYPE_30b:
    ex_buf[0]  = m_value & 0xFF;                                                  // lo byte
    ex_buf[1]  = (m_value >> 8 ) & 0xFF;
    ex_buf[2]  = (m_value >> 16 ) & 0xFF;
    ex_buf[3]  = ( (m_value >> 24) & 0x1F) | ((m_value < 0) ? 0x80 :0x00 );       // 5 bit hi byte and sign 
    ex_buf[3] |= m_precision;                                                     // precision in bit 5/6 (0, 20, 40)
    return 4;
  case TYPE_GPS:
    ex_buf[0]  = m_value & 0xFF;                                                  // value has been prepared by SetSensorValueGPS 
    ex_buf[1]  = (m_value >> 8 ) & 0xFF;                                          
    ex_buf[2]  = (m_value >> 16) & 0xFF;
    ex_buf[3]  = (m_value >> 24) & 0xFF;
    return 4;
  }
  return 0;
}


// copy sensor label to ex buffer
uint8_t JetiSensor::jetiCopyLabel(uint8_t *ex_buf) {
	uint8_t i =0;
	while (m_label[i] != '\0'){
		ex_buf[i] = m_label[i];
		i++;
	}
	return(i); 						// number of bytes copied
}



// **************************************
// Jeti helpers
// **************************************
// Published in "JETI Telemetry Protocol EN V1.06"
//* Jeti EX Protocol: Calculate 8-bit CRC polynomial X^8 + X^2 + X + 1
uint8_t JetiExProtocol::update_crc (uint8_t crc, uint8_t crc_seed) {
  unsigned char crc_u;
  unsigned char i;
  crc_u = crc;
  crc_u ^= crc_seed;
  for (i=0; i<8; i++)
    crc_u = ( crc_u & 0x80 ) ? POLY ^ ( crc_u << 1 ) : ( crc_u << 1 );
  return (crc_u);
}

//* Calculate CRC8 Checksum over EX-Frame, Original code by Jeti
uint8_t JetiExProtocol::jeti_crc8(uint8_t *exbuf, unsigned char framelen) {
	uint8_t crc = 0;
	uint8_t c;
	for (c = 2; c<framelen; c++)
		crc = update_crc(exbuf[c], crc);
	return (crc);
}
