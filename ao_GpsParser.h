/*
 * This lib provides a basic GPS parser for the RMC and GGA sentence.
 * 
 Licence:

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.

*/
#ifndef AO_GPSPARSER_H
#define AO_GPSPARSER_H

#include "ao_Parser.h"

#define GPS_WORD_LENGTH 12

// Bits in newData
#define	GPS_SENTENCE_RMC	01
#define GPS_SENTENCE_GGA	02

const char GPS_Header[] PROGMEM = {
	6,4,						// string length and number of lines
  'G' ,'P' ,'R' ,'M' ,'C' ,'\0',
  'G' ,'N' ,'R' ,'M' ,'C' ,'\0',
  'G' ,'P' ,'G' ,'G' ,'A' ,'\0',
  'G' ,'N' ,'G' ,'G' ,'A' ,'\0',
};


union Clock {
	struct {
		uint8_t	year;			// 00 - 99
		uint8_t	month;			// 01 - 12
		uint8_t	day;			// 01 - 31
	};
	struct {
		uint8_t	hour;			// 00 - 23
		uint8_t	minute;			// 00 - 59
		uint8_t	second;			// 00 - 59
	};
	uint8_t c[3];
};

union ValGPS {
	struct {
		uint16_t	minute;		// 00xx * 4096 + nachkomma
		uint8_t		degs;		// degrees
		uint8_t		nesw;		// 0b0xx0.0000
	};
	uint32_t	d;				// for quick access
};

struct GpsData {				// main gps data structure
	Clock		time;
	Clock		date;
	char		valid;			// A:okay V:invalid	
	ValGPS		latitude; 		
	ValGPS		longitude;		
	uint16_t	altitude;		//
	uint16_t	speed;			//
	uint16_t	course;
//	uint8_t		satellites;		//
	uint8_t		fix;			
};


// -- generic sentence
class GpsInterface{

public:
	GpsInterface(void);
	virtual void    transferData(void)=0;
	virtual void	checkWord(void)=0;

	bool 	nextChar(char c);
	uint8_t doCrc(char c);
	uint8_t getCharToNibble(char c);

// sentence
	uint8_t wordCnt;
	uint8_t charCnt;
	char	wordBuf[GPS_WORD_LENGTH];
	uint8_t status;
// CRC
	uint8_t crcCntDat;
	uint8_t crcSum;
	uint8_t crcValue;

	static volatile uint8_t	newData;	// there is only one status for all interfaces
										// RMC=01, GGA=02
};


// -- GPRMC sentence specific
class GPS_Rmc: public GpsInterface {
public:
	GPS_Rmc(GpsData *gpd);
	virtual void checkWord(void);
	virtual void transferData(void);
private:
	void readClock(Clock &clock);
	void readValGPS(ValGPS &deg);
	GpsData *gpdata;	
	
// transfer these data if CRC is okay
	Clock		date;
	Clock		time;
	char 		valid;
	ValGPS		latitude; 		
	ValGPS		longitude;		
	uint16_t	speed;
	uint16_t	course;
};

// -- GPGGA sentence specific 
class GPS_Gga: public GpsInterface {
public:
	GPS_Gga(GpsData *gpd);
	virtual void checkWord(void);
	virtual void transferData(void);
	GpsData *gpdata;

private:
// transfer these data if CRC is okay
	uint8_t		fix;
//	uint8_t		satellites;
	uint16_t	altitude;	
};


//-- Master class
class GPS_Parser{
public:
	GPS_Parser(void);
	uint8_t 	parseStream(char c);	// call to parse character
	uint8_t 	available(void);		// returns parser status per sentence, see newData
	void 		done(uint8_t what);		// clears parser status sentence number in newData
	GpsData 	gpsData;				// gps data store
private:
	Parser 		pa;						// header parser
	GpsInterface  *gpi;					// pointer to sentence parser
	uint8_t 	state;					// detected header number
};

#endif