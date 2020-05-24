
#include "ao_GpsParser.h"
#include "Arduino.h"


volatile uint8_t	GpsInterface::newData;
	
	
GpsInterface::GpsInterface(void) 
	: crcCntDat(0), crcSum(0), wordCnt(0), charCnt(0) {
}

bool GpsInterface::nextChar(char c){
    uint8_t result = doCrc(c);
    if (result==0) {
		if (c==',') {				// new word
			checkWord();
			wordBuf[charCnt]='\0';	// terminate string
//Serial.print(wordCnt);
//Serial.print(" ");
//Serial.println(wordBuf);
			wordCnt++;
			charCnt=0;
		} else {
			wordBuf[charCnt]=c;
			if (charCnt<GPS_WORD_LENGTH-1) charCnt++;
		}
	} else if (result==255){
		return true;				// sentence end
    } else if (result==1) {
		transferData();				// tranfer sentence data
		return true;				// sentence end
    } else if (result==3){
		checkWord();				// finalize last word in sentence
	}
    return false;					// continuie
}

uint8_t GpsInterface::doCrc(char c){
  if (c=='*'){
    crcCntDat=2;					// detected CRC start
    return 3;
  } else if (crcCntDat){
    crcCntDat--;
    if (crcCntDat) {
      crcValue=getCharToNibble(c)<<4;
      return 2;
    } else {
      crcValue|=getCharToNibble(c);
      return (crcSum==crcValue?1:255);
    }
  } else {
    crcSum^=c;						// Streamdata: add data to crc
  }
  return 0;
}

// takes a char and returns it as a lower hex nibble
uint8_t GpsInterface::getCharToNibble(char c){
    return (c>'9'?(c>'Z'?c-'a'+10:c-'A'+10):c-'0')&0x0F;
}



//------------------------------------------------
GPS_Rmc::GPS_Rmc(GpsData *gpd){
	gpdata=gpd;
}

void GPS_Rmc::checkWord(void){
	switch (wordCnt) {
		case 1:{		// time
			readClock(time);
		break;}
		case 2:{		// vali A:ok, V:invalid
			valid=wordBuf[0];
		break;}
		case 3:{		// Latitude 90
			readValGPS(latitude);
		break;}
		case 4:{		// North/South
			latitude.nesw=(wordBuf[0]=='N'?0x00:0x40);
		break;}
		case 5:{		// Longitude 180
			readValGPS(longitude);
		break;}
		case 6:{		// East/West
			longitude.nesw=(wordBuf[0]=='E'?0x20:0x60);
		break;}
		case 7:{		// Speed
			speed=(uint16_t)(18.52 * atof(wordBuf));
		break;}
		case 8:{		// course
			course=(uint16_t)(10 * atof(wordBuf));
		break;}
		case 9:{		// Date
			readClock(date);
		break;}
//		case 10:{		// Variation
//		break;}
//		case 11:{		// East/West
//		break;}
	}
}

void GPS_Rmc::transferData(){	// set new sentence to data struct
	gpdata->time=time;
	gpdata->date=date;
	gpdata->valid=valid;
	if (valid=='A'||valid=='a'){
		gpdata->latitude=latitude;
		gpdata->longitude=longitude;
		gpdata->speed=speed;
		gpdata->course=course;
	}
	GpsInterface::newData|=GPS_SENTENCE_RMC;
}

void GPS_Rmc::readClock(Clock &clock){
	char b[3];
	b[2]='\0';
	for (uint8_t i=0; i<3; i++){
		b[0]=wordBuf[2*i];
		b[1]=wordBuf[2*i+1];		
		clock.c[i]=atoi(b);
	}
}

void GPS_Rmc::readValGPS(ValGPS &deg){
	uint8_t offset=0;			// code gps to jeti format
	char b[6];
	b[0]=wordBuf[0];
	b[1]=wordBuf[1];
	if (wordBuf[4]=='.') {
		b[2]='\0';
	} else {
		offset=1;
		b[2]=wordBuf[2];
		b[3]='\0';		
	}
	deg.degs=(uint8_t)atoi(b);
//
	b[0]=wordBuf[2+offset];
	b[1]=wordBuf[3+offset];
	// skip the '.'
	b[2]=wordBuf[5+offset];
	b[3]=wordBuf[6+offset];
	b[4]=wordBuf[7+offset];
	b[5]='\0';
	deg.minute=(uint16_t)atoi(b);
}


//------------------------------------------------
GPS_Gga::GPS_Gga(GpsData *gpd){
	gpdata=gpd;
}

void GPS_Gga::checkWord(void){
	switch (wordCnt) {
		case 6:{		// fix
			fix=atoi(wordBuf);
		break;}
//		case 7:{		// number of satellites
//			satellites=atoi(wordBuf);
//		break;}
		case 9:{		// Antenna altitude
			altitude=(uint16_t)(10* atof(wordBuf));
		break;}
	}
}

void    GPS_Gga::transferData(){	// set new sentence to data struct
	if (gpdata->valid=='A'||gpdata->valid=='a'){
		gpdata->altitude=altitude;
	}
	gpdata->fix=fix;
//	gpdata->satellites=satellites;
	GpsInterface::newData|=GPS_SENTENCE_GGA;
}


//------------------------------------------------

GPS_Parser::GPS_Parser(void) : state(0) {
	pa.setTable(GPS_Header);
}

uint8_t GPS_Parser::parseStream(char c){
	if (c=='$') {					// start new sentence?
      state=0;						// sentence finisched
      if (gpi) delete gpi;
      gpi=nullptr;
	} else if (state){
    if (gpi->nextChar(c)){
      state=0;						// sentence finisched
      if (gpi) delete gpi;
      gpi=nullptr;
    }
  } else {
    state=pa.parse(c);				// find sentence Header
    if (state) {					// new header found
      switch (state){				// assign new handler object
        case 1: 
        case 2: {
          gpi = new GPS_Rmc(&gpsData);
          break;}
        case 3: 
        case 4: {
          gpi = new GPS_Gga(&gpsData);
          break;}
      }
      if (gpi) {
        char buf[10];
        pa.getString(state, buf);	// add header to crc
        uint8_t i=0;
        while (buf[i]) gpi->doCrc(buf[i++]);
      } else {
        state=0;					// somthing went wrong
      }
    }
  }
}


uint8_t GPS_Parser::available(void){
	return GpsInterface::newData;		//RMC=01, GGA=02
}

void GPS_Parser::done(uint8_t what){
	uint8_t cSREG= SREG;
	cli();
	GpsInterface::newData&=(~what);
	SREG = cSREG;						// restore SREG value (I-bit)
}

//
