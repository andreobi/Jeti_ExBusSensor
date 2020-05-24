#include "ao_ADInterrupt.h"
#include "ao_In_Output.h"
#include "ao_LedFlash.h"
#include "ao_SystemConf.h"

#ifndef JETIEXBUSSENSOR_H
#define JETIEXBUSSENSOR_H

#define WITH_KONTRONIK


// SW_VERSION is shown in JB info
#define AO_SW_VERSION  1

// must aslo defined in Jeti Protocol
#define JETI_MAX_SENSORS  32

// define system Flags 0-7
#define MAIN_1S         0x01
#define BLINK_1S        0x02
//???
#define FREExyz     0x04

#define READY_MASSAGE   0x08
#define BME_INIT        0x10
#define SWUSART_ERROR   0x20
#define RESET_WATCHDOG  0x40
#define RESET_BROWNOUT  0x80


// EEPROM Address definition, each value is [uint8_t]
enum {
// BYTE:32bit/8 *3 =12 LED Flash light pattern + period
  LEDPATTRERN=1,

// WORD: adjustment for ACD (4x), current max (1x), timer (2x), BNO (2x)
  ADJUSTVALUE = LEDPATTRERN + 13,

// BYTE: switch - channel selection - leve for output control
  IO_CHANNEL = ADJUSTVALUE + 2*JBMADJUST_TAB_LENGTH,

// turn ON/OFF Hardware components
  HARDWARE_ACTIVE = IO_CHANNEL + JBMIOCH_TAB_LENGTH + JBMIOCS_TAB_LENGTH,
// turn ON/OFS Jeti sensors (active) (array for 64 sensors reserved)
  JETI_SENSOR_ACTIVE = HARDWARE_ACTIVE +2,

//BNO055 calibration data 22byte + 1 checksum
  BNO_CALIB_EEPROM =JETI_SENSOR_ACTIVE +8,

// used in the main program
  EEPROM_MAIN=BNO_CALIB_EEPROM + 23,
// 4 are used
// 6 free
XXXXXXX=EEPROM_MAIN+10
};

const char JBMH[] PROGMEM = "all in one";         // jetiBox Main mene Header (line0)
const char JBM0[] PROGMEM = "Info";               //
const char JBM1[] PROGMEM = "I/O conf";        //
const char JBM2[] PROGMEM = "Flash";
const char JBM3[] PROGMEM = "Adjust";
const char JBM4[] PROGMEM = "Config";
const char *const JBM_Main_table[] PROGMEM ={JBMH,JBM0,JBM1,JBM2,JBM3,JBM4}; //,JBM5,JBM6
#define JBMMAIN_TAB_LENGTH (sizeof(JBM_Main_table)/sizeof(JBM_Main_table[0]))

union EE_data{
  struct{
    uint16_t resetCntBO;            // resetConter BrownOut
    uint16_t resetCntWD;            // resetConter WatchDog
  };
  uint8_t eepromData[10];
};

void mainIO(void);
void mainFlash(void);
void mainPressure(void);
void mainSoftSerialIn(void);
void mainFastSerialIn(void);
void mainADC(void);
void mainJetiData(void);
void mainJetiBox(void);
void mainWatchDog(void);
void mainSystemMessage(void);
//
bool getFlag(uint8_t id);
void setFlag(uint8_t id, bool state);
//
void eepromResetBitCnt(void);
void eepromIncBitCnt(uint16_t &adr);
uint8_t eepromReadBitCnt(uint16_t &adr);

bool  BNOsafeCalibData(void);
bool  BNOreadCalibData(void);

// jeti display messages
//0 Basic informative message (really unimportant messages)
//1 Status message (device ready, motors armed, GPS position fix etc.)
//2 Warning (alarm, high vibrations, preflight conditions check, …)
//3 Recoverable error (loss of GPS position, erratic sensor data, …)
//4 Nonrecoverable error (peripheral failure, unexpected hardware fault, …)

#define JB_MESSAGE_0_CLASS  3
#define JB_MESSAGE_1_CLASS  3
#define JB_MESSAGE_2_CLASS  3
#define JB_MESSAGE_3_CLASS  3
#define JB_MESSAGE_4_CLASS  2
#define JB_MESSAGE_5_CLASS  2
#define JB_MESSAGE_6_CLASS  1
#define JB_MESSAGE_7_CLASS  0
// waiting time[s] + class level before another message will be send, must be <8
#define JB_MESSAGE_OVERWRITE_T 2
//
const char JB_Message0[] PROGMEM = "no EX>0.3s";
const char JB_Message1[] PROGMEM = "ESC signal";
const char JB_Message2[] PROGMEM = "ESC Warning";
const char JB_Message3[] PROGMEM = "Sensor Err";      // BNO BME GPS ...
const char JB_Message4[] PROGMEM = "BrownOut";
const char JB_Message5[] PROGMEM = "WatchDog";
const char JB_Message6[] PROGMEM = "PIN low";
const char JB_Message7[] PROGMEM = "takeoff";
const char JB_Message10[] PROGMEM = "?";

const char *const JB_Message_table[] PROGMEM = {JB_Message0,JB_Message1,JB_Message2,JB_Message3,JB_Message4,JB_Message5,JB_Message6,JB_Message7};
#define JBMMESSAGE_TAB_LENGTH (sizeof(JB_Message_table)/sizeof(JB_Message_table[0]))
//

// name plus unit must be < 20 characters
// precision = 0 --> 0, precision = 1 --> 0.0, precision = 2 --> 0.00
JETISENSOR_CONST sensors[] PROGMEM = {
//   name        unit         data type             precision 
// ESC Part 1
  { "rpm",        JetiSensor::TYPE_22b, 0 },   //  1
  { "V",          JetiSensor::TYPE_14b, 2 },
  { "A",          JetiSensor::TYPE_14b, 2 },
  { "mAh",        JetiSensor::TYPE_14b, 0 },
  { "V",          JetiSensor::TYPE_14b, 3 },
  { "A",          JetiSensor::TYPE_14b, 2 },
  { "\xB0\x43",   JetiSensor::TYPE_14b, 0 },
  { "\xB0\x43",   JetiSensor::TYPE_14b, 0 },
  { "n",          JetiSensor::TYPE_6b , 0 },
//BNO055
  { "\xB0",       JetiSensor::TYPE_14b, 0 },  // 10
  { "\xB0",       JetiSensor::TYPE_14b, 0 },
  { "\xB0",       JetiSensor::TYPE_14b, 0 },
  { "Nm",         JetiSensor::TYPE_14b, 1 },
  { "Nm",         JetiSensor::TYPE_14b, 1 },
  { "Nm",         JetiSensor::TYPE_14b, 1 },
  { "Nm",         JetiSensor::TYPE_14b, 1 },
// BME280
  { "m",          JetiSensor::TYPE_14b, 1 },  // 17
  { "m/s",        JetiSensor::TYPE_14b, 1 },
  { "\xB0\x43",   JetiSensor::TYPE_6b , 0 },
// GPS
  { " ",          JetiSensor::TYPE_GPS, 0 },  // 20
  { " ",          JetiSensor::TYPE_GPS, 0 },
  { "m",          JetiSensor::TYPE_14b, 1 },
  { "kmh",        JetiSensor::TYPE_14b, 1 },
  { "\xB0\x43",   JetiSensor::TYPE_14b, 1 },  
  { " ",          JetiSensor::TYPE_DT , 0 },
// internal ADC
  { "V",          JetiSensor::TYPE_14b, 1 },  // 26
  { "V",          JetiSensor::TYPE_14b, 1 },
  { "V",          JetiSensor::TYPE_14b, 1 },
  { "A",          JetiSensor::TYPE_14b, 1 },
  { "mAh",        JetiSensor::TYPE_14b, 2 },
  { "V",          JetiSensor::TYPE_14b, 1 },
  { "V",          JetiSensor::TYPE_14b, 1 },
// RPM
  { "n",          JetiSensor::TYPE_22b, 0 }, // 32
  { "n",          JetiSensor::TYPE_22b, 0 },
// ESC Part 2
  { "ms",         JetiSensor::TYPE_14b, 0 },  // 34
  { "%",          JetiSensor::TYPE_6b , 0 },
  { "\xB0",       JetiSensor::TYPE_6b , 0 },
  { "n",          JetiSensor::TYPE_6b , 0 },
  { "n",          JetiSensor::TYPE_6b , 0 },
  { "n",          JetiSensor::TYPE_6b , 0 },
  { "n",          JetiSensor::TYPE_6b , 0 },

/*
//   name        unit         data type             precision 
// ESC Part 1
  { "RPM"       , "Upm",        JetiSensor::TYPE_14b, 0 },   //  1
  { "U Batt"    , "V",          JetiSensor::TYPE_14b, 2 },
  { "I Batt"    , "A",          JetiSensor::TYPE_14b, 2 },
  { "Capacity"  , "mAh",        JetiSensor::TYPE_14b, 0 },
  { "U BEC"     , "V",          JetiSensor::TYPE_14b, 3 },
  { "I BEC"     , "A",          JetiSensor::TYPE_14b, 2 },
  { "T Motor"   , "\xB0\x43",   JetiSensor::TYPE_14b, 0 },
  { "T BEC"     , "\xB0\x43",   JetiSensor::TYPE_14b, 0 },
  { "Status"    , "n",          JetiSensor::TYPE_6b , 0 },
//BNO055
  { "Headding"  , "\xB0",       JetiSensor::TYPE_14b, 0 },  // 10
  { "Roll"      , "\xB0",       JetiSensor::TYPE_14b, 0 },
  { "Nick"      , "\xB0",       JetiSensor::TYPE_14b, 0 },
  { "ACC X"     , "Nm",         JetiSensor::TYPE_14b, 1 },
  { "ACC Y"     , "Nm",         JetiSensor::TYPE_14b, 1 },
  { "ACC Z"     , "Nm",         JetiSensor::TYPE_14b, 1 },
// BME280
  { "Altitude"  , "m",          JetiSensor::TYPE_14b, 1 },  // 16
  { "Vario"     , "m/s",        JetiSensor::TYPE_14b, 1 },
  { "Temp."     , "\xB0\x43",   JetiSensor::TYPE_14b, 0 },
// GPS
  { "Latitude"  , " ",          JetiSensor::TYPE_GPS, 0 },  // 19
  { "Longitude" , " ",          JetiSensor::TYPE_GPS, 0 },
  { "Altitude"  , "m",          JetiSensor::TYPE_14b, 1 },
  { "Speed"     , "kmh",        JetiSensor::TYPE_14b, 1 },
  { "Course"    , "\xB0\x43",   JetiSensor::TYPE_14b, 1 },  
  { "Time"      , " ",          JetiSensor::TYPE_DT , 0 },
// internal ADC
  { "ADC 0"     , "V",          JetiSensor::TYPE_14b, 1 },  // 26
  { "ADC 1"     , "V",          JetiSensor::TYPE_14b, 1 },
  { "ADC 2"     , "V",          JetiSensor::TYPE_14b, 1 },
  { "I    AD3"  , "A",          JetiSensor::TYPE_14b, 1 },
  { "Capacity"  , "mAh",        JetiSensor::TYPE_14b, 2 },
  { "ADC 0 Min" , "V",          JetiSensor::TYPE_14b, 1 },
  { "ADC 0 Max" , "V",          JetiSensor::TYPE_14b, 1 },
// RPM
  { "RPM low"   , "n",          JetiSensor::TYPE_14b, 0 }, // 32
  { "RPM high"  , "n",          JetiSensor::TYPE_22b, 0 },
// ESC Part 2
  { "PWM in"    , "ms",         JetiSensor::TYPE_14b, 0 },  // 34
  { "PWM Out"   , "%",          JetiSensor::TYPE_14b, 0 },
  { "PHASE"     , "\xB0",       JetiSensor::TYPE_14b, 0 },
  { "n Cell"    , "n",          JetiSensor::TYPE_6b , 0 },
  { "n 38"    , "n",          JetiSensor::TYPE_6b , 0 },
  { "n 39"    , "n",          JetiSensor::TYPE_6b , 0 },
  { "n 40"    , "n",          JetiSensor::TYPE_6b , 0 },
*/
{ 0 } // end of array
};

// jeti sensor id's
enum JETISEN_NUM {
// ECU
  SEN_ECU_RPM   =1,
  SEN_ECU_UBAT,
  SEN_ECU_IBAT,
  SEN_ECU_CAPA,
  SEN_ECU_UBEC,
  SEN_ECU_IBEC,
  SEN_ECU_TMOT,
  SEN_ECU_TBEC,
  SEN_ECU_STATE,
//
  SEM_ORI_HEAD,
  SEM_ORI_ROLL,
  SEM_ORI_NICK,
  SEM_ORI_AC_X,
  SEM_ORI_AC_Y,
  SEM_ORI_AC_Z,
  SEM_ORI_AC_T,
//
  SEN_PRE_ALTI,
  SEN_PRE_VARI,
  SEN_PRE_TEMP,
//
  SEN_GPS_LATT,
  SEN_GPS_LONG,
  SEN_GPS_ALTI,
  SEN_GPS_SPEED,
  SEN_GPS_COURS,
  SEN_GPS_TIME,
//
  SEN_ADC_U0,
  SEN_ADC_U1,
  SEN_ADC_U2,
  SEN_ADC_I,
  SEN_ADC_C,
  SEN_AD_MI,
  SEN_AD_MX,
//
  SEN_RPM_LOW,
  SEN_RPM_HIGH,
//
  SEN_ECU_PIN,
  SEN_ECU_POUT,
  SEN_ECU_PHASE,
  SEN_ECU_CELL,
//
  SEN_GPS_SAT,
  x40
};

#endif
