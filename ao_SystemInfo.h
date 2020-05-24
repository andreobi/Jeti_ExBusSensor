
#ifndef AO_SYSTEM_INFO_H
#define AO_SYSTEM_INFO_H

#include <Arduino.h>
#include "ao_JBMenue.h"

// Sensor output on/off should fit to sensor list in JetiBus...
const char JBMSysInfo0[] PROGMEM =  "Blink Code";
const char JBMSysInfo1[] PROGMEM =  "RX exbus";
const char JBMSysInfo2[] PROGMEM =  "RX ESC";
const char JBMSysInfo3[] PROGMEM =  "RX GPS";
const char JBMSysInfo4[] PROGMEM =  "Reset CNT";
const char JBMSysInfo5[] PROGMEM =  "Pressure";
const char JBMSysInfo6[] PROGMEM =  "Orient";
//const char JBMSysInfo7[] PROGMEM =  "ESC Err.3"; // is only 0
const char JBMSysInfo8[] PROGMEM =  "ESC Hi";
const char JBMSysInfo9[] PROGMEM =  "ESC Mid";
const char JBMSysInfo10[] PROGMEM = "ESC Lo";
const char JBMSysInfo11[] PROGMEM = "ESC Version";
const char JBMSysInfo12[] PROGMEM = "AO Version";
const char *const JBM_SysInfo_table[] PROGMEM = 
	{JBMSysInfo0,JBMSysInfo1,JBMSysInfo2,JBMSysInfo3,JBMSysInfo4,JBMSysInfo5,JBMSysInfo6,
//JBMSysInfo7,
  JBMSysInfo8,JBMSysInfo9, JBMSysInfo10, JBMSysInfo11,JBMSysInfo12
	};
#define JBMSYSINFO_TAB_LENGTH (sizeof(JBM_SysInfo_table)/sizeof(JBM_SysInfo_table[0]))

const char JBMInfoMessageEXB[] PROGMEM = {"exbus"};
const char JBMInfoMessageESC[] PROGMEM = {"ESC"};
const char JBMInfoMessageGPS[] PROGMEM = {"RMC:  GGA:  RX:"};
// init failed
const char JBMInfoMessage_IF [] PROGMEM = {"INIT failed"};
const char JBMInfoMessage_NM [] PROGMEM = {"Normal Mode"};
const char JBMInfoMessage_NR [] PROGMEM = {"Init Mode"};
const char JBMInfoMessage_OFF[] PROGMEM = {"Off"};

//-----------------------
class SystemInfoFactory : public JBMFactory {
public:
    virtual JBMenue *getNewMenue(void);
//    virtual void  deleteNewMenue(void);
};

class SystemInfoInterface : public JBMInterface {
public:
  virtual void    JBMGetParameter(JBMenueSel *jbms,uint8_t num);
//  virtual void    JBMRetParameter(uint16_t value, uint8_t num); // could be used to retrieve new value
//  virtual void    JBMEnd(void);
};

class SystemInfo {
friend SystemInfoInterface;
public:

};

//static SystemInfo *p_SystemInfo;      // no interface class!
#endif // 
