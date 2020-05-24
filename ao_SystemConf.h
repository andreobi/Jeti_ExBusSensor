
#ifndef SYSTEM_CONF_H
#define SYSTEM_CONF_H

#include <Arduino.h>
#include "ao_JBMenue.h"

// define Hardware configuration flags
#define SYS_CONF_HW_ESC           0
#define SYS_CONF_HW_GPS           1
#define SYS_CONF_HW_PRESSURE      2
#define SYS_CONF_HW_ORIENTATION   3
#define SYS_CONF_HW_ORIENT_CALI   4
#define SYS_CONF_HW_OSLBUZZER     5
#define SYS_CONF_HW_OSLLEDFLASH   6
#define SYS_CONF_HW_BLINK_P0      7
#define SYS_CONF_HW_BLINK_P1      8

#define SYS_CONF_HW_AM_EXBUS       9
#define SYS_CONF_HW_AM_ESC        10
#define SYS_CONF_HW_AM_SENSOR     11
#define SYS_CONF_HW_WM2           12
#define SYS_CONF_HW_IM1           13
#define SYS_CONF_HW_AM_ESC_F      14
#define SYS_CONF_HW_CL_ERR        15

// which configuration is only temporarely valid not safed to eeprom
#define SYS_CONF_DONT_SAVE  ((1<<SYS_CONF_HW_ORIENT_CALI)|(1<<SYS_CONF_HW_CL_ERR))
//

// Table names(x+8) should fit Sensor Names(x)
const char JBMSysConL0[] PROGMEM = "HW:ESC";          // 0 (sysConf.GetHardwareActive(1))
const char JBMSysConL1[] PROGMEM = "HW:GPS";          // 1 (sysConf.GetHardwareActive(1))
const char JBMSysConL2[] PROGMEM = "HW:Pressure";     // 2 (sysConf.GetHardwareActive(2))
const char JBMSysConL3[] PROGMEM = "HW:Orient";       // 3 (sysConf.GetHardwareActive(3))
const char JBMSysConL4[] PROGMEM = "Orient.Calib";    // 4  enable to safe BNO calibration data
const char JBMSysConL5[] PROGMEM = "OSL:buzzer";
const char JBMSysConL6[] PROGMEM = "OSL:flash";       // 6 osl: on signal lost
const char JBMSysConL7[] PROGMEM = "Pin0 Blink";
const char JBMSysConH0[] PROGMEM = "Pin1 Blink";      // 8 stable or 1Hz blinking output
const char JBMSysConH1[] PROGMEM = "Alarm EXbus";     // 9
const char JBMSysConH2[] PROGMEM = "Alarm ESC";       // 10
const char JBMSysConH3[] PROGMEM = "Alarm Sensor";    // 11
const char JBMSysConH4[] PROGMEM = "Warn Reset";      // 12
const char JBMSysConH5[] PROGMEM = "Info Pin";        // 13

const char JBMSysConH6[] PROGMEM = "?";             // 14   ??? free
const char JBMSysConH7[] PROGMEM = "Reset Error";     // 15 Reset
// Sensor output on/off should fit to sensor list in JetiBus...
const char JBMSysConS01[] PROGMEM = "ecu RPM";
const char JBMSysConS02[] PROGMEM = "ecu U Batt";
const char JBMSysConS03[] PROGMEM = "ecu I Batt";
const char JBMSysConS04[] PROGMEM = "ecu Capacity";
const char JBMSysConS05[] PROGMEM = "ecu U BEC";
const char JBMSysConS06[] PROGMEM = "ecu I BEC";
const char JBMSysConS07[] PROGMEM = "ecu T Motor";
const char JBMSysConS08[] PROGMEM = "ecu T BEC";
const char JBMSysConS09[] PROGMEM = "ecu State";
//
const char JBMSysConS10[] PROGMEM = "o Headding";
const char JBMSysConS11[] PROGMEM = "o Roll";
const char JBMSysConS12[] PROGMEM = "o Nick";
const char JBMSysConS13[] PROGMEM = "o ACC X";
const char JBMSysConS14[] PROGMEM = "o ACC Y";
const char JBMSysConS15[] PROGMEM = "o ACC Z";
const char JBMSysConS16[] PROGMEM = "o ACC T";
//
const char JBMSysConS17[] PROGMEM = "p Altitude";
const char JBMSysConS18[] PROGMEM = "p Vario";
const char JBMSysConS19[] PROGMEM = "p Temp";
//
const char JBMSysConS20[] PROGMEM = "gps Latitude";
const char JBMSysConS21[] PROGMEM = "gps Longitude";
const char JBMSysConS22[] PROGMEM = "gps Altitude";
const char JBMSysConS23[] PROGMEM = "gps Speed";
const char JBMSysConS24[] PROGMEM = "gps Course";
const char JBMSysConS25[] PROGMEM = "gps Time";
//
const char JBMSysConS26[] PROGMEM = "s AD0";
const char JBMSysConS27[] PROGMEM = "s AD1";
const char JBMSysConS28[] PROGMEM = "s AD2";
const char JBMSysConS29[] PROGMEM = "s Current";
const char JBMSysConS30[] PROGMEM = "s Capacity";
const char JBMSysConS31[] PROGMEM = "s AD0min";
const char JBMSysConS32[] PROGMEM = "s AD0max";
//
const char JBMSysConS33[] PROGMEM = "s RPM lo";
const char JBMSysConS34[] PROGMEM = "s RPM hi";
//
const char JBMSysConS35[] PROGMEM = "ecu PWM in";
const char JBMSysConS36[] PROGMEM = "ecu PWM out";
const char JBMSysConS37[] PROGMEM = "ecu Timing";
const char JBMSysConS38[] PROGMEM = "ecu nCell";
//
const char JBMSysConS39[] PROGMEM = "gps Fix";
const char JBMSysConS40[] PROGMEM = "S: End";

const char *const JBM_SysCon_table[] PROGMEM = 
	{JBMSysConL0,JBMSysConL1,JBMSysConL2,JBMSysConL3,JBMSysConL4,JBMSysConL5,JBMSysConL6,JBMSysConL7,
   JBMSysConH0,JBMSysConH1,JBMSysConH2,JBMSysConH3,JBMSysConH4,JBMSysConH5,JBMSysConH6,JBMSysConH7,
//
                JBMSysConS01,JBMSysConS02,JBMSysConS03,JBMSysConS04,JBMSysConS05,JBMSysConS06,JBMSysConS07,JBMSysConS08,JBMSysConS09,
	 JBMSysConS10,JBMSysConS11,JBMSysConS12,JBMSysConS13,JBMSysConS14,JBMSysConS15,JBMSysConS16,JBMSysConS17,JBMSysConS18,JBMSysConS19,
	 JBMSysConS20,JBMSysConS21,JBMSysConS22,JBMSysConS23,JBMSysConS24,JBMSysConS25,JBMSysConS26,JBMSysConS27,JBMSysConS28,JBMSysConS29,
   JBMSysConS30,JBMSysConS31,JBMSysConS32,JBMSysConS33,JBMSysConS34,JBMSysConS35,JBMSysConS36,JBMSysConS37,JBMSysConS38,JBMSysConS39,
   JBMSysConS40,
	};
#define JBMSYSCONF_TAB_LENGTH (sizeof(JBM_SysCon_table)/sizeof(JBM_SysCon_table[0]))

class SystemConfFactory : public JBMFactory {
public:
    virtual JBMenue *getNewMenue(void);
//    virtual void  deleteNewMenue(void);
};

class SystemConfInterface : public JBMInterface {
public:
  virtual void    JBMGetParameter(JBMenueSel *jbms,uint8_t num);
  virtual void    JBMRetParameter(uint16_t value, uint8_t num); // could be used to retrieve new value
  virtual void    JBMEnd(void);
};

//-----------------------
class SystemConf {
  friend SystemConfInterface;
  public:
         SystemConf(void);
    void start(void);
    void  doMain(void);                         // called from main loop 

    void setHardwareActive(uint8_t id, bool state);
    bool getHardwareActive(uint8_t id);
    bool getHardwareChanged(uint8_t id);
  protected:
    uint8_t eepromJob;
  private:
    union{
      uint16_t hardwareActive;
      uint8_t  eepromData[2];
    };
    uint16_t hardwareChange;
};

static SystemConf *p_SystemConf;
#endif // 
