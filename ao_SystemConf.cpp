
#include "ao_SystemConf.h"
#include "ao_JetiExBusSensor.h"
#include "ao_JetiExBus.h"
#include "ao_Eeprom.h"

extern JetiExBus    exBus;          // Jeti EX Bus

JBMenue* SystemConfFactory::getNewMenue(void){
  jbmObA = new JBMBase[1];
  jbmObA[0].i = new SystemConfInterface;
  return new JBMenueSel(jbmObA, JBM_SysCon_table, JBMSYSCONF_TAB_LENGTH);
}

void  SystemConfInterface::JBMGetParameter(JBMenueSel *jbms, uint8_t num){
  if (num<16){
    jbms->value=p_SystemConf->getHardwareActive(num);
  } else {
	  jbms->value=exBus.getSensorEnable(num+1-16);             // jeti starts with 1
  }
  jbms->confMode=CONF_MODE_PRINT_O_O;      // ON / OFF
}

void  SystemConfInterface::JBMRetParameter(uint16_t value, uint8_t num){
  if (num<16){
		p_SystemConf->setHardwareActive(num, (bool)value);
  } else {
	  exBus.setSensorEnable(num+1-16, (bool)value);      // jeti starts with 1
  }
}

void  SystemConfInterface::JBMEnd(void){
  p_SystemConf->eepromJob=10;
}

//---------------------------------------------------------------------
SystemConf::SystemConf(void){
  p_SystemConf=this;
}

//---------------------------------------------------------------------
void SystemConf::start(void) {
// read EEPROM
  for(uint8_t i=0; i<2; i++){
    eepromData[i]=AEEPROM.read(HARDWARE_ACTIVE +i);
  }
  hardwareActive &= ~SYS_CONF_DONT_SAVE;
}

//---------------------------------------------------------------------
void SystemConf::doMain(void) {
// eeprom jeti sensor enabled
  if (eepromJob>2){
    eepromJob--;
    uint8_t jse=0;
    for (uint8_t i=0; i<8; i++) {
      if (exBus.getSensorEnable((eepromJob-2)*8+i+1))
        jse|=(1<<i);
    }
    if (!AEEPROM.update(JETI_SENSOR_ACTIVE+eepromJob-2, jse))
      eepromJob++;
  } else if (eepromJob){
    eepromJob--;
    uint16_t tConf= hardwareActive & ~SYS_CONF_DONT_SAVE;
    if (eepromJob) tConf>>=8;
    if (!AEEPROM.update(HARDWARE_ACTIVE +eepromJob, (uint8_t)tConf)){
      eepromJob++;
    }
  }
}

//---------------------------------------------------------------------

void SystemConf::setHardwareActive(uint8_t id, bool state){
  if (id<16){
    if (state){
      if (!(hardwareActive&(1<<id))){
        hardwareActive|=(1<<id);
        hardwareChange|=(1<<id);
      }
    } else {
      if ((hardwareActive&(1<<id))){
        hardwareActive&=~(1<<id);
        hardwareChange|=(1<<id);
      }
    }
  }
}

bool SystemConf::getHardwareActive(uint8_t id){
  if ((id<16) && (hardwareActive&(1<<id))) return true;
  return false;
}

bool SystemConf::getHardwareChanged(uint8_t id){
  if ((id<16) && (hardwareChange&(1<<id))){
    hardwareChange&=(~(1<<id));
    return true;
  }
  return false;
}

//
