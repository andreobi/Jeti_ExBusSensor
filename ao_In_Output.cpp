
#include "ao_In_Output.h"
#include "ao_JetiExBusSensor.h"
#include "ao_JetiExBus.h"
#include "ao_Eeprom.h"

// inner Class channel assignment ----------------
JBMenue* InOutput::JBIOchselectFactory::getNewMenue(void){
  jbmObA = new JBMBase[1]; 
  jbmObA[0].i= new JBIOchselectInterface;
  return new JBMenueSel(jbmObA, JBM_IOcs_table, JBMIOCS_TAB_LENGTH);
}

void  InOutput::JBIOchselectInterface::JBMGetParameter(JBMenueSel *jbm, uint8_t num){
  jbm->value=p_InOutput->ioChSel[num];
  jbm->mini=1;
  jbm->maxi=24;
  jbm->inc=1;
  jbm->confMode=CONF_MODE_TURN_OFF;
}

void  InOutput::JBIOchselectInterface::JBMRetParameter(uint16_t value, uint8_t num){
  p_InOutput->ioChSel[num]=value;
}

void  InOutput::JBIOchselectInterface::JBMEnd(void){
  p_InOutput->eepromJob=JBMIOCS_TAB_LENGTH+JBMIOCH_TAB_LENGTH;
}

// inner Class time definition ----------------
JBMenue* InOutput::JBIOswitchFactory::getNewMenue(void){
  jbmObA = new JBMBase[1];
  jbmObA[0].i = new JBIOswitchInterface;
  return new JBMenueSel(jbmObA, JBM_IOch_table, JBMIOCH_TAB_LENGTH);
}

void  InOutput::JBIOswitchInterface::JBMGetParameter(JBMenueSel *jbm, uint8_t num){
  jbm->value=(p_InOutput->ioChVal[num]+49)*10;
  jbm->mini=500;
  jbm->maxi=2500;
  jbm->inc=10;
  jbm->confMode=CONF_MODE_TURN_OFF;
  jbm->unit[0]='u';
  jbm->unit[1]='s';
  jbm->unit[2]=' ';
}

void  InOutput::JBIOswitchInterface::JBMRetParameter(uint16_t value, uint8_t num){
  if (value)
    p_InOutput->ioChVal[num]=value/10-49;
  else
    p_InOutput->ioChVal[num]=0;
}

void  InOutput::JBIOswitchInterface::JBMEnd(void){
  p_InOutput->eepromJob=JBMIOCS_TAB_LENGTH+JBMIOCH_TAB_LENGTH;
}

//  Class InOutput ----------------
JBMenue* InOutputFactory::getNewMenue(void){
  jbmObA = new JBMBase[JBMI_O_TAB_LENGTH-1];
  jbmObA[0].f= new InOutput::JBIOchselectFactory;
  jbmObA[1].f= new InOutput::JBIOswitchFactory;
  return new JBMenue(jbmObA, JBM_I_O_table, JBMI_O_TAB_LENGTH);
}

InOutput::InOutput(void){
  p_InOutput=this;
}

// call to initialise
void InOutput::start(void) {
// read EEPROM
  for(uint8_t i=0; i<JBMIOCH_TAB_LENGTH; i++){
    eepromData[i]=AEEPROM.read(IO_CHANNEL +i);
  }

// init & set output port PIN_0 und PIN_1 
//  PORTD &=(~(1<<PD4));    // done in main program
//  DDRD |=(1<<PD4);
//  PORTB &=(~(1<<PB1));
//  DDRB |=(1<<PB1);
}

void InOutput::doMain(void){
// eeprom
  if (eepromJob){
    eepromJob--;
    if (!AEEPROM.update(IO_CHANNEL +eepromJob, eepromData[eepromJob])){
      eepromJob++;
    }
  }
}

void InOutput::setPin0(bool o){
  if (o)
    PORTD |=(1<<PD4);
  else
    PORTD &=(~(1<<PD4));  
}

void InOutput::setPin1(bool o){
  if (o)
    PORTB |=(1<<PB1);
  else
    PORTB &=(~(1<<PB1));  
}

// true: tx channel level in range 
bool InOutput::getIOStatus(uint8_t functionNum){
  if (functionNum<JBMIOCS_TAB_LENGTH){
    uint8_t channelNum=ioChSel[functionNum];
    if (channelNum && channelNum<24){               // 0: OFF, max channel number of Jeti is 24
      uint16_t rxValue=exBus.getChannel(channelNum-1);
      uint16_t mini=ioChVal[functionNum*2];
      if (mini) mini=((mini-1)*80)+4000;            // 0: OFF for mini
      uint16_t maxi=ioChVal[functionNum*2+1];
      if (maxi) maxi=((maxi-1)*80)+4000;            // o: OFF for maxi
      if (mini && maxi){                            // both limits turned ON?
        if (mini<=maxi){                            // select switch logic
          if (mini<=rxValue && rxValue<=maxi) return true;  // turn ON if value is in Window
        } else {
          if (mini<=rxValue || rxValue<=maxi) return true;  // turn ON if value is outside Window
        }
      } else {                                      // only one limit is on
        if (mini && mini<=rxValue) return true; 
        if (maxi && rxValue<=maxi) return true;
      }
    }
  }
  return false;
}
