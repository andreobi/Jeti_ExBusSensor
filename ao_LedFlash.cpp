
#include "ao_LedFlash.h"
#include "ao_JetiExBusSensor.h"
#include "ao_JetiExBus.h"
#include "ao_Eeprom.h"

// inner Class Period ----------------
JBMenue* LedFlash::JBPeriodFactory::getNewMenue(void){
  jbmObA = new JBMBase[1]; 
  jbmObA[0].i= new JBPeriodInterface;
  return new JBMenueSel(jbmObA, JBM_LedPer_table, JBMLEDPER_TAB_LENGTH);
}

void  LedFlash::JBPeriodInterface::JBMGetParameter(JBMenueSel *jbm, uint8_t num){
  jbm->value=((uint16_t)p_JBPattern->ledPeriod)*128;
  jbm->mini=128;
  jbm->maxi=8192;
  jbm->inc=128;
  jbm->unit[0]='m';
  jbm->unit[1]='s';
  jbm->unit[2]=' ';
  jbm->confMode=CONF_MODE_TURN_OFF;
}

void  LedFlash::JBPeriodInterface::JBMRetParameter(uint16_t value, uint8_t num){
  p_JBPattern->ledPeriod=uint8_t(value/128);
}

void  LedFlash::JBPeriodInterface::JBMEnd(void){
  p_JBPattern->eepromJob=13;
}
  
// inner Class Pattern----------------
JBMenue* LedFlash::JBPatternFactory::getNewMenue(void){
  jbmObA = new JBMBase[1]; 
  jbmObA[0].i= new JBPatternInterface;
  return new JBPattern(jbmObA);
}

void  LedFlash::JBPatternInterface::JBMEnd(void){
  p_JBPattern->eepromJob=13;
}

// LED Flash pattern selection
LedFlash::JBPattern::JBPattern(JBMBase *jbmOb) :
  JBMenue(jbmOb, 0, 0), ledBitPosJB(0), ledLine(0), line1Active(false) {
}

bool LedFlash::JBPattern::JBMDo(uint8_t key){
  if (line1Active) {                          // set / clear bit on line A,B,C
    if (key&JB_K_U) {
      line1Active=false;
    } else if (key&JB_K_R) {
      if (ledLine<2) ledLine++;
    } else if (key&JB_K_L) {
      if (ledLine) ledLine--;
    } else if (key&JB_K_D) {
      uint8_t i=(ledBitPosJB>>3)+4*ledLine;
      uint8_t bitMask=1<<(ledBitPosJB&0x07);
      p_JBPattern->ledPattern[i]=(p_JBPattern->ledPattern[i]&(~bitMask))|((~p_JBPattern->ledPattern[i])&bitMask);
    }
  } else {                                    // select pattern bit 0-31
    if (key&JB_K_D){
        line1Active=true;
    } else if (key&JB_K_U) {
      (jbmObject[0].i)->JBMEnd();
      delete jbmObject[0].i;
      delete [] jbmObject;
      return true;
    } else if (key&JB_K_R) {
      if (ledBitPosJB<31) ledBitPosJB++;
    } else if (key&JB_K_L) {
      if (ledBitPosJB) ledBitPosJB--;
    }
  }
  JBMShow();
  return false;
}

void  LedFlash::JBPattern::JBMShow(void){
  uint8_t segment=ledBitPosJB>>3;
  uint8_t bitMask=1<<(ledBitPosJB&0x07);
    char line[17];
    line[0]=(line1Active ?' ':'#');
    line[1]=(line1Active ?' ':(ledBitPosJB ?'<':' '));
    line[2]='0'+(uint8_t)(ledBitPosJB/10);
    line[3]='0'+(uint8_t)(ledBitPosJB%10);
    line[4]=(line1Active ?' ':(ledBitPosJB<31 ?'>':' '));
    line[5]=':';
    uint8_t offset=0;
    for (uint8_t i=0; i<8; i++) {
      if (i==(ledBitPosJB&0x07)) {
        line[i+6]='-';
        offset=1;
      }
      line[i+offset+6]='0'+(p_JBPattern->ledPattern[segment]&(1<<i)   ? 1:0)
                   +(p_JBPattern->ledPattern[segment+4]&(1<<i) ? 2:0)
                   +(p_JBPattern->ledPattern[segment+8]&(1<<i) ? 4:0);    
      if (i==(ledBitPosJB&0x07)) {
        line[i+8]='-';
        offset=2;
      }
    }
    line[16]='\0';
    exBus.setJetiboxText(0, line);
//
    line[0]=(line1Active ?'#':' ');
    line[1]=(line1Active ?(ledLine ?'<':' '):' ');
    line[2]=' ';
    line[3]='A'+ledLine;
    line[4]=(line1Active ?(ledLine<2 ?'>':' '):' ');
    line[5]=':';
    uint8_t b0=(p_JBPattern->ledPattern[segment]&bitMask ? 1:0);
    uint8_t b1=(p_JBPattern->ledPattern[segment+4]&bitMask ? 2:0);
    uint8_t b2=(p_JBPattern->ledPattern[segment+8]&bitMask ? 4:0);
    line[15]='0'+b0+b1+b2;
    line[6] =(ledLine==0 ? '-'   :'0'+b0);
    line[7] =(ledLine==0 ? '0'+b0:(ledLine==1 ? '-'   :'0'+b1));
    line[8] =(ledLine==0 ? '-'   :(ledLine==1 ? '0'+b1:'-'   ));
    line[9] =(ledLine==1 ? '-'   :(ledLine==2 ? '0'+b2:'0'+b1));
    line[10]=(ledLine==2 ? '-'   :'0'+b2);
    line[11]=' ';
    line[12]=' ';
    line[13]=' ';
    line[14]='=';
    exBus.setJetiboxText(1, line);
}

//---------------------------------
//  Class LEDFlash ----------------

JBMenue* LedFlashFactory::getNewMenue(void){
  jbmObA = new JBMBase[JBMLED_TAB_LENGTH-1];
  jbmObA[0].f= new LedFlash::JBPatternFactory;
  jbmObA[1].f= new LedFlash::JBPeriodFactory;    // list of JB Menue objects
  return new JBMenue(jbmObA, JBM_Led_table, JBMLED_TAB_LENGTH);
}

//
LedFlash::LedFlash(void){
  p_JBPattern=this;
}

// call to initialise
void LedFlash::start(void) {
// read EEPROM
  for(uint8_t i=0; i<13; i++)
    eepromData[i]=(uint8_t)AEEPROM.read(LEDPATTRERN +i);

// init valus
  ledOff=false;
  ledBitPos=0;
// init output port
//  PORTD=PORTD&(~(LEDFLASHBIT2|LEDFLASHBIT1|LEDFLASHBIT0));
//  DDRD|=(LEDFLASHBIT2|LEDFLASHBIT1|LEDFLASHBIT0);
}

void  LedFlash::doMain(void){
  // eeprom
  if (eepromJob){
    eepromJob--;
    if (!AEEPROM.update(LEDPATTRERN +eepromJob, eepromData[eepromJob])){
      eepromJob++;
    }
  }
}

// must be called every 4ms, typicly from an Timer interrupt
void LedFlash::interrupt(void){
  if (ledOff||!ledPeriod){                                  // LED flash off
    ledBitPos=0;
    PORTD&=(~(LEDFLASHBIT2|LEDFLASHBIT1|LEDFLASHBIT0));
    return; 
  }
  ledPeriodCnt--;                                           // Period
  if (ledPeriodCnt) return;
  ledPeriodCnt=ledPeriod;

	uint8_t segment=ledBitPos>>3;                             // set Output port
	uint8_t bitMask=(1<<(ledBitPos&0x07));
	uint8_t ledPins=0;
  if (ledPattern[segment]  &bitMask) ledPins=LEDFLASHBIT0;
  if (ledPattern[segment+4]&bitMask) ledPins|=LEDFLASHBIT1;
  if (ledPattern[segment+8]&bitMask) ledPins|=LEDFLASHBIT2;
	
	PORTD=PORTD&(~(LEDFLASHBIT2|LEDFLASHBIT1|LEDFLASHBIT0))|ledPins;
	ledBitPos++;
	if (ledBitPos>31) ledBitPos=0;
}

void LedFlash::setLedOn(bool led){                          // trun on/off
  ledOff=!led;
}
//-----
