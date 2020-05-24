
#ifndef LEDFLASH_H
#define LEDFLASH_H

#include <Arduino.h>
#include "ao_JBMenue.h"

// LED flash Pins on PortD
#define LEDFLASHBIT0	0x20          /* A */
#define LEDFLASHBIT1	0x40          /* B */
#define LEDFLASHBIT2	0x80          /* C */


extern JetiExBus    exBus;          // Jeti EX Bus
// menue text
const char JBMLedH[] PROGMEM = "Flash";
const char JBMLed0[] PROGMEM = "Pattern";
const char JBMLed1[] PROGMEM = "Period";
const char *const JBM_Led_table[] PROGMEM = {JBMLedH,JBMLed0,JBMLed1};
#define JBMLED_TAB_LENGTH (sizeof(JBM_Led_table)/sizeof(JBM_Led_table[0]))
//
const char *const JBM_LedPer_table[] PROGMEM = {JBMLed1};
#define JBMLEDPER_TAB_LENGTH (sizeof(JBM_LedPer_table)/sizeof(JBM_LedPer_table[0]))
//

class LedFlashFactory : public JBMFactory {
public:
  virtual JBMenue *getNewMenue(void);
//  virtual void  deleteNewMenue(void);
};

class LedFlash {
public:
// do the Flash period menue
  class JBPeriodFactory : public JBMFactory {
    public:
    virtual JBMenue *getNewMenue(void);
//    virtual void  deleteNewMenue(void);
  };

  class JBPeriodInterface : public JBMInterface {
    public:
    virtual void    JBMGetParameter(JBMenueSel *jbms,uint8_t num);
    virtual void    JBMRetParameter(uint16_t value, uint8_t num);
    virtual void    JBMEnd(void);
  };

  class JBPeriod {
    friend JBPeriodInterface;
    public:
  };
  
//----------------------------
// do the Pattern definition
  class JBPatternFactory : public JBMFactory {
    public:
    virtual JBMenue *getNewMenue(void);
//    virtual void  deleteNewMenue(void);
  };

  class JBPatternInterface : public JBMInterface {
    public:
//    virtual void    JBMGetParameter(JBMenueSel *jbms,uint8_t num);
//    virtual void    JBMRetParameter(uint16_t value, uint8_t num);
    virtual void    JBMEnd(void);
  };

  class JBPattern : public JBMenue {     // handels the Pattern definition 
	friend JBPatternInterface;
    public:
	  JBPattern(JBMBase *jbmOb);
      virtual bool    JBMDo(uint8_t key);
      virtual void    JBMShow(void);
    private:
      JBPattern(void);
      uint8_t   line1Active;
      uint8_t   ledBitPosJB;                  // bit position in menue
      uint8_t   ledLine;                      // Output A,B,C
  };
//----------------------------
public:
        LedFlash(void);
  void  start(void);                          // call only once to initialise
  void  doMain(void);                         // called from main loop 
  void  interrupt(void);                      // should be called all 4ms from timer
  void  setLedOn(bool led);                   // turn on/off the LED function
private:
  union{
    struct{
      uint8_t  ledPattern[12];
      uint8_t  ledPeriod;  
    };
    uint8_t   eepromData[12+1];
  };
  uint8_t  eepromJob;
	uint8_t  ledPeriodCnt;
	bool     ledOff;                           // if true: LED flash is off
	uint8_t  ledBitPos;                        // output A,B,C
};

static LedFlash *p_JBPattern;

#endif
