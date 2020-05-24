
#ifndef AO_IN_OUTPUT_H
#define AO_IN_OUTPUT_H

#include <Arduino.h>
#include "ao_JBMenue.h"

extern JetiExBus    exBus;          // Jeti EX Bus
// menue text
const char JBMI_OH[] PROGMEM = "IO Control";
const char JBMI_O0[] PROGMEM = "Ch Select";
const char JBMI_O1[] PROGMEM = "Ch Level";
const char *const JBM_I_O_table[] PROGMEM = {JBMI_OH,JBMI_O0,JBMI_O1};
#define JBMI_O_TAB_LENGTH (sizeof(JBM_I_O_table)/sizeof(JBM_I_O_table[0]))
//
const char JBMIOcs0[] PROGMEM = "Ch Buzzer";
const char JBMIOcs1[] PROGMEM = "Ch Flash";
const char JBMIOcs2[] PROGMEM = "Ch Pin0";
const char JBMIOcs3[] PROGMEM = "Ch Pin1";
const char *const JBM_IOcs_table[] PROGMEM = {JBMIOcs0,JBMIOcs1,JBMIOcs2,JBMIOcs3};
#define JBMIOCS_TAB_LENGTH (sizeof(JBM_IOcs_table)/sizeof(JBM_IOcs_table[0]))
//
const char JBMIOch0[] PROGMEM = "min Buzzer";
const char JBMIOch1[] PROGMEM = "MAX Buzzer";
const char JBMIOch2[] PROGMEM = "min Flash";
const char JBMIOch3[] PROGMEM = "MAX Flash";
const char JBMIOch4[] PROGMEM = "min Pin0";
const char JBMIOch5[] PROGMEM = "MAX Pin0";
const char JBMIOch6[] PROGMEM = "min Pin1";
const char JBMIOch7[] PROGMEM = "MAX Pin1";
const char *const JBM_IOch_table[] PROGMEM = {JBMIOch0,JBMIOch1,JBMIOch2,JBMIOch3,JBMIOch4,JBMIOch5,JBMIOch6,JBMIOch7};
#define JBMIOCH_TAB_LENGTH (sizeof(JBM_IOch_table)/sizeof(JBM_IOch_table[0]))
//

class InOutputFactory : public JBMFactory {
public:
  virtual JBMenue *getNewMenue(void);
//  virtual void  deleteNewMenue(void);
};

class InOutput {
public:
// do the channel asssignment menue
  class JBIOchselectFactory : public JBMFactory {
    public:
    virtual JBMenue *getNewMenue(void);
//    virtual void  deleteNewMenue(void);
  };

  class JBIOchselectInterface : public JBMInterface {
    public:
    virtual void    JBMGetParameter(JBMenueSel *jbms,uint8_t num);
    virtual void    JBMRetParameter(uint16_t value, uint8_t num);
    virtual void    JBMEnd(void);
  };

  class JBIOchselect {
    friend JBIOchselectInterface;
    public:
  };

//----------------------------
// do the Min and Max to switch the channel
  class JBIOswitchFactory : public JBMFactory {
    public:
    virtual JBMenue *getNewMenue(void);
//    virtual void  deleteNewMenue(void);
  };

  class JBIOswitchInterface : public JBMInterface {
    public:
    virtual void    JBMGetParameter(JBMenueSel *jbms,uint8_t num);
    virtual void    JBMRetParameter(uint16_t value, uint8_t num);
    virtual void    JBMEnd(void);
  };

  class JBIOswitch {
    friend JBIOswitchInterface;
    public:

  };
//----------------------------
public:
        InOutput(void);
  void  start(void);                          // call only once to initialise
  void  doMain(void);                         // called from main loop 
  bool  getIOStatus(uint8_t functionNum);     // true: tx channel level in range 
  void  setPin0(bool o);                      // set Status o to PIN0
  void  setPin1(bool o);                      // set Status o to PIN1
protected:
  uint8_t            eepromJob;
private:
  union{
    struct{
    uint8_t   ioChSel[JBMIOCS_TAB_LENGTH];      // 0: OFF , 1-24 are valid channel num
    uint8_t   ioChVal[JBMIOCH_TAB_LENGTH];      //
    };
    uint8_t   eepromData[JBMIOCS_TAB_LENGTH+JBMIOCH_TAB_LENGTH];
  };
};

static InOutput *p_InOutput;

#endif
