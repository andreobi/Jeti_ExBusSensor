
#ifndef AO_ADInterrupt_H
#define AO_ADInterrupt_H

#include <Arduino.h>
#include "ao_JBMenue.h"

const char JBMAdMax0[] PROGMEM = "U-ADC0";
const char JBMAdMax1[] PROGMEM = "U-ADC1";
const char JBMAdMax2[] PROGMEM = "U-ADC2";
const char JBMAdMax3[] PROGMEM = "I-Batt";
const char JBMAdMax4[] PROGMEM = "I-Offset";
const char JBMAdMax5[] PROGMEM = "RPM lo";
const char JBMAdMax6[] PROGMEM = "RPM hi";
const char JBMAdMax7[] PROGMEM = "BNO axes";
const char JBMAdMax8[] PROGMEM = "BNO sign XYZ";
const char *const JBM_AdMax_table[] PROGMEM = 
  {JBMAdMax0,JBMAdMax1,JBMAdMax2,JBMAdMax3,JBMAdMax4,JBMAdMax5,JBMAdMax6,JBMAdMax7,JBMAdMax8};
#define JBMADJUST_TAB_LENGTH (sizeof(JBM_AdMax_table)/sizeof(JBM_AdMax_table[0]))

//-----------------------
#if  F_CPU==16000000L
#define CLOCKS_FOR_1MS  16000
#define CLOCKS_FOR_3MS  48000
#define ADC_PRESCALER   (1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0)
//-----------------------
#elif F_CPU==8000000L
#define CLOCKS_FOR_1MS   8000
#define CLOCKS_FOR_3MS  24000
#define ADC_PRESCALER   (1<<ADPS2)|(1<<ADPS1)
//-----------------------
#else 
 #error ("ONLY_defined_for_8_or_16MHz")
#endif
// hardware
//!!! don't forget to change DIDR0 !!!!
#define ADC_CHANNEL_NO_DIGIT (1<<ADC3D)|(1<<ADC2D)|(1<<ADC1D)|(1<<ADC0D)
#define AD_START_CHANNEL    0
#define AD_MAX_NUM_CHANNEL  4

//-----------------------
class ADFactory : public JBMFactory {
public:
    virtual JBMenue *getNewMenue(void);
//    virtual void  deleteNewMenue(void);
};

class ADInterface : public JBMInterface {
public:
  virtual void    JBMGetParameter(JBMenueSel *jbms,uint8_t num);
  virtual void    JBMRetParameter(uint16_t value, uint8_t num); // could be used to retrieve new value
  virtual void    JBMEnd(void);
};

class ADInt {
  friend ADInterface;
public:
  void  start(void);
  void  adMain(void);
  void  adTimerJob(void);

  volatile bool      adDone;                     // true: new Values available
  volatile uint16_t  ad[AD_MAX_NUM_CHANNEL];     // final ad result
//
static  volatile uint16_t  adc0Max;
static  volatile uint16_t  adc0Min;
  volatile uint16_t  adc0MaxOut;
  volatile uint16_t  adc0MinOut;

static  volatile uint16_t  adSum[AD_MAX_NUM_CHANNEL];
static  volatile uint8_t   adPointer;

  union{
    uint16_t         adjustValue[JBMADJUST_TAB_LENGTH];
    uint8_t          eepromData[2*JBMADJUST_TAB_LENGTH];
  };
  uint8_t            eepromJob;
private:
  int32_t            capacity;
};

static ADInt    *p_ADInterrupt;

#endif // 
