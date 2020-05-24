
#include "ao_ADInterrupt.h"
#include "ao_JetiExBusSensor.h"
#include "ao_JetiExBus.h"
#include "ao_LedFlash.h"
#include "ao_BNO055.h"
#include "ao_Eeprom.h"

extern JetiExBus    exBus;                // Jeti EX Bus
extern AO_BNO       bno;
extern LedFlash     ledFlash;
//
extern volatile uint32_t cntMs;           // system  1 ms counter
extern volatile uint8_t  lastSecondCnt;   // trigger counter for 1s in main
//
extern volatile uint8_t  exbusNoDataCnt;  // counts exBus packet lost, based on 100Hz
extern volatile uint8_t  escNoDataCnt;    // counts esc packet lost, based on 100Hz
extern volatile uint8_t  bnoTimerCnt;     // counter to next bno measurement
extern volatile uint16_t waitToAction;

volatile uint8_t         compare10ms;
//
volatile uint16_t  timer3OvfCnt;
volatile uint32_t  ic3Period;
volatile uint32_t  ic3LastTime;
//
volatile uint16_t  tcnt4diff;
volatile uint16_t  tcnt4Last;
volatile uint16_t  compare250ms;
volatile uint8_t   rpmMeasure;
//
volatile uint16_t  ADInt::adSum[AD_MAX_NUM_CHANNEL];
volatile uint8_t   ADInt::adPointer;
volatile uint16_t  ADInt::adc0Max;
volatile uint16_t  ADInt::adc0Min;

JBMenue* ADFactory::getNewMenue(void){
  jbmObA = new JBMBase[1];
  jbmObA[0].i = new ADInterface;
  return new JBMenueSel(jbmObA, JBM_AdMax_table, JBMADJUST_TAB_LENGTH);
}

void  ADInterface::JBMGetParameter(JBMenueSel *jbms, uint8_t num){
  jbms->unit[0]=' ';
  jbms->unit[1]=' ';
  jbms->unit[2]=' ';
  if (num<AD_MAX_NUM_CHANNEL-1){        // voltage
    jbms->mini=300;                     // min 3V
    jbms->maxi=8191;                    // max 81V
    jbms->unit[1]='V';
    jbms->value=p_ADInterrupt->adjustValue[num];
    jbms->showValue=((uint32_t)p_ADInterrupt->ad[num]*(uint32_t)p_ADInterrupt->adjustValue[num])>>10;
    jbms->confMode=0x20|0x00|0x02;      // decimal 2, 2 values, unsigned int
  } else if (num==AD_MAX_NUM_CHANNEL-1){ // current
    jbms->mini=10;                      // min 1.0A
    jbms->maxi=1050;                    // max 105.0A
    jbms->unit[1]='A';
    jbms->value=p_ADInterrupt->adjustValue[num];
    jbms->showValue=p_ADInterrupt->ad[num]-p_ADInterrupt->adjustValue[num+1];  // value-Offset
    jbms->showValue=(uint16_t)((((int32_t)((int16_t)jbms->showValue))*(uint32_t)p_ADInterrupt->adjustValue[num])>>9);
    jbms->confMode=0x10|0x08|0x02;      // decimal 1, signed int, 2 values
  } else if (num==AD_MAX_NUM_CHANNEL){  // AD offset for current
    jbms->mini=412;                     // ideal value would be 512= 2**10Bit ADC resolution/2 
    jbms->maxi=612;
    jbms->value=p_ADInterrupt->adjustValue[num];
    jbms->confMode=0x00;                // decimal 0 pure uint
  } else if (num<AD_MAX_NUM_CHANNEL+3){ // Timer
    jbms->mini=1;                       // min 0.01
    jbms->maxi=10500;                   // max 105
    jbms->unit[1]='x';
    jbms->value=p_ADInterrupt->adjustValue[num];
    jbms->confMode=0x30|0x00|0x01;      // decimal 3, 1 values, unsigned in
    if (num==AD_MAX_NUM_CHANNEL+2) jbms->unit[0]='.';
  } else if (num==AD_MAX_NUM_CHANNEL+3){ // BNO Mapper
    jbms->mini=0;                       // min 0
    jbms->maxi=5;                       // max 5
    jbms->value=p_ADInterrupt->adjustValue[num];
    jbms->confMode=0x00;                // decimal 0 pure uint
    uint8_t a=bno.axisMap[jbms->value];
    jbms->unit[0]='X'+(a&0x03);
    jbms->unit[1]='X'+((a>>2)&0x03);
    jbms->unit[2]='X'+((a>>4)&0x03);
  } else {                              // BNO aches sign
    jbms->mini=0;                       // min 0
    jbms->maxi=7;                       // max 8
    jbms->value=p_ADInterrupt->adjustValue[num];
    jbms->confMode=0x00;                // decimal 0 pure uint
    jbms->unit[0]=(jbms->value&0x04 ? '-':'+');
    jbms->unit[1]=(jbms->value&0x02 ? '-':'+');
    jbms->unit[2]=(jbms->value&0x01 ? '-':'+');
  }
}

void  ADInterface::JBMRetParameter(uint16_t value, uint8_t num){
  p_ADInterrupt->adjustValue[num]=value;
}

void  ADInterface::JBMEnd(void){
  p_ADInterrupt->eepromJob=2*JBMADJUST_TAB_LENGTH;
//
  uint8_t a= (p_ADInterrupt->adjustValue[AD_MAX_NUM_CHANNEL+4]<<4)
             | p_ADInterrupt->adjustValue[AD_MAX_NUM_CHANNEL+3];
  bno.setAxes(a);
}
//---------------------------------------------------------------------

void ADInt::start(void) {
// read EEPROM
    for(uint8_t i=0; i<2*JBMADJUST_TAB_LENGTH; i++){
      eepromData[i]=AEEPROM.read(ADJUSTVALUE+i);
    }
// init values
    p_ADInterrupt=this;
    adDone=0;
// system time init
    cntMs=compare10ms=0;
// config ADC
    DIDR0|=ADC_CHANNEL_NO_DIGIT;                        // ADC no digital input
    DIDR1=0;                                            // AIN0 & AIN1 not used
    ADCSRA = (1<<ADEN)|(1<<ADIF)|ADC_PRESCALER;         // ADC enable
    ADCSRB = 0;                                         // one shot mode
    ADMUX = (1<<REFS0);                                 // Uref=AVCC, links kanal 0
// init Interrupts
    cli();
    ADCSRA|=(1<<ADIE);                      // enable ADC Interrupt
//-- Timer 3 init
//  OC3A: 1ms system counter
//  IC3 : falling edge
//  OC3B: int 1ms from ic3                  // avoid ic3 flooding
    TCCR3A=0x00;
    TCCR3B=(1<<ICNC3)|(1<<CS30);            // IC3: Noise, falling edge; NO prescaler
    OCR3A=TCNT3+(uint16_t)CLOCKS_FOR_1MS;
    OCR3B=TCNT3+(uint16_t)51000;            // delay start measure
    TIFR3=(1<<ICF3)|(1<<OCF3B)|(1<<OCF3A)|(1<<TOV3); // clear old flags
    TIMSK3=(1<<OCIE3B)|(1<<OCIE3A)|(1<<TOIE3); // enable OCA3,OCB3,TOI3 Interrupt
// counter 4
    TCCR4A=0x00;
    TCCR4B=(1<<ICNC4)|(1<<CS42)|(1<<CS41);   // IC4: falling edge; T4:external clock falling edge
//
    sei();
//
    capacity=0;
    eepromJob=0;
}

// Sensor main Timer 1 ms
ISR(TIMER3_COMPA_vect){
  OCR3A+=(uint16_t)CLOCKS_FOR_1MS;          // period [micro_s]
  cntMs++;                                  // increment 1ms counter
  if (AO_BNO::waitToAction) AO_BNO::waitToAction--;
//
  if (int16_t((uint16_t)cntMs-compare250ms)>=250){ // count 250ms 
    compare250ms+=250;
    if (rpmMeasure&0x02){
      uint16_t tcnt=TCNT4;
      tcnt4diff=tcnt-tcnt4Last;             // frequency per 500ms
      tcnt4Last=tcnt;
    }
    rpmMeasure++;
  }

//0 AD interrupt
//1 
//2 LED Flash
//3 
  uint8_t cycle=(uint8_t)cntMs&0x03;
  if (!(cycle&0x03)){                       // every 4ms do job number 0: start ADC
    p_ADInterrupt->adTimerJob();
  } else {
    sei();
    switch (cycle) {
//    case 1: {                             // nothing to do here!
//    break;}
    case 2: {
        ledFlash.interrupt();               // handle LED Flash output
      break;}
    case 3: {
      if (bnoTimerCnt<0xFF) bnoTimerCnt++;  // cnt the time to next bno measure
      break;}
    }
  }
  sei();
//
  if (int8_t(cntMs-compare10ms)>=10){             // count 10ms 
    compare10ms+=10;
    if (lastSecondCnt<0xff)  lastSecondCnt++;     // count to 100 => second flag in main
//
    if (cntMs>15000) {                            // no error counting in the first 15s
      if (exbusNoDataCnt<0xff) exbusNoDataCnt++;  // signal lost: exbus
      if (escNoDataCnt<0xff)   escNoDataCnt++;    // signal lost: esc
    }
  }
}

ISR(TIMER3_CAPT_vect){
  uint16_t icTime=ICR3;
  if ((!(icTime&0x8000)) && (TIFR3&(1<<TOV3))){ // check for pending timer overflow
    TIFR3=(1<<TOV3);
    timer3OvfCnt++;
  }
  OCR3B=icTime+(uint16_t)(CLOCKS_FOR_3MS);
  TIFR3=(1<<ICF3)|(1<<OCF3B);                   // clear old flags
  TIMSK3=(1<<OCIE3B) |(TIMSK3 & (~(1<<ICIE3))); // turn ic3 off, oc3b on
  uint32_t t=((uint32_t)timer3OvfCnt<<16)|icTime;
  ic3Period=t-ic3LastTime;
  ic3LastTime=t;
}

ISR(TIMER3_COMPB_vect){
  if (TIFR3&(1<<ICF3)){                           // ic flag set=> f to high
    uint16_t icTime=ICR3;
    TIFR3=(1<<ICF3)|(1<<OCF3B);                   // clear old flags
    if ((!(icTime&0x8000)) && (TIFR3&(1<<TOV3))){ // check for pending timer overflow
      TIFR3=(1<<TOV3);
      timer3OvfCnt++;
    }
    uint32_t t=((uint32_t)timer3OvfCnt<<16)|icTime;
    if ((int16_t)(OCR3B-icTime) < 0){             // happend ic3 after oc3b?
      ic3Period=t-ic3LastTime;
    } else {
      ic3Period=48000;                            // min period time
    }
    OCR3B=icTime+(uint16_t)(CLOCKS_FOR_3MS);
    ic3LastTime=t;
  } else {
    TIMSK3=(1<<ICIE3) |(TIMSK3 & (~(1<<OCIE3B))); // turn ic3 on, oc3b off
  }
}

ISR(TIMER3_OVF_vect){
  timer3OvfCnt++;                                 // extend timer 3 cnt

  if (timer3OvfCnt-(ic3LastTime>>16) > 5000){     // detect no signal
    ic3LastTime=(uint32_t)timer3OvfCnt<<16;
    ic3Period=0xFFFFFFFF;
  }
}

// start AD conversion
void ADInt::adTimerJob(void){
  ADMUX=(ADMUX&0xf0)|AD_START_CHANNEL;            // select the first AD Channel
  ADInt::adPointer=0;
  if ((cntMs&(31*4))==0){                         // 32 mesurement every 4ms
    for (uint8_t p=0; p< AD_MAX_NUM_CHANNEL; p++){
      ad[p]=ADInt::adSum[p]/32;
      ADInt::adSum[p]=0;
    }
    adc0MaxOut=ADInt::adc0Max;
    adc0MinOut=ADInt::adc0Min;
    ADInt::adc0Min=0x03FF;
    ADInt::adc0Max=0;
    adDone=true;
  }
  ADCSRA|=(1<<ADSC);                              // Start new conversation period
}


// read AD register and start next conversion
ISR(ADC_vect){
  ADMUX=(ADMUX&0xf0)|((ADMUX+1)&0x0f);            // do it first to stabilize the input, switch to next channel
  uint16_t ad=ADC;
  if (!ADInt::adPointer){                         // channel 0?
    if (ad<ADInt::adc0Min) ADInt::adc0Min=ad;  
    if (ad>ADInt::adc0Max) ADInt::adc0Max=ad;
  }
  ADInt::adSum[ADInt::adPointer]+= ad;
//
  ADInt::adPointer++;
  if (ADInt::adPointer<AD_MAX_NUM_CHANNEL) {      // start next conversion?
    ADCSRA|=(1<<ADSC); 
  }
}


// sets the measurement to the exbus values
void ADInt::adMain(void){
// RPM -------------
  cli();
  if (rpmMeasure&0x01){
    rpmMeasure++;
    uint32_t value=ic3Period;
    sei();
    value=((uint32_t)(1920000000/value)*(uint32_t)adjustValue[AD_MAX_NUM_CHANNEL+1])/2000;
    exBus.setSensorValue(SEN_RPM_LOW , value);
    cli();
    value=tcnt4diff<<1;
    sei();
    value=(value*(uint32_t)adjustValue[AD_MAX_NUM_CHANNEL+2]*6)/1000;
    exBus.setSensorValue(SEN_RPM_HIGH, value);
  }
  sei();
  if (rpmMeasure>3) rpmMeasure=0;

  
// ADC -------------
  cli();
  bool retValue=adDone;
  adDone=false;                           // reset status
  sei();
  if (retValue){                          // valid data: convert digital data to voltage
    for (uint8_t p=0; p < AD_MAX_NUM_CHANNEL-1; p++){
      exBus.setSensorValue(SEN_ADC_U0+p, ((((uint32_t)ad[p]*(uint32_t)adjustValue[p])>>10)+5)/10);
    }
    int32_t current=int16_t(ad[AD_MAX_NUM_CHANNEL-1]-adjustValue[AD_MAX_NUM_CHANNEL]);  // ADC - offset
    capacity+=current;
// set current
    exBus.setSensorValue(SEN_ADC_I,((int32_t)(current*(uint32_t)adjustValue[AD_MAX_NUM_CHANNEL-1])/512)); // >>9 because 512 is max
// an hour has 3600s and there is every 128ms a sample => 28125 samples/h
    int16_t out=(((20*capacity/28125)*(int32_t)(adjustValue[AD_MAX_NUM_CHANNEL-1]))>>10);
    exBus.setSensorValue(SEN_ADC_C,out);
// adc0 min / max
    exBus.setSensorValue(SEN_AD_MI, ((((uint32_t)adc0MinOut*(uint32_t)adjustValue[0])>>10)+5)/10);
    exBus.setSensorValue(SEN_AD_MX, ((((uint32_t)adc0MaxOut*(uint32_t)adjustValue[0])>>10)+5)/10);
  }
// eeprom
  if (eepromJob){
    eepromJob--;
    if (!AEEPROM.update(ADJUSTVALUE+eepromJob, eepromData[eepromJob])){
      eepromJob++;
    }
  }
}
