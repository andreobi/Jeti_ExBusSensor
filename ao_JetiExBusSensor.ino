/*
 * Pin usage
 * PD0    RX0
 * PD1    TX0
 * PD2    OC3B (PWM / Servo out) ??? free
 * PD3    OC2B      BuZZER
 * PD4    I/O       PIN Output PIN 0
 * PD5    I/O       LED Flash A
 * PD6    I/O       LED Flash B
 * PD7    I/O       LED Flash C
 * 
 * PB0    IC1       RX (SW-USART)
 * PB1    I/O       PIN Output PIN 1
 * PB2    OC1B      TX (SW-USART)
 * PB3    USART1    TX1
 * PB4    USART1    RX1
 * PB5    I/O       System Status LED
 * 
 * PC0    ADC0      Voltage 0 
 * PC1    ADC1      Voltage 1
 * PC2    ADC2      Voltage 2
 * PC3    ADC3      Current I
 * PC4    I2C0      SDA
 * PC5    I2C0      SCL
 * 
 * PE0    IC4       PIN input => "PIN low" Message
 * PE1    T4        RPM high: falling edge, 4Hz resolution
 * PE2    IC3       RPM low : falling edge, 3ms minimum period
 * PE3    I/O     PIN input free ???
 * 
 * used internal resources
 * Timer  0 no pins       millis()      free
 *        1 IC1 OCA OCB   SW USART
 *        2 OC2B          Buzzer
 *        3 TCNT3         1ms Job Timer
 *        4 TCNT4         RPM Counter high
 * 
 *        I2C 0           Pressure sensor BME280
 *        
 *        USART 0         EX_Bus Receiver
 *        USART 1         Kontronik / (EXBus input) future release
 *        SW-USART        GPS / (EX Telemetry input) future release
 *        
 */
//

#include <avr/wdt.h> 
#include "ao_WireLight.h"
#include "ao_ADInterrupt.h"
#include "ao_JetiExBusSensor.h"
#include "ao_JetiExBus.h"
#include "ao_JBMenue.h"
#include "ao_SystemConf.h"
#include "ao_SystemInfo.h"
#include "ao_In_Output.h"
#include "ao_BuzzerOC2.h"
#include "ao_PessureBME.h"
#include "ao_BNO055.h"
#include "ao_SW_USART.h"
#include "ao_GpsParser.h"
#include "ao_LedFlash.h"
#include "ao_Eeprom.h"

#ifdef  WITH_KONTRONIK
#include "ao_Control_K.h"
#endif

//-----------------------
#if  F_CPU==16000000L
#define JETI_SERIAL_SPEED 0007
//-----------------------
#elif F_CPU==8000000L
#define JETI_SERIAL_SPEED 0003
//-----------------------
#else 
  ONLY_defined_for_8_or_16MHz
#endif

//-----------------------
// first Memory allocation because it blocks the stack
JBMBase jbmObA[JBMMAIN_TAB_LENGTH-1]; //= new JBMBase[5]; but fix  // jetibox first level factory pointer array
JBMBase *jbmObAMain=jbmObA;
// system objects
ADInt         adi;            // A1.ADC and Timer 
JetiExBus     exBus;          // A2.Jeti EX Bus
JBMenue       *jbm;           // jetibox menue
SystemConf    sysConf;        // A3.Systemconfiguration
GPS_Parser    ggp;            // first sw serial then gps
WireLight     wireLight;      // B1.I2C Bus  
LedFlash      ledFlash;       // LED Flash control
InOutput      ioPut;          // io channel
SystemInfo    sysInfo;        // jetibox info menue
                              // static: B2. Pressure
AO_BNO        bno;            // B3. BNO

#ifdef  WITH_KONTRONIK
Control_K     esc;            // Kontronik esc
#endif


// system time handled in an ISR 3 routine
volatile uint32_t  cntMs;           // 1ms counter
// handled in main
volatile uint8_t lastSecondCnt;     // cleared every 1s
// Function data
uint8_t          exbusNoDataMax;    // stores the maximum timeout
uint8_t          exbusNoDataFrq;    // how often over 40ms a signal was lost
volatile uint8_t exbusNoDataCnt;    // counts exBus packet lost, based on 100Hz
volatile uint8_t escNoDataCnt;      // counts esc packet lost, based on 100Hz
// status LED
volatile uint8_t errorNumber;       // might be set in an interrupt
uint8_t          errorLEDstate;
// gps
uint8_t          gpsStatus;         // RMC [7,6] GGA[5,4] Char[4..0] counter
//NO GGA:0b11xx.xxxx; NO RMC:0bxx11.xxxx; NO serial data:0bxxxx.0000 ; NO valid serial Data:0b0000.1111
// BNO
volatile uint8_t bnoTimerCnt;       // get incremented every 4ms
//
EE_data mp;
uint8_t eepromJobM;                 // eeprom in Main
//
uint8_t *calData;
uint8_t eepromJobBNO;               // eeprom used for BNO
//
uint8_t systemFlags;                // stores Flags for system status
//
uint8_t jbMessage;                  // upper nibble messageNum, lower nibble messageClass, 0xFF: no Message
uint8_t jbMessageLast;              // upper nibble messageNum, lower nibble secondCnt, 0xF0: no Message
uint8_t pinLowMessage;              // 
//
extern uint8_t bmeStatus;
//
//----------------------------------------------------------------------
void setup() {
// works this way only with ISP (without bootloade) programming
  uint8_t mcusr_copy=MCUSR;   // copy reset reason
  MCUSR = 0;
//
// set default values to the output this might be changed later on
// ??? use internal pullup, instead of external R on the 5V side for level shifter 
  PORTB=0x1D;           // IC1, OC1B, RX,TX high
//  PORTB=0x05;           // IC1, OC1B high (RX,TX no Pullup because of level shift
  DDRB=0x26;            // IC1, TX, RX input

  PORTC=0x30;           // SCL0, SDA0 high
  DDRC=0x00;            // all input
  PORTD=0x03;           // RX, TX high
  DDRD=0xFC;            // RX, TX input
  PORTE=0x0F;           // T4, IC4, T3, IC3 high
  DDRE=0x00;            // all input
//
// Status LED: output and off
  errorLEDstate=0;
  systemFlags=0;
  eepromJobM=0;
// enable watchdog : 1s
  wdt_enable(WDTO_1S);
//

// ??? test
#ifndef WITH_KONTRONIK
	Serial.begin(115200);
  Serial.println("R E S E T");
  Serial.println(mcusr_copy,HEX);
#endif
  
  initADC();                    // must be done early: SYSTEM TIME!!!
  initIO();
  sysConf.start();
  initFlash();
  initWire();
  initPressure();
  initSoftSerialIn();         // GPS
  initFastSerialIn();         // esc
  initJeti();
  initSystemMessage();
  BuzzerOC2::start();
  initEEprom();
  initOrientation();          // do it at last, sensor needs time to start
//
/* MCUSR reset reason
 Bit 3 – WDRF  Watchdog System
 Bit 2 – BORF  Brown-out
 Bit 1 – EXTRF External
 Bit 0 – PORF  Power-on
*/
  if (mcusr_copy&(1<<BORF)){                           // BrownOut?
    eepromIncBitCnt(mp.resetCntBO);
    setFlag(RESET_WATCHDOG,true);
//Serial.println("watchdog");
  } else if (mcusr_copy&(1<<WDRF)){                    // watchdog ?
    eepromIncBitCnt(mp.resetCntWD);
    setFlag(RESET_BROWNOUT,true);
//Serial.println("brown");
  }
  wdt_reset();                // reset watchdog counter
  wdt_enable(WDTO_15MS);    // is a problem when writing eeprom
}

void loop() {
  if (lastSecondCnt>=100){    // do once in a second
    cli();
    lastSecondCnt-=100;
    sei();
    setFlag(MAIN_1S,true);    // set MAIN_1S flag for one main period
  } else {
    setFlag(MAIN_1S,false);
  }
//
  mainIO();               // 
  mainFlash();            // 
  mainPressure();         // BME
  mainOrientation();      // BNO
  mainSoftSerialIn();     // GPS
  mainFastSerialIn();     // ESC
  mainADC();              // ADC and RPM counter
  mainJetiData();         // set IO control 
  mainJetiBox();
//
  if (getFlag(MAIN_1S)){        // any thing what has to be done once a second
    setFlag(BLINK_1S, !getFlag(BLINK_1S));       // alternate status every second
    setFlag(SWUSART_ERROR, SwUsart::getError()); // provide status for a second
//  Timer for pinLowMessage try to send
    if (pinLowMessage&&PORTE&0x01) pinLowMessage--; // only decrement when pin PE0 is high
// clear error counter
    if(sysConf.getHardwareActive(SYS_CONF_HW_CL_ERR)){
      eepromResetBitCnt();
      exbusNoDataCnt=0;
      exbusNoDataMax=0;
      exbusNoDataFrq=0;
      escNoDataCnt=0;
      setFlag(SWUSART_ERROR,false);
      setFlag(RESET_WATCHDOG,false);
      setFlag(RESET_BROWNOUT,false);
      sysConf.setHardwareActive(SYS_CONF_HW_CL_ERR,false);
    }
  }
//
// put your code here but less then 2ms per loop
// 
  mainDoErrorMessage_LED();     // checks status ans sets status led and message
  mainSystemMessage();          // send message to TX
  mainDoStatusLED();            // PIN 13 - Status Flag
  sysConf.doMain();
  mainEEprom();

  mainWatchDog();
}

//----------------------------------------------------------------------
void initEEprom(void){
// read EEPROM
    for(uint8_t i=0; i<10; i++){
      mp.eepromData[i]=AEEPROM.read(EEPROM_MAIN+i);
    } 
}

//----------------------------------------------------------------------
void initWire(void){
  wireLight.begin();
  wireLight.setClock(125000);                    // maximum possible speed=400000
}
//----------------------------------------------------------------------
void initIO(void){
    ioPut.start();
}
//----------------------------------------------------------------------
void initFlash(void){
    ledFlash.start();
}
//----------------------------------------------------------------------
void initPressure(void){
  if (sysConf.getHardwareActive(SYS_CONF_HW_PRESSURE)){
    setFlag(BME_INIT,startPessureBME());
  }
}
//----------------------------------------------------------------------
void initOrientation(void){
  if (sysConf.getHardwareActive(SYS_CONF_HW_ORIENTATION)){
    uint8_t a= (adi.adjustValue[AD_MAX_NUM_CHANNEL+4]<<4)
              | adi.adjustValue[AD_MAX_NUM_CHANNEL+3];
    bno.setAxes(a);
    bno.start();
  }
}

//----------------------------------------------------------------------
void initSoftSerialIn(void){
    SwUsart::start();
    gpsStatus =0;             // GPS diagnose conter for GGA,RMC, and usart char received
}
//----------------------------------------------------------------------
void initFastSerialIn(void){
  if (sysConf.getHardwareActive(SYS_CONF_HW_ESC)){
#ifdef  WITH_KONTRONIK
  esc.start();
#endif
  escNoDataCnt=0;
  }
}
//----------------------------------------------------------------------
void initADC(void){
    adi.start();
}
//----------------------------------------------------------------------
void initJeti(void){
    exBus.setDeviceId(0x76, 0x32);      // 0x3276 Sensor ID
    exBus.start("AO", sensors);  // Sensor name
// base menue init
    jbmObA[0].f= new SystemInfoFactory;     // list of JBClassInterfaces must fit the tabel order JBM_Main_table
    jbmObA[1].f= new InOutputFactory;
    jbmObA[2].f= new LedFlashFactory;
    jbmObA[3].f= new ADFactory;
    jbmObA[4].f= new SystemConfFactory;
    jbm=new JBMenue(jbmObA, JBM_Main_table, JBMMAIN_TAB_LENGTH);            // jetibox menue
    jbm->JBMShow();
//
    for (uint8_t i=0; i<(JETI_MAX_SENSORS/8)+1; i++) {
      uint8_t jsa=AEEPROM.read(JETI_SENSOR_ACTIVE+i);
      for (uint8_t j=0; j<8; j++) {
        exBus.setSensorEnable(i*8+j+1, jsa&(1<<j));
      }
    }

    exbusNoDataCnt=0;     // counts exBus packet lost, based on 100Hz
    exbusNoDataMax=0;     // maximum lost time
    exbusNoDataFrq=0;
}
//----------------------------------------------------------------------
void initSystemMessage(void){
    jbMessage=pinLowMessage=0;        // no message to send
    jbMessageLast=0xF0;               // done
}
//----------------------------------------------------------------------
void mainEEprom(void){
  // eeprom
  if (eepromJobM){
    eepromJobM--;
    if (!AEEPROM.update(EEPROM_MAIN +eepromJobM, mp.eepromData[eepromJobM])){
      eepromJobM++;
    }
  }
}
//----------------------------------------------------------------------
void mainIO(void){
    ioPut.doMain();   // is handeld as set... in mainJetiData();
}
//----------------------------------------------------------------------
void mainFlash(void){
   ledFlash.doMain();                     // 
}
//----------------------------------------------------------------------
void mainPressure(void){
  if (sysConf.getHardwareActive(SYS_CONF_HW_PRESSURE)){
    if (sysConf.getHardwareChanged(SYS_CONF_HW_PRESSURE)){
      setFlag(BME_INIT,startPessureBME());
    } else {
      if (getFlag(BME_INIT)){       // sensor not initialiesed: no measurement!
        mainPessureBME();           // sets altitude, vario to Jeti Telemetry
      }
    }
  }
}
//----------------------------------------------------------------------
void mainOrientation(void){
  bno.doMain();
}
//----------------------------------------------------------------------
void mainSoftSerialIn(void){
// GPS diagnose
  if (getFlag(MAIN_1S)){                  // diagnose inc Cnt per second clear if received
    uint8_t tempCnt=gpsStatus&0x30;       // RMC cnt
    if (tempCnt<0x30){
      tempCnt+=0x10;
      gpsStatus=(gpsStatus&0xCF)|tempCnt;
    }
    tempCnt=gpsStatus&0xC0;
    if (tempCnt<0xC0){                    // GGA cnt
      tempCnt+=0x40;
      gpsStatus=(gpsStatus&0x3F)|tempCnt;
    }
    tempCnt=gpsStatus&0x0F;
    if (tempCnt>0x02){                    // rx cnt
      tempCnt=0x02;                       // so it takes 2 seconds to clear
    } else {
      if (tempCnt) tempCnt--;
    }
    gpsStatus=(gpsStatus&0xF0)|tempCnt;
  }

  if (sysConf.getHardwareActive(SYS_CONF_HW_GPS)){  // GPS ON?
// read char from serial and parse it
    if (SwUsart::available()>0){           // charater to read?
      ggp.parseStream(SwUsart::read());    // put character to stream
      if (gpsStatus&0x0F<0x0F){
        gpsStatus++;                      // diagnose: got char then inc counter
      }
    }
// do RMC data
    if (ggp.available()&GPS_SENTENCE_RMC){
      exBus.setSensorValue(SEN_GPS_LATT, ggp.gpsData.latitude.d); 
      exBus.setSensorValue(SEN_GPS_LONG, ggp.gpsData.longitude.d);
      exBus.setSensorValue(SEN_GPS_COURS, ggp.gpsData.course);
      exBus.setSensorValue(SEN_GPS_SPEED, ggp.gpsData.speed);
// 
      exBus.setSensorValueTime(SEN_GPS_TIME, ggp.gpsData.time.hour, ggp.gpsData.time.minute, ggp.gpsData.time.second);
//      exBus.SetSensorValueDate(31, ggp.gpsData.date.year, ggp.gpsData.date.month, ggp.gpsData.date.day);
      ggp.done(GPS_SENTENCE_RMC);           // set status RMC data handeld
      gpsStatus=(gpsStatus&0xC0)|0x03;      //diagnose:got sentence clear RMC Cnt & char Cnt=3
    }
// do GGA data
    if (ggp.available()&GPS_SENTENCE_GGA){
      exBus.setSensorValue(SEN_GPS_ALTI, ggp.gpsData.altitude);
      exBus.setSensorValue(SEN_GPS_SAT, ggp.gpsData.fix);
//      exBus.setSensorValue(SEN_GPS_SAT, ggp.gpsData.satellites);
      ggp.done(GPS_SENTENCE_GGA);           // set status GGA data handeld
      gpsStatus&=(gpsStatus&0x30)|0x03;     //diagnose:got sentence clear GGA Cnt & char Cnt=3
    }
  } else {
    gpsStatus=0;                            // device off: no Error
  }
}
//----------------------------------------------------------------------
void mainFastSerialIn(void){
// converts Kontronik input to Jeti Telemetry
#ifdef  WITH_KONTRONIK
  if (sysConf.getHardwareActive(SYS_CONF_HW_ESC)){
    if (esc.hasNewLiveData()){
      escNoDataCnt&=(~0x07);                         // got data => lost counter=0
      exBus.setSensorValue(SEN_ECU_RPM ,esc.getLiveValue32(LD_CONTROL_RPM));
      exBus.setSensorValue(SEN_ECU_UBAT,esc.getLiveValue16(LD_CONTROL_UBATT));
      exBus.setSensorValue(SEN_ECU_IBAT,esc.getLiveValue16(LD_CONTROL_IBATT));
      exBus.setSensorValue(SEN_ECU_CAPA,esc.getLiveValue16(LD_CONTROL_CAPCITY));
      exBus.setSensorValue(SEN_ECU_UBEC,esc.getLiveValue16(LD_CONTROL_UBEC));
      exBus.setSensorValue(SEN_ECU_IBEC,esc.getLiveValue16(LD_CONTROL_IBEC));
      exBus.setSensorValue(SEN_ECU_TMOT,esc.getLiveValue8(LD_CONTROL_T_MOT));
      exBus.setSensorValue(SEN_ECU_TBEC,esc.getLiveValue8(LD_CONTROL_T_BEC));
      exBus.setSensorValue(SEN_ECU_STATE,esc.getLiveValue8(LD_CONTROL_STATUS));
// part 2
      exBus.setSensorValue(SEN_ECU_PIN ,esc.getLiveValue16(LD_CONTROL_PWM_IN));
      exBus.setSensorValue(SEN_ECU_POUT,esc.getLiveValue8(LD_CONTROL_PWM_OUT));
      exBus.setSensorValue(SEN_ECU_PHASE,esc.getLiveValue8(LD_CONTROL_TIMING));
    }
    if (esc.hasNewInfoData()){
      exBus.setSensorValue(SEN_ECU_CELL,esc.getInfoValue8(ID_CONTROL_N_CELL));
    }
  } else {
    escNoDataCnt=0;                               // device off: no Error
  }
// errorflags direct in mainDoErrorMessage_LED
#endif
}
//----------------------------------------------------------------------
void mainADC(void){
  adi.adMain();          //  set ADC measurements to Jeti Telemetry
}
//----------------------------------------------------------------------
void mainJetiData(void){
  if (exBus.hasNewChannelData()){
    if (exbusNoDataCnt>exbusNoDataMax) exbusNoDataMax=exbusNoDataCnt;
    if (exbusNoDataCnt>4 && exbusNoDataFrq<0xFF) exbusNoDataFrq++;
    exbusNoDataCnt=0;                         // got data => lost counter=0

// set Buzzer, LED Flash, PIN 0, PIN 1 according to the TX signal
    BuzzerOC2::buzzerOC2set(ioPut.getIOStatus(0));  // check rx value to set Buzzer
    ledFlash.setLedOn(ioPut.getIOStatus(1));    // check rx value to set LED Flash
     // check rx value and configuration to set PIN 0 & PIN 1
    ioPut.setPin0((ioPut.getIOStatus(2)?(sysConf.getHardwareActive(SYS_CONF_HW_BLINK_P0)?getFlag(BLINK_1S):1):0));
    ioPut.setPin1((ioPut.getIOStatus(3)?(sysConf.getHardwareActive(SYS_CONF_HW_BLINK_P1)?getFlag(BLINK_1S):1):0));
  } else if (exbusNoDataCnt==255){              // signal lost on (255*10ms)?
    if (sysConf.getHardwareActive(SYS_CONF_HW_OSLBUZZER))
      BuzzerOC2::buzzerOC2set(true);
    if (sysConf.getHardwareActive(SYS_CONF_HW_OSLLEDFLASH))
      ledFlash.setLedOn(true);
    ioPut.setPin0(0);                           // no EX signal will turn off PIN0 & PIN1
    ioPut.setPin1(0);
  }
}
//----------------------------------------------------------------------
void mainJetiBox(void){
  uint8_t jbKey=exBus.getJetiboxKey();
  if (getFlag(MAIN_1S) || jbKey){                   // call every second or when key is pressed
    jbm->JBMDo(jbKey);
  }
}
//----------------------------------------------------------------------
void mainDoErrorMessage_LED(void){
// highest priority needs to be set as the last one
// num  class reason
// 1      3   exbus lost >0,3 s
// 2      1   esc   lost >0,4 s
// 3      2   esc internal error flags
// 4      2   internal sensor error (BNO,BME,GPS)
// 5      3   
// 6      4
// 7      4   info Input PE0 low

  errorNumber=0;                               // assume everything is okay 
//
  if (sysConf.getHardwareActive(SYS_CONF_HW_IM1)){
// Status 7: Ready takeoff
    if ((jbMessage==0xFF)&&(jbMessageLast&0xF0)==(7<<4)){
      setFlag(READY_MASSAGE,true);               // don't send it twice
    }
    if (!getFlag(READY_MASSAGE)){
      if (sysConf.getHardwareActive(SYS_CONF_HW_ORIENTATION)){
        if (bno.getStatus()==(BNO_PRESENT|BNO_READY)){
          jbMessage=7<<4|JB_MESSAGE_7_CLASS;
        }
      } else {
        jbMessage=7<<4|JB_MESSAGE_7_CLASS;
      }
    }

// Status 6:
    if ((jbMessage==0xFF)&&(jbMessageLast&0xF0)==(6<<4)){
      pinLowMessage=0;                      // Message was send
    }
    if (TIFR4&(1<<ICF4)){                   // falling edge on PE0/IC4?
      TIFR4=(1<<ICF4);                      // clear flag
      pinLowMessage=10;                     // max try time is 10s
    }
    if (pinLowMessage){
      jbMessage=6<<4|JB_MESSAGE_6_CLASS;    // try to send message
    }
  }
//
  if (sysConf.getHardwareActive(SYS_CONF_HW_WM2)){
// Status 5:
    if (getFlag(RESET_WATCHDOG)){
      jbMessage=5<<4|JB_MESSAGE_5_CLASS;
      errorNumber=5+1;
    }
// Status 4: 
    if (getFlag(RESET_BROWNOUT)){
      jbMessage=4<<4|JB_MESSAGE_4_CLASS;
      errorNumber=4+1;
    }
    setFlag(RESET_WATCHDOG|RESET_BROWNOUT,false);
  }

// Status 3: internal BNO || BME || GPS
  if (bmeStatus || bno.getStatus()&BNO_ERROR || ((gpsStatus&0x30)==0x30)||((gpsStatus&0xC0)==0xC0)){
    if (sysConf.getHardwareActive(SYS_CONF_HW_AM_SENSOR)){
      jbMessage=3<<4|JB_MESSAGE_2_CLASS;
    }
    errorNumber=3+1;
  }
// Status 1 & 2 ESC
#ifdef  WITH_KONTRONIK
  if (sysConf.getHardwareActive(SYS_CONF_HW_ESC)){        // ESC active?

// ??? bit mask for error flags
    if (esc.getLiveValue32(LD_CONTROL_ERROR)&(uint32_t)0x0001FFFF){ // check ESC status

      if (sysConf.getHardwareActive(SYS_CONF_HW_AM_ESC)){
        jbMessage=2<<4|JB_MESSAGE_3_CLASS;
      }
      errorNumber=2+1;
    }
    if (escNoDataCnt>48){                                 // total 0,48s without valid esc signal
      if (sysConf.getHardwareActive(SYS_CONF_HW_AM_ESC)){
        jbMessage=1<<4|JB_MESSAGE_1_CLASS;
        escNoDataCnt=32;                                  // keep LED status
      }
    }
    if (escNoDataCnt>=32) errorNumber=1+1;                // LED status without valid esc signal
  }
#endif

// Status 0: no EX >300ms 
  if (sysConf.getHardwareActive(SYS_CONF_HW_AM_EXBUS)){
    if (exbusNoDataCnt>30){                               // 0,3s without valid exbus signal
      jbMessage=0<<4|JB_MESSAGE_0_CLASS;
      errorNumber=0+1;
    }
  }
}
//----------------------------------------------------------------------
void mainWatchDog(void){
  // if eachsection is okay reset watchdog
  wdt_reset();  // only call the watchdog reset once per loop in the main routine
}
//----------------------------------------------------------------------
// send display message 0-7, 0highest priority
void  mainSystemMessage(void){
  if (jbMessage!=0xff){                                                     // new JB Message to send?
    if ((jbMessage&0xF0)<(jbMessageLast&0xF0) || (jbMessageLast&0x0f)==0){  // has new message higher priority or time over?
      char mText[21];
      strcpy_P(mText,(char *)pgm_read_word(&(JB_Message_table[((jbMessage>>4)&0x07)])));
      exBus.setMessage(mText, jbMessage&0x07);                              // TX Display message + message class
      jbMessageLast=jbMessage + JB_MESSAGE_OVERWRITE_T;                     // + time before a new message will be send
      jbMessage=0xff;
    }
  }
  if (getFlag(MAIN_1S) && (jbMessageLast&0x0f)){                            // decrement waiting time
    jbMessageLast--;
  }
}
//----------------------------------------------------------------------
void mainDoStatusLED(void){
// --- test only: toggle LED once per main period
//signal cpu usage - main duration
//digitalWrite(13, false);
//digitalWrite(13, true);
//return;

// ??? miner to do: wrong timing after status change
//errorLEDstate: 
// 0b10000000 done Flag, per bit set/clear period
// 0b01110000 Pause counter
// 0b00001111 Flash counter
  bool switchOn=false;
  if (errorNumber==0){
    if (((uint8_t)cntMs&0x80)==0) switchOn=true;        // every 128ms
  } else {
    if (((uint16_t)cntMs&0x0100)==0) switchOn=true;     // every 256ms
  }
  if (switchOn){
    if ((errorLEDstate&0x80)==0){                       // done Flag
      uint8_t temp=errorLEDstate&0x70;                  // Pause counter
      if (temp){
        errorLEDstate=(temp-0x10)|(errorLEDstate&0x8F); // Pause -1
      } else {
        temp=errorLEDstate&0x0F;                        // Flash counter
        if (temp){
          errorLEDstate=(temp-0x01)|(errorLEDstate&0xF0);
          PORTB|=(1<<PB5);                              // turn LED on
        } else {                                        // set new Output values
          errorLEDstate=(errorNumber<0x0f? errorNumber+1: 0x0f)|0x50;
        }
      }
      errorLEDstate|=0x80;                              // done Flag
    }
  } else {
    errorLEDstate&=0x7f;
    PORTB&=~(1<<PB5);
  }
}
//----------------------------------------------------------------------
// Helper functions

// just to save ram space
void setFlag(uint8_t id, bool state){
  uint8_t cSREG= SREG;
  cli();
  if (state)  systemFlags|=id;
  else        systemFlags&=~id;
  SREG = cSREG;           // restore SREG value (I-bit)
}

bool getFlag(uint8_t id){
  return (systemFlags&id);
}

//----------------------------------------------------------------------
// clear next bit in the bit counter: 1111=0; 1100=2; 1000=3 
// the counter can count max 16
void eepromIncBitCnt(uint16_t &cnt){
  for (uint8_t i=0; i<16; i++){                     // find first 1-bit
    if (cnt&(1<<i)){
      cnt&=(~(1<<i));   // clear next bit
      break;
    }
  }
  eepromJobM=10;
}

// read bit counter: 1111=0; 1100=2; 1000=3 
// the counter can count max 16
uint8_t eepromReadBitCnt(uint16_t &adr){
  uint8_t i;
  for (i=0; i<16; i++){                     // find first 1-bit
    if (adr&(1<<i)) break;
  }
  return i;
}

// reset bit couter = 0xFFFF
inline void eepromResetBitCnt(void){
  mp.resetCntBO=mp.resetCntWD=0xFFFF;
  eepromJobM=10;
}



// -end
