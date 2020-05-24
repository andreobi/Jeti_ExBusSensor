
#include "ao_SystemInfo.h"
#include "ao_JetiExBusSensor.h"
#include "ao_JetiExBus.h"
#include "ao_BNO055.h"
#include "ao_SystemConf.h"
#ifdef  WITH_KONTRONIK
#include "ao_Control_K.h"
extern Control_K     esc;         // Kontronik esc
#endif



extern JetiExBus    exBus;          // Jeti EX Bus
extern AO_BNO       bno;
extern SystemConf   sysConf;
extern EE_data      mp;
extern volatile uint8_t  errorNumber; 
extern volatile uint8_t  escNoDataCnt;
extern uint8_t  exbusNoDataMax;             // stores the maximum timeout
extern uint8_t  exbusNoDataFrq;             // how often over 40ms a signal was lost
extern uint8_t  gpsStatus;
extern uint8_t  bmeStatus;
extern uint16_t bnoerrorcnt;

JBMenue* SystemInfoFactory::getNewMenue(void){
  jbmObA = new JBMBase[1];
  jbmObA[0].i= new SystemInfoInterface;
  return new JBMenueSel(jbmObA, JBM_SysInfo_table, JBMSYSINFO_TAB_LENGTH);
}

void  SystemInfoInterface::JBMGetParameter(JBMenueSel *jbms, uint8_t num){
  jbms->unit[0]=' ';
  jbms->unit[1]=' ';
  jbms->unit[2]=' ';
  jbms->mini=15;
  jbms->maxi=15;
  jbms->inc=15;
  jbms->confMode=CONF_MODE_SHOW_ONLY|CONF_MODE_PRINT_INFO;
  switch (num){
    case 0:{                // Blink Code
      switch (errorNumber&0x0F) {
        case 0: { jbms->jbmInfoStr=JBMInfoMessage_NM; break;}
        case 1: { jbms->jbmInfoStr=JB_Message0; break;}
        case 2: { jbms->jbmInfoStr=JB_Message1; break;}
        case 3: { jbms->jbmInfoStr=JB_Message2; break;}
        case 4: { jbms->jbmInfoStr=JB_Message3; break;}
        case 5: { jbms->jbmInfoStr=JB_Message4; break;}
        case 6: { jbms->jbmInfoStr=JB_Message5; break;}
        case 7: { jbms->jbmInfoStr=JB_Message6; break;}
        case 8: { jbms->jbmInfoStr=JB_Message7; break;}
        default:{ jbms->jbmInfoStr=JB_Message10; break;}
      }
    break;}
    case 1:{      // RX exbus
      jbms->showValue=exbusNoDataFrq;
      jbms->value=(exbusNoDataMax>3 ? exbusNoDataMax : 0);
      jbms->unit[0]='m';
      jbms->unit[1]='x';
      jbms->mini=13;
      jbms->maxi=14;
      jbms->confMode=CONF_MODE_SHOW_ONLY|CONF_MODE_PRINT_ID16;      
    break;}
    case 2:{      // RX ESC
      if (sysConf.getHardwareActive(SYS_CONF_HW_ESC)){
        jbms->value=(escNoDataCnt<2 ? 0 : escNoDataCnt);
        jbms->unit[0]='c';
        jbms->unit[1]='n';
        jbms->unit[2]='t';
        jbms->confMode=CONF_MODE_SHOW_ONLY|CONF_MODE_PRINT_UD16;      
      }else{
          jbms->jbmInfoStr=JBMInfoMessage_OFF;
      }
    break;}
    case 3:{      // RX GPS Status Protocoll RMC, GGA and RX line
      if (sysConf.getHardwareActive(SYS_CONF_HW_GPS)){
        jbms->unit[0]=((gpsStatus&0x30)==0x30 ? 'E' : '0'|((gpsStatus&0x30)>>4));
        jbms->unit[1]=((gpsStatus&0xC0)==0xC0 ? 'E' : '0'|((gpsStatus&0xC0)>>6));
        jbms->unit[2]=(getFlag(SWUSART_ERROR)?'E':((gpsStatus&0x0F)==0x00 ? 'n' : 'y'));
        jbms->mini=4;
        jbms->maxi=10;
//      jbms->inc=15;
        jbms->jbmInfoStr=JBMInfoMessageGPS;
      }else{
          jbms->jbmInfoStr=JBMInfoMessage_OFF;
      }
    break;}
    case 4:{      //RESET couter BrownOut WatchDog
      jbms->showValue=eepromReadBitCnt(mp.resetCntBO);
      jbms->value=eepromReadBitCnt(mp.resetCntWD);
      jbms->unit[0]='B';
      jbms->unit[1]='W';
      jbms->mini=7;
      jbms->maxi=13;
      jbms->confMode=CONF_MODE_SHOW_ONLY|CONF_MODE_PRINT_ID16;                
    break;}
    case 5:{    // BME
      if (sysConf.getHardwareActive(SYS_CONF_HW_PRESSURE)){
        if (bmeStatus){
          jbms->jbmInfoStr=JBMInfoMessage_IF;
        }else{
          jbms->jbmInfoStr=JBMInfoMessage_NM;
        }
      }else{
          jbms->jbmInfoStr=JBMInfoMessage_OFF;
      }
    break;}
    case 6:{    // BNO
      if (sysConf.getHardwareActive(SYS_CONF_HW_ORIENTATION)){
        if (bno.getStatus()&BNO_ERROR){
          jbms->jbmInfoStr=JBMInfoMessage_IF;
        }else if (!(bno.getStatus()&BNO_READY)){
          jbms->jbmInfoStr=JBMInfoMessage_NR;
        }else if (sysConf.getHardwareActive(SYS_CONF_HW_ORIENT_CALI)){
          jbms->value=bno.getCalibSatus();
          jbms->unit[0]='c';
          jbms->unit[1]='a';
          jbms->unit[2]='l';
          jbms->mini=13;
          jbms->maxi=14;
//        jbms->inc=15;
          jbms->confMode=CONF_MODE_SHOW_ONLY|CONF_MODE_PRINT_BINARY;
        }else{
          jbms->jbmInfoStr=JBMInfoMessage_NM;
        }
      }else{
        jbms->jbmInfoStr=JBMInfoMessage_OFF;
      }
    break;}
#ifdef  WITH_KONTRONIK
// contains only 0
//    case 7:{
//      jbms->value=esc.getLiveValue32(LD_CONTROL_ERROR)>>24;
//      jbms->confMode=CONF_MODE_SHOW_ONLY|CONF_MODE_PRINT_BINARY;
//      jbms->unit[0]='.';
//      jbms->mini=6;
//    break;}
    case 8-1:{
      jbms->value=(esc.getLiveValue32(LD_CONTROL_ERROR)>>16)&0x00FF;
      jbms->confMode=CONF_MODE_SHOW_ONLY|CONF_MODE_PRINT_BINARY;
      jbms->unit[0]='.';
      jbms->mini=6;
    break;}
    case 9-1:{                     // ESC Error Low Word
      jbms->value=(esc.getLiveValue32(LD_CONTROL_ERROR)>>8)&0x00FF;
      jbms->confMode=CONF_MODE_SHOW_ONLY|CONF_MODE_PRINT_BINARY;
      jbms->unit[0]='.';
      jbms->mini=6;    break;}
    case 10-1:{                     // ESC Error High Word
      jbms->value=(esc.getLiveValue32(LD_CONTROL_ERROR))&0x00FF;
      jbms->confMode=CONF_MODE_SHOW_ONLY|CONF_MODE_PRINT_BINARY;
      jbms->unit[0]='.';
      jbms->mini=6;
    break;}
    case 11-1:{                     // ESC Version
      uint16_t raw=esc.getInfoValue16(ID_CONTROL_VERSION);
      jbms->value=100*(raw>>8)+(raw&0x00FF);
//
      jbms->unit[0]='0'+(esc.getInfoValue16(ID_CONTROL_DEVICE)>>10);
      jbms->mini=2;
//
      jbms->showValue=esc.getInfoValue16(ID_CONTROL_DEVICE)&0x03FF;
      jbms->confMode=CONF_MODE_SHOW_ONLY|CONF_MODE_PRINT_ID16;      // 2 values, unsigned int
#else
//    case  7:
    case  8-1:
    case  9-1: 
    case 10-1:
    case 11-1:
    {
      jbms->jbmInfoStr=JBMInfoMessage_OFF;
#endif
    break;}
    case 12-1:{                     // AO Version
      jbms->value=AO_SW_VERSION;
      jbms->confMode=CONF_MODE_SHOW_ONLY|CONF_MODE_PRINT_UD16|0x20;      
    break;}
	}
}
//---------------------------------------------------------------------
//
