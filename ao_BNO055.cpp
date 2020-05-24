
#include "Arduino.h"
#include "ao_JetiExBusSensor.h"
#include "ao_BNO055.h"
#include "ao_WireLight.h"
#include "ao_JetiExBus.h"
#include "ao_Eeprom.h"
#include "ao_SystemConf.h"

uint16_t bnoerrorcnt=0;


extern WireLight 	  wireLight;
extern SystemConf   sysConf;
extern volatile uint8_t  bnoTimerCnt;

//data
#define BNO_CLK_SEL		0x80
#define BNO_RST_SYS		0x20

#define POWER_MODE_NORMAL		    0X00
// status
#define BNO_CALIB_STAT_ADR		  0X35
#define BNO_SELFTEST_RESULT_ADR	0X36
#define BNO_INTR_STAT_ADR		    0X37
#define BNO_SYS_CLK_STAT_ADR	  0X38
#define BNO_SYS_STAT_ADR		    0X39
#define BNO_SYS_ERR_ADR			    0X3A
// config
#define BNO_UNIT_SEL_ADR		    0X3B
#define BNO_DATA_SELECT_ADR		  0X3C
#define BNO_OPR_MODE_ADR		    0X3D
#define BNO_PWR_MODE_ADR		    0X3E
#define BNO_SYS_TRIGGER_ADR		  0X3F
#define BNO_TEMP_SOURCE_ADR		  0X40
#define BNO_AXIS_MAP_ASSIGN_ADR	0X41
#define BNO_AXIS_MAP_SIGN_ADR	  0X42
// control register
#define BNO_CHIP_ID_ADR			    0x00
#define BNO_ACCEL_REV_ID_ADR	  0x01
#define BNO_MAG_REV_ID_ADR		  0x02
#define BNO_GYRO_REV_ID_ADR		  0x03
#define BNO_SW_REV_ID_LSB_ADR	  0x04
#define BNO_SW_REV_ID_MSB_ADR	  0x05
#define BNO_BL_REV_ID_ADR		    0X06
#define BNO_PAGE_ID_ADR			    0X07
// calibration
#define BNO_CALIBDATA_REG		    0x55
//

uint8_t const 	    AO_BNO::axisMap[];
volatile uint16_t		AO_BNO::waitToAction;

void AO_BNO::doMain(void){
	if ((waitToAction==0)&&((stateMachine&BNO_STATE_CNT)==0)){		    // wait for BNO chip or stateMachine
		if (sysConf.getHardwareActive(SYS_CONF_HW_ORIENTATION)){
			if (sysConf.getHardwareChanged(SYS_CONF_HW_ORIENTATION)){
				start();									                                  // enabled while running
			} else {
				if (bnoTimerCnt>24){							                          // 100ms passed?
					bnoTimerCnt=0;
					if ((stateMachine&(BNO_ERROR|BNO_PRESENT|BNO_READY))==(BNO_PRESENT|BNO_READY)){	// chip present, no error
/*            uint8_t stat;                                         // reinit bno if not running
            getSensorRegister(&stat, nullptr, nullptr);
            if (stat!=5){                                           // is BNO still running?
              start();                                              // restart BNO
              return;
            }
*/
// Headding, roll, nick
						DataVector dv;
						getSensor(dv, BNO_REG_EULER);
						exBus.setSensorValue(SEM_ORI_HEAD,dv.x);
						exBus.setSensorValue(SEM_ORI_NICK,dv.y);
            exBus.setSensorValue(SEM_ORI_ROLL,dv.z);
// Acceleration
						getSensor(dv, BNO_REG_ACCELEROMETER);
						exBus.setSensorValue(SEM_ORI_AC_X,dv.x);
						exBus.setSensorValue(SEM_ORI_AC_Y,dv.y);
						exBus.setSensorValue(SEM_ORI_AC_Z,dv.z);
// total acceleration
            uint32_t totalG=sqrt((uint32_t)(dv.x*dv.x)+(uint32_t)(dv.y*dv.y)+(uint32_t)(dv.z*dv.z));
            exBus.setSensorValue(SEM_ORI_AC_T,totalG);
					}
					if (axesMap&0x80){                                        // new axes data from menue?
						startAxes();
					} else if (sysConf.getHardwareActive(SYS_CONF_HW_ORIENT_CALI)&&!eepromJobBNO){ // request and previous job done?
            if (getCalibSatus()==0xFF){
              startCalib2EE();
            }
					}
				}
			}
		} else {
			if (!eepromJobBNO){
			  stateMachine=0;                                             // all done then turn off
			}
		}
	}
 doStateMachine();
// write calibration data to eeprom
	if (eepromJobBNO){
		if (safeCalibData()){
			sysConf.setHardwareActive(SYS_CONF_HW_ORIENT_CALI,false);     // reset JB requestflag for calibration
		}
	}
}

//----------------------------------------------------------------------
bool  AO_BNO::safeCalibData(void){
	bool calib=false;
	if (eepromJobBNO==BNO_CALIBDATA_LENGTH+3) {
			eepromJobBNO--;
			calData = new uint8_t[BNO_CALIBDATA_LENGTH+1];		// get memory
			calData[BNO_CALIBDATA_LENGTH]=57;					        // simple checksum
	} else if (eepromJobBNO==BNO_CALIBDATA_LENGTH+2) {
		if (stateMachine==(BNO_PRESENT|BNO_READY)){				  // data from BNO ready
			for (uint8_t i=0; i<BNO_CALIBDATA_LENGTH; i++)
				calData[BNO_CALIBDATA_LENGTH]+=calData[i];
			eepromJobBNO--;
		}
	} else if (eepromJobBNO){                             // write data to eeprom
		eepromJobBNO--;
		if (!AEEPROM.update(BNO_CALIB_EEPROM+eepromJobBNO, calData[eepromJobBNO])){
			eepromJobBNO++;
		} else if (!eepromJobBNO){
			delete [] calData;                                // done, return memory
			calib=true;
		}
	}
	return calib;
}


void AO_BNO::start(void){
  stateMachine=BNO_START|BNO_CNT0; 
  axesMap&=0x7F;
  waitToAction=1000;                                              // delay for next action >650ms
}

void AO_BNO::startAxes(void){
	stateMachine=BNO_PRESENT|BNO_AXES|BNO_CNT1;
  axesMap&=0x7F;
	setModeConfig();
}

void AO_BNO::startCalib2EE(void){
	stateMachine=BNO_PRESENT|BNO_CALIB2EE|BNO_CNT1;
	setModeConfig();
	eepromJobBNO=BNO_CALIBDATA_LENGTH+3;						// start Calibration to EEprom
}

void AO_BNO::startOff(void){									    // no further action on BNO
	stateMachine=0;
}


void AO_BNO::doStateMachine(void){
	if (waitToAction)	return;										    // wait for BNO chip 
//
	switch (stateMachine&0x1F){
    case (BNO_START|BNO_CNT0) :{                  // start 0
      if (isPresent()){
        stateMachine=BNO_PRESENT|BNO_START|BNO_CNT1;  // start reset
        setReset();
      } else {
        stateMachine=BNO_ERROR;                       // if not present then error
      }
      stateMachine=BNO_PRESENT|BNO_START|BNO_CNT1;// start => 1
      break;}
		case (BNO_START|BNO_CNT1) :{					        // start 1
			setPowerMode();
			writeAxes();
			writeCalibData();
			setModeOperation();
			stateMachine=BNO_PRESENT|BNO_START|BNO_CNT2;// start => 2
			break;}
    case (BNO_START|BNO_CNT2) :{                  // start 2
      uint8_t stat;
      getSensorRegister(&stat, nullptr, nullptr);
      if (stat==5){
        stateMachine=BNO_PRESENT|BNO_READY;       // mode is running
      } else {
        waitToAction=10000;                       // error, try again in 10s
        stateMachine=BNO_PRESENT|BNO_START|BNO_CNT3;// start => 3
      }
      break;}
    case (BNO_START|BNO_CNT3) :{                  // restart 3
      start();
      break;}
		case (BNO_AXES|BNO_CNT1) :{						        // set axes
			writeAxes();
			setModeOperation();
			stateMachine=BNO_PRESENT|BNO_READY;
			break;}
		case (BNO_CALIB2EE|BNO_CNT1) :{					      // read calib data from BNO
			readCalibData(calData);
			setModeOperation();
			stateMachine=BNO_PRESENT|BNO_READY;
			break;}
	}
}


// --- funtion blocks -------------------
// check chip ID
inline bool AO_BNO::isPresent(void) {
	uint8_t value;
 	wireLight.requestFrom(BNO_CHIP_ADRESS, BNO_CHIP_ID_ADR, &value);
	return (value == BNO_ID ? true:false);
}

// do chip reset
inline void AO_BNO::setReset(void){
	uint8_t value=0;
	wireLight.sendTo(BNO_CHIP_ADRESS, BNO_PAGE_ID_ADR, &value);		  // Page 0
	value=BNO_RST_SYS;
	wireLight.sendTo(BNO_CHIP_ADRESS, BNO_SYS_TRIGGER_ADR, &value);	// reset BNO
	waitToAction=1000;			                                        // delay for next action >650ms
}

// cconfig: set power mode
inline void AO_BNO::setPowerMode(void) {
	uint8_t value= BNO_CLK_SEL;									                    // external crystal
	wireLight.sendTo(BNO_CHIP_ADRESS, BNO_SYS_TRIGGER_ADR, &value);
	value=POWER_MODE_NORMAL;									                      // NORMAL Power mode
	wireLight.sendTo(BNO_CHIP_ADRESS, BNO_PWR_MODE_ADR, &value);
}

// set config mode
void AO_BNO::setModeConfig(void) {
	uint8_t mode= BNO_OPMODE_CONFIG;
	wireLight.sendTo(BNO_CHIP_ADRESS, BNO_OPR_MODE_ADR, &mode);
	waitToAction=15;				                                        // >8 delay for mode change
}

// set measure mode
void AO_BNO::setModeOperation(void) {
  uint8_t mode= BNO_OPMODE_NDOF;
	wireLight.sendTo(BNO_CHIP_ADRESS, BNO_OPR_MODE_ADR, &mode);
	waitToAction=1500;			                                        // NDOF needs about 400ms (simple> 19ms) delay for mode change
}

// config: upper nibble: axis signs xyz 0-7, lower nibble: axis remapping 0-5
void AO_BNO::writeAxes(void) {
	uint8_t am=(axesMap>>4)&0x07;
	wireLight.sendTo(BNO_CHIP_ADRESS, BNO_AXIS_MAP_SIGN_ADR, &am);
	am=axisMap[axesMap&0x07];
	wireLight.sendTo(BNO_CHIP_ADRESS, BNO_AXIS_MAP_ASSIGN_ADR, &am);
}

// config: 
void AO_BNO::readCalibData(uint8_t *calibData) {
	wireLight.requestFrom(BNO_CHIP_ADRESS, BNO_CALIBDATA_REG, calibData, BNO_CALIBDATA_LENGTH);
}

// config: 
void AO_BNO::writeCalibData(void){
  uint8_t calData[BNO_CALIBDATA_LENGTH+1];
  uint8_t checkSum=57;
  for(uint8_t i=0; i<BNO_CALIBDATA_LENGTH; i++){
    calData[i]=AEEPROM.read(BNO_CALIB_EEPROM+i);
    checkSum+=calData[i];
  }
  uint8_t checkSumEE=AEEPROM.read(BNO_CALIB_EEPROM+BNO_CALIBDATA_LENGTH);
  if (checkSumEE==checkSum){			// write only calibration data to chip with valid checksum
    setCalibOffsets(calData);
  }
}

// config: 
void AO_BNO::setCalibOffsets(uint8_t *calibData) {
	wireLight.sendTo(BNO_CHIP_ADRESS, BNO_CALIBDATA_REG, calibData, BNO_CALIBDATA_LENGTH);
}

// config: 
void AO_BNO::clearCalibration(void){
	uint8_t calibData[BNO_CALIBDATA_LENGTH];
	memset(calibData,0,BNO_CALIBDATA_LENGTH);
	wireLight.sendTo(BNO_CHIP_ADRESS, BNO_CALIBDATA_REG, calibData, BNO_CALIBDATA_LENGTH);
}

// --- interfaces -------------------
void AO_BNO::setAxes(uint8_t axesMapping){
	axesMap=axesMapping|0x80; // mark as new
}

void AO_BNO::getSensorRegister(uint8_t *status, uint8_t *selftest, uint8_t *error) {
	if (status)
 		wireLight.requestFrom(BNO_CHIP_ADRESS, BNO_SYS_STAT_ADR, status);
	if (selftest)
		wireLight.requestFrom(BNO_CHIP_ADRESS, BNO_SELFTEST_RESULT_ADR, selftest);
	if (error)
		wireLight.requestFrom(BNO_CHIP_ADRESS, BNO_SYS_ERR_ADR, error);
}

// reads measured data from register
void AO_BNO::getSensor(DataVector &dv, uint8_t regAdr) {
	uint8_t rxBuffer[6];
	wireLight.requestFrom(BNO_CHIP_ADRESS, regAdr, rxBuffer, 6);
	int16_t x = ((int16_t)rxBuffer[0]) | (((int16_t)rxBuffer[1]) << 8);
	int16_t y = ((int16_t)rxBuffer[2]) | (((int16_t)rxBuffer[3]) << 8);
	int16_t z = ((int16_t)rxBuffer[4]) | (((int16_t)rxBuffer[5]) << 8);
	int16_t divisor;
	switch (regAdr) {
//		case BNO_REG_MAGNETOMETER:
//		case BNO_REG_GYROSCOPE:
		case BNO_REG_EULER:
			divisor=16;
		break;
		default:
		divisor=10;
	}
	dv.x = x/ divisor;
	dv.y = y/ divisor;
	dv.z = z/ divisor;
}

//
uint8_t AO_BNO::getCalibSatus(void) {
	uint8_t clibStatus = 0;
	wireLight.requestFrom(BNO_CHIP_ADRESS, BNO_CALIB_STAT_ADR, &clibStatus);
	return clibStatus;
}

uint8_t AO_BNO::getStatus(void){
  return stateMachine&0xE0;
}
