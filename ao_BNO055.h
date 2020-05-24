
#ifndef __AO_BNO_H__
#define __AO_BNO_H__

#include "Arduino.h"

// chip I2C address
#define BNO_CHIP_ADRESS				0x28
#define BNO_ID 					      0xA0
// Operation mode
#define BNO_OPMODE_CONFIG		  0X00
#define BNO_OPMODE_ACCONLY		0X01
#define BNO_OPMODE_MAGONLY		0X02
#define BNO_OPMODE_GYRONLY		0X03
#define BNO_OPMODE_ACCMAG		  0X04
#define BNO_OPMODE_ACCGYRO		0X05
#define BNO_OPMODE_MAGGYRO		0X06
#define BNO_OPMODE_AMG			  0X07
#define BNO_OPMODE_IMUPLUS		0X08
#define BNO_OPMODE_COMPASS		0X09
#define BNO_OPMODE_M4G			  0X0A
#define BNO_OPMODE_NDOF_FMC_OFF	0X0B
#define BNO_OPMODE_NDOF			  0X0C
// result register 3x
#define BNO_REG_ACCELEROMETER	0x08
#define BNO_REG_MAGNETOMETER	0x0E
#define BNO_REG_GYROSCOPE		  0x14
#define BNO_REG_EULER			    0x1A
#define BNO_REG_LINEARACCEL		0x28
#define BNO_REG_GRAVITY			  0x2E
// Quaternion 4x
#define BNO_QUATERNION_ADR		0X20
// Temperature 1x
#define BNO_TEMP_ADR			    0X34
// number of calibration space
#define BNO_CALIBDATA_LENGTH	22

// flags
#define BNO_ERROR     0x80
#define BNO_PRESENT   0x40
#define BNO_READY     0X20
//#define BNO_CALIB     0x10
// BNO_STATE
#define BNO_STATE     0x1C
#define BNO_START     0x04
#define BNO_AXES      0x08
#define BNO_CALIB2EE  0x10
// counter state machine
#define BNO_STATE_CNT 0x03
#define BNO_CNT0      0x00
#define BNO_CNT1      0x01
#define BNO_CNT2      0x02
#define BNO_CNT3      0x03
//
#define BNO_OFF       0x00

struct DataVector {
	int16_t x;
	int16_t y;
	int16_t z;
};

class AO_BNO {
public:
	void	start(void);            
	void	doMain(void);
	void	setAxes(uint8_t axes);
	void  getSensorRegister(uint8_t *status, uint8_t *selftest, uint8_t *error);
	void	getSensor(DataVector &dv, uint8_t regAdr);
	uint8_t	getCalibSatus(void);
  uint8_t getStatus(void);

	constexpr static uint8_t axisMap[]={0b00100100, 0b00100001, 0b00011000, 0b00010010, 0b00001001, 0b00000110}; // default (0)=zyx, zxy, yzx, yxz, xzy, xyz
  static  volatile uint16_t waitToAction;   // wait until next action in ms

private:
  void  startOff(void);
  void  startCalib2EE(void);
  void  startAxes(void);
  void  doStateMachine(void);

	bool	isPresent(void);
	void	setReset(void);
	void	setPowerMode(void);
	void	setModeConfig(void);
	void	setModeOperation(void);
	void	writeAxes(void);
	void	writeCalibData(void);
	void	setCalibOffsets(uint8_t *calibData);	// integrieren
	
  bool  safeCalibData(void);
	void	readCalibData(uint8_t *calibData);
	void	clearCalibration(void);
	
  uint8_t   stateMachine;         // status of state machine
  uint8_t   eepromJobBNO=0;       // which eeprom cell to write and status
	uint8_t		axesMap;
	uint8_t 	*calData=nullptr;     // temporary calib data buffer
};

#endif
