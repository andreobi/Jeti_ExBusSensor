#include "ao_PessureBME.h"
#include "ao_JetiExBusSensor.h"
#include "ao_JetiExBus.h"

extern volatile uint32_t cntMs;        // 1ms counter
extern JetiExBus        exBus;        // Jeti EX Bus
extern WireLight        wireLight;    // I2C bus

BME280CalibData calDat;

uint8_t  bmeStatus;
uint8_t  bmeNextTime;
uint8_t  bmeMeasureCnt;
uint8_t  bmeWorkState;
uint32_t bmePressureSum;
float    previousAltitude;
float    startAltitude;

union {
  uint8_t   regData[6];
  BMEValue  bmeValue;
} bmeCache;

// internal functions
void     bmeGetCalibdata(void);
void     bmeStartMeasure(void);
bool     bmeDataready(void);
void     bmeReadValue(uint8_t *reg_data);
BMEValue bmeCalibrateValue(uint8_t *reg_data);

/*
 * start all 67 ms a new measurement
 * take 5 measures in the arithmetic mean, so there will be 3 results per second
 * Pressure   oversampling  16
 * temerature oversampling   4
 * IRR filter                8
 * 
 * workstate is used to distribute the workload over 3 main cycles
 * this has an major effect when using a 8MHz cpu
 */
bool startPessureBME() {  
  bmeMeasureCnt=0;
  bmePressureSum=0;
  bmeWorkState=0;                           // start with a new workload cycle

  uint8_t regData;
  wireLight.requestFrom(BME280_I2C_ADR, BME280_CHIP_ID_ADDR, &regData, 1);
  if (regData == BME280_CHIP_ID) {
    uint8_t try_cnt = 3;
    uint8_t cmd = 0xB6;                                   // Software RESET
    wireLight.sendTo(BME280_I2C_ADR, BME280_RESET_ADDR, &cmd, 1);
    do {
      uint8_t msTime=(uint8_t)cntMs;
      while ((int8_t)((uint8_t)cntMs-msTime)<3);          //  delay(2);
      wireLight.requestFrom(BME280_I2C_ADR, BME280_STATUS_REG_ADDR, &regData, 1);
    } while ((try_cnt--) && (regData & BME280_STATUS_IM_UPDATE));
    if (!(regData&BME280_STATUS_IM_UPDATE)) {                           // 0 when done
      bmeGetCalibdata();
      uint8_t cmd=0x08;                                                 // Tsb=0.5; IRR=8;
      wireLight.sendTo(BME280_I2C_ADR, BME280_CONFIG_ADDR, &cmd, 1);    // config
      cmd=0x00;
      wireLight.sendTo(BME280_I2C_ADR, BME280_CTRL_HUM_ADDR, &cmd, 1);  // ctrl hum
      cmd=0x74;                                                         // 4x temp + 16x pressure
      wireLight.sendTo(BME280_I2C_ADR, BME280_CTRL_MEAS_ADDR, &cmd, 1); // ctrl meas
      return true;
    }
  }
  return false;
}


void mainPessureBME() {
  if (bmeWorkState==0){
    if (int8_t((uint8_t)cntMs-bmeNextTime)<66){
      return;                               // less than 67ms
    } else {
      if (!bmeDataready()) return;          // data not ready (shoudn't happen)
    }
    bmeWorkState++;
    bmeNextTime+=67;
// ca 1ms @16MHz
    bmeReadValue(bmeCache.regData);         // get data
    bmeStartMeasure();                      // start new measurement
  } else   if (bmeWorkState==1){
    bmeWorkState++;
// ca 1.2ms
// convert data to physical value
    bmeCache.bmeValue=bmeCalibrateValue(bmeCache.regData);
  } else {
    bmeWorkState=0;                         // start with a new workstate cycle
    if (bmeMeasureCnt>=5){                  // 5*67ms?
      bmeCache.bmeValue.pressure=bmePressureSum/5;
      bmePressureSum=0;
      bmeMeasureCnt=0;
    } else {
      bmePressureSum+=bmeCache.bmeValue.pressure;
      bmeMeasureCnt++;
      return;
  }
// ca 0.8ms
// calculate altitude and vario
    float pressure =((float)bmeCache.bmeValue.pressure)/10000;
    float altitude = 44330 * (1.0 - pow(pressure / 1013.25, 0.1903));
    float vario = (altitude-previousAltitude)*3;
    if (0.15>vario && vario>-0.15) vario=0;
    previousAltitude=altitude;
// set exbus data
// ??? don' overwrite start altitude in case of watchdog or brownout
    if (cntMs<3000) {
      startAltitude=altitude;           // remenber start altitude time < 3s
    } else {
      exBus.setSensorValue(SEN_PRE_ALTI, int16_t((altitude-startAltitude)*10));
      exBus.setSensorValue(SEN_PRE_VARI, int16_t(vario*10));
      exBus.setSensorValue(SEN_PRE_TEMP, int16_t(bmeCache.bmeValue.temperature));
    }
  }
}



void bmeGetCalibdata(void) {
  uint8_t reg_data[(0x9f-0x88)+1] = { 0 }; // 24 without H1
  wireLight.requestFrom(BME280_I2C_ADR, BME280_TEMP_PRESS_CALIB_DATA_ADDR, reg_data,(0x9f-0x88)+1);
  calDat.dig_t1 =          BME280_CONCAT_BYTES(reg_data[1], reg_data[0]);
  calDat.dig_t2 = (int16_t)BME280_CONCAT_BYTES(reg_data[3], reg_data[2]);
  calDat.dig_t3 = (int16_t)BME280_CONCAT_BYTES(reg_data[5], reg_data[4]);
  calDat.dig_p1 =          BME280_CONCAT_BYTES(reg_data[7], reg_data[6]);
  calDat.dig_p2 = (int16_t)BME280_CONCAT_BYTES(reg_data[9], reg_data[8]);
  calDat.dig_p3 = (int16_t)BME280_CONCAT_BYTES(reg_data[11], reg_data[10]);
  calDat.dig_p4 = (int16_t)BME280_CONCAT_BYTES(reg_data[13], reg_data[12]);
  calDat.dig_p5 = (int16_t)BME280_CONCAT_BYTES(reg_data[15], reg_data[14]);
  calDat.dig_p6 = (int16_t)BME280_CONCAT_BYTES(reg_data[17], reg_data[16]);
  calDat.dig_p7 = (int16_t)BME280_CONCAT_BYTES(reg_data[19], reg_data[18]);
  calDat.dig_p8 = (int16_t)BME280_CONCAT_BYTES(reg_data[21], reg_data[20]);
  calDat.dig_p9 = (int16_t)BME280_CONCAT_BYTES(reg_data[23], reg_data[22]);
}


void bmeStartMeasure(void){
  uint8_t cmd=0x75;
  wireLight.sendTo(BME280_I2C_ADR, BME280_CTRL_MEAS_ADDR, &cmd, 1); // ctrl meas
}

bool bmeDataready(void){
  uint8_t status_reg; 
  wireLight.requestFrom(BME280_I2C_ADR, BME280_STATUS_REG_ADDR, &status_reg, 1);
  return ((status_reg&BME280_STATUS_DATA_READY)==0);
}

void bmeReadValue(uint8_t *reg_data){
  wireLight.requestFrom(BME280_I2C_ADR, BME280_DATA_ADDR, reg_data, 6);
}

// value calibration according to Bosch
BMEValue bmeCalibrateValue(uint8_t *reg_data){
  int32_t temperature;
  int32_t t_fine;
  uint32_t utemperature = ((uint32_t)reg_data[3] << 12) 
                        | ((uint32_t)reg_data[4] << 4) 
                        | ((uint32_t)reg_data[5] >> 4);
  int32_t varT1;
  int32_t varT2;
  int32_t temperature_min = -4000;
  int32_t temperature_max = 8500;
  varT1 = (int32_t)((utemperature / 8) - ((int32_t)calDat.dig_t1 * 2));
  varT1 = (varT1 * ((int32_t)calDat.dig_t2)) / 2048;
  varT2 = (int32_t)((utemperature / 16) - ((int32_t)calDat.dig_t1));
  varT2 = (((varT2 * varT2) / 4096) * ((int32_t)calDat.dig_t3)) / 16384;
  t_fine = varT1 + varT2;
  temperature = (t_fine * 5 + 128) / 256;
  if (temperature < temperature_min){
    temperature = temperature_min;
  } else if (temperature > temperature_max) {
    temperature = temperature_max;
  }
//
  uint32_t pressure;
  uint32_t upressure = ((uint32_t)reg_data[0] << 12)
                     | ((uint32_t)reg_data[1] << 4)
                     | ((uint32_t)reg_data[2] >> 4);
  int64_t varP1;
  int64_t varP2;
  int64_t varP3;
  int64_t varP4;
  uint32_t pressure_min = 3000000;
  uint32_t pressure_max = 11000000;
  varP1 = ((int64_t)t_fine) - 128000;
  varP2 = varP1 * varP1 * (int64_t)calDat.dig_p6;
  varP2 = varP2 + ((varP1 * (int64_t)calDat.dig_p5) * 131072);
  varP2 = varP2 + (((int64_t)calDat.dig_p4) * 34359738368);
  varP1 = ((varP1 * varP1 * (int64_t)calDat.dig_p3) / 256) + ((varP1 * ((int64_t)calDat.dig_p2) * 4096));
  varP3 = ((int64_t)1) * 140737488355328;
  varP1 = (varP3 + varP1) * ((int64_t)calDat.dig_p1) / 8589934592;
  if (varP1 != 0) {
  varP4 = 1048576 - upressure;
  varP4 = (((varP4 * INT64_C(2147483648)) - varP2) * 3125) / varP1;
  varP1 = (((int64_t)calDat.dig_p9) * (varP4 / 8192) * (varP4 / 8192)) / 33554432;
  varP2 = (((int64_t)calDat.dig_p8) * varP4) / 524288;
  varP4 = ((varP4 + varP1 + varP2) / 256) + (((int64_t)calDat.dig_p7) * 16);
  pressure = (uint32_t)(((varP4 / 2) * 100) / 128);
    if (pressure < pressure_min) {
        pressure = pressure_min;
    } else if (pressure > pressure_max) {
      pressure = pressure_max;
    } 
  } else {
    pressure = pressure_min;
  }

  BMEValue bme;
  bme.temperature=temperature/100;
  bme.pressure=pressure;
  return bme;
}

//
