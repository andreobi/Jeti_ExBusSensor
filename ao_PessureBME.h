#ifndef PRESSUREBME_H
#define PRESSUREBME_H

#include "Arduino.h"
#include "ao_WireLight.h"

extern volatile uint32_t cntMs;         // 1ms counter
bool startPessureBME(void);
void mainPessureBME(void);

// register address
#define BME280_CHIP_ID_ADDR               uint8_t(0xD0)
#define BME280_RESET_ADDR                 uint8_t(0xE0)
#define BME280_TEMP_PRESS_CALIB_DATA_ADDR uint8_t(0x88)
#define BME280_HUMIDITY_CALIB_DATA_ADDR   uint8_t(0xE1)
#define BME280_CTRL_HUM_ADDR              uint8_t(0xF2)
#define BME280_CTRL_MEAS_ADDR             uint8_t(0xF4)
#define BME280_CONFIG_ADDR                uint8_t(0xF5)
#define BME280_DATA_ADDR                  uint8_t(0xF7)
#define BME280_STATUS_REG_ADDR            uint8_t(0xF3)
// Values
#define BME280_I2C_ADR                    uint8_t(0x76)
#define BME280_CHIP_ID                    uint8_t(0x60)
#define BME280_STATUS_IM_UPDATE           uint8_t(0x01)
#define BME280_STATUS_DATA_READY          uint8_t(0x08)
// helper macro
#define BME280_CONCAT_BYTES(msb, lsb)     (((uint16_t)msb << 8) | (uint16_t)lsb)
// calibration data structure
struct BME280CalibData {
    uint16_t  dig_t1;
    int16_t   dig_t2;
    int16_t   dig_t3;
    uint16_t  dig_p1;
    int16_t   dig_p2;
    int16_t   dig_p3;
    int16_t   dig_p4;
    int16_t   dig_p5;
    int16_t   dig_p6;
    int16_t   dig_p7;
    int16_t   dig_p8;
    int16_t   dig_p9;
};

struct BMEValue {
  int8_t   temperature;
  uint32_t pressure;
};

#endif
//
