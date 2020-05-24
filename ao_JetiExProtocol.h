
#ifndef JETIEXPROTOCOL_H
#define JETIEXPROTOCOL_H

#include <Arduino.h>

// constant data
typedef struct {
//  uint8_t id;
//  char    text[20];
  char    unit[4];
  uint8_t dataType;
  uint8_t precision;
}
JetiSensorConst;
typedef const JetiSensorConst JETISENSOR_CONST; 

// dynamic sensor data
class JetiValue {
  friend class JetiSensor;
  friend class JetiExProtocol;
public:
  JetiValue() : m_value( -1 ) {}
protected:
  // value
  int32_t m_value;
};

// complete data for a sensor to fill ex frame buffer
class JetiExProtocol;

class JetiSensor {
public:
  // Jeti data types
  enum enDataType {
    TYPE_6b   = 0, // int6_t  Data type 6b (-31 ¸31)
    TYPE_14b  = 1, // int14_t Data type 14b (-8191 ¸8191)
    TYPE_22b  = 4, // int22_t Data type 22b (-2097151 ¸2097151)
    TYPE_DT   = 5, // int22_t Special data type – time and date
    TYPE_30b  = 8, // int30_t Data type 30b (-536870911 ¸536870911) 
    TYPE_GPS  = 9, // int30_t Special data type – GPS coordinates:  lo/hi minute - lo/hi degree. 
  }
  EN_DATA_TYPE;

  JetiSensor(uint8_t arrIdx, JetiExProtocol * pProtocol);

//  uint8_t m_id;				// sensor id
  int32_t m_value;			// value
  uint8_t m_enabled;		// sensor value is enabled to be send
  uint8_t m_active;			// sensor value is fresh
  uint8_t m_label[20];		// label/description of value
  uint8_t m_textLen;
  uint8_t m_unitLen;
  uint8_t m_dataType;		// format
  uint8_t m_precision; 
  uint8_t m_bufLen;
  // helpers
  void    copyLabel(const uint8_t * text, const uint8_t * unit,  uint8_t * label, uint8_t label_size, uint8_t * textLen, uint8_t * unitLen);
  uint8_t jetiCopyLabel(uint8_t * ex_buf);
  uint8_t jetiEncodeValue(uint8_t * ex_buf);
};

// Definition of Jeti EX protocol
class JetiExProtocol {
  friend class JetiSensor;
public:
  JetiExProtocol();

  void init(const char * name,  JETISENSOR_CONST * pSensorArray); 
  void setDeviceId(uint8_t idLo, uint8_t idHi) { m_devIdLow = idLo; m_devIdHi = idHi; } // adapt it, when you have multiple sensor devices connected to your REX
  void setMessage(const char *message, uint8_t messageClass=0);
  void setSensorValue(uint8_t id, int32_t value);
  void setSensorValueGPS(uint8_t id, bool bLongitude, float value);
  void setSensorValueGPS(uint8_t id, uint32_t value);
  void setSensorValueDate(uint8_t id, uint8_t day, uint8_t month, uint16_t year);
  void setSensorValueTime(uint8_t id, uint8_t hour, uint8_t minute, uint8_t second);
  void setSensorEnable(uint8_t id, bool bEnable);
  bool getSensorEnable(uint8_t id);

protected:
  enum {
    MAX_SENSORS   = 40, 							// increase up to 255 if necessary, 31 is max for DC16/DS/16
    MAX_SENSORBYTES = (MAX_SENSORS /8) + ( MAX_SENSORS %8 ? 1:0)
  };

  uint8_t	setupExFrame(uint8_t * exBuffer);

  uint16_t			 m_FrameCnt;					// frameCnt to decide what should be send next
  char               m_message[20];					// sensor message
  uint8_t            m_messageLength;					
  uint8_t			 m_messageClass;
  bool				 m_messageNew;					// true if new message needs to be send

  char               m_name[20];					// sensor name
  uint8_t            m_nameLen;

  JETISENSOR_CONST * m_pSensorsConst;               // array to constant sensor definitions
  JetiValue        * m_pValues;                     // sensor value array, same order as constant data array
  uint8_t            m_sensorIdx;                   // current index to sensor array to send value
  uint8_t            m_dictIdx;                     // current index to sensor array to send sensor dictionary
  uint8_t            m_enabledSensors[MAX_SENSORBYTES]; // bit array for enabled sensor bit field
  uint8_t            m_activeSensors[MAX_SENSORBYTES]; 	// bit array for active sensor bit field (value was set)

  //crypt and crc
  enum {
    // Jeti Duplex EX Ids: Manufacturer and device
    MANUFACTURER_ID_LOW = 0x0A, // 0xA409 (Jeti recommended range for 3rd party sensors is 0xA400 – 0xA41F)
    MANUFACTURER_ID_HI  = 0xA4,
    DEVICE_ID_LOW       = 0x4F, // random number: 0x414F
    DEVICE_ID_HI        = 0x41,
    POLY                = 0x07, // constant for for "crypt"
  };
  uint8_t m_devIdLow;
  uint8_t m_devIdHi;
  uint8_t update_crc (uint8_t crc, uint8_t crc_seed);
  uint8_t jeti_crc8(uint8_t *exbuf, unsigned char framelen);
};

#endif // JETIEXPROTOCOLBUF_H
