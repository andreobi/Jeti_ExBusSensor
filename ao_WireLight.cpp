
#include "ao_WireLight.h"
#include "Arduino.h"
#include <compat/twi.h>

#define SDA0		PC5
#define SCL0		PC4

#define TWI_FREQ 	125000L

#define TWI_READY	0
#define TWI_MRX		1
#define TWI_MTX		2
#define TWI_SRX		3
#define TWI_STX		4

volatile uint8_t WireLight::twState;		// will be set by ISR statemachine
volatile uint8_t WireLight::twSlaRW;		// chipAdr + R/W Bit
volatile uint8_t WireLight::twSendStop;		// keep bus, send no Stop
volatile uint8_t WireLight::twNoStart;		// already bus owner, send no Start
volatile uint8_t WireLight::twWait;			// wait until send is done
volatile uint8_t WireLight::twError;		// Error
volatile uint8_t WireLight::twRegAdr;		// register Address
volatile uint8_t *WireLight::twBuffer;		// Pointer to data buffer 
volatile uint8_t WireLight::twBufferIndex;	// current position in buffer
volatile uint8_t WireLight::twBufferLength;	// buffer length


WireLight::WireLight(void){						// constructor 
}


void WireLight::begin(void){
	WireLight::twState = TWI_READY;				// initialize state
	WireLight::twSendStop = true;
	WireLight::twNoStart = false;
	WireLight::twWait = true;					// wait until send job is done
	digitalWrite(SDA0, 1);						// activate internal pullups
	digitalWrite(SCL0, 1);
//
	TWSR&=(~((1<<TWPS0)|(1<<TWPS1)));			// prescaler =1
	TWBR = ((F_CPU / TWI_FREQ) - 16)/2;			// set bus rate
	TWCR =(1<<TWEN)|(1<<TWIE)|(1<<TWEA);		// enable twi module, acks, and twi interrupt
}


void  WireLight::setWaitMode(uint8_t mode){
  WireLight::twWait=mode;						// true: send will return when done
}


bool  WireLight::getSendActive(void){
  return (TWI_READY != WireLight::twState);		// returns active status
}


void WireLight::setClock(uint32_t speed){
	TWBR = ((F_CPU / speed) - 16)/2;			// set bus rate
}


uint8_t WireLight::requestFrom(uint8_t chipAdr, uint8_t regAdr, uint8_t *rxBuffer, uint8_t length, bool adrStop, bool stop){
	sendTo(chipAdr, regAdr,0, 0,adrStop);		// send only reg address
	while(TWI_READY != twState);				// previous job done?
	WireLight::twBuffer=rxBuffer;
	WireLight::twBufferIndex = 0;				// set buffer conditions
	WireLight::twBufferLength = length-1;		// last byte will be read with ACK
	WireLight::twState = TWI_MRX;
	WireLight::twSendStop = stop;
	WireLight::twError = 0xFF;					// reset error state
	twSlaRW = TW_READ;							// slave chipAdr + Read bit
	twSlaRW |= chipAdr << 1;
	if (true == twNoStart) {					// continue, alread bus owner?
		WireLight::twNoStart = false;
		do {
			TWDR = twSlaRW;						// send chipAdr +R/W
		} while(TWCR & (1<<TWWC));
		TWCR =(1<<TWINT)|(1<<TWEA)|(1<<TWEN)|(1<<TWIE);	// enable INTs, but not START
	} else {
		TWCR =(1<<TWEN)|(1<<TWIE)|(1<<TWEA)|(1<<TWINT)|(1<<TWSTA);	// start transmitting
	}
	while(TWI_MRX == twState);					// wait for read operation to complete
	if (WireLight::twBufferIndex < length)		// got less?
		length = WireLight::twBufferIndex;
	return length;								// return number of received bytes
}


uint8_t WireLight::sendTo(uint8_t chipAdr, uint8_t regAdr, uint8_t *txBuffer, uint8_t length, bool stop){
	while(TWI_READY != twState);				// previous job done?
	WireLight::twRegAdr =regAdr;
	WireLight::twBuffer =txBuffer;
	WireLight::twBufferIndex =0xFF;				// start to send register address
	WireLight::twBufferLength =length;
	WireLight::twState =TWI_MTX;
	WireLight::twSendStop =stop;
	WireLight::twError =0xFF;
	WireLight::twSlaRW =TW_WRITE;				// slave chipAdr + Write bit
	twSlaRW |= chipAdr << 1;
	if (true == twNoStart) {					// continue, alread bus owner?
		WireLight::twNoStart = false;
		do {
			TWDR = twSlaRW;						// send chipAdr +R/W
		} while(TWCR & (1<<TWWC));
		TWCR =(1<<TWINT)|(1<<TWEA)|(1<<TWEN)|(1<<TWIE);	// enable INTs, but not START
	} else {
		TWCR =(1<<TWINT)|(1<<TWEA)|(1<<TWEN)|(1<<TWIE)|(1<<TWSTA);	// enable INTs
	}
	while(WireLight::twWait && (TWI_MTX == WireLight::twState));	// wait for write operation to complete

	return getError();
}


uint8_t	WireLight::getError(void){
  if (twError == 0xFF)
    return 0;									// success
  else if (WireLight::twError == TW_MT_SLA_NACK)
    return 2;									// error: address send, nack received
  else if (WireLight::twError == TW_MT_DATA_NACK)
    return 3;									// error: data send, nack received
  else
    return 4;									// other twi error
}


inline void WireLight::twStop(void){
  TWCR =(1<<TWEN)|(1<<TWIE)|(1<<TWEA)|(1<<TWINT)|(1<<TWSTO);  // send stop condition
  sei();										// don't block other interrups
  while(TWCR & (1<<TWSTO));						// wait for stop condition to be exectued on bus
  WireLight::twState = TWI_READY;
}


ISR(TWI_vect){
	switch(TW_STATUS){								// Check status register
// All Master
		case TW_START:								// start condition
		case TW_REP_START:							// repeated start condition
			TWDR = WireLight::twSlaRW;				// send chipAdr +R/W
			TWCR =(1<<TWEN)|(1<<TWIE)|(1<<TWINT)|(1<<TWEA);
		break;
// Master Transmitter
		case TW_MT_SLA_ACK:							// slave receiver acked address
		case TW_MT_DATA_ACK:						// slave receiver acked data
			if(WireLight::twBufferIndex == 0xFF){	// register to send?
				TWDR = WireLight::twRegAdr;
				TWCR =(1<<TWEN)|(1<<TWIE)|(1<<TWINT)|(1<<TWEA);
				WireLight::twBufferIndex++;
			} else if(WireLight::twBufferIndex < WireLight::twBufferLength){	// data to send?
				TWDR = WireLight::twBuffer[WireLight::twBufferIndex++];
				TWCR =(1<<TWEN)|(1<<TWIE)|(1<<TWINT)|(1<<TWEA);
			} else {
				if (WireLight::twSendStop){
					WireLight::twStop();			// done
				} else {
					WireLight::twNoStart = true;	// keep bus and continue
					TWCR =(1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
					WireLight::twState = TWI_READY;
				}
			}
		break;
		case TW_MT_SLA_NACK:						// address sent, nack received
			WireLight::twError = TW_MT_SLA_NACK;
			WireLight::twStop();
		break;
		case TW_MT_DATA_NACK:						// data sent, nack received
			WireLight::twError = TW_MT_DATA_NACK;
			WireLight::twStop();
		break;
		case TW_MT_ARB_LOST:						// lost bus arbitration
			WireLight::twError = TW_MT_ARB_LOST;
			TWCR =(1<<TWEN)|(1<<TWIE)|(1<<TWEA)|(1<<TWINT);
			WireLight::twState = TWI_READY;
		break;
// Master Receiver
		case TW_MR_DATA_ACK:						// data received, ack sent
			WireLight::twBuffer[WireLight::twBufferIndex++] = TWDR;
		case TW_MR_SLA_ACK:							// address sent, ack received
			if(WireLight::twBufferIndex < WireLight::twBufferLength){
				TWCR =(1<<TWEN)|(1<<TWIE)|(1<<TWINT)|(1<<TWEA);
			}else{
				TWCR =(1<<TWEN)|(1<<TWIE)|(1<<TWINT);
			}
		break;
		case TW_MR_DATA_NACK:						// data received, nack sent
			WireLight::twBuffer[WireLight::twBufferIndex++] = TWDR;
			if (WireLight::twSendStop) {
				WireLight::twStop();
			} else {
				WireLight::twNoStart = true;		// keep bus and continue
				TWCR =(1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
				WireLight::twState = TWI_READY;
			}    
		break;
		case TW_MR_SLA_NACK:						// address sent, nack received
			WireLight::twStop();
		break;
													// TW_MR_ARB_LOST handled by TW_MT_ARB_LOST case
// All
		case TW_NO_INFO:							// no state information
			break;
		case TW_BUS_ERROR:							// bus error, illegal stop/start
			WireLight::twError = TW_BUS_ERROR;
			WireLight::twStop();
		break;
	}
}
