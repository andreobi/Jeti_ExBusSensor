
#include "ao_Control_K.h"

// CLI max time = 180us!!!
#define AO_USRAT_CONTROL

#ifdef AO_USRAT_CONTROL
#define	UBRRn	UBRR0
#define UCSRnA	UCSR0A
#define	UCSRnB	UCSR0B
#define UCSRnC	UCSR0C
#define UDRn	UDR0
#define RXENn	RXEN0
#define RXCIEn	RXCIE0
#define RXCn	RXC0
#define UPEn	UPE0
#define U2Xn	U2X0
#define USARTn_RX_vect	USART0_RX_vect

#else
#define	UBRRn	UBRR1
#define UCSRnA	UCSR1A
#define	UCSRnB	UCSR1B
#define UCSRnC	UCSR1C
#define UDRn	UDR1
#define RXENn	RXEN1
#define RXCIEn	RXCIE1
#define RXCn	RXC1
#define UPEn	UPE1
#define U2Xn	U2X1
#define USARTn_RX_vect	USART1_RX_vect

#endif


Control_K::Control_K() {
	_ThisControl_K = this;
}

void Control_K::start() {
	resetPacket();
	rx_irq	=false;
	UBRRn	=16;							//115200Baud
	UCSRnA	=(1<<U2Xn);
	UCSRnC	=SERIAL_DEF;					// Data format
	UCSRnB	=(1<<RXENn)|(1<<RXCIEn);		// RX on
}

// defined(USARTn_RX_vect)
ISR(USARTn_RX_vect) {
    _ThisControl_K->rx_udr_rxc();
}

// write recived data into Buffer and do RX protocol
void Control_K::rx_udr_rxc(void){
	while (UCSRnA&(1<<RXCn)){
		uint8_t i =(rx_buffer_head +1) % RX_BUF_SIZE_RX1;
		if ((UCSRnA&(1<<UPEn))==0) {	// Parity okay?
			if (i != rx_buffer_tail) {
				rx_buffer[rx_buffer_head] = UDRn;
				rx_buffer_head = i;
			} else {
				break;					// SW Buffer full
			}
		} else {
			unsigned char c = UDRn;		// waste broken data
		}
	}

	if (rx_irq==false) {    			// don't reenter statemachine
		rx_irq=true;
		sei();
		stateMachine();
		cli();
		rx_irq=false;
	}
}

void Control_K::stateMachine(void){
  while (rx_buffer_head != rx_buffer_tail) {
    unsigned char c;
    c = rx_buffer[rx_buffer_tail];
    rx_buffer_tail = (rx_buffer_tail + 1) % RX_BUF_SIZE_RX1;
    if (b_state == WAIT_DATA) {
      b_kBusBuffer[b_nBytes++] = c;
      if (b_nBytes == b_nPacketLen) {
        if (b_kBusBuffer[3]=='L') {			// LIve data 
          decodeLiveData();
        } else if (b_kBusBuffer[3]=='I') {	// Info data 
          decodeInfoData();
        }
        b_state = WAIT_HDR_0;
      }
    } else if (b_state == WAIT_HDR_0) {
      if (c == 0x4b) {						// K
        b_state = WAIT_HDR_1;
        b_kBusBuffer[0] = c;
      }
    } else if (b_state == WAIT_HDR_1) {
      if (c == 0x4f) {						// O
        b_state = WAIT_HDR_2;
        b_kBusBuffer[1] = c;
	  }
    } else if (b_state == WAIT_HDR_2) {
      if (c == 0x44) {						// D
        b_state = WAIT_HDR_3;
        b_kBusBuffer[2] = c;
	  }
    } else if (b_state == WAIT_HDR_3) {
      if (c == 0x4c  || c == 0x49) {		// L or I
        b_state = WAIT_DATA;
        b_kBusBuffer[3] = c;
		b_nBytes=4;
        b_nPacketLen =(c == 'L' ? SENSOR_LIVEDATA : SENSOR_INFODATA);
	  }
	} else {
      b_state = WAIT_HDR_0; // --> Error
    }
  }
}

void Control_K::decodeLiveData(){
	if (receiveCRCCheck(b_nPacketLen-7)){
		memcpy(m_LiveData,&b_kBusBuffer[4],SENSOR_LIVEDATA-8);	// without header and crc
		m_LiveDataNew = true;
		m_LiveDataTime=millis();
	}
}

void Control_K::decodeInfoData(){
	if (receiveCRCCheck(b_nPacketLen-6)){
		memcpy(m_InfoData,&b_kBusBuffer[4],SENSOR_INFODATA-8);	// without header and crc
		m_InfoDataNew = true;
		m_InfoDataTime=millis();
	}
}

bool Control_K::receiveCRCCheck(uint8_t length){  //-6 für Info  -7 für Live (4 CRC)
	reset();
	for (uint8_t i=0; i<=length;i++)
		update(b_kBusBuffer[i]);

	uint32_t crcBus=	((uint32_t)(b_kBusBuffer[b_nPacketLen-1])<<24)|((uint32_t)(b_kBusBuffer[b_nPacketLen-2])<<16)
			|((uint32_t)(b_kBusBuffer[b_nPacketLen-3])<<8)|((uint32_t)(b_kBusBuffer[b_nPacketLen-4]));
	return crcBus==finalize();
}

bool Control_K::hasNewLiveData(void) {
	uint8_t cSREG= SREG;
	cli();
	bool b = m_LiveDataNew; 
	m_LiveDataNew = false;
	SREG = cSREG;			// restore SREG value (I-bit)
	return b;
}

bool Control_K::hasNewInfoData(void) {
	uint8_t cSREG= SREG;
	cli();
	bool b = m_InfoDataNew; 
	m_InfoDataNew = false;
	SREG = cSREG;			// restore SREG value (I-bit)
	return b;
}

// get live data
uint32_t Control_K::getLiveValue32(uint8_t pos){
	uint8_t cSREG= SREG;
	cli();
	uint32_t value=((uint32_t)(m_LiveData[pos+3])<<24)|((uint32_t)(m_LiveData[pos+2])<<16)|((uint32_t)(m_LiveData[pos+1])<<8)|((uint32_t)(m_LiveData[pos]));
	SREG = cSREG;			// restore SREG value (I-bit)
	return value;
}

uint16_t Control_K::getLiveValue16(uint8_t pos){
	uint8_t cSREG= SREG;
	cli();
	uint16_t value=((uint16_t)(m_LiveData[pos+1])<<8)|((uint32_t)(m_LiveData[pos]));
	SREG = cSREG;			// restore SREG value (I-bit)
	return value;
}
uint8_t Control_K::getLiveValue8(uint8_t pos){
	uint8_t cSREG= SREG;
	cli();
	uint8_t value=m_LiveData[pos];
	SREG = cSREG;			// restore SREG value (I-bit)
	return value;
}

// get info data
uint32_t Control_K::getInfoValue32(uint8_t pos){
	uint8_t cSREG= SREG;
	cli();
	uint32_t value=((uint32_t)(m_InfoData[pos+3])<<24)|((uint32_t)(m_InfoData[pos+2])<<16)|((uint32_t)(m_InfoData[pos+1])<<8)|((uint32_t)(m_InfoData[pos]));
	SREG = cSREG;			// restore SREG value (I-bit)
	return value;
}

uint16_t Control_K::getInfoValue16(uint8_t pos){
	uint8_t cSREG= SREG;
	cli();
	uint16_t value=((uint16_t)(m_InfoData[pos+1])<<8)|((uint32_t)(m_InfoData[pos]));
	SREG = cSREG;			// restore SREG value (I-bit)
	return value;
}
uint8_t Control_K::getInfoValue8(uint8_t pos){
	uint8_t cSREG= SREG;
	cli();
	uint8_t value=m_InfoData[pos];
	SREG = cSREG;			// restore SREG value (I-bit)
	return value;
}
