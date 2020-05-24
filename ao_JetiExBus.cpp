#include "ao_JetiExBus.h"
#include "ao_JetiExProtocol.h"

// CLI max time = 170us!!!

JetiExBus::JetiExBus() : m_nButtons(0), m_newChannelData(0), m_nNumChannels(0){
	_ThisJetiExBus = this;
	resetPacket();
	memset(m_channelValues, 0, sizeof(m_channelValues));
}

void JetiExBus::start(const char * name, JETISENSOR_CONST * pSensorArray){
	memset(m_textBuffer, ' ', sizeof(m_textBuffer));	// init jetibox text memory
	JetiExProtocol::init(name, pSensorArray);			// init EX protocol handler
	resetPacket();
	pinMode(11, INPUT);						// init EX bus serial port 
	digitalWrite(11, true);
	rx_irq=false;
	UBRR1= JETI_SERIAL_SPEED;				//125.000Baud
	UCSR1C = SERIAL_8N1;
	UCSR1B= (1<<RXEN1)|(1<<RXCIE1);			// RX on
}

// prepare TX to send buffer
void JetiExBus::writeTx(uint8_t* txBuffer, uint8_t nBytes){
	if (nBytes>0) {
		tx_buffer_pointer=1;
		tx_buffer_end=nBytes;
		tx_buffer=txBuffer;
		cli();
		while (!(UCSR1A & (1<<UDRE1)));
		UCSR1A=(1<<TXC1);
		UDR1 = tx_buffer[0];
		UCSR1B= (1<< TXEN1)|(1<<UDRIE1);	// TX on, RX off
		sei();
	}
}

ISR(USART1_UDRE_vect) {
	_ThisJetiExBus->tx_udr_empty_irq();
}

// send Buffer
void JetiExBus::tx_udr_empty_irq(void) {
	if (tx_buffer_pointer < tx_buffer_end) {
		UCSR1A=(1<<TXC1);
		UDR1 = tx_buffer[tx_buffer_pointer];
		tx_buffer_pointer++;
	} else {
		UCSR1B= (1<<TXCIE1);				// TX off, Buffer empty ISR on
	}
}

ISR(USART1_TX_vect) {
	UCSR1B= (1<<RXEN1)|(1<<RXCIE1);			// RX on
}

ISR(USART1_RX_vect) {
    _ThisJetiExBus->rx_udr_rxc();
}

// write recived data into Buffer and do RX protocol
void JetiExBus::rx_udr_rxc(void){
	while (UCSR1A&(1<<RXC1)){
		uint8_t i =(rx_buffer_head + 1) % EX_RX_BUF_SIZE;
		if (bit_is_clear(UCSR1A, UPE1)) {
			if (i != rx_buffer_tail) {
				rx_buffer[rx_buffer_head] = UDR1;
				rx_buffer_head = i;
			} else {
				break;					// SW Buffer full
			}
		} else {
			unsigned char c = UDR1;
		}
	}

	if (rx_irq==false) {    // only enter statemachine once
		rx_irq=true;
		sei();
		stateMachine();
		cli();
		rx_irq=false;
	}
}

void JetiExBus::stateMachine(void){
  // master header data
  // 0x3x 0x01 --> release bus
  // 0x3x 0x03 --> keep allocated
  // 0x3e 0x0x --> channel data
  // 0x3d 0x01 --> EX telemetry or jetibox request
  while (rx_buffer_head != rx_buffer_tail) {
    unsigned char c;
    c = rx_buffer[rx_buffer_tail];
    rx_buffer_tail = (rx_buffer_tail + 1) % EX_RX_BUF_SIZE;
    if (m_state == WAIT_END) {
      m_exBusBuffer[m_nBytes++] = c;
      if (m_nBytes == m_nPacketLen) {
        if (receiveCRCCheck()) {
		  m_nPacketId = m_exBusBuffer[3];
          if (m_exBusBuffer[0]==0x3e && (m_exBusBuffer[4]==0x31)) {			// packet contains channel data 
            decodeChannelData();
          } else if (m_exBusBuffer[1]==0x01 && m_exBusBuffer[4]==0x3a) {	// packet is a telemetry request
            sendTelemetryData();
          } else if (m_exBusBuffer[1]==0x01 && m_exBusBuffer[4]==0x3b) {	// packet is a Jetibox request
            sendJetiBoxData();
          }
        }
        m_state = WAIT_HDR_START;
      }
    } else if (m_state == WAIT_HDR_START) {
      if (c == 0x3d || c == 0x3e) {
        m_state = WAIT_HDR_TYPE;
        m_exBusBuffer[0] = c;
      }
    } else if (m_state == WAIT_HDR_TYPE) {
      if (c == 0x01 || c == 0x03 ) {
        m_state = WAIT_LEN;
        m_exBusBuffer[1] = c;
      } else {
        m_state = WAIT_HDR_START; // --> Error
      }
    } else if (m_state == WAIT_LEN) {
      m_state = WAIT_END;
      m_nPacketLen = (uint8_t)c;
      m_exBusBuffer[2] = c;
      m_nBytes = 3;
      if (m_nPacketLen > sizeof( m_exBusBuffer)) {
        m_state = WAIT_HDR_START; // --> Error
      }
    } else {
	   m_state = WAIT_HDR_START; // --> Error
	}
  }
}

void JetiExBus::decodeChannelData(){
	m_nNumChannels = m_exBusBuffer[5] / 2;  						// number of channels
	uint16_t * pChannel = (uint16_t*)&m_exBusBuffer[6];  			// first channel data position
	for (uint8_t i = 0; i < m_nNumChannels; i++)
		m_channelValues[i] = *(pChannel++);
	m_newChannelData = 0x03;
}

void JetiExBus::sendJetiBoxData(){
	uint8_t bt=((~m_exBusBuffer[6])&0xF0);		// 0bKKKK...N	K:key, N:new Data

	m_nBytes = 40;								// send jetibox packet
	m_exBusBuffer[0] = 0x3b;
	m_exBusBuffer[1] = 0x01;
	m_exBusBuffer[2] = m_nBytes;
	m_exBusBuffer[3] = m_nPacketId;
	m_exBusBuffer[4] = 0x3b; 					// jetibox
	m_exBusBuffer[5] = 32;						// SUB_LEN
	memcpy(&m_exBusBuffer[6], m_textBuffer, 32 );

	uint16_t crcCalc = 0;
	for (uint8_t i = 0; i < m_nBytes - 2; i++)
		crcCalc = crc_ccitt_update(crcCalc, m_exBusBuffer[i]);

	m_exBusBuffer[38] = (uint8_t)(crcCalc & 0xFF);
	m_exBusBuffer[39] = (uint8_t)(crcCalc >> 8);

	writeTx(m_exBusBuffer, m_nBytes);

	if (bt && bt==buttonOld){
		buttonCnt++;
		if (buttonPress==0){
			if (buttonCnt>=8){				// first repeat delay
				buttonPress=1;
				buttonCnt=0;
				m_nButtons=bt&0xf0;
			}
		} else {
			if (buttonCnt>=1){				// further delays
				if (buttonPress <0xff) buttonPress++;
				buttonCnt=0;
				m_nButtons=bt&0xf0;
			}
		}
		m_nButtons|=JB_K_LP;				// last Keys still pressed
		if (buttonPress > JB_KEY_VLONG_PRESS)
			m_nButtons|=JB_K_VLP;			// last Keys very long pressed
	} else {
		buttonPress=0;
		buttonCnt=0;
		m_nButtons=bt&0xf0;
	}
	buttonOld=bt;
}

void JetiExBus::sendTelemetryData(){
	uint8_t len = setupExFrame(&m_exBusBuffer[6]);

	m_nBytes = 8 + len;
	m_exBusBuffer[0] = 0x3b;
	m_exBusBuffer[1] = 0x01;
	m_exBusBuffer[2] = m_nBytes;
	m_exBusBuffer[3] = m_nPacketId;
	m_exBusBuffer[4] = 0x3a; // telemetry data
	m_exBusBuffer[5] = len;  // SUB_LEN

	// crc
	uint16_t crcCalc = 0;
	for (uint8_t i = 0; i < m_nBytes - 2; i++)
		crcCalc = crc_ccitt_update(crcCalc, m_exBusBuffer[i]);

	m_exBusBuffer[m_nBytes - 2] = (uint8_t)(crcCalc & 0xFF);
	m_exBusBuffer[m_nBytes - 1] = (uint8_t)(crcCalc >> 8);

	writeTx(m_exBusBuffer, m_nBytes);
}

// calc crc...
bool JetiExBus::receiveCRCCheck(){
	uint16_t crcCalc = 0;
	for (uint8_t i = 0; i < m_nBytes-2; i++)
		crcCalc = crc_ccitt_update(crcCalc, m_exBusBuffer[i]);

	// ...and compare with crc from packet
	uint16_t crcPacket  = m_exBusBuffer[m_nBytes - 1] << 8;
	         crcPacket |= m_exBusBuffer[m_nBytes - 2];
	return (crcCalc == crcPacket);
}

// Jeti Box
void JetiExBus::setJetiboxText(uint8_t lineNo, const char* text){
	if (text == 0) return;
	char * pStart = 0;
	switch (lineNo) {
	default:
	case 0: pStart = m_textBuffer; break;
	case 1: pStart = m_textBuffer + 16; break;
	}

	bool bPadding = false;
	for (uint8_t i = 0; i < 16; i++) {
		if (text[i] == '\0') bPadding = true;
		if (!bPadding) 
			pStart[i] = text[i];
		else
			pStart[i] = ' ';
	}
}

// CRC calculation
uint16_t JetiExBus::crc_ccitt_update(uint16_t crc, uint8_t data) {
	uint16_t ret_val;
	data ^= (uint8_t)(crc) & (uint8_t)(0xFF);
	data ^= data << 4;
	ret_val = ((((uint16_t)data << 8) | ((crc & 0xFF00) >> 8))
		       ^ (uint8_t)(data >> 4)
		       ^ ((uint16_t)data << 3));
	return ret_val;
}

bool JetiExBus::hasNewChannelData(void) { 
	uint8_t cSREG= SREG;
	cli();
	bool chS = m_newChannelData&0x01; 
	m_newChannelData &= (~0x01); 
	SREG = cSREG;			// restore SREG value (I-bit)
	return chS;
	}
	
// get channel data
uint16_t JetiExBus::getChannel(uint8_t nChannel){
	if (nChannel >= JETIEXBUS_COUNT_OF(m_channelValues)) return 0;
	uint8_t cSREG= SREG;
	cli();
	uint16_t chV=m_channelValues[nChannel];
	SREG = cSREG;			// restore SREG value (I-bit)
	return chV;
}

uint8_t  JetiExBus::getJetiboxKey(){
	uint8_t cSREG= SREG;
	cli();
	uint8_t b=m_nButtons;
	m_nButtons=0;
	SREG = cSREG;			// restore SREG value (I-bit)
	return b;
}
