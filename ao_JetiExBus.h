/*
 *
 This lib provides a Jeti ExBus protocol. It is optimized for the 328p(b).
 
 Maijor changes to the parent software are:
 - Integrated the new message protocol to be displayed directly on the tx screen.
 - Send only current sensor data (so old data will blink on the tx display).
 - new API for GPS.
 - new API to turn a sensor on / off.
 - new API to get the JetiBox key with repeat function(long press).
 - Optimizes sensor definition to save memory space (but more a bit uncomfortable).
 - some minor internal changes to save RAM.
 - No call from the main program anymore.
 - Protocoll check take place when data are comming in. (Interrupt driven).
 
 This lib is derived from:  https://github.com/Sepp62/JetiExSensor
 
Licence:

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/
#ifndef JETIEXBUS_H
#define JETIEXBUS_H

#include <Arduino.h>
#include "ao_JetiExProtocol.h"

#if  F_CPU==16000000L

#define JETI_SERIAL_SPEED 0007

#elif F_CPU==8000000L
#define JETI_SERIAL_SPEED 0003

#else 
#error ("ONLY_defined_for_8_or_16MHz")

#endif

#define SERIAL_8N1 0x06
#define EX_RX_BUF_SIZE 32

#define JETIEXBUS_COUNT_OF(x) ((sizeof(x)/sizeof(0[x])) / ((size_t)(!(sizeof(x) % sizeof(0[x]))))) // number of elements in an array

// Jetikey
#define JB_K_R   0x10
#define JB_K_L   0x80
#define JB_K_U   0x20
#define JB_K_D   0x40
#define JB_K_VLP 0x02
#define JB_K_LP  0x01
// key inputs to set very log press
#define JB_KEY_VLONG_PRESS 50

class JetiExBus : public JetiExProtocol {
public:
			 JetiExBus();	// NOP contructor

	void     start(const char * name, JETISENSOR_CONST * pSensorArray);	// call once in setup()

	bool 	 hasNewChannelData(void);									// to check data availablity
	uint8_t  getNumChannels() { return m_nNumChannels; }
	uint16_t getChannel(uint8_t nChannel);

	void     setJetiboxText(uint8_t lineNo, const char* text);
	uint8_t  getJetiboxKey();

	void	 rx_udr_rxc(void);					// class interrupt handler
	void	 tx_udr_empty_irq(void);

protected:
	void 	 writeTx(uint8_t* buffer, uint8_t nBytes);
	void	 stateMachine(void);
	void	 resetPacket() { m_state = WAIT_HDR_START; m_nPacketLen = 0; m_nBytes = 0; m_nPacketId = 0; }
	
	volatile unsigned char 	rx_buffer[EX_RX_BUF_SIZE];
	volatile uint8_t	rx_buffer_head;
	volatile uint8_t	rx_buffer_tail;
	volatile bool 		rx_irq;

	unsigned char 		* tx_buffer;
	volatile uint8_t	tx_buffer_pointer;
	uint8_t 			tx_buffer_end;

	enum enPacketState{
		WAIT_HDR_START = 0,
		WAIT_HDR_TYPE = 1,
		WAIT_LEN = 2,
		WAIT_END = 3,
	};

	enPacketState m_state;

	uint8_t		m_nPacketLen;
	uint8_t		m_nBytes;
	uint8_t		m_nPacketId;
	uint8_t		m_exBusBuffer[64];  // 7 bytes header + 2 bytes crc + 24*2bytes channels

	// channel and button data
	volatile uint8_t	m_newChannelData;
	uint8_t			m_nNumChannels;
	uint16_t		m_channelValues[24];

	uint8_t		m_nButtons;
	uint8_t  	buttonOld;
	uint8_t  	buttonCnt;
	uint8_t  	buttonPress;

	// jetibox text buffer
	char		m_textBuffer[32];

	// helpers
	void		decodeChannelData(void);
	void		sendJetiBoxData(void);
	void		sendTelemetryData(void);
	bool		receiveCRCCheck(void);
	uint16_t 	crc_ccitt_update(uint16_t crc, uint8_t data);
};

static	JetiExBus *	_ThisJetiExBus;	// needed to call class interrupt handler

#endif // JETIEXBUS_H