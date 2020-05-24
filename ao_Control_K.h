/*
 * This lib provides an interpretation of the Kontronik Telemetry protocol.
 * This lib is optimized for the 328p(b).
 
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

#ifndef AO_CONTROL_K
#define AO_CONTROL_K

#include <Arduino.h>
#include "CRC32.h"

#if  F_CPU==16000000L

#else 
#error ("ONLY_defined_for_16MHz")
#endif

#define SERIAL_DEF 	(1<<UPM11)|(1<<UCSZ11)|(1<<UCSZ10)	/* 8E1 */
#define RX_BUF_SIZE_RX1 32								/* 2^x; min 4 max 64*/

#define SENSOR_LIVEDATA	38
#define SENSOR_INFODATA	41


class Control_K : CRC32 {
public:
				Control_K();				// NOP contructor
	void    	start();					// call once in setup()

	bool		hasNewLiveData(void);
	uint32_t	getLiveValue32(uint8_t pos);
	uint16_t	getLiveValue16(uint8_t pos);
	uint8_t 	getLiveValue8(uint8_t pos);

	bool		hasNewInfoData(void);
	uint32_t	getInfoValue32(uint8_t pos);
	uint16_t	getInfoValue16(uint8_t pos);
	uint8_t 	getInfoValue8(uint8_t pos);
	
	void	 	rx_udr_rxc(void);			// class interrupt handler

protected:
	void		resetPacket() { b_state = WAIT_HDR_0; b_nPacketLen = 0; b_nBytes = 0; }
	void		stateMachine(void);
	
	volatile unsigned char 	rx_buffer[RX_BUF_SIZE_RX1];
	volatile uint8_t		rx_buffer_head;
	volatile uint8_t		rx_buffer_tail;
	volatile bool 			rx_irq;

	enum enPacketState{
		WAIT_HDR_0 = 0,
		WAIT_HDR_1 = 1,
		WAIT_HDR_2 = 2,
		WAIT_HDR_3 = 3,
		WAIT_DATA  = 4
	};

	enPacketState b_state;

	uint8_t		b_nPacketLen;
	uint8_t		b_nBytes;
	uint8_t		b_kBusBuffer[45];  		// max von Info or Live Data

	uint8_t 	m_LiveData[SENSOR_LIVEDATA-4];
	uint8_t		m_InfoData[SENSOR_INFODATA-4];
	
	// channel and button data
	volatile bool		m_LiveDataNew;
	volatile uint32_t	m_LiveDataTime;

	volatile bool		m_InfoDataNew;
	volatile uint32_t	m_InfoDataTime;

	// helpers
	void		decodeLiveData(void);
	void		decodeInfoData(void);
	bool		receiveCRCCheck(uint8_t length);
};

static	Control_K *	_ThisControl_K;	// needed to call class interrupt handler

// Live
#define	LD_CONTROL_RPM		0	/*32		*/
#define	LD_CONTROL_UBATT	4	/*16		*/
#define	LD_CONTROL_IBATT	6	/*16 signed */
#define	LD_CONTROL_IMOTOR	8	/*16 signed */
#define	LD_CONTROL_IPMOTOR	10	/*16 signed */
#define	LD_CONTROL_CAPCITY	12	/*16		*/
#define	LD_CONTROL_IBEC		14	/*16		*/
#define	LD_CONTROL_UBEC		16	/*16		*/
#define	LD_CONTROL_PWM_IN	18	/*16		*/
#define	LD_CONTROL_GAS_IN	20	/* 8 signed */
#define	LD_CONTROL_PWM_OUT	21	/* 8		*/
#define	LD_CONTROL_T_MOT	22	/* 8 signed */
#define	LD_CONTROL_T_BEC	23	/* 8 signed */
#define	LD_CONTROL_ERROR	24	/*32		*/
#define	LD_CONTROL_STATUS	28	/*32		*/
#define	LD_CONTROL_TIMING	29	/* 8		*/

// Info
#define	ID_CONTROL_DEVICE	0	/*16		*/
#define	ID_CONTROL_VERSION	2	/*16		*/
#define	ID_CONTROL_ERROR	4	/*32		*/
#define	ID_CONTROL_N_CELL	8	/* 8		*/
#define	ID_CONTROL_RPM		9	/*32		*/
#define	ID_CONTROL_UBATT_N	13	/*16		*/
#define	ID_CONTROL_UBATT_X	15	/*16		*/
#define	ID_CONTROL_IBATT	17	/*16 signed */
#define	ID_CONTROL_IMOTOR	19	/*16 signed */
#define	ID_CONTROL_IPMOTOR	21	/*16 signed */
#define	ID_CONTROL_IBEC		23	/*16		*/
#define	ID_CONTROL_UBEC		25	/*16		*/
#define	ID_CONTROL_GAS_IN	27	/* 8 signed	*/
#define	ID_CONTROL_PWM_OUT	28	/* 8		*/
#define	ID_CONTROL_T_MOT_N	29	/* 8 signed */
#define	ID_CONTROL_T_MOT_X	30	/* 8 signed */
#define	ID_CONTROL_T_BEC_N	31	/* 8 signed */
#define	ID_CONTROL_T_BEC_X	32	/* 8 signed */

#endif // AO_CONTROL_K