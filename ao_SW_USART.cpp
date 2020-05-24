
#include "ao_SW_USART.h"

// based on Timer 1:
// RX	Port.B0 = ICP1 	
// 		Port.B1 = free (OC1A used)
// TX	Port.B2 = OC1B

	uint8_t SwUsart::rxLevel;
	uint8_t SwUsart::rxRegister;
	uint8_t SwUsart::rxBitCnt=0;
	uint8_t SwUsart::rxFlags=0;

	uint16_t SwUsart::ocMinTime;
	uint16_t SwUsart::baud = OC1_BAUD;
	uint16_t SwUsart::ocNextTolerance;
	uint8_t SwUsart::rx_buffer_head=0;
	uint8_t SwUsart::rx_buffer_tail=0;
	uint8_t SwUsart::rx_buffer[SW_USART_RX_BUF_SIZE];
//
	uint8_t SwUsart::txRegister;
	uint8_t SwUsart::txBitCnt;
	uint8_t SwUsart::tx_buffer[SW_USART_TX_BUF_SIZE];
	uint8_t SwUsart::tx_buffer_tail=0;
	volatile uint8_t	SwUsart::tx_buffer_head=0;
	volatile bool 		SwUsart::txWriteDone=true;


//-- TX/RX init ------------------------------
void SwUsart::start(void){
	PORTB|=(1<<PB2);					// TX line to 1
	DDRB|=(1<<PB2);

#ifdef RX_PULL_UP
	DDRB&=~(1<<PB0);
	PORTB|=(1<<PB0);					// RX line with pullup
#elif
	DDRB&=~(1<<PB0);
	PORTB&=~(1<<PB0);					// RX line no Pullup
#endif

	cli();
	TCCR1B = (1<<ICNC1)|(1<<CS10);				// IC with noise canceler, no prescaler
	TIFR1 = (1<<ICF1)|(1<<OCF1B)|(1<<OCF1A);	// clear all IRQ Flags for timer 1
	TIMSK1 |= (1<<ICIE1);
	TCCR1A = 0;									// no output compare at init
	sei();
}

//-- RX methods ------------------------------
 uint8_t SwUsart::getError(void){
	uint8_t cSREG= SREG;
	cli();
	uint8_t error=SwUsart::rxFlags&RX_ERROR_BITS;
	SwUsart::rxFlags&=(~RX_ERROR_BITS);
	SREG = cSREG;						// restore SREG value (I-bit)
	return error;
}
 
 int8_t SwUsart::available(void){
//	uint8_t cSREG= SREG;
//	cli();
	int8_t ra=SwUsart::rx_buffer_head-SwUsart::rx_buffer_tail;
//	SREG = cSREG;						// restore SREG value (I-bit)
	if (ra<0) ra=SW_USART_RX_BUF_SIZE+ra;
	return ra;
}

int16_t SwUsart::read(void){
	uint8_t cSREG= SREG;
	cli();
	int16_t c;
	if (SwUsart::rx_buffer_head != SwUsart::rx_buffer_tail){
		c = (int16_t)((uint16_t)SwUsart::rx_buffer[SwUsart::rx_buffer_tail]);
		SwUsart::rx_buffer_tail = (SwUsart::rx_buffer_tail + 1) % SW_USART_RX_BUF_SIZE;
	} else {
		c=-1;
	}
	SREG = cSREG;						// restore SREG value (I-bit)
	return c;
}

//-- RX receive ------------------------------
// timer 1 input capture ISR
ISR (TIMER1_CAPT_vect){
	uint16_t capture = ICR1 ; 
	SwUsart::rxLevel=TCCR1B&(1<<ICES1);
	if (SwUsart::rxLevel){
		TCCR1B&=~(1<<ICES1);
	} else {
		TCCR1B|=(1<<ICES1);
	}
// check if previous OCint is pending
	if (TIFR1&(1<<OCF1A) && SwUsart::rxBitCnt && ((int16_t)(capture-OCR1A))>0){
		SwUsart::rxRegister=(SwUsart::rxRegister>>1);
	if (!SwUsart::rxLevel) SwUsart::rxRegister|=0x80;		// level is now inverted
	SwUsart::rxBitCnt++;
	}
 	SwUsart::ocNextTolerance=(uint16_t)(OC1_BAUD*0.25);
	OCR1A=capture+SwUsart::baud+SwUsart::ocNextTolerance;	// max time, otherwise cnt next bit
	TIFR1=(1<<OCF1A);										// start new bit after change

	if (SwUsart::rxBitCnt>8){								// got 8 bits
		TCCR1B &= ~(1<<ICES1);								// next trigger is: start bit
		TIMSK1 &= ~(1<<OCIE1A);								// no level interrupt
		SwUsart::rxBitCnt=0x00;
// ??? check frame time
// rxFlags|=RX_FRAME_ERROR_I;              // frame timing error
		if (!SwUsart::rxLevel) SwUsart::rxFlags|=RX_ERROR_STOP;	// Frame Error Stopbit expected
		if (SwUsart::rxFlags&RX_BIT_ERROR_I) {
			SwUsart::rxFlags|=RX_ERROR_TIMING;				// bit timing error
			SwUsart::rxFlags&=(~RX_BIT_ERROR_I);
		} else {											// got valid data
			uint8_t i =(SwUsart::rx_buffer_head + 1) % SW_USART_RX_BUF_SIZE;
			if (i != SwUsart::rx_buffer_tail) {
				SwUsart::rx_buffer[SwUsart::rx_buffer_head] = SwUsart::rxRegister;
				SwUsart::rx_buffer_head = i;
			}
		}
		SwUsart::rxRegister=0;
	} else {
		if (SwUsart::rxBitCnt) {							// next bit
			SwUsart::rxRegister=(SwUsart::rxRegister>>1);
			if (SwUsart::rxLevel) SwUsart::rxRegister|=0x80;
			if (((int16_t)(capture-SwUsart::ocMinTime))<0){	// level change too early
				SwUsart::rxFlags|=RX_BIT_ERROR_I;
			}
		} else {											// start bit
			TIMSK1 |=(1<<OCIE1A);							// start level detection IRQ OCA
//			icFrameStart=capture;
		}
		SwUsart::rxBitCnt++;
	}
	SwUsart::ocMinTime=capture+SwUsart::baud-SwUsart::ocNextTolerance;	// min time to be stable
	SwUsart::ocNextTolerance>>=1;
}

// shift rxByte when no level change happens
ISR(TIMER1_COMPA_vect){
	OCR1A=OCR1A+SwUsart::ocNextTolerance+OC1_BAUD;
	if (SwUsart::rxBitCnt>8){
		TCCR1B &= ~(1<<ICES1); 
		TIMSK1 &= ~(1<<OCIE1A);								// trigger for start bit
		SwUsart::rxBitCnt=0x00;
// ??? check frame time
// rxFlags|=RX_FRAME_ERROR_I;								// frame timing error
		if (!SwUsart::rxLevel){
			if (SwUsart::rxRegister) 
				SwUsart::rxFlags|=RX_ERROR_STOP;			// Stopbit expected
			else
				SwUsart::rxFlags|=RX_ERROR_BREAK;			// got BREAK?
		}
		if (SwUsart::rxFlags&RX_BIT_ERROR_I) {
			SwUsart::rxFlags|=RX_ERROR_TIMING;				// bit timing error
			SwUsart::rxFlags&=(~RX_BIT_ERROR_I);
		} else {											// got valid data
			uint8_t i =(SwUsart::rx_buffer_head + 1) % SW_USART_RX_BUF_SIZE;
			if (i != SwUsart::rx_buffer_tail) {
				SwUsart::rx_buffer[SwUsart::rx_buffer_head] = SwUsart::rxRegister;
				SwUsart::rx_buffer_head = i;
			}
		}
		SwUsart::rxRegister=0;
	} else {
		if (SwUsart::rxBitCnt) {							// not start bit
			SwUsart::rxRegister=(SwUsart::rxRegister>>1);
			if (SwUsart::rxLevel) SwUsart::rxRegister|=0x80;
		}
		SwUsart::rxBitCnt++;								// next bit
		SwUsart::ocMinTime+=(OC1_BAUD-SwUsart::ocNextTolerance);
		SwUsart::ocNextTolerance>>=1;
	}
}

//-- TX write ------------------------------
void SwUsart::write(uint8_t c) {
	if (SwUsart::tx_buffer_head==SwUsart::tx_buffer_tail && SwUsart::txWriteDone) {
		SwUsart::txWriteDone=false;
		SwUsart::txRegister=c;
		SwUsart::txBitCnt=8;
		cli();          									// buffer empty start right now
		TCCR1A=(TCCR1A&(~((1<<COM1B1)|(1<<COM1B0))))|(1<<COM1B1); // Startbit to 0
		TCCR1C=(1<<FOC1B);
		OCR1B=TCNT1+OC1_BAUD-2;								// 2: is just estimated
		if (SwUsart::txRegister&0x01) {
			TCCR1A|=((1<<COM1B0)|(1<<COM1B1));				// bit 0 to 1
		}
		TIFR1=(1<<OCF1B);
		TIMSK1|=(1<<OCIE1B);
		SwUsart::txRegister=SwUsart::txRegister>>1;    
		sei();
	} else {
		uint8_t pNext=(SwUsart::tx_buffer_tail +1) % SW_USART_TX_BUF_SIZE;
		while (pNext==SwUsart::tx_buffer_head);				// wait for space in write buffer
		SwUsart::tx_buffer[pNext] =c;
		SwUsart::tx_buffer_tail=pNext;
	}
}

//-- TX send ------------------------------
ISR(TIMER1_COMPB_vect){  
	OCR1B+=OC1_BAUD;
	if (SwUsart::txBitCnt){													// send txRegister
		if (SwUsart::txBitCnt!=1){
			if (!(SwUsart::txRegister&0x01)) {
				TCCR1A=(TCCR1A&(~((1<<COM1B1)|(1<<COM1B0))))|(1<<COM1B1);	// bit to 0
			} else {
				TCCR1A|=((1<<COM1B0)|(1<<COM1B1));							// bit to 1
			}
			SwUsart::txRegister=SwUsart::txRegister>>1;
		} else {
			TCCR1A|=((1<<COM1B0)|(1<<COM1B1));								// Stop Bit
		}
		SwUsart::txBitCnt--;
	} else {                
		if (SwUsart::tx_buffer_head != SwUsart::tx_buffer_tail) {			// next Byte
			TCCR1A=(TCCR1A&(~((1<<COM1B1)|(1<<COM1B0))))|(1<<COM1B1);		// Startbit to 0
			SwUsart::tx_buffer_head = (SwUsart::tx_buffer_head +1) % SW_USART_TX_BUF_SIZE;
			SwUsart::txRegister = SwUsart::tx_buffer[SwUsart::tx_buffer_head];
			SwUsart::txBitCnt=9;											// start + 8 bit
		} else {															// done
			TIMSK1 &=~(1<<OCIE1B);
			SwUsart::txWriteDone=true;
		}
	}
}
