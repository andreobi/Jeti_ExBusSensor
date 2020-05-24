/*
 * This lib provide a software USART upto 19200 baud. (depending on the cpu speed)
 * The lib uses timer 1 input capture and output compare functions and is interrupt based.
 * The baud rate determenes the maximum blocking time for interrupts.
 * The advantage is, that the cpu can do other things while sending or receiving serial data.
 
*/

#ifndef SW_USART_H
#define SW_USART_H

#include <Arduino.h>


#if  F_CPU==16000000L
							// at 16MHz 1667=9600Baud
#define OC1_BAUD 1667

#elif F_CPU==8000000L
							// at 8MHz 1667/2=9600Baud
#define OC1_BAUD 1667/2

#else 
#error ("ONLY_defined_for_8_or_16MHz")

#endif


#define RX_PULL_UP
#define SW_USART_RX_BUF_SIZE 8
#define SW_USART_TX_BUF_SIZE 4

//#define RX_FRAME_ERROR_I	0x20
#define RX_BIT_ERROR_I		0x10
#define RX_ERROR_BITS		0x07
// Flags will be returnd by getError
#define RX_ERROR_TIMING		0x04
#define RX_ERROR_STOP		0x02
#define RX_ERROR_BREAK		0x01

class SwUsart {
			SwUsart(void);
public:
static	void 	start(void);
static	int8_t 	available(void);
static	void	write(uint8_t c);
static	int16_t	read(void);
static	uint8_t getError(void);
//
static	uint8_t rxLevel;
static	uint8_t rxRegister;
static	uint8_t rxBitCnt;
static	uint8_t rxFlags;
//
static	uint16_t ocMinTime;
static	uint16_t baud;
static	uint16_t ocNextTolerance;
static	uint8_t rx_buffer_head;
static	uint8_t rx_buffer_tail;
static	uint8_t rx_buffer[SW_USART_RX_BUF_SIZE];
//
static	uint8_t txRegister;
static	uint8_t txBitCnt;
static	uint8_t tx_buffer[SW_USART_TX_BUF_SIZE];
static	uint8_t tx_buffer_tail;
static	volatile uint8_t	tx_buffer_head;
static	volatile bool 		txWriteDone;
};

#endif