/*
 * This LIB provides a lightweight implementation which only works in master mode and 
 * has a simplyfied API. Only the Master Mode is implemented.
 * 
 * the lib has no own buffer! The provided buffer must be valid until the job is done!
 * Typicly it is no challenge, because the lib get the buffer pointer and starts
 * to read and return when done. The returned value is the real number of read bytes.
 * 
 * For sending data the default mode (setSendMode(true)) waits until all data are send and 
 * the return value reprensents the status. 
 * 
 * The non blocking send mode (setWaitMode(false)) will return as soon the job is started.
 * The return value represent the status at that time. An error can also happen later on. 
 * The buffer memory must be valid until getSendActive() returns false.
 * 
 * first create an Object like:
 * WireLight Wire;
 * 
 * in the init part call:
 * Wire.begin();
 * 
 * to change the bus speed call:
 * Wire.setClock(xxx); // in Hz
 * 
 * send data 
 * Wire.sendTo(chipAdr, regAdr, *txBuffer, length, stop);
 * chipAdr      is the chip's bus address
 * regAdr		is the register address
 * txBuffer[0..n] is the buffer pointer 
 * length       is the buffer length, max 254
 * stop			sends an I2C Stop after send is done (default:true)
 * 
 * Wire.requestFrom(chipAdr, regAdr, *rxBuffer, length, adrStop, stop);
 * chipAdr      is the chip's bus address
 * regAdr       is the register address
 * rxBuffer     is the buffer pointer
 * length       is the buffer length
 * adrStop		sends an I2C STOP after the register address (default:true)
 * stop			sends an I2C STOP after read is done (default:true)
 * 
 * There is no beginTransmisson, endTransmission ...
 
 This lib is derived from:
 
   twi.c - TWI/I2C library for Wiring & Arduino
  Copyright (c) 2006 Nicholas Zambetti.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

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

#ifndef AO_WIRE_LIGHT_H
#define AO_WIRE_LIGHT_H

#include "Arduino.h"

class WireLight {
  public:
    WireLight(void);
    void	begin(void);
	void	setClock(uint32_t speed);
	void	setWaitMode(uint8_t mode);
	bool	getSendActive(void);
	uint8_t	getError(void);
    uint8_t	sendTo(uint8_t chipAdr, uint8_t regAdr, uint8_t *txBuffer, uint8_t length=1, bool stop=true);	// max length =254!!!
	uint8_t	requestFrom(uint8_t chipAdr, uint8_t regAdr, uint8_t *rxBuffer, uint8_t length=1, bool adrStop=true, bool stop=true);
//
    static	void 	twStop(void);
	static volatile uint8_t twState;			// will be set by ISR statemachine
	static volatile uint8_t twSlaRW;			// chipAdr + R/W Bit
	static volatile uint8_t twSendStop;			// keep bus, send no Stop
	static volatile uint8_t twNoStart;			// already bus owner, send no Start
	static volatile uint8_t twWait;				// wait until send is done
	static volatile uint8_t twError;			// Error
	static volatile uint8_t twRegAdr;			// register Address
	static volatile uint8_t *twBuffer;			// Pointer to data buffer 
	static volatile uint8_t twBufferIndex;		// current position in buffer
	static volatile uint8_t twBufferLength;		// buffer length
};
#endif
