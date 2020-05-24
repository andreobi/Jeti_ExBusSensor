
#include "ao_Eeprom.h"

// wait until done and read
uint8_t Eeprom::read(uint16_t address){
	while(EECR & (1<<EEPE));				// Wait for completion of previous write
	EEAR = address;
	EECR |= (1<<EERE);						// Start eeprom read
	return EEDR;
}

// return false if busy
bool Eeprom::write(uint16_t address, uint8_t  data){
	if (EECR & (1<<EEPE)) return false;		// eeprom busy return false
	EEAR = address;
	EEDR = data;
	EECR = 0x00;							// erase and write
	char cSREG;								// disable interrupts during timed sequence
	cSREG = SREG;
	cli();
	EECR |= (1<<EEMPE);						// start EEPROM write
	EECR |= (1<<EEPE);
	SREG = cSREG;
	return true;
}

// try to avoid an earase cycle
bool Eeprom::update(uint16_t address, uint8_t  data){
	if (EECR & (1<<EEPE)) return false;		// eeprom busy return false
	EEAR = address;
	EECR |= (1<<EERE);						// Start eeprom read
	uint8_t eed=EEDR;
	if (eed==data) return true;				// data is already in eeprom
	EEDR = data;
	if ((eed&data)==data){
		EECR = (1<<EEPM1);					// write only
	}else{						
		EECR = 0x00;						// erase and write
	}
	char cSREG;								// disable interrupts during timed sequence
	cSREG = SREG;
	cli();
	EECR |= (1<<EEMPE);						// start EEPROM write
	EECR |= (1<<EEPE);
	SREG = cSREG;
	return true;
}

// erase address
bool Eeprom::erase(uint16_t address){
	if (EECR & (1<<EEPE)) return false;		// eeprom busy return false
	EEAR = address;
	EEDR = 0xff;
	EECR = (1<<EEPM0);						// erase and write
	char cSREG;								// disable interrupts during timed sequence
	cSREG = SREG;
	cli();
	EECR |= (1<<EEMPE);						// start EEPROM erase
	EECR |= (1<<EEPE);
	SREG = cSREG;
	return true;
}
