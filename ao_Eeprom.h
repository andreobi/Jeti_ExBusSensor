/*
 * This lib provides a non blocking (no waiting) eeprom write routine.
 * The calling program has to retry the writing in case false was returned.
 * The read routine will wait until a write process is done.
 *
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
#ifndef AO_EEPROM_H
#define AO_EEPROM_H
#include <Arduino.h>

class Eeprom{
public:
static	uint8_t read(uint16_t address);
static	bool write(uint16_t address, uint8_t  data);
static	bool update(uint16_t address, uint8_t  data);
static	bool erase(uint16_t address);
};

static	Eeprom AEEPROM;
#endif