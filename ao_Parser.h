/*
 * This lib provides a stream parser with memory, that means in case of a mismatch 
 * the parser goes back in its memory and tries to find the sync point.
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
#ifndef AO_PARSER_H
#define AO_PARSER_H

#include "Arduino.h"

#define IN_BUFFER_LENGTH 15
// unused charater mus be filled, \0 determined the end of the search string
/* example Header table
const char GPS_Header[] PROGMEM = {
	6,4,						// string length and number of lines
  'G' ,'P' ,'R' ,'M' ,'C' ,'\0',
  'G' ,'N' ,'R' ,'M' ,'C' ,'\0',
  'G' ,'P' ,'G' ,'G' ,'A' ,'\0',
  'G' ,'N' ,'G' ,'G' ,'A' ,'\0',
};
*/

class Parser{
private:
	const char * headerTable;
	uint8_t	cntLines;				// 
	uint8_t cntChar;
	char 	*inBuffer;				// pointer to memory
	uint8_t *hp;					// curent header position

public:
	Parser(void);
	Parser(const char * headerTab);
	~Parser(void);
	void	setTable(const char * headerTab);
	void	resetTable(void);			// must be called bevor setting another new table or destroying the object 
	void	getString(uint8_t headerNum, char *buffer);
	uint8_t parse(char c);
};

#endif
