/*
 * This lib provides a framework and some views for the JetiBox menu.
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

#ifndef AO_JBMENUE_H
#define AO_JBMENUE_H

#include <Arduino.h>
#include "ao_JetiExBus.h"
#include <avr/pgmspace.h>

#define CONF_MODE_TURN_OFF		0x80	/* value==0: turn Off 		*/
#define CONF_MODE_SHOW_ONLY		0x40	/* value can't be changed, must be set for mode 6	*/
#define CONF_MODE_DECIMALS		0x30	/* decimal dot				*/
#define CONF_MODE_SIGNED		0x08	/* value is signed int		*/
#define CONF_MODE_PRINT_METH	0x07	/* Print method				*/
#define CONF_MODE_PRINT_U16		0x00	/* uint16 */
#define CONF_MODE_PRINT_UD16	0x01	/* uint16 value decimal*/
#define CONF_MODE_PRINT_ID16	0x02	/* int16 showValue,  int16 value decimal*/
#define CONF_MODE_PRINT_BINARY	0x03	/* binary format plus 3 unit character @mini, maxi, inc*/
#define CONF_MODE_PRINT_xx4		0x04	/*free*/
#define CONF_MODE_PRINT_O_O		0x05	/* ON / OFF^*/
#define CONF_MODE_PRINT_INFO	0x06	/* INFO Line + 3 char unit[3]*/
#define CONF_MODE_PRINT_xx7		0x07	/*free*/


extern JetiExBus    exBus;          // Jeti EX Bus object

class JBMenue;
class JBMenueSel;
class JBMFactory;
class JBMInterface;


// the factory knows the next menue level and delivers a JBMenue object this could be 
// - JBMenue 		to select a function block and jbmObject[..] points to the next level factories
// - JBMenueSel		edit an array of values and jbmObject[0] points to the <function> interface
// - or something class <function> : public JBMenue specific
union JBMBase{
	JBMFactory 		*f;		// for a menue structure whithout the intention to change values
	JBMInterface	*i;		// for a menue leafes to change values
};

class JBMFactory {
public:
	virtual JBMenue *getNewMenue(void)=0;
	virtual void	deleteNewMenue(uint8_t numJbmObA);		// 1: interface, 2: factory ...
protected:
	JBMBase  		*jbmObA;
};

// derived interface needs to be a friend of the class <function> 
class JBMInterface {
public:
	JBMInterface(void);
	virtual void	JBMGetParameter(JBMenueSel *jbms, uint8_t num);	// needs to be overriden to set specific values
	virtual void	JBMRetParameter(uint16_t value, uint8_t num);	// could be used to retrieve new value
	virtual void 	JBMEnd(void);
};


class JBMenue {
public:
	JBMenue(void);
	JBMenue(JBMBase *jbmOb, const char * const* menueTab, uint8_t m_len);
	virtual bool 		JBMDo(uint8_t key);
	virtual void 		JBMShow(void);
protected:
	const char *const	*menueTable;				// JBMenueSel has no Header
	JBMBase				*jbmObject;				// pointer table for the next menue job
	uint8_t				menueTarget;				// selected menue tab
	uint8_t 			menueLength;				// number of tabs 
	JBMenue				*subMenue;					// pointer to sub menue level
};
//---------------------------------------------------------------------------
// standard menue leafe to modify and show content
class JBMenueSel : public JBMenue {
public:
	JBMenueSel(void);
	JBMenueSel(JBMBase *jbmOb, const char * const* menueTab, uint8_t m_len);
	virtual bool 	JBMDo(uint8_t key);		// overriden to JBMSelValue(key)
	virtual void 	JBMShow(void);			// overriden to JBMSelValue(0)

// these values must be set according to JBMGetParameter(...)
	union {
	const char *jbmInfoStr; // Pointer to progmem string for info message
	struct{
	uint16_t	showValue;	// could be displayed as a reference value 
	uint16_t	value;		// value to be modified by inc
	}; };
	uint16_t	mini;		// minimum for value
	uint16_t	maxi;		// maximum for value
	uint16_t	inc;		// increment or decrement for value
	uint8_t		confMode;	// see CONF_MODE_...
	char		unit[3];	// contains the unit or 3 status character position are mini, maxi, inc
//----------
// could be overridden to modify the repesentaion of the second line
public:
	virtual void	JBMSelShowLine1_7(char *line, uint8_t num);		// free

private:
	void		JBMSelShowLine1_0(char *line);	// value: 0..65536‬ unit
	void		JBMSelShowLine1_1(char *line);	// showValue  value 0..9999 +decimal unit
	void		JBMSelShowLine1_2(char *line);	// showValue  value -9999..9999 +decimal unit
	void		JBMSelShowLine1_3(char *line);	// free
	void		JBMSelShowLine1_4(char *line);
	void		JBMSelShowLine1_5(char *line);	// ON / OFF
	void		JBMSelShowLine1_6(char *line);	// info string: jbmInfoStr + unit@mini +unit@maxi +..

	void		JBMprint(char *buf, uint16_t out);		// needs 5 character
	uint8_t		line1Active;	
};
//---------------------------------------------------------------------------

#endif