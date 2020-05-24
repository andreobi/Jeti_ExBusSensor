
#include "ao_JBMenue.h"

extern JBMBase *jbmObAMain;

/*
JBMenue JBMFactory::getNewMenue(void){
  jbmObA = new JBMBase*[JBMIOCS_TAB_LENGTH-1];  // each entry but not the header
  jbmObA[0].f= new JBMFactory_0;
  jbmObA[1].f= new JBMFactory_1;
  return new JBMenue(jbmObA, JBM_IOcs_table, JBMIOCS_TAB_LENGTH);
} */

/*
JBMenue JBMFactory::getNewMenue(void){
  jbmObA = new JBMBase*[1]; 
  jbmObA[0].i= new JBMInterface;
  return new JBMenueSel(jbmObA, JBM_IOcs_table, JBMIOCS_TAB_LENGTH);
} */

void	JBMFactory::deleteNewMenue(uint8_t numJbmObA){
}

//---------------------------------------------------------------------------
JBMInterface::JBMInterface(){
}

// needs to set the Menue parameter for the selected tab
void	JBMInterface::JBMGetParameter(JBMenueSel *jbms, uint8_t num){
//	jbms->value=paratmeter[num];
// and other
}

// could be used to retrieve new value
void	JBMInterface::JBMRetParameter(uint16_t value, uint8_t num){
	// paratmeter[num] = value
}

void 	JBMInterface::JBMEnd(void){		// save data to EEPROM
}

//----------------------------------------------------------------------------
JBMenue::JBMenue(void) :subMenue(nullptr), menueTarget(0) {
}

JBMenue::JBMenue(JBMBase *jbmOb, const char * const* menueTab, uint8_t m_len) :
	subMenue(nullptr), menueTarget(0), jbmObject(jbmOb), menueTable(menueTab), menueLength(m_len) {
}

bool 	JBMenue::JBMDo(uint8_t key){
	if (subMenue) {						// call next sub menue
		if (subMenue->JBMDo(key)){		// was UP Key pressed?
			delete subMenue;
			subMenue=nullptr;
			JBMShow();					// show own menue
		}
	} else {							// no submenue: handel input
		if (key&JB_K_R && menueTarget<menueLength-2){
			menueTarget++;
		} else if(key&JB_K_L && menueTarget>0) {
			menueTarget--;
		} else if(key&JB_K_U) {
			if (jbmObAMain != jbmObject){			// don't delete the main menue
				uint8_t numJbmObA=menueLength-1;	// clean up factory store
				do {
					delete jbmObject[--numJbmObA].f;
				} while (numJbmObA);
				delete [] jbmObject;
			}
			return true;		// signal upper menue the end of sub menue
		} else if(key&JB_K_D) {
			if (jbmObject[menueTarget].f) {
				subMenue=(jbmObject[menueTarget].f)->getNewMenue();
				subMenue->JBMDo(0);					// get parameter and show
				return false;
			} 
		}
		JBMShow();
	}
	return false;
}


void 	JBMenue::JBMShow(void){	// show menue
	char line[18];
	line[0]=(menueTarget>0 ? '<' : ' ');
	line[1]=' ';
	strcpy_P(&line[2],(char *)pgm_read_word(&(menueTable[menueTarget+1]))); // +1 because of Headder
	bool padding=false;
	for (uint8_t i=2; i<15; i++){
		if (line[i]=='\0') padding =true;
		if (padding)line[i]=' ';
	}
	line[15]=(menueTarget<menueLength-2 ? '>' : ' ');
	line[16]='\0';

	exBus.setJetiboxText(1, line);
	strcpy_P(line,(char *)pgm_read_word(&(menueTable[0])));
	line[16]='\0';
	exBus.setJetiboxText(0, line);
}


//---------------------------------------------------------------------------

JBMenueSel::JBMenueSel(void) {
}


JBMenueSel::JBMenueSel(JBMBase *jbmOb, const char * const* menueTab, uint8_t m_len) :
	JBMenue(jbmOb, menueTab, m_len),
	line1Active(false), confMode(0), mini(0),maxi(1), inc(1), unit{' ',' ',' '} {
}


bool 	JBMenueSel::JBMDo(uint8_t key){
	if ((!line1Active||menueLength<2)&&(key&JB_K_U)) {
		(jbmObject[0].i)->JBMEnd();			// store value to EEPROM
		delete jbmObject[0].i;
		delete [] jbmObject;
		return true;						// signal upper menue the end of sub menue
	} 
	(jbmObject[0].i)->JBMGetParameter(this, menueTarget);	// get value[num] & parameter[num]
	if (confMode&CONF_MODE_SIGNED){
		value+=0x8000;mini+=0x8000;maxi+=0x8000;
	}
	if (menueLength<2) line1Active=true;	// no selection on line 0 possible: goto line1
	if (line1Active&&key&JB_K_U) {
		line1Active=false;					// JB buttons change line 0
	} else	if(key&JB_K_D) {
		if (!(confMode&CONF_MODE_SHOW_ONLY))// only if there is something to change
			line1Active=true;				// JB buttons change line 1
	} else if (line1Active){				// line1: change selected Value
// key&JB_K_LP
		uint8_t keyRepeat=0;
		
		if (key&JB_K_VLP)
			keyRepeat=10;		// key is pressed increase speed by 10
		do{
			if (key&JB_K_R && value<maxi){
				if (value<mini) {
					value=mini;
				} else {
					if (value<maxi-inc)
						value+=inc;
					else
					value=maxi;
				}
			} else if(key&JB_K_L){
				if (value>=inc) value-=inc;
				if (confMode&CONF_MODE_TURN_OFF) {
					if (value <mini) value=0;
				} else {
					if (value <mini) value=mini;
				}
			}
		} while(keyRepeat--);
//		
	} else {								// line0: select Value to change
		if (key&JB_K_R && menueTarget<menueLength-1)
			menueTarget++;
		else if(key&JB_K_L && menueTarget>0)
			menueTarget--;
		(jbmObject[0].i)->JBMGetParameter(this, menueTarget);		// get value & confMode parameter (num)
		if (confMode&CONF_MODE_SIGNED){
			value+=0x8000;mini+=0x8000;maxi+=0x8000;
		}
	}
	if (!(confMode&CONF_MODE_SHOW_ONLY)){
		if (confMode&CONF_MODE_TURN_OFF) {
			if (value <mini) value=0;
		} else {
			if (value <mini) value=mini;
		}
		if (value>maxi) value=maxi;
	}
	
	if (confMode&CONF_MODE_SIGNED){
		value+=0x8000;mini+=0x8000;maxi+=0x8000;
	}
	(jbmObject[0].i)->JBMRetParameter(value, menueTarget);		// could be used to retrieve new value

	JBMShow();
	return false;
}


void 	JBMenueSel::JBMShow(void){
// create Display content line 0
	char line[18];
	strcpy_P(&line[2],(char *)pgm_read_word(&(menueTable[menueTarget])));
	bool padding=false;
	for (uint8_t i=2; i<15; i++){
		if (line[i]=='\0') padding =true;
		if (padding) line[i]=' ';
	}
	line[1]=' ';
	if (!line1Active){
		line[0]=(menueTarget>0 ? '<' : ' ');
		line[15]=(menueTarget<menueLength-1 ? '>' : ' ');
	} else {
		line[0]=' ';
		line[15]=' ';
	}
	line[16]='\0';
	exBus.setJetiboxText(0, line);
// create Display content line 1
	for(uint8_t i=0; i<16; i++) line[i]=' ';
	if (line1Active){
		if (confMode&CONF_MODE_TURN_OFF)
			line[0]=(value>=mini ? '<':' ');
		else
			line[0]=(value>mini ? '<':' ');
		
		line[15]=(value<maxi ? '>':' ');
	}

	switch (confMode&CONF_MODE_PRINT_METH) {
	case 0:{
		JBMSelShowLine1_0(line);
		break;}
	case 1:{
		JBMSelShowLine1_1(line);
		break;}
	case 2:{
		JBMSelShowLine1_2(line);
		break;}
	case 3:{
		JBMSelShowLine1_3(line);
		break;}
	case 4:{
		JBMSelShowLine1_4(line);
		break;}
	case 5:{
		JBMSelShowLine1_5(line);
		break;}
	case 6:{
		JBMSelShowLine1_6(line);
		break;}
	default: {
		JBMSelShowLine1_7(line, menueTarget);}
	}

	line[16]='\0';
	exBus.setJetiboxText(1, line);
}

// show uint16 with OFF option
void	JBMenueSel::JBMSelShowLine1_0(char *line){
	if (value || (!(confMode&CONF_MODE_TURN_OFF))) {
		if (value>=10000) line[5]='0'+(value/10000)%10;
		if (value>=1000) line[6]='0'+(value/1000)%10;
		if (value>=100) line[7]='0'+(value/100)%10;
		if (value>=10) line[8]='0'+(value/10)%10;
		line[9]='0'+value%10;
		line[11]=unit[0];
		line[12]=unit[1];
		line[13]=unit[2];
	} else {
		line[6]='O';
		line[7]='F';
		line[8]='F';
	}
}

// show 1 value and 3 unit character
void	JBMenueSel::JBMSelShowLine1_1(char *line){
	JBMprint(line+4, value);
	line[12]=unit[0];
	line[13]=unit[1];
	line[14]=unit[2];
}

// show 2 values and 2 unit character
void	JBMenueSel::JBMSelShowLine1_2(char *line){
	JBMprint(line+1, showValue);
	JBMprint(line+7, value);
	if (confMode&CONF_MODE_SHOW_ONLY){
		line[mini&0x000F]=unit[0];					// set char @ position(mini)
		line[maxi&0x000F]=unit[1];
	} else {
		line[13]=unit[0];
		line[14]=unit[1];
//		line[15]=unit[2];
	}
}

// show value as a binary bit format and place the 3 unit charater into it
void	JBMenueSel::JBMSelShowLine1_3(char *line){
	uint8_t offset=0;
	for(uint8_t i=2; i<8+2; i++){
		if (i+offset==mini) offset++;
		if (i+offset==maxi) offset++;
		if (i+offset==inc)  offset++;
		line[i+offset]=(value&(1<<(7-(i-2))) ? '1' : '0');
	}
	line[mini&0x000F]=unit[0];					// set char @ position(mini)
	line[maxi&0x000F]=unit[1];
	line[inc &0x000F]=unit[2];	
}

void	JBMenueSel::JBMSelShowLine1_4(char *line){

}

// show ON / OFF option
void	JBMenueSel::JBMSelShowLine1_5(char *line){
    if (value){
		line[ 9]='O';
		line[10]='N';
		line[11]=' ';
	} else {
		line[ 4]='O';
		line[ 5]='F';
		line[ 6]='F';
	}
}

// show an info string and place 3 unit character into the string
void	JBMenueSel::JBMSelShowLine1_6(char *line){
	char c;
	uint8_t i=0;										// copy info string
	do{
		c= pgm_read_byte_near(jbmInfoStr + i);
		if (c) line[i]=c;
		i++;
	} while (c && (i <=16));
	line[mini&0x000F]=unit[0];							// set char @ position(mini)
	line[maxi&0x000F]=unit[1];
	line[inc &0x000F]=unit[2];
}

// could be overridden to modify the repesentaion of value
void	JBMenueSel::JBMSelShowLine1_7(char *line, uint8_t num){

}


// needs 6 character, prints unsinged integer to buffer
// no minus sign if value >= 10000
void	JBMenueSel::JBMprint(char *buf, uint16_t out){
	if (confMode&CONF_MODE_SIGNED){
		if (((int16_t)out)<0){
			out*=-1;
			buf[0]='-';
		}
	}
	switch ((confMode&CONF_MODE_DECIMALS)>>4) {
	case 0:{
		if (out>=10000) buf[1]='0'+(out/10000)%10;
		if (out>=1000) buf[2]='0'+(out/1000)%10;
		if (out>=100)  buf[3]='0'+(out/100)%10;
		if (out>=10)   buf[4]='0'+(out/10)%10;
		buf[5]='0'+out%10;
		break;}
	case 1:{
		if (out>=10000) buf[0]='0'+(out/10000)%10;
		if (out>=1000) buf[1]='0'+(out/1000)%10;
		if (out>=100)  buf[2]='0'+(out/100)%10;
		buf[3]='0'+(out/10)%10;
		buf[4]='.';
		buf[5]='0'+out%10;
		break;}
	case 2:{
		if (out>=10000) buf[0]='0'+(out/10000)%10;
		if (out>=1000) buf[1]='0'+(out/1000)%10;
		buf[2]='0'+(out/100)%10;
		buf[3]='.';
		buf[4]='0'+(out/10)%10;
		buf[5]='0'+out%10;
		break;}
	case 3:{
		if (out>=10000) buf[0]='0'+(out/10000)%10;
		buf[1]='0'+(out/1000)%10;
		buf[2]='.';
		buf[3]='0'+(out/100)%10;
		buf[4]='0'+(out/10)%10;
		buf[5]='0'+out%10;
		break;}
	}
}
