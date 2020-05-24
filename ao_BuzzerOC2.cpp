
#include "ao_BuzzerOC2.h"

#define BUZZER_FREQUENCE  62

//-----------------------
#if  F_CPU==16000000L
#define BUZZER_2KHZ  (1<<CS22)
//-----------------------
#elif F_CPU==8000000L
#define BUZZER_2KHZ  (1<<CS21)|(1<<CS20)
//-----------------------
#else 
 #error ("ONLY_defined_for_8_or_16MHz")
#endif

void BuzzerOC2::start(void){
	DDRD|=(1<<PD3);
	TIMSK2=0;
	TCCR2B=BUZZER_2KHZ;								      // 128 or 64 prescaler => 125 kHz input clock
	TCCR2A=(1<<COM2B0)|(1<<WGM21);					// toggle Output
	OCR2A=BUZZER_FREQUENCE;
	OCR2B=3;
}

void BuzzerOC2::buzzerOC2set(bool o){
	if (o)
		TCCR2A=(1<<COM2B0)|(1<<WGM21);				// toggle Output
	else
		TCCR2A=(1<<COM2B1)|(1<<WGM21);	      // clear pin
}
