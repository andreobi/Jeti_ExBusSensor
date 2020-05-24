
#include "ao_Parser.h"

Parser::Parser(void){
	headerTable=0;
	cntLines=0;
	cntChar=0;
}

Parser::Parser(const char * headerTab){
	setTable(headerTab);
}


Parser::~Parser(void){
// remove the comments if you destroy the Parser object and don't want to call resetTable
//	delete [] hp;
//	delete [] inBuffer;	
}


void Parser::setTable(const char *headerTab){
	headerTable=headerTab;
	cntChar=pgm_read_byte_near(headerTable++);					// read table size 
	cntLines=pgm_read_byte_near(headerTable++);

	cntChar=(cntChar<IN_BUFFER_LENGTH?cntChar:IN_BUFFER_LENGTH); // any limit?
	inBuffer=new char[cntChar];									//reserve memory
	memset(inBuffer,0,cntChar);
	hp=new uint8_t[cntLines];
	memset(hp,0,cntLines);
}

void Parser::resetTable(void){
	delete [] hp;
	delete [] inBuffer;	
}


uint8_t Parser::parse(char c){
    uint8_t j=cntChar-1;										// shift buffer 
    do {
      inBuffer[j]=inBuffer[j-1];
    } while (--j);
    inBuffer[0]=c;

	for(uint8_t i=0; i<cntLines; i++){
	uint8_t idx=i*cntChar;
	if(c==pgm_read_byte_near(headerTable+idx+hp[i])){			// does c match header(position)
		hp[i]++;
		if (pgm_read_byte_near(headerTable+idx+hp[i])=='\0'){	// header end

			memset(hp,0,cntLines);								// set all hp to start
			return i+1;
		}
	} else {													// no match
		bool found=false;
		uint8_t bp=(hp[i]<cntChar?hp[i]:cntChar-1);				// limit step back to the maximum menory length and dont find the same again
		hp[i]=0;
		while(bp && !found){									// reached memory depth 0 then skip
			bool sync=true;
			uint8_t sbp=bp;
			while (sbp && sync){
				if (inBuffer[sbp-1]==pgm_read_byte_near(headerTable+idx+hp[i])){
					sbp--;
					hp[i]++;
				} else {
					sync=false;
					hp[i]=0;
				}
			}
		found=sync;
		bp--;													// next test is memory depth -1
		}
	}
}
return 0;
}

void Parser::getString(uint8_t headerNum, char *buffer){
	if (!cntLines || !cntChar) {
		buffer[0]='\0';
	} else {
		uint8_t idx=(headerNum-1)*cntChar;
		uint8_t i=0;
		char c;
		do {
			c=pgm_read_byte_near(headerTable+idx+i);
			buffer[i]=c;
			i++;
		} while (c);
	}
}

