/***************************************************************
    RingBuff.c
	ver. 2.0 : 2007.12.31
	by Dae-Woong Chung
****************************************************************/
#include "RingBuff.h"

#define ERR_INVALID_CMD 	0x01
#define ERR_INVALID_OFFSET	0x02

#define BUFFER_COUNT 30		// minimum 18 for SCI (16 byte comm + 1 byte feedback + 1 byte margin)

char ezDSP_ringBuffer[BUFFER_COUNT];
int ezDSP_nStartPos=0, ezDSP_nEndPos=0;
int ezDSP_nMaxBuffSize = 0;

void ResetRing(void) {
	int i;
	for(i=0; i < BUFFER_COUNT; i++)
		ezDSP_ringBuffer[i] = 0;

	ezDSP_nStartPos = ezDSP_nEndPos = 0;
}

void AddRing(char y) {
	int nBuffSize;

	ezDSP_ringBuffer[ezDSP_nEndPos]	= y;
	ezDSP_nEndPos++;
	if(ezDSP_nEndPos == BUFFER_COUNT) ezDSP_nEndPos = 0;

	// check if buffer is full
	if(ezDSP_nStartPos == ezDSP_nEndPos) {
		ezDSP_nStartPos++;
		if(ezDSP_nStartPos == BUFFER_COUNT) ezDSP_nStartPos = 0;
	}
	// check maximum buff size
	else {
		if(ezDSP_nEndPos >= ezDSP_nStartPos) 
			nBuffSize =  ezDSP_nEndPos - ezDSP_nStartPos;
		else 
			nBuffSize = BUFFER_COUNT + ezDSP_nEndPos - ezDSP_nStartPos;

		if(nBuffSize > ezDSP_nMaxBuffSize) 
			ezDSP_nMaxBuffSize = nBuffSize;
	}
}

char ExtractRing(void) {
	char rv;

	rv = ezDSP_ringBuffer[ezDSP_nStartPos++];
	if(ezDSP_nStartPos == BUFFER_COUNT) ezDSP_nStartPos = 0;

	return rv;
}

char ReadRing(int offset) {
	int pos;

	if(offset >= GetSizeRing()) {
		return 0; // invalid data
	}
	pos = ezDSP_nStartPos + offset;
	if(pos >= BUFFER_COUNT) pos -= BUFFER_COUNT;
	return ezDSP_ringBuffer[pos];
}

int GetSizeRing(void) {
	if(ezDSP_nEndPos >= ezDSP_nStartPos) 
		return ezDSP_nEndPos - ezDSP_nStartPos;
	else 
		return BUFFER_COUNT + ezDSP_nEndPos - ezDSP_nStartPos;
}

int IsRingEmpty(void)
{
	return ezDSP_nEndPos == ezDSP_nStartPos;
}
/*--------------------------------------------------------------------------*/

