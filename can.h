/* 
 * File:    can.h
 * Author: 	Ian Hogg
 * Comments:	Header file for the CAN interface
 * Revision history: 
 */

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef XC_CAN_H
#define	XC_CAN_H

#include <xc.h> // include processor files - each processor file is guarded.  
#include "main.h"

#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */

#define ECAN1_MSG_BUF_LENGTH    32
typedef unsigned int ECAN1MSGBUF[ECAN1_MSG_BUF_LENGTH][8];
    
extern ECAN1MSGBUF ecan1MsgBuf __attribute__((space(dma), aligned(ECAN1_MSG_BUF_LENGTH*16)));

// bits in the C1TRmnCON register
#define TXEN    7
#define TXABT   6
#define TXLARB  5
#define TXERR   4
#define TXREQ   3
#define RTREN   2

    
    
extern void initECAN();

extern int queueForTransmit(unsigned int sid, unsigned int rtr, unsigned int dlc,
    unsigned int b0, unsigned int b1, unsigned int b2, unsigned int b3,
    unsigned int b4, unsigned int b5, unsigned int b6, unsigned int b7);

extern void setTxBuffer(unsigned int buf, 
    unsigned int sid, unsigned int rtr, unsigned int dlc,
    unsigned int b0, unsigned int b1, unsigned int b2, unsigned int b3,
    unsigned int b4, unsigned int b5, unsigned int b6, unsigned int b7);

extern int getRxMessageBufferNumber();
extern void releaseRxMessageBuffer(unsigned int buf);
extern unsigned int getSID(unsigned int buf);
extern unsigned int getDLC(unsigned int buf);
extern unsigned int getSRR(unsigned int buf);
extern unsigned int getRTR(unsigned int buf);
extern void setC1TRCONbit(unsigned int buf, unsigned int bit);

#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	/* XC_CAN_H */

