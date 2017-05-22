/* 
 * File:   sense.h
 * Author: 	Ian Hogg
 * Comments:	Header file for the current sensing
 * Revision history: 
 */

#ifndef SENSE_H
#define	SENSE_H

#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */

#include <xc.h> 
#include "main.h"

extern void senseT5interrupt();
extern unsigned char getFaultState(unsigned char no);
extern unsigned char getTotiState(unsigned char no);
extern unsigned int getAverageCurrent(unsigned char no);

extern void initADC();

#define CURRENT_READINGS_BUF_SIZE        10   // number of readings to keep

typedef struct {
    unsigned int readings[CURRENT_READINGS_BUF_SIZE];
    unsigned char nextReadingOffset;
    unsigned char faultCount;
    unsigned char toti;
} SenseVar;

#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	/* SENSE_H */

