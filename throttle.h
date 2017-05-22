/* 
 * File:   throttle.h
 * Author: 	Ian Hogg
 * Comments:	Header file for the throttle functionality
 * Revision history: 
 */

#ifndef THROTTLE_H
#define	THROTTLE_H

#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */

#include "xc.h"

#define NUM_THROTTLES       4

typedef struct {
    volatile unsigned int *port;
    volatile unsigned int *tris;
    unsigned char bit;
    unsigned char remappable;
    unsigned char analogueBit;
} PinConfig;

typedef struct {
    unsigned char pwmNumber;
    PinConfig alarm;
    PinConfig outputA;
    PinConfig outputB;
    PinConfig sense;
} ThrottleConfig;

extern void initThrottles();
extern void stop(unsigned char no);
extern void faultOff(unsigned char no);
extern void faultOn(unsigned char no);
extern void setCurrentSpeed(unsigned char no, char s, unsigned char cored);
extern char getCurrentSpeed(unsigned char no);
extern void setRequiredSpeed(unsigned char no, char s, unsigned char cored, char accel);
extern unsigned char getSpeed(unsigned char no);
extern unsigned char getCored(unsigned char no);
extern unsigned char getAccel(unsigned char no);
extern void stop(unsigned char no);
extern void stopAll();
extern void synch();
extern void serviceAccel(unsigned char no);
extern void initTimer2();
extern void initTimer3();


#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	/* THROTTLE_H */

