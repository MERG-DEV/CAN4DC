/* 
 * File:   throttleVar.h
 * Author: 	Ian Hogg
 * Comments:	Variables associated with the thottles
 * Revision history: 
 */

#ifndef THROTTLEVAR_H
#define	THROTTLEVAR_H

#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */

#include <xc.h> 

#define NUM_READINGS        5   // number of readings to keep

typedef struct {
    volatile char currentSpeed;
    char requestedSpeed;
    char acceleration;
    int accelCount;
    char cored;
    char currentCored;
    unsigned char readings[NUM_READINGS];
    char nextReadNo;
    char bemff;
} ThrottleVar;

#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	/* THROTTLEVAR_H */

