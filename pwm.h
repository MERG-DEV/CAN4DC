/* 
 * File:   pwm.h
 * Author: 	Ian Hogg
 * Comments:	Header file for the PWM functionality
 * Revision history: 
 */

#ifndef XC_PWM_H
#define	XC_PWM_H

#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */

#include <xc.h> 

extern void initPWMindexes();
extern unsigned int getCoredPrescalar();
extern unsigned int getCorelessPrescalar();
extern unsigned int getCoredMultiplier();
extern unsigned int getCorelessMultiplier();

extern void setPWMwidth(unsigned char no, char s, unsigned int cored);
extern void initPWM(unsigned char no);
extern void startPWM(unsigned char no, char cored);
extern void stopPWM(unsigned char no);


#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	/* XC_PWM_H */

