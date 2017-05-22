/* 
 * File:   main.h
 * Author: 	Ian Hogg
 * Comments:	A variety of definitions needed for the CAN4DC firmware
 * Revision history: 
 */

#ifndef XC_MAIN_H
#define	XC_MAIN_H

#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */

#include <xc.h> // include processor files - each processor file is guarded.  

#define FCY   16000000L  //define your instruction frequency, FCY = FOSC/2

// The start of the EN ranges
#define TOTI_EN         0x1000
#define FAULT_EN        0x2000
#define THROTTLE_EN     0x0000

#define EN_TYPE_MASK    0xF000
    
// Yellow LED states
#define LED_OFF         0
#define LED_FLASH_ON    1
#define LED_FLASH       2
#define LED_FLASH_OFF   3
#define LED_ON          4
extern int led;

#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	/* XC_MAIN_H */

