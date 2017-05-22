/* 
 * File:   frequencyConfig.h
 * Author: 
 * Comments:	Structure definition used to convert between PWM frequency
 *		and the multiplier and prescalar.
 * Revision history: 
 */

#ifndef XC_FREQUENCYCONFIG_H
#define	XC_FREQUENCYCONFIG_H

#include <xc.h>

#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */

typedef struct {
    int frequency;
    int multiplier;
    int prescalar;
} FrequencyConfig;

#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	/* XC_FREQUENCYCONFIG_H */

