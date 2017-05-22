/* 
 * File:   nodeVarables.h
 * Author: 	Ian Hogg
 * Comments:	Definitions for NVs
 * Revision history: 
 */

#ifndef XC_NODE_VARIABLES_H
#define	XC_NODE_VARIABLES_H

#include <xc.h> 

#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */

#define NV_CORELESS_FREQ_PARAM_HI   1
#define NV_CORELESS_FREQ_PARAM_LO   2
#define NV_CORED_FREQ_PARAM_HI      3
#define NV_CORED_FREQ_PARAM_LO      4
#define NV_ALARM_THRESHOLD_HI       5
#define NV_ALARM_THRESHOLD_LO       6
#define NV_ALARM_CUTOFF_TIME        7
#define NV_TOTI_THRESHOLD_HI        8
#define NV_TOTI_THRESHOLD_LO        9
#define NV_NUMBER_TO_AVERAGE        10
#define NV_MIN_CORELESS_MEASURE     11
#define NV_MIN_CORED_MEASURE        12

#define NUM_NV  NV_MIN_CORED_MEASURE   // Number of Node Variables

extern unsigned int getNodeVar(unsigned int index);
extern void setNodeVar(unsigned int index, unsigned int value);


#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	/* XC_NODE_VARAIABLES_H */

