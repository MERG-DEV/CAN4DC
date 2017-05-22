/* 
 * File:   persist.h
 * Author: 	Ian Hogg
 * Comments:	Header file for persist into flash functionality
 * Revision history: 
 */

#ifndef PERSIST_H
#define	PERSIST_H

#include <xc.h>
#include <libpic30.h>

#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */

extern void writeFlash();


#define FLASH_KEY	0xA4DC	// used to determine if Flash holds valid data

#define FLASH_INDEX_KEY			0
#define FLASH_INDEX_CANID		1
#define FLASH_INDEX_NODEID		2
#define FLASH_INDEX_CORELESS_FREQUENCY	3
#define FLASH_INDEX_CORED_FREQUENCY	4
#define FLASH_INDEX_ALARM_THRESHOLD	5
#define FLASH_INDEX_ALARM_CUTOFF_TIME	6
#define FLASH_INDEX_TOTI_THRESHOLD	7
#define FLASH_INDEX_NUMBER_TO_AVERAGE	8
#define FLASH_INDEX_MIN_CORELESS	9
#define FLASH_INDEX_MIN_CORED		10
#define FLASH_MAX_INDEX		FLASH_INDEX_MIN_CORED

// These are the in-RAM version of the persisted data

extern unsigned int persist[_FLASH_ROW];

// the default values
#define DEFAULT_CORED_FREQUENCY         100 //Hz
#define DEFAULT_CORELESS_FREQUENCY      20000   //Hz
#define DEFAULT_ALARM_THRESHOLD         512
#define DEFAULT_ALARM_CUTOFF_TIME       250 //in fiftyths of second
#define DEFAULT_TOTI_THRESHOLD          16
#define DEFAULT_NUMBER_TO_AVERAGE       5
#define DEFAULT_MIN_CORED_MEASURE       0x12    // below this measurements get unreliable
#define DEFAULT_MIN_CORELESS_MEASURE    0x20    // motor stops above this
#define DEFAULT_CANID 0x7F
#define DEFAULT_NODEID 100

extern void initPersist();
extern void setCanId(unsigned int id);
extern void setNN(unsigned int id);

#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	/* PERSIST_H */

