/*
 * File:   persist.c
 * Author: Ian Hogg
 *
 * Created on 21 January 2016, 20:50
 */
 
/**
 * Funcationality associated with persisting the node variables to Flash
 */

#include "xc.h"
#include "persist.h"
#include "sense.h"
#include <libpic30.h>
#include "throttle.h"
#include "cbus.h"

#define FLASH_ROWS_PER_PAGE   _FLASH_PAGE/_FLASH_ROW

#define NV_PERSIST_ADDRESS	0x800	// first page of usable program flash
_prog_addressT flashStore;			// FlashStore is the in-flash storage
unsigned int flashSpace[_FLASH_PAGE] __attribute__((space(prog), aligned(_FLASH_PAGE*2),address(NV_PERSIST_ADDRESS)));

unsigned int persist[_FLASH_ROW];	// persist is the RAM cache of the Flash data


// forward references
void readFlash();
void writeFlash();

/**
 * Initialise the persisted data. If the persisted data isn't valid then set to defaults.
 */
void initPersist() {
    
    _init_prog_address(flashStore, flashSpace);
    
    readFlash();
    if (persist[FLASH_INDEX_KEY] != FLASH_KEY) {
    	// flash didn't contain valid data
	// put the defaults into RAM
	persist[FLASH_INDEX_KEY]                = FLASH_KEY;
        persist[FLASH_INDEX_CANID]              = DEFAULT_CANID;
        persist[FLASH_INDEX_NODEID]             = DEFAULT_NODEID;
        persist[FLASH_INDEX_CORELESS_FREQUENCY] = DEFAULT_CORELESS_FREQUENCY;
        persist[FLASH_INDEX_CORED_FREQUENCY]    = DEFAULT_CORED_FREQUENCY;
        persist[FLASH_INDEX_ALARM_THRESHOLD]    = DEFAULT_ALARM_THRESHOLD;
        persist[FLASH_INDEX_ALARM_CUTOFF_TIME]  = DEFAULT_ALARM_CUTOFF_TIME;
        persist[FLASH_INDEX_TOTI_THRESHOLD]     = DEFAULT_TOTI_THRESHOLD;
        persist[FLASH_INDEX_NUMBER_TO_AVERAGE]  = DEFAULT_NUMBER_TO_AVERAGE;
        persist[FLASH_INDEX_MIN_CORELESS]       = DEFAULT_MIN_CORELESS_MEASURE;
        persist[FLASH_INDEX_MIN_CORED]          = DEFAULT_MIN_CORED_MEASURE;
	// save to flash
        writeFlash();
    }
}

/**
 * Set the CanId.
 */
void setCanId(unsigned int id) {
    persist[FLASH_INDEX_CANID] = id;
    writeFlash();
}

/**
 * Set the NodeId.
 */
void setNN(unsigned int id) {
    persist[FLASH_INDEX_NODEID] = id;
    writeFlash();
}

/**
 * Read the Flash and put into persist RAM cache.
 */
void readFlash() {
    // work back through the rows looking for the latest valid row
    int i;
    for (i=FLASH_ROWS_PER_PAGE-1; i>=0; i--) {
        _memcpy_p2d16((int *)persist, flashStore+2*(i*_FLASH_ROW), 2*(FLASH_MAX_INDEX+1));
        if (persist[FLASH_INDEX_KEY] == FLASH_KEY) return;
    }
    // Didn't find a valid row - will use defaults instead
}

/**
 * Write the persist RAM cache to Flash.
 * This uses all the rows in the page before erasing the page and starting at first row again.
 */
void writeFlash() {
    // find an empty row to store the latest copy
    int i;
    unsigned int key;
    for (i=0; i< FLASH_ROWS_PER_PAGE; i++) {
        _memcpy_p2d16((int *)(&key), flashStore+2*(i*_FLASH_ROW), 2); // get the key
        if (key == 0xFFFF) {
            // found an empty row    
            _write_flash16(flashStore+2*(i*_FLASH_ROW), (int *)persist);		// all the data fits into a single Flash row
            return;
        }
    }
    // no empty row so erase and store at first row
    _erase_flash(flashStore);			// erase the entire flash page
    _write_flash16(flashStore, (int *)persist);		// all the data fits into a single Flash row
}
