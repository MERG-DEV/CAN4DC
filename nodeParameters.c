/*
 * File:   nodeVariables.c
 * Author: Ian Hogg
 *
 * Created on 05 March 2016, 18:14
 */

/**
 * Handle the node parameters. Most of the functionality specified in the parameters specification is supported.
 * Unfortunately we cannot lay these out in the HEX file as required by FCU.
 */
#include "xc.h"
#include <libpic30.h>
#include "cbusdefs8j.h"
#include "nodeParameters.h"
#include "nodeVariables.h"

#define NUM_PV  PAR_BETA   // Number of Node Parameters

#define MANUFACTURER    MANU_MERG
#define MODULE          47
#define MJ_VERSION      0x02
#define MN_VERSION      'b'

#define DEVID_ADDRESS 0xFF0000
#define DEVREV_ADDRESS 0xFF0002

extern void main(void);
// Locate at 0x0810
char module_name[] = {'4', 'D', 'C',' ', ' ',' ', ' '};


// These must be placed at 0x0820 with no zeros between bytes.
// Can't do that on a PIC24 so this isn't used for now...
//__pack_upper_byte
const Params params = {
    MANUFACTURER,   // manufacturer
    MN_VERSION,     // minor version
    MODULE,         // module id
    0,              // number of events
    0,              // number of event variable per event
    NUM_NV,         // number of node variables
    MJ_VERSION,     // Major version
    (PF_COMBI | PF_FLiM /*| PF_BOOT*/),     // flags
    0,              // Processor Id - none assigned to PIC24HJ128GP502
    PB_CAN,         // Interface protocol
    (void *)(&main),//  load address
    0,              // padding
    0x067D,         // processor code
    CPUM_MICROCHIP, // manufacturer code
    0,              // beta release flag
    {0,0,0,0},      // unused
    NUM_PV,         // number of parameters
    module_name,    // name string address   = 0x0810
    0,              // padding
    (MANUFACTURER + MN_VERSION + MODULE + NUM_NV + MJ_VERSION + 
            (PF_COMBI | PF_FLiM /*| PF_BOOT*/)+
            PB_CAN + 0x06 + 0x7D + CPUM_MICROCHIP + NUM_PV + 0x08 + 0x10) // checksum 
};

unsigned int devid;
unsigned int devrev;

/**
 * Initialise the Node Parameters.
 */
inline void initNP() {
    _prog_addressT src;
    src=DEVID_ADDRESS;
    _memcpy_p2d16((char *)&devid, src, 2);
    src=DEVREV_ADDRESS;
    _memcpy_p2d16((char *)&devrev, src, 2);
}

/**
 * Get a node parameter.
 */
inline unsigned int getNodeParam(unsigned int index) {
    switch(index) {
        case 0:
            return NUM_PV;
        case PAR_MANU:
            return params.manufacturer;
        case PAR_MINVER:
            return params.minor_version;
        case PAR_MTYP:
            return params.module_id;
        case PAR_EVTNUM:
            return params.no_events;
        case PAR_EVNUM:
            return params.no_event_vars_per_event;
        case PAR_NVNUM:
            return params.no_node_variables;
        case PAR_MAJVER:
            return params.major_version;
        case PAR_FLAGS:
            return PF_NOEVENTS | PF_COMBI	| PF_FLiM | PF_BOOT;
        case PAR_CPUID:
            return params.processor_id;
        case PAR_BUSTYPE:
            return params.interface_protocol;
        case PAR_LOAD:
            return ((unsigned long)(params.load_address))&0xFF;
        case (PAR_LOAD+1):
            return (((unsigned long)(params.load_address)) >> 8) &0xFF;
        case (PAR_LOAD+2):
            return (((unsigned long)(params.load_address)) >> 16) &0xFF;
        case (PAR_LOAD+3):
            return (((unsigned long)(params.load_address)) >> 24) &0xFF;
        case PAR_CPUMID:
            return devid & 0xFF; 
        case (PAR_CPUMID+1):
            return (devid>>8) &0xFF;
        case (PAR_CPUMID+2):
            return devrev & 0xFF; 
        case (PAR_CPUMID+3):
            return (devrev>>8) &0xFF;
        case PAR_CPUMAN:
            return params.manufacturer_code;
        case PAR_BETA:
            return params.beta_release_flag;
    }
    return 0;
}


