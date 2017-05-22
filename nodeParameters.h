/* 
 * File:   nodeParameters.h
 * Author: 	Ian Hogg
 * Comments:	Definitions required for Node Parameters
 * Revision history: 
 */

#ifndef XC_NODE_PARAMETERS_H
#define	XC_NODE_PARAMETERS_H

#include <xc.h> 
#include "cbusdefs8j.h"

#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */

typedef struct {
    unsigned char manufacturer;
    char minor_version;
    unsigned char module_id;
    unsigned char no_events;
    unsigned char no_event_vars_per_event;
    unsigned char no_node_variables;
    unsigned char major_version;
    unsigned char flags;
    unsigned char processor_id;
    unsigned char interface_protocol;
    void * load_address;    // a void * is 16 bits so this is needed for a 4 byte address
    unsigned int pad1;
    long manufacturer_processor_code;
    unsigned char manufacturer_code;
    unsigned char beta_release_flag;
    unsigned char unused[4];
    unsigned int number_of_parameters;
    char * module_name_address;
    unsigned int pad2;          // a char* is 16 bits so this is needed for 4 byte address
    unsigned int checksum;
} Params;

extern const Params params;
extern char module_name[];

extern unsigned int getNodeParam(unsigned int index);
extern void initNP();


#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	/* XC_NODE_PARAMETERS_H */

