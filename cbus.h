/* 
 * File:   cbus.h
 * Author: 	Ian Hogg
 * Comments:	Header file for the CBUS 
 * Revision history: 
 */

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef XC_CBUS_H
#define	XC_CBUS_H

#include <xc.h> 
#include "cbusdefs8j.h"
#include "can.h"

#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */

extern void initCBUS();

extern void startSelfEnumeration();
extern void handleSelfEnumerationResponse(unsigned int buf);
extern int doingSelfEnumeration();
extern void selfEnumerationTimeout();

extern void setAutoRemoteResponse();
extern void cbusTransmit(unsigned int len, unsigned int canid, unsigned int pri,
    unsigned int opc, unsigned int nn, unsigned int en, unsigned int b1,
    unsigned int b2, unsigned int b3);

extern unsigned int getOPC(unsigned int buf);
extern unsigned int getNN(unsigned int buf);
extern unsigned int getEN(unsigned int buf);
extern unsigned int getB1(unsigned int buf);
extern unsigned int getB2(unsigned int buf);
extern unsigned int getB3(unsigned int buf);
    
// OPC codes to handle - splitting these into TX and RX
// makes it easier to spot any mistakes on those for reception
// and those for transmission
#define OPC_RX_TOF      OPC_TOF    // Track off
#define OPC_RX_TON      OPC_TON    // Track on
#define OPC_RX_RESTP    OPC_RESTP  // Request stop
#define OPC_RX_QNN      OPC_QNN    // Query node number
#define OPC_RX_RQNP     OPC_RQNP   // Request Node Parameter
#define OPC_RX_RQMN     OPC_RQMN   // Request Name
#define OPC_RX_SNN      OPC_SNN    // Set Node Number
#define OPC_RX_BOOTM    OPC_BOOT   // Bootloader Mode
#define OPC_RX_ENUM     OPC_ENUM   // Self Enumeration of CANID
#define OPC_RX_NVRD     OPC_NVRD   // Node Variable Read
#define OPC_RX_RQNPN    OPC_RQNPN  // Request Node Parameter by Index
#define OPC_RX_CANID    OPC_CANID  // Set 
#define OPC_RX_ACOF     OPC_ACOF   // zero speed
#define OPC_RX_AREQ     OPC_AREQ   // Request Long event
#define OPC_RX_NVSET    OPC_NVSET  // Set a Node Variable
#define OPC_RX_ACOF1    OPC_ACOF1  // zero speed
#define OPC_RX_ACOF2    OPC_ACOF2  // zero speed
#define OPC_RX_ACON3    OPC_ACON3  // Set speed
#define OPC_RX_ACOF3    OPC_ACOF3  // zero speed
    
#define OPC_TX_ESTOP    OPC_ESTOP  // Reply to RESTP
#define OPC_TX_RQNN     OPC_RQNN   // Request Node Number
#define OPC_TX_NNREL    OPC_NNREL  // Release Node Number
#define OPC_TX_NNACK    OPC_NNACK  // Node Number Acknowledge
#define OPC_TX_WRACK    OPC_WRACK  // Write acknowledge
#define OPC_TX_CMDERR   OPC_CMDERR // Command error
#define OPC_TX_ACON     OPC_ACON   // Long event ON
#define OPC_TX_ACOF     OPC_ACOF   // Long Event OFF
#define OPC_TX_ARON     OPC_ARON   // Response to AREQ with 0 data bytes
#define OPC_TX_AROF     OPC_AROF   // Response to AREQ with 0 data bytes
#define OPC_TX_NVANS    OPC_NVANS  // Response to NVRD
#define OPC_TX_PARAN    OPC_PARAN  // response to RQNPN
#define OPC_TX_PNN      OPC_PNN    // Response to QNN
#define OPC_TX_ARON2    OPC_ARON2  // Long event ON
#define OPC_TX_AROF2    OPC_AROF2  // Long event ON
#define OPC_TX_NAME     OPC_NAME   // Resposne to RQMN
#define OPC_TX_PARAMS   OPC_PARAMS // Response to RQNP
#define OPC_TX_ARON3    OPC_ARON3  // Response to AREQ with 3 data bytes
    
#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	/* XC_CBUS_H */

