/*
 * File:   cbus.c
 * Author: Ian Hogg
 *
 * Created on 19 February 2016, 15:18
 */

/**
 * Functionality for CBUS. Makes use of can.c
 * Hanles self enumeration.
 */
#include "xc.h"
#include "persist.h"
#include "can.h"
#include "cbus.h"

// forward declarations
unsigned int testBit(unsigned int b);
void setBit(unsigned int b);
void setAutoRemoteResponse();

unsigned char usedIds[16];  // 128 bits
int doingEnum;

/**
 * Initialise the CBUS. 
 * Specifically initialise the CAN and the self enumeration.
 */
inline void initCBUS() {
    doingEnum = 0;
    // Set up the auto remote response
    setAutoRemoteResponse();
    initECAN();
}

void cbusTransmit(unsigned int len, unsigned int canid, unsigned int pri, 
        unsigned int opc, unsigned int nn, unsigned int en, 
        unsigned int b1, unsigned int b2, unsigned int b3) {

    queueForTransmit(canid | (pri << 7), 0, len, opc,
        (nn>>8)&0xFF, nn&0xFF, (en>>8)&0xFF, en&0xFF, b1, b2, b3);
}

/**
 * Start the self enumeration process to get a new CANID
 * Send a RTR frame and prepare to collect responses. Set a timeout for 500ms.
 */
void startSelfEnumeration() {
    doingEnum = 10; // acts as counter - decremented every 50ms by Timer5
    
    unsigned int id;
    for (id = 0; id <16; id++) {
        usedIds[id] = 0;
    }
    
    // Transmit a Remote Transmit Request
    // Using SID=0x5FF (CAN_ID=7F, MjPri=2, MnPri=3), RTR=1,DLC=0
    queueForTransmit(0x5FF, 1, 0, 0,0,0,0,0,0,0,0);
}

/**
 * Process the response to the RTR.
 * @param buf
 */
void handleSelfEnumerationResponse(unsigned int buf) {
    if (doingEnum == 0) return;
    unsigned int canid = getSID(buf) & 0x7F;
    setBit(canid);
}

/**
 * Finish the self enumeration process. Determine our CAN_ID, save it and 
 * set up the self enum auto response with the new CAN_ID.
 */
void stopSelfEnumeration() {
    // determine our new CAN_ID
    // start at 1 looking for an used Id
    int id;
    for (id=1; id<100; id++) {
        if (! testBit(id)) break;
    }
    if (id < 100) {
        // success
    
        // Save the CAN_ID
        setCanId(id);   // saves in the persist store
        setAutoRemoteResponse();
        // send ACK
        cbusTransmit(3, persist[FLASH_INDEX_CANID], 11, OPC_TX_NNACK, persist[FLASH_INDEX_NODEID], 0, 0,0,0);

    } else {
        cbusTransmit(4, persist[FLASH_INDEX_CANID], 11, OPC_TX_CMDERR, persist[FLASH_INDEX_NODEID],
                CMDERR_INVALID_EVENT<<8, 0,0,0);
    }
    // Now request a new NN - we should still be in setup mode with LED flashing
    cbusTransmit(3, persist[FLASH_INDEX_CANID], 11, OPC_TX_RQNN, persist[FLASH_INDEX_NODEID], 0, 0,0,0);
}

/**
 * Is a self enumeration process currently running?
 * @return 1 if self enum is in process otherwise 0
 */
inline int doingSelfEnumeration() {
    return doingEnum;
}

/**
 * Call regularly during self enum process to determine if a timeout has occurred
 * and the stopSelfEnumeration needs to be called.
 * @return 
 */
inline void selfEnumerationTimeout() {
    if (doingEnum == 0) return;
    doingEnum--;
    if (doingEnum == 0) stopSelfEnumeration();
    return;
}

/**
 * Helper method to test a CAN_ID in the bit field.
 * @param id
 * @return 
 */
unsigned int testBit(unsigned int id) {
    unsigned int offset = id/8;
    unsigned int bit = id%8;
    unsigned int mask = 1 << bit;
    
    return (usedIds[offset] & mask);
}

/**
 * Helper method to set a CAN_ID in the bit field.
 * @param id
 */
void setBit(unsigned int id) {
    unsigned int offset = id/8;
    unsigned int bit = id%8;
    unsigned int mask = 1 << bit;
    
    usedIds[offset] |= mask;
}

/**
 * Buffer 0 is used as the automatic self enumeration response. This must be
 * kept up to date with the CAN_ID.
 */
void setAutoRemoteResponse() {
    // Buffer 0 is Automatic Remote Response
    // 0x580 sets the MjPri and MnPri
    // DLC = 0
    setTxBuffer(0, persist[FLASH_INDEX_CANID] | 0x580, 0, 0, 0,0,0,0,0,0,0,0);
}

    /* Message buffers:
     * word0 0,0,0,SID<10:0>,SRR,IDE
     * word1 0,0,0,0,EID<17:6>
     * word2 EID<5:0>,RTR,0,0,0,0,0,DLC
     * word3 Message Byte1, Message Byte0
     * word4 Message Byte3, Message Byte2
     * word5 Message Byte5, Message Byte4
     * word6 Message Byte7, Message Byte6
     * word7 0,0,0,FilterHitBits<4:0>, 0,0,0,0,0,0,0,0
     */


/**
 * Get the OPC code from the message.
 * @param buf
 * @return 
 */
inline unsigned int getOPC(unsigned int buf) {
    return ecan1MsgBuf[buf][3] & 0xFF;
} 

/**
 * Get the NN from the message.
 * @param buf
 * @return 
 */
inline unsigned int getNN(unsigned int buf) {
    return ((ecan1MsgBuf[buf][3] & 0xFF00) | (ecan1MsgBuf[buf][4] & 0xFF));
} 

/** Get the EN from the message.
 * @param buf
 * @return 
 */
inline unsigned int getEN(unsigned int buf) {
    return ((ecan1MsgBuf[buf][4] & 0xFF00) | (ecan1MsgBuf[buf][5] & 0xFF));
}

/** Get the additional byte1 from the message.
 * @param buf
 * @return 
 */
inline unsigned int getB1(unsigned int buf) {
    return ((ecan1MsgBuf[buf][5] >> 8) & 0xFF);
}

/** Get the additional byte2 from the message.
 * @param buf
 * @return 
 */
inline unsigned int getB2(unsigned int buf) {
    return ((ecan1MsgBuf[buf][6]) & 0xFF);
}

/** Get the additional byte3 from the message.
 * @param buf
 * @return 
 */
inline unsigned int getB3(unsigned int buf) {
    return ((ecan1MsgBuf[buf][6] >> 8) & 0xFF);
}
