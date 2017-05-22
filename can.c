/*
 * File:   can.c
 * Author: Ian Hogg
 *
 * Created on 19 February 2016, 15:17
 */

/**
 * Functionality for the ECAN peripheral.
 */

#include "xc.h"
#include "can.h"
#include "dma.h"

extern void initDMA();
extern void faultOn(unsigned char no);
extern void faultOff(unsigned char no);

#define FCAN FCY        // Can clock frequency
#define BITRATE 125000  // CAN Bit rate
#define NTQ 16          // Number of Time Quanta per bit
#define BRP_VAL ((FCAN/(2*NTQ*BITRATE))-1)

#define MIN_TX_FIFO_BUFFER  1
#define MAX_TX_FIFO_BUFFER  7


// forward declarations
void setC1TRCONbit(unsigned int buf, unsigned int bit);
void clearC1TRCONbit(unsigned int buf, unsigned int bit);
unsigned int testC1TRCONbit(unsigned int buf, unsigned int bit);

ECAN1MSGBUF ecan1MsgBuf __attribute__((space(dma), aligned(ECAN1_MSG_BUF_LENGTH*16)));

int nextBufferToQueue;
int lastBufferTransmitted;

/**
 * Initialise the ECAN1 module for CBUS.
 */
inline void initECAN() {
    // set up the clock and timing parameters
    C1CTRL1bits.REQOP = 4;              // INIT mode
    while (C1CTRL1bits.OPMODE != 4) ;   // wait for INIT mode
    C1CFG1bits.BRP = BRP_VAL;           // bit rate pre-scalar
    C1CFG2bits.SEG1PH = 3;              // Phase 1 is 4 x TQ
    C1CFG2bits.SEG2PHTS = 1;            // Segment 2 is programmable
    C1CFG2bits.SEG2PH = 3;              // Segment 2 is 4 x TQ
    C1CFG2bits.PRSEG = 6;               // Propagation segment is 7 x TQ
    C1CFG2bits.SAM = 1;                 // 3 samples
    C1CFG1bits.SJW = 0;                 // Step jump width of 1
    
    C1CTRL2bits.DNCNT = 0;              // No deviceNET (unfortunately)
    C1CTRL1bits.CSIDL = 1;              // Stop in idle
    
    C1CTRL1bits.WIN = 1;                // Access the filters
    {        
        // config the masks
        C1RXM0SIDbits.SID = 0x0;            // Don't care about SID bits
        C1RXM0SIDbits.MIDE = 1;             // Only standard addresses
    
        // config the filters
        C1FEN1 = 0;                         // disable all for now
        
        // Filter 0 is for RTR messages
        C1FMSKSEL1bits.F0MSK = 0;           // Filter 0 uses Mask 0
        C1RXF0SIDbits.SID = 0;              // Filter 0 match any SID
        C1RXF0SIDbits.EXIDE = 0;            // Filter 0 doesn't use Extended addressing
        C1BUFPNT1bits.F0BP = 0;             // Filter 0 matches are Tx RTR response Buffer 0
        C1FEN1bits.FLTEN0 = 1;              // Enable Filter 0
    
        // Filter 1 is for normal messages 
        C1FMSKSEL1bits.F1MSK = 0;           // Filter 1 uses Mask 0
        C1RXF1SIDbits.SID = 0;              // Filter 1 match any SID
        C1RXF1SIDbits.EXIDE = 0;            // Filter 1 doesn't use Extended addressing
        C1BUFPNT1bits.F1BP = 15;            // Filter 1 matches go into Rx FIFO
        C1FEN1bits.FLTEN1 = 1;              // Enable Filter 1
    
        C1CTRL1bits.WIN = 0;                // Access the buffers
    } 
    // set up the buffers
    // We use a FIFO of 24 buffers to receive messages using FILTER0.
    // 7 buffers are used for Tx
    // 1 buffer is used for an auto remote response
    //
    // 0    Auto remote response
    // 1..7 Tx buffer with FIFO
    // 8..31    Rx FIFO buffer
    //
    C1FCTRLbits.DMABS = 6;          // 32 buffers in DMA RAM
    C1FCTRLbits.FSA = 8;            // FIFO area starts at buffer 8
    //C1FCTRL = 0xC8;               // THIS DOES NOT WORK - arrgghh

    // Set up buffer 0 as a High priority RTR TX buffer
    setC1TRCONbit(0, (RTREN|TXEN|3));
    
    // Make the buffers of the FIFO TX buffers
    int b;
    for (b=MIN_TX_FIFO_BUFFER; b<=MAX_TX_FIFO_BUFFER; b++) {
        setC1TRCONbit(b, TXEN);
    }
    // make sure RX FIFO buffers are released and available for reuse
    for (b=8; b<32; b++) {
        releaseRxMessageBuffer(b);
    }
    
    nextBufferToQueue = MAX_TX_FIFO_BUFFER;
    lastBufferTransmitted = MAX_TX_FIFO_BUFFER+1;
    
    initDMA();

    C1CTRL1bits.REQOP = 0;              // NORMAL mode
    while (C1CTRL1bits.OPMODE != 0) ;   // wait for NORMAL mode
    
    // Now set up the pins
    TRISBbits.TRISB4 = 1;           // Pin 11 set to input
    RPINR26bits.C1RXR = 4;       // ECAN1 RX set to RP4 pin (pin 11)
    RPOR2bits.RP5R = 0x10;  // Pin 14 (RP5) set to C1TX

    IEC2bits.C1IE = 1;      //Enable ECAN1 interrupts for CPU
    //IEC4bits.C1TXIE = 1;    //Enable ECAN1 interrupts for CPU
    C1INTEbits.TBIE = 1;    // Enable TX buffer interrupt for DMA module
    //IEC2bits.C1RXIE = 1;    //Enable ECAN1 interrupts for CPU
    C1INTEbits.RBIE = 1;    // Enable RX buffer interrupt for DMA module
}
/**
 * The ECAN1 ISR. Service the ECAN module and enable any waiting
 * buffer for transmission.
 */
void _ISR _C1Interrupt(void) {
    if (C1INTFbits.TBIF) {
        // Interrupt due to a transmit completion
        unsigned int icode = C1VECbits.ICODE;
        if ((icode >= MIN_TX_FIFO_BUFFER) && (icode <= MAX_TX_FIFO_BUFFER)) {
            lastBufferTransmitted = icode;
            // check if it was the last
            if (lastBufferTransmitted == MIN_TX_FIFO_BUFFER) {
                // request for the ones already in buffers to be sent
                int b;
                for (b = MAX_TX_FIFO_BUFFER; b > nextBufferToQueue; b--) {
                    setC1TRCONbit(b, TXREQ);    // request send
                }
            }
        }
        C1INTFbits.TBIF = 0;        // clear the TX buffer interrupt flag
        //IFS4bits.C1TXIF = 0;
    }
    if (C1INTFbits.RBIF) {
        // Interrupt due to a receive buffer
        // no need to do anything
        C1INTFbits.RBIF = 0;        // clear the RX buffer interrupt flag
        //IFS2bits.C1RXIF = 0;
    }
    IFS2bits.C1IF = 0;          // Clear the ECAN interrupt flag
}

/**
 * Queue a CAN message ready for transmission.
 * @param sid
 * @param rtr
 * @param dlc
 * @param b0
 * @param b1
 * @param b2
 * @param b3
 * @param b4
 * @param b5
 * @param b6
 * @param b7
 */
inline int queueForTransmit(unsigned int sid, unsigned int rtr, unsigned int dlc,
        unsigned int b0, unsigned int b1, unsigned int b2, unsigned int b3,
        unsigned int b4, unsigned int b5, unsigned int b6, unsigned int b7) {
    // is the next buffer available?
    if (testC1TRCONbit(nextBufferToQueue, TXREQ)) {
        // not available
        led = LED_FLASH_ON;
        return 1;
    }
    
    // write the data
    setTxBuffer(nextBufferToQueue, sid, rtr, dlc, b0, b1, b2, b3, b4, b5, b6, b7);
    
    // if possible set it ready to transmit
    if ((nextBufferToQueue <= lastBufferTransmitted) || 
            (lastBufferTransmitted == MIN_TX_FIFO_BUFFER)) {
        setC1TRCONbit(nextBufferToQueue, TXREQ);
    }
    
    // move the index ready for next request
    nextBufferToQueue--;
    if (nextBufferToQueue < MIN_TX_FIFO_BUFFER) {
        nextBufferToQueue = MAX_TX_FIFO_BUFFER;
    }
    return 0;    // success
}

/**
 * Write the data into the specified transmit buffer.
 * @param buf
 * @param sid
 * @param rtr
 * @param dlc
 * @param b0
 * @param b1
 * @param b2
 * @param b3
 * @param b4
 * @param b5
 * @param b6
 * @param b7
 */
inline void setTxBuffer(unsigned int buf, 
        unsigned int sid, unsigned int rtr, unsigned int dlc, 
        unsigned int b0, unsigned int b1, unsigned int b2, unsigned int b3,
        unsigned int b4, unsigned int b5, unsigned int b6, unsigned int b7) {
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
    // The RTR is actually stored in SRR or standard frames
    ecan1MsgBuf[buf][0] = (sid << 2) | (rtr << 1);
    ecan1MsgBuf[buf][1] = 0;
    ecan1MsgBuf[buf][2] = dlc;
    ecan1MsgBuf[buf][3] = ((b1 << 8) | b0);
    ecan1MsgBuf[buf][4] = ((b3 << 8) | b2);
    ecan1MsgBuf[buf][5] = ((b5 << 8) | b4);
    ecan1MsgBuf[buf][6] = ((b7 << 8) | b6);
    ecan1MsgBuf[buf][7] = 0;
}


/**
 * Get the buffer number of the next received message.
 * @return  -1 if no message available or the buffer number

 */
inline int getRxMessageBufferNumber() {
    unsigned int buf;
    // get the pointer
    buf = C1FIFObits.FNRB;
    // Is there data available in the buffer?
    if (buf < 16) {
        if (C1RXFUL1 & (1 << (buf))) {
            return buf;
        }
    } else {
        if (C1RXFUL2 & (1 << (buf-16))) {
            return buf;
        }
    }
    return -1;
}

/** 
 * Indicate we have finished with the RX buffer. 
 */
inline void releaseRxMessageBuffer(unsigned int buf) {
    if (buf < 16) {
        C1RXFUL1 &= ~(1 << (buf));
    } else {
        C1RXFUL2 &= ~(1 << (buf-16));
    }
}

/**
 * Extract the SID from a message buffer
 * @param buf
 * @return 
 */
inline unsigned int getSID(unsigned int buf) {
    return (ecan1MsgBuf[buf][0] >> 2) & 0x7ff;
}

/**
 * Extract the DLC from a message buffer
 * @param buf
 * @return 
 */
inline unsigned int getDLC(unsigned int buf) {
    return ecan1MsgBuf[buf][2] & 0x0F;
}

/**
 * Get the SRR bit from a message buffer
 * @param buf
 * @return 
 */
inline unsigned int getSRR(unsigned int buf) {
    return (ecan1MsgBuf[buf][0]) & 0x02;
}

/**
 * Get the RTR bit from a message buffer
 * @param buf
 * @return 
 */
inline unsigned int getRTR(unsigned int buf) {
    return (ecan1MsgBuf[buf][2]) & 0x200;
}

/**
 * Helper method to set a bit within the ECAN1 Transmit control registers.
 * @param buf
 * @param bit
 */
inline void setC1TRCONbit(unsigned int buf, unsigned int bit) {
    switch(buf) {
        case 0:
            C1TR01CON |= (1 << bit);
            break;
        case 1:
            C1TR01CON |= (1 << (bit+8));
            break;
        case 2:
            C1TR23CON |= (1 << bit);
            break;
        case 3:
            C1TR23CON |= (1 << (bit+8));
            break;
        case 4:
            C1TR45CON |= (1 << bit);
            break;
        case 5:
            C1TR45CON |= (1 << (bit+8));
            break;
        case 6:
            C1TR67CON |= (1 << bit);
            break;
        case 7:
            C1TR67CON |= (1 << (bit+8));
            break;
    }
}

/**
 * Helper method to clear a bit within the ECAN1 Transmit control registers.
 * @param buf
 * @param bit
 */
inline void clearC1TRCONbit(unsigned int buf, unsigned int bit) {
    switch(buf) {
        case 0:
            C1TR01CON &= ~(1 << bit);
            break;
        case 1:
            C1TR01CON &= ~(1 << (bit+8));
            break;
        case 2:
            C1TR23CON &= ~(1 << bit);
            break;
        case 3:
            C1TR23CON &= ~(1 << (bit+8));
            break;
        case 4:
            C1TR45CON &= ~(1 << bit);
            break;
        case 5:
            C1TR45CON &= ~(1 << (bit+8));
            break;
        case 6:
            C1TR67CON &= ~(1 << bit);
            break;
        case 7:
            C1TR67CON &= ~(1 << (bit+8));
            break;
    }
}

/**
 * Helper method to test a bit within the ECAN1 Transmit control registers.
 * @param buf
 * @param bit
 */
unsigned int testC1TRCONbit(unsigned int buf, unsigned int bit) {
    switch(buf) {
        case 0:
            return (C1TR01CON & (1 << bit));
        case 1:
            return (C1TR01CON & (1 << (bit+8)));
        case 2:
            return (C1TR23CON & (1 << bit));
        case 3:
            return (C1TR23CON & (1 << (bit+8)));
        case 4:
            return (C1TR45CON & (1 << bit));
        case 5:
            return (C1TR45CON & (1 << (bit+8)));
        case 6:
            return (C1TR67CON & (1 << bit));
        case 7:
            return (C1TR67CON & (1 << (bit+8)));
    }
    return 0;
}
