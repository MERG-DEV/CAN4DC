/*
 * File:   dma.c
 * Author: Ian Hogg
 *
 * Created on 21 February 2016, 12:04
 */

/**
 * DMA functionality. All done by the hardware so just initialise the DMA peripheral.
 */

#include "xc.h"
#include "can.h"

inline void initDMA() {
    // set up the DMA controller for ECAN
    // Channel 0 is used to transmit data
    DMACS0 = 0;         // clear collision flags
    
    DMA0CONbits.CHEN = 0;   // disabled for now
    DMA0CONbits.SIZE = 0;   // word
    DMA0CONbits.DIR = 1;    // to peripheral
    DMA0CONbits.HALF = 0;   // interrupt on complete
    DMA0CONbits.NULLW = 0;
    DMA0CONbits.AMODE = 2;  //Peripheral indirect addressing mode
    DMA0CONbits.MODE = 0;   // Continuous and no ping-pong
    
    DMA0REQbits.FORCE = 0;  // Automatic
    DMA0REQbits.IRQSEL = 0x46;  // ECAN1 TX Data Request
    
    DMA0STA = __builtin_dmaoffset(&ecan1MsgBuf);
    DMA0STB = __builtin_dmaoffset(&ecan1MsgBuf);
    DMA0PAD = (volatile unsigned int)&C1TXD;   // ECAN1 TXD register
    DMA0CNT = 7;        // 8 words
    //
    //
    // Channel 1 is used to receive data
    DMACS0 = 0;         // clear collision flags

    DMA1CONbits.CHEN = 0;   // disabled for now
    DMA1CONbits.SIZE = 0;   // word
    DMA1CONbits.DIR = 0;    // from peripheral
    DMA1CONbits.HALF = 0;   // interrupt on complete
    DMA1CONbits.NULLW = 0;
    DMA1CONbits.AMODE = 2;  //Peripheral indirect addressing mode
    DMA1CONbits.MODE = 0;   // Continuous and no ping-pong
    
    DMA1REQbits.FORCE = 0;  // Automatic
    DMA1REQbits.IRQSEL = 0x22;  // ECAN1 RX Data Request
    
    DMA1STA = __builtin_dmaoffset(&ecan1MsgBuf);
    DMA1STB = __builtin_dmaoffset(&ecan1MsgBuf);
    DMA1PAD = (volatile unsigned int)&C1RXD;   // ECAN1 RXD register
    DMA1CNT = 7;        // 8 words
    
    // Enable both channels
    DMA0CONbits.CHEN = 1;   // enable channel
    DMA1CONbits.CHEN = 1;   // enable channel
}
