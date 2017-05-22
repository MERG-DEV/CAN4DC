/*
 * File:   main.c
 * Author: Ian Hogg
 *
 * Created on 02 January 2016, 10:26
 */
/** TODOs
 * Bootloader
 * Node parameter checksum with main()
 * Persist NVs
 * Location of NVs in hex file
 */

/**
 *	The Main CAN4DC program.
 */


// PIC24HJ128GP502 Configuration Bit Settings

// 'C' source line config statements

#include <xc.h>

// FBS
#pragma config BWRP = WRPROTECT_OFF     // Boot Segment Write Protect (Boot Segment may be written)
#pragma config BSS = NO_FLASH           // Boot Segment Program Flash Code Protection (No Boot program Flash segment)
#pragma config RBS = NO_RAM             // Boot Segment RAM Protection (No Boot RAM)

// FSS
#pragma config SWRP = WRPROTECT_OFF     // Secure Segment Program Write Protect (Secure segment may be written)
#pragma config SSS = NO_FLASH           // Secure Segment Program Flash Code Protection (No Secure Segment)
#pragma config RSS = NO_RAM             // Secure Segment Data RAM Protection (No Secure RAM)

// FGS
#pragma config GWRP = OFF               // General Code Segment Write Protect (User program memory is not write-protected)
#pragma config GSS = OFF                // General Segment Code Protection (User program memory is not code-protected)

// FOSCSEL
#pragma config FNOSC = PRIPLL           // Oscillator Mode (Primary Oscillator (XT, HS, EC) with PLL)
#pragma config IESO = ON                // Internal External Switch Over Mode (Start-up device with FRC, then automatically switch to user-selected oscillator source when ready)

// FOSC
#pragma config POSCMD = XT              // Primary Oscillator Source (XT Oscillator Mode)
#pragma config OSCIOFNC = OFF           // OSC2 Pin Function (OSC2 pin has clock out function)
//#pragma config POSCMD = EC              // Primary Oscillator Source (XT Oscillator Mode)
//#pragma config OSCIOFNC = ON           // OSC2 Pin Function (OSC2 pin has clock out function)
#pragma config IOL1WAY = OFF            // Peripheral Pin Select Configuration (Disallow Only One Re-configuration)
#pragma config FCKSM = CSECME           // Clock Switching and Monitor (Both Clock Switching and Fail-Safe Clock Monitor are enabled)

// FWDT
#pragma config WDTPOST = PS32768        // Watchdog Timer Postscaler (1:32,768)
#pragma config WDTPRE = PR128           // WDT Prescaler (1:128)
#pragma config WINDIS = OFF             // Watchdog Timer Window (Watchdog Timer in Non-Window mode)
#pragma config FWDTEN = OFF             // Watchdog Timer Enable (Watchdog timer enabled/disabled by user software)

// FPOR
#pragma config FPWRT = PWR128           // POR Timer Value (128ms)
#pragma config ALTI2C = OFF             // Alternate I2C  pins (I2C mapped to SDA1/SCL1 pins)

// FICD
#pragma config ICS = PGD1               // Comm Channel Select (Communicate on PGC1/EMUC1 and PGD1/EMUD1)
#pragma config JTAGEN = OFF             // JTAG Port Enable (JTAG is Disabled)



#include "cbusdefs8j.h"
#include <libpic30.h>
#include "main.h"
#include "persist.h"
#include "throttle.h"
#include "can.h"
#include "cbus.h"
#include "sense.h"
#include "throttle.h"
#include "nodeVariables.h"
#include "nodeParameters.h"

#define DELAY 500

  
#define CYCLES_PER_MS ((unsigned long)(FCY * 0.001))        //instruction cycles per millisecond
#define CYCLES_PER_US ((unsigned long)(FCY * 0.000001))   //instruction cycles per microsecond
#define DELAY_MS(ms)  __delay32(CYCLES_PER_MS * ((unsigned long) ms));   //__delay32 is provided by the compiler, delay some # of milliseconds
 
#define DELAY_US(us)  __delay32(CYCLES_PER_US * ((unsigned long) us));    //delay some number of microseconds

// Modes
#define MODE_FLIM   0
#define MODE_SLIM   1
#define MODE_SETUP  2
#define MODE_LEARN  3

// Time PB must be pressed for
#define PB_TIME 100     // about 2 seconds

// forward declarations
void initialise();
void initTimer5();
void ledOn();
void ledOff();
void serviceLED();
void servicePBstart();
void servicePBstop();
void processCommand(unsigned int buf);

int mode = MODE_FLIM;
int led = LED_OFF;
int ledCounter = 0;    // used for led flashing timings
int pbCounter = 0;      // used to time how long PB has been pressed
unsigned char cored = 0;


/**
 * It is all run from here.
 * Initialise everything and then loop receiving and processing CAN messages.
 */
int main(void) {
    initialise();
    
    unsigned int l;
    for (l=0; l<4; l++) {
        faultOn(l);
        DELAY_MS(1000);
        faultOff(l);
    }

    while (1) {
        int buf;

        
        buf = getRxMessageBufferNumber();
        if (buf >= 0) {
 //           cbusTransmit(8, persist[FLASH_INDEX_CANID], 11, 0xfc, persist[FLASH_INDEX_NODEID],
 //                       C1RXFUL1, (C1RXFUL2 >> 8),(C1RXFUL2&0xff),C1FIFObits.FNRB);
 //           cbusTransmit(8, persist[FLASH_INDEX_CANID], 11, 0xfd, persist[FLASH_INDEX_NODEID],
 //                       C1RXOVF1, (C1RXOVF2 >> 8),(C1RXOVF2&0xff),C1FIFObits.FNRB);
            // get the CBUS message and process it
            // check to see if this is an empty message - used for self enumeration
            if (getDLC(buf) == 0) {
                // check the SRR
                if (getSRR(buf)) {
                    // Another node is doing self Enum so we need to reply
                    // with our canId.
                    // This should be done automatically but the automatic response
                    // doesn't seem to work with SRR - only RTR
                    // Transmit buffer 0
                    setC1TRCONbit(0, TXREQ);
                }
                // check the RTR
                if (getRTR(buf)) {
                    // This probably isn't needed and may be done automatically
                    setC1TRCONbit(0, TXREQ);
                }
                // response to RTR self enumerate
                handleSelfEnumerationResponse(buf);
            } else {
                processCommand(buf);
            }

            releaseRxMessageBuffer(buf);
        }
    } // main loop
    return 0;
} // main

/**
 * Handle each of the CBUS messages from the CAN interface.
 */
inline void processCommand(unsigned int buf) {      
    unsigned int opc = getOPC(buf);
 
    //cbusTransmit(8, persist.canId, 11, opc, 0, 0, 0,0,0);
    // first check for commands without any arguments
    switch(opc) {
        case OPC_RX_TON:      // Track on. We use to synchronise PWM phase between nodes.
            synch();
            return;
        case OPC_RX_TOF:      // Track off
        case OPC_RX_RESTP:    // Request stop
            stopAll();  // we don't send a ESTOP response as there would be lots
            return;
        case OPC_RX_QNN:      // Query node number
            cbusTransmit(6, persist[FLASH_INDEX_CANID], 11, OPC_TX_PNN, persist[FLASH_INDEX_NODEID], 
                        ((params.manufacturer<<8)|params.module_id), params.flags,0,0);
            return;
        case OPC_RX_RQNP:     // Request Node Parameter
            if (mode == MODE_SETUP) {
                cbusTransmit(8, persist[FLASH_INDEX_CANID], 11, OPC_TX_PARAMS, // NO NN
                        ((params.manufacturer<<8)|params.minor_version), 
                        ((params.module_id<<8)|params.no_events),
                        params.no_event_vars_per_event,params.no_node_variables,params.major_version);
            }
            return;
        case OPC_RX_RQMN:     // Request Name
            if (mode == MODE_SETUP) {
                cbusTransmit(8, persist[FLASH_INDEX_CANID], 11, OPC_TX_NAME,  
                        ((module_name[0]<<8) | module_name[1]), 
                        ((module_name[2]<<8) | module_name[3]),
                        module_name[4], module_name[5], module_name[6]);
            }
            return;
        case OPC_RX_SNN:      // Set Node Number
            if (mode == MODE_SETUP) {
                setNN(getNN(buf));
                mode = MODE_FLIM;  // out of setup
                led = LED_ON;
                // send ack
                cbusTransmit(3, persist[FLASH_INDEX_CANID], 11, OPC_TX_NNACK, persist[FLASH_INDEX_NODEID], 0, 0,0,0);
            }
            return;
    }
            
    // Now commands which have an nn
    unsigned int nn = getNN(buf);
    if (nn != persist[FLASH_INDEX_NODEID]) return;   // not for us
    unsigned int en = getEN(buf);
    unsigned char no;
    unsigned int ent;
    unsigned int current;

    switch(opc) {
        case OPC_RX_BOOTM:    // Bootloader Mode
            // TODO
            return;
        case OPC_RX_ENUM:     // Self Enumeration of CANID
            mode = MODE_SETUP;
            led = LED_FLASH;
            startSelfEnumeration();
            return;
            
        // Commands having a 1 byte parameter. This will be in en high byte.
        case OPC_RX_NVRD:     // Node Variable Read
            en = en >> 8;
            if (en > NUM_NV) {
                cbusTransmit(4, persist[FLASH_INDEX_CANID], 11, OPC_CMDERR, persist[FLASH_INDEX_NODEID],
                        CMDERR_INV_NV_IDX<<8, 0,0,0);
            } else {
                unsigned int ans = getNodeVar(en);
                cbusTransmit(5, persist[FLASH_INDEX_CANID], 11, OPC_TX_NVANS, persist[FLASH_INDEX_NODEID],
                        ((en << 8) | ans), 0,0,0);
            }
            return;
        case OPC_RX_RQNPN:    // Request Node Parameter by Index
            en = en >> 8;
            if (en == 0) {
                // number of parameters
                cbusTransmit(5, persist[FLASH_INDEX_CANID], 11, OPC_TX_PARAN, persist[FLASH_INDEX_NODEID], 
                        getNodeParam(0), 0,0,0);
            }
            if (en > getNodeParam(0)) {
                cbusTransmit(4, persist[FLASH_INDEX_CANID], 11, OPC_CMDERR, persist[FLASH_INDEX_NODEID],
                        CMDERR_INV_PARAM_IDX<<8, 0,0,0);
            } else {
                unsigned int ans = getNodeParam(en);
                cbusTransmit(5, persist[FLASH_INDEX_CANID], 11, OPC_TX_PARAN, persist[FLASH_INDEX_NODEID], 
                        ((en << 8) | ans), 0,0,0);
            }
            return;
        case OPC_RX_CANID:    // Set CANID 
            en = en >> 8;
            if ((en > 99) || (en < 1)) {
                // outside permitted range
                cbusTransmit(4, persist[FLASH_INDEX_CANID], 11, OPC_CMDERR, persist[FLASH_INDEX_NODEID],
                        CMDERR_INVALID_EVENT<<8, 0,0,0);
            }
            // Save the CAN_ID
            setCanId(en);   // saves in the persist store
            setAutoRemoteResponse();
            // send ACK - XXX spec doesn't sat to do this
            cbusTransmit(3, persist[FLASH_INDEX_CANID], 11, OPC_TX_NNACK, persist[FLASH_INDEX_NODEID], 0, 0,0,0);
            return;
            
    // Now for commands having an nn and en
        case OPC_RX_ACOF:     // zero speed
        case OPC_RX_ACOF1:    // zero speed
        case OPC_RX_ACOF2:    // zero speed
        case OPC_RX_ACOF3:    // zero speed
            no = en % NUM_THROTTLES;
            setRequiredSpeed(no, 0, 0, 0);
            return;
        case OPC_RX_AREQ:     // Request Long event
            no = en % NUM_THROTTLES;
            ent = en & EN_TYPE_MASK;
            switch (ent) {
                case TOTI_EN:       // get the Toti state
                    current = getAverageCurrent(no);
                    if (getTotiState(no)) {
                        cbusTransmit(7, persist[FLASH_INDEX_CANID], 11, OPC_TX_ARON2, persist[FLASH_INDEX_NODEID],
                        en, (current>>8)&0xFF,current&0xFF,0);
                    } else {
                        cbusTransmit(7, persist[FLASH_INDEX_CANID], 11, OPC_TX_AROF2, persist[FLASH_INDEX_NODEID],
                        en, (current>>8)&0xFF,current&0xFF,0);
                    }
                    return;
                case FAULT_EN:      // Get the Fault state
                    if (getFaultState(no)) {
                        cbusTransmit(5, persist[FLASH_INDEX_CANID], 11, OPC_TX_ARON, persist[FLASH_INDEX_NODEID],
                        en, 0,0,0);
                    } else {
                        cbusTransmit(5, persist[FLASH_INDEX_CANID], 11, OPC_TX_AROF, persist[FLASH_INDEX_NODEID],
                        en, 0,0,0);
                    }
                    return;
                case THROTTLE_EN:   // Get the Throttle state
                    cbusTransmit(8, persist[FLASH_INDEX_CANID], 11, OPC_TX_ARON3, persist[FLASH_INDEX_NODEID],
                        en, getSpeed(no),getCored(no),getAccel(no));
                    return;
            }
            return;
        case OPC_RX_NVSET:    // Set a Node Variable
            no = (en >> 8) & 0xFF;
            if ((no < 1) || (no > NUM_NV)) {
                cbusTransmit(4, persist[FLASH_INDEX_CANID], 11, OPC_CMDERR, persist[FLASH_INDEX_NODEID],
                        CMDERR_INV_NV_IDX<<8, 0,0,0);
            } else {
                setNodeVar(no, en&0xFF);
                cbusTransmit(3, persist[FLASH_INDEX_CANID], 11, OPC_TX_WRACK, persist[FLASH_INDEX_NODEID], 0, 0,0,0);
            }
            return;
        case OPC_RX_ACON3:    // Set speed
            no = en % NUM_THROTTLES;
            setRequiredSpeed(no, getB1(buf), getB2(buf), getB3(buf));
            return;
    }
}

/**
 * Go into Setup mode where we do self enumeration and request a Node Number.
 */
inline void setup() {
    mode = MODE_SETUP;
    led = LED_FLASH;
    //Transmit a RQNN
    cbusTransmit(3, persist[FLASH_INDEX_CANID], 11, OPC_TX_RQNN, persist[FLASH_INDEX_NODEID], 0, 0,0,0);
}

/**
 * Initialise each of the modules.
 */
void initialise() {
    // Set up the oscillator
    CLKDIVbits.PLLPOST = 0x01;    // Divide by 4  N2=4
    CLKDIVbits.PLLPRE = 0x002;    // Divide by 4  N1=4
    PLLFBD = 126;                 // M=128

    // rest of setup
    initPersist();
    initNP();
	T1CON = 0x8030;
    initThrottles();
    initCBUS();
    TRISAbits.TRISA4 = 0;   // Yellow LED output port
    initTimer5();
    led = LED_ON;
    IFS0 = 0;
    IFS1 = 0;
    IFS2 = 0;
    IFS3 = 0;
    IFS4 = 0;
}

/**
 * Set up Timer 5 for 50Hz clock - used for various timing such as acceleration
 * and self enum timouts.
 */
inline void initTimer5() {
    T5CONbits.TON = 0;
    //T3CONbits.T32 = 0;      // 16 bit timer
    T5CONbits.TCKPS = 3;    // prescalar = 256 
    T5CONbits.TCS = 0;      // No Sync
    T5CONbits.TGATE = 0;    // No Gate
    PR5 = 1250;              // period about 20ms = 50Hz
    IEC1bits.T5IE = 1;      // Enable interrupts
    IPC7bits.T5IP = 2;          // low priority
    T5CONbits.TSIDL = 1;    // Stop on sleep
    TMR5 = 0;               // zero the timer
    T5CONbits.TON = 1;      // timer
}

/**
 * Called from Timer5 ISR. Service the acceleration requirements.
 */
void _ISR _T5Interrupt(void) {
    unsigned char i;

    // Process acceleration and deacceleration
    for (i=0; i< NUM_THROTTLES; i++) {
        serviceAccel(i);
    }
    
    // check the sense inputs
    senseT5interrupt();
    
    // check the self enumeration timeout
    if (doingSelfEnumeration()) {
        selfEnumerationTimeout();
    }
    // start to service the PB
    servicePBstart();
    
    serviceLED();
    
    // check push button
    servicePBstop();
    
/*if ((ledCounter & 0x3f) == 0) {
// transmit an ACK regularly
cbusTransmit(1, persist.canId, 11, OPC_ACK, 0, 0, 0,0,0);
}*/
    
    // done
    IFS1bits.T5IF = 0;  // clear flag
} 

inline void serviceLED() {
    ledCounter++;
    
    switch (led) {
        case LED_OFF:
            ledOff();
            break;
        case LED_FLASH_ON:
            if ((ledCounter & 0x08) && (ledCounter & 0x04)) {
                ledOn();
            } else {
                ledOff();
            }
            break;
        case LED_FLASH:
            if (ledCounter & 0x08) {
                ledOn();
            } else {
                ledOff();
            }
            break;
        case LED_FLASH_OFF:
            if ((ledCounter & 0x08) || (ledCounter & 0x04)) {
                ledOn();
            } else {
                ledOff();
            }
            break;
        case LED_ON:
            ledOn();
            break;
    }
}

/**
 * Turn the yellow LED on.
 */
inline void ledOn() {
    LATAbits.LATA4 = 1;
}

/**
 * Turn the yellow LED off.
 */
inline void ledOff() {
    LATAbits.LATA4 = 0;
}

/**
 * Start the Push button service. This is split out as it needs a bit of time for input to settle before reading
 */
inline void servicePBstart() {
    TRISAbits.TRISA4 = 1;   // Switch Yellow LED output port to input
}

/**
 * Finish servicing the push button. If button was held down long enough then change mode.
 */
inline void servicePBstop() {
    if (PORTAbits.RA4 == 0) {
        //pressed
        TRISAbits.TRISA4 = 0;   // back to output
	
        if (pbCounter == 0) {
            cbusTransmit(3, persist[FLASH_INDEX_CANID], 11, OPC_TX_NNACK, persist[FLASH_INDEX_NODEID], 0, 0,0,0);
//            cbusTransmit(8, persist[FLASH_INDEX_CANID], 11, 0xFB, persist[FLASH_INDEX_NODEID], getAverageCurrent(0), 0,0,0);
            /* debug code
             * 
             * We turn on the leds for debugging. Pressing the PB turns them all off again.  
            faultOff(3);
            faultOff(2);
            faultOff(1);
            faultOff(0); */
        }
        if (pbCounter > PB_TIME) {
            if (mode == MODE_SETUP) {
                // this press was to get node out of setup and continue to use its
                // original NN
                mode = MODE_FLIM;
                led = LED_ON;
            } else {
                // this press was to put module into setup and do self enum
                mode = MODE_SETUP;
                led = LED_FLASH;
                // start to self enum to get a CAN_ID
                startSelfEnumeration();
            }
            pbCounter = 0;
        }
        pbCounter++;
    } else {
        TRISAbits.TRISA4 = 0;   // back to output
        // released

        pbCounter = 0;                       
    }
    
}

