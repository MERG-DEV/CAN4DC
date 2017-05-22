/*
 * File:   throttle.c
 * Author: Ian Hogg
 *
 * Created on 10 January 2016, 13:32
 */

/**
 * The throttle routines to set up the PIC's ports based upon direction setting of throttle.
 * Set the PWM based upon throttle setting.
 */

#include "throttle.h"
#include "persist.h"
#include "throttleVar.h"
#include "frequencyConfig.h"
#include "sense.h"
#include "pwm.h"

#define PPS_PORT            0
#define PPS_PWM1            0b010010
#define OCM_DISABLED        0
#define OCM_PWM_NO_FAULT    6


/**
 * The ThrottleConfig provides the logical device (port) numbers that are being 
 * used based upon which pins of the device are connected to the throttle
 * circuits.
 * 
 */
const ThrottleConfig throttleConfig[NUM_THROTTLES] = { 
    {
        1,          // PWM number
        {   // alarm PIN 4
                &LATB,       // alarm port
                &TRISB,      // alarm tris
                0,          // alarm pin
                0,           // remappable 
                2           // analogue bit
        },
        {   // Output A PIN 6
                &LATB,       // Output A port
                &TRISB,      // Output A tris
                2,          // Output A pin
                2,           // remappable
                4           // analogue bit
        },
        {   // Output B PIN 7
                &LATB,       // Output B port
                &TRISB,      // Output B tris
                3,          // Output B pin
                3,           // remappable
                5           // analogue bit
        },
        {   // Sense PIN 2
                &LATA,       // Sense port
                &TRISA,      // Sense Tris
                0,           // Sense pin
                -1,           // remappable 
                0           // analogue bit
        }
    },
    {
        2,          // PWM number
        {   // Alarm PIN 5
                &LATB,       // alarm port
                &TRISB,      // alarm TRIS
                1,          // alarm pin
                1,           // remappable - unused
                3           // analogue bit
        },
        {   // Output A PIN 15
                &LATB,       // Output A port
                &TRISB,      // Output B tris
                6,          // Output A pin
                6,           // remappable
                -1          // analogue bit
        },
        {   // Output B PIN 26
                &LATB,       // Output B port
                &TRISB,      // Output B tri
                15,          // Output B pin
                15,          // remappable
                9           // analogue bit
        },
        {   // Sense PIN 3
                &LATA,       // Sense port
                &TRISA,      // Sense TRIS
                1,           // Sense pin
                -1,           // remappable - unused
                1           // analogue bit
        }
    },
    {
        3,          // PWM number
        {   // Alarm PIN 22
                &LATB,      // alarm port
                &TRISB,      // alarm tris
                11,         // alarm pin
                11,          // remappable
                -1           // analogue bit
        },  // Due to a cock-up in the PCB layout I have had to swap the track 3 & track 4 outputs and sense
        {   // Output A PIN 18
                &LATB,      // Output A port
                &TRISB,      // Output A tris
                9,          // Output A pin
                9,          // remappable
                -1           // analogue bit
        },
        {   // Output B PIN 21
                &LATB,      // Output B port
                &TRISB,      // Output B tris
                10,         // Output B pin
                10,          // remappable
                -1           // analogue bit
        },
        {   // Sense PIN 24
                &LATB,      // Sense port
                &TRISB,      // Sense TRIS
                13,          // Sense pin
                13,          // remappable
                11           // analogue bit
        }
    },
    {
        4,          // PWM number
        {   // Alarm PIN 25
                &LATB,      // alarm port
                &TRISB,      // alarm tris
                14,         // alarm pin
                14,          // remappable
                10           // analogue bit
        },
                // Due to a cock-up in the PCB layout I have had to swap the track 3 & track 4 outputs and sense
        {   // Output A PIN 16
                &LATB,      // Output A port
                &TRISB,      // Output A tris
                7,          // Output A pin
                7,          // remappable
                -1           // analogue bit
        },
        {   // Output B PIN 17
                &LATB,      // Output B port
                &TRISB,      // Output B tris
                8,          // Output B pin
                8,          // remappable
                -1           // analogue bit
        },
        {   // Sense PIN 23
                &LATB,      // Sense port
                &TRISB,      // Sense tris
                12,           // Sense pin
                12,          // remappable
                12           // analogue bit
        }

    }
};

/**
 * A set of variables used to manage the throttle. Grouped into a struct
 * for convenience.
 */
ThrottleVar vars[NUM_THROTTLES];


void stop(unsigned char no);
void faultOff(unsigned char no);
void faultOn(unsigned char no);
void setCurrentSpeed(unsigned char no, char s, unsigned char cored);
void setDirection(unsigned char no, char s);
void mapPPSoutput(unsigned char PRn, unsigned char peripheral);


/**
 * Initialise a single throttle.
 * 
 * @param no the throttle number (0-3)
 */
void initThrottle(unsigned char no) {
    // set speed to 0
    vars[no].accelCount = 0;
    vars[no].currentSpeed = 0;
    vars[no].currentCored = 0x7F;   // invalid so will reset

    // make alarm a digital output
    *(throttleConfig[no].alarm.tris) &= ~(1 << (throttleConfig[no].alarm.bit));
    if (throttleConfig[no].alarm.analogueBit >= 0) {
        AD1PCFGL |= (1 << (throttleConfig[no].alarm.analogueBit));
    }
    // set pin mapping to PORT
    mapPPSoutput(throttleConfig[no].alarm.remappable, PPS_PORT);
    
    
    // make Output A and Output B digital outputs
    *(throttleConfig[no].outputA.tris) &= ~(1 << (throttleConfig[no].outputA.bit));
    if (throttleConfig[no].outputA.analogueBit >= 0) {
        AD1PCFGL |= (1 << (throttleConfig[no].outputA.analogueBit));
    }
    // set pin mapping to PORT
    mapPPSoutput(throttleConfig[no].outputA.remappable, PPS_PORT);

    *(throttleConfig[no].outputB.tris) &= ~(1 << (throttleConfig[no].outputB.bit));
    if (throttleConfig[no].outputB.analogueBit >= 0) {
        AD1PCFGL |= (1 << (throttleConfig[no].outputB.analogueBit));
    }
    // set pin mapping to PORT
    mapPPSoutput(throttleConfig[no].outputB.remappable, PPS_PORT);
        
    // configure sense analogue input
    *(throttleConfig[no].sense.tris) |= (1 << (throttleConfig[no].sense.bit));
    // set pin mapping to analogue
    AD1PCFGL &= ~(1 << (throttleConfig[no].sense.analogueBit));
    
    vars[no].nextReadNo = 0;
    
    initPWM(no);

    stop(no);
    faultOff(no);  
}

/**
 * Initialse all the throttles.
 * Set up the various timers, interrupts and ADC.
 */
void initThrottles() {
    unsigned char i;
 
    for (i=0; i<NUM_THROTTLES; i++) {
        initThrottle(i);
    }
    
    // Timers must be configured after PWM
    // set up timer 2 for cored motor
    initTimer2();

    // set up timer 3 for coreless motors
    initTimer3();
    
    initADC();

}

/**
 * This is the main way to request a throttle change.
 * @param no the throttle number (0-3)
 * @param s the required speed
 * @param cored boolean to determine which speed PWM clock to use
 * @param accel time between speed changes - actually 1/acceleration
 */
void setRequiredSpeed(unsigned char no, char b1, unsigned char b2, char b3) {
    vars[no].requestedSpeed = b1;
    unsigned char accel = b2 & 0x7f;
    vars[no].acceleration = accel;    // lower 7 bits are the acceleration
    char cored = (b2>>7) & 0x01;
    vars[no].cored = cored;        // msb is the cored flag
    vars[no].bemff = b3;
    if (getFaultState(no)) return;  // currentSpeed should be zero
    if (accel == 0) {
        // if accel == 0 then this is an immediate change
        setCurrentSpeed(no, b1, cored);
    } else if ((vars[no].currentSpeed == vars[no].requestedSpeed) && (vars[no].cored != vars[no].currentCored)) {
        setCurrentSpeed(no, vars[no].currentSpeed, vars[no].cored);
    }
/*    if (vars[no].requestedSpeed == 0) {
        faultOff(no);
    } else {
        faultOn(no);
    }*/
}

inline unsigned char getSpeed(unsigned char no) {
    return vars[no].currentSpeed;
}

inline unsigned char getCored(unsigned char no) {
    return vars[no].cored;
}

inline unsigned char getAccel(unsigned char no) {
    return vars[no].acceleration;
}
/**
 * Called by Timer5 ISR to accelerate or deaccelerate periodically.
 * @param no
 */
void serviceAccel(unsigned char no) {
    if ((vars[no].currentSpeed == vars[no].requestedSpeed) && (vars[no].cored == vars[no].currentCored)) {
        // no change
        return;
    }
    if (getFaultState(no)) return;  // currentSpeed should be zero
    if (vars[no].accelCount >= vars[no].acceleration) {
        vars[no].accelCount = 0;
        if (vars[no].currentSpeed < vars[no].requestedSpeed) {
            setCurrentSpeed(no, vars[no].currentSpeed+1, vars[no].cored);
        } else if (vars[no].currentSpeed > vars[no].requestedSpeed) {
            setCurrentSpeed(no, vars[no].currentSpeed-1, vars[no].cored);
        }
    } else {
        vars[no].accelCount++;
    }
}


/**
 * Get the current speed setting of a throttle.
 * @param no throttle number (0-3)
 * @return the current speed
 */
char getCurrentSpeed(unsigned char no) {
    return vars[no].currentSpeed;
}

/**
 * Set the current speed of a throttle.
 * This will connect the PWM generator to the correct pins, the right way around
 * based upon the sign of the speed. If the new speed is zero the PWM generator 
 * is disconnected.
 * 
 * @param no the throttle number (0-3)
 * @param s the speed
 * @param cored boolean to indicate which PWM clock frequency to use
 */
void setCurrentSpeed(unsigned char no, char s, unsigned char cored) {
    vars[no].cored = cored;
    
    // set direction if it has changed
    if ((s > 0) && (vars[no].currentSpeed <= 0)) {
        setDirection(no, s);
    }
    if ((s < 0) && (vars[no].currentSpeed >= 0)) {
        setDirection(no, s);
    }
    if (s == 0) {
        stop(no);
    } else {
        if ((vars[no].currentSpeed == 0) || (vars[no].cored != vars[no].currentCored)) {
            // start PWM
            startPWM(no, cored);
            vars[no].currentCored = vars[no].cored;
        }
        
        // set pulse width
        if (s < 0) {
            setPWMwidth(no, -s, cored);
            // OC1RS = -s;
        } else {
            setPWMwidth(no, s, cored);
            // OC1RS = s;
        }
    }
    vars[no].currentSpeed = s;
}

/**
 * Connect the PWM generator to the correct pin and the other pin to ground.
 * This is accomplished by using the PPS (peripheral pin select) functionality
 * of the PIC.
 * 
 * @param no the throttle number (0-3)
 * @param s the speed - only the sign of the speed is used.
 */
void setDirection(unsigned char no, char s) {
    // connect the PWM output to the correct pin
    if (s > 0) {
        // set PWM to outputA
        mapPPSoutput(throttleConfig[no].outputA.remappable, (PPS_PWM1 + no));
        // set outputB low
        mapPPSoutput(throttleConfig[no].outputB.remappable, PPS_PORT);
    } else {
        // set PWM to outputB
        mapPPSoutput(throttleConfig[no].outputB.remappable, (PPS_PWM1 + no));
        // set outputA low
        mapPPSoutput(throttleConfig[no].outputA.remappable, PPS_PORT);
    }
}

/**
 * Disconnect the PWM from the pins so that both output pins are at ground.
 * 
 * @param no throttle number (0-3)
 */
void stop(unsigned char no) {
    // disconnect PWM from both outputA and outputB
    mapPPSoutput(throttleConfig[no].outputA.remappable, PPS_PORT);
    mapPPSoutput(throttleConfig[no].outputB.remappable, PPS_PORT);
    
    // turn off PWM
    stopPWM(no);
}

/**
 * Synchronise the throttles. This to ensure that all throttles on the layout
 * are at the same phase so that when a loco moves from one PWM to another there
 * is no shorting out across the block boundary. In reality there will be some 
 * mismatch but the alarm functionality ignore brief shorts.
 * 
 */
void synch() {
    TMR2 = 0;
    TMR3 = 0;
}

/**
 * Turn the alarm fault LED on.
 * @param no throttle number (0-3)
 */
void faultOn(unsigned char no) {
    *(throttleConfig[no].alarm.port) |= (1 << (throttleConfig[no].alarm.bit));
}

/**
 * Turn the alarm fault LED off.
 * @param no throttle number (0-3)
 */
void faultOff(unsigned char no) {
    *(throttleConfig[no].alarm.port) &= ~(1 << (throttleConfig[no].alarm.bit));
}

/**
 * Unlock the PPS so that PPS changes can be made. 
 */
inline void unlockPPS() {
    __builtin_disi(0x3fff);
    __builtin_write_OSCCONL(OSCCON & 0xBF);    // clear IOLOCK
    __builtin_disi(0x0);
}
/**
 * Relock the PPS so that changes are not made unintentially.
 */
inline void lockPPS() {
    __builtin_disi(0x3fff);
    __builtin_write_OSCCONL(OSCCON & 0x40);    // set IOLOCK
    __builtin_disi(0x0);
}

/**
 * Change the PPS setting.
 * @param PRn   PPS port number
 * @param peripheral identifier for a particular peripheral
 */
inline void mapPPSoutput(unsigned char PRn, unsigned char peripheral) {
    unsigned char * p;
    unlockPPS();
    p = ((unsigned char *)&RPOR0) + PRn;
    *p = peripheral;
    lockPPS();
}


/**
 * Set up Timer 2 for 100Hz PWM
 */
inline void initTimer2() {
    T2CONbits.TON = 0;      // Timer off
    T2CONbits.T32 = 0;      // 16 bit timer
    T2CONbits.TCKPS = getCoredPrescalar();    // prescalar
    T2CONbits.TCS = 0;      // No Sync
    T2CONbits.TGATE = 0;    // No Gate
    PR2 = 128*getCoredMultiplier();              // period
    IEC0bits.T2IE = 0;      // Disable interrupts - will get enabled by T5
    IPC1bits.T2IP = 4;      // interrupt priority
    T2CONbits.TSIDL = 1;    // Stop on sleep
    TMR2 = 0;               // zero the timer
    T2CONbits.TON = 1;      // timer on 
}


/**
 * Set up Timer 3 for 20kHz PWM
 */
inline void initTimer3() {
    T3CONbits.TON = 0;      // Timer off
    //T3CONbits.T32 = 0;      // 16 bit timer
    T3CONbits.TCKPS = getCorelessPrescalar();    // prescalar
    T3CONbits.TCS = 0;      // No Sync
    T3CONbits.TGATE = 0;    // No Gate
    PR3 = 128*getCorelessMultiplier();              // period
    IEC0bits.T3IE = 0;      // Disable interrupts - will get enabled by T5
    IPC2bits.T3IP = 5;      // interrupt priority
    T3CONbits.TSIDL = 1;    // Stop on sleep
    TMR3 = 0;               // zero the timer
    T3CONbits.TON = 1;      // timer on
}

/**
 * Stop all.
 */
inline void stopAll() {
    unsigned char i;
    for (i=0; i<NUM_THROTTLES; i++) {
        setRequiredSpeed(i, 0, 0, 0);
        stop(i);
    }
}

