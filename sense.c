/*
 * File:   sense.c
 * Author: Ian Hogg
 * 
 * We use the T2 and T3 interrupts to indicate that a PWM cycle has started.
 * At the start of the PWM cycle the PWM output should be high so that the
 * current can be measured.
 * 
 * In the T2 and T3 service routines we find the next throttle to be measured
 * and the start the ADC sample. The ADC conversion will start automatically
 * after the sample period.
 * 
 * When the conversion is complete we get the current reading and store it
 * in the readings buffer.
 *
 * Created on 20 January 2016, 19:58
 */


#include "xc.h"
#include "throttle.h"
#include "sense.h"
#include "persist.h"
#include "throttleVar.h"
#include "cbus.h"


volatile unsigned char throttleDoingADC;        // modified in T5 interrupt
volatile unsigned char thisThrottleDoingADC;    // modified in T2/T3 and ADC interrupt
SenseVar senseVars[NUM_THROTTLES];
extern ThrottleConfig throttleConfig[NUM_THROTTLES];
extern ThrottleVar vars[];
void saveReading(unsigned char no, unsigned int r);
unsigned int getAverage(unsigned char no);
extern void faultOn(unsigned char no);
extern void faultOff(unsigned char no);
extern void stop(unsigned char no);

/**
 * Timer 5 is used as the time base of ADC measurements.
 * Each T5 tick the next throttle is checked to see if it needs ADC sense 
 * measurement - i.e. if its speed is >= MIN_MEASURE.
 * 
 * If it is then either T2 or T3 interrupt is enabled depending upon which
 * clock source the PWM for that throttle is using.
 * 
 * The T2 or T3 interrupt is triggered at the start of the next PWM pulse
 * and this is used to start an ADC Sample.
 */

inline void senseT5interrupt(void) {
    // move on to next throttle
    throttleDoingADC ++;
    throttleDoingADC %= NUM_THROTTLES;
    // check if already in fault
    if (senseVars[throttleDoingADC].faultCount) {
        senseVars[throttleDoingADC].faultCount--;
        // want to save a value close to the threshold so that if the fault still
        // persists at end of timeout we don't spend too long until the fault state
        // is re-entered.
        saveReading(throttleDoingADC, persist[FLASH_INDEX_ALARM_THRESHOLD]-20);
        if (senseVars[throttleDoingADC].faultCount == 0) {
            // end of fault timeout
            faultOff(throttleDoingADC);
            cbusTransmit(5, persist[FLASH_INDEX_CANID], 11, OPC_TX_ACOF, persist[FLASH_INDEX_NODEID], 
                    FAULT_EN+throttleDoingADC, 0,0,0);
        }
    } else {
        int minMeasure;
        if (vars[throttleDoingADC].cored) {
            minMeasure = persist[FLASH_INDEX_MIN_CORED];
        } else {
            minMeasure = persist[FLASH_INDEX_MIN_CORELESS];
        }
        if ((vars[throttleDoingADC].currentSpeed <= -minMeasure) || 
                (vars[throttleDoingADC].currentSpeed >= minMeasure)) {
            // we can do a sense measurement so enable to correct T2/3 interrupt
            // to be called at the start of the next PWM cycle
            if (vars[throttleDoingADC].cored) {
                IEC0bits.T2IE = 1;
            } else {
                IEC0bits.T3IE = 1;
            }
        }
    }
    IFS0bits.T2IF = 0;
    IFS0bits.T3IF = 0;
}

/**
 * Called from Timer2 ISR. Start an ADC sample/conversion.
 * We only do an ADC every 20ms so we don't need every possible Timer2 interrupt.
 */
void _ISR _T2Interrupt(void) {
    IEC0bits.T2IE = 0;      // disable again   
    // set up to sample throttle sense input
    thisThrottleDoingADC = throttleDoingADC;
    AD1CHS0bits.CH0SA = throttleConfig[thisThrottleDoingADC].sense.analogueBit;
    // start ADC conversion
    AD1CON1bits.SAMP = 1;
    IFS0bits.T2IF = 0;  // clear flag
} 

/**
 * Called from Timer3 ISR. Start an ADC sample/conversion.
 * We only do an ADC every 20ms so we don't need every possible Timer3 interrupt.
 */
void _ISR _T3Interrupt(void) {
    IEC0bits.T3IE = 0;      // disable again
    // set up to sample throttle sense input
    thisThrottleDoingADC = throttleDoingADC;
    AD1CHS0bits.CH0SA = throttleConfig[thisThrottleDoingADC].sense.analogueBit;
    // start ADC conversion
    AD1CON1bits.SAMP = 1;
    IFS0bits.T3IF = 0;  // clear flag
} 

/**
 * Initialise the ADC.
 */
inline void initADC() {
    throttleDoingADC = 0;
    thisThrottleDoingADC = 0;
    
/*    AD1CON1bits.ADON = 0;   // Turn the ADC off
    AD1CON1bits.AD12B = 0;  // 10bit mode
    //AD1PCFGL has already been set in initThrottle()
    AD1CON2bits.VCFG = 0;   // voltage reference
    AD1CON3bits.ADRC = 0;   // analogue conversion clock
    AD1CON2bits.CHPS = 0;   // prescalar
    AD1CON1bits.SSRC = 0;   // trigger sample by CPU
    AD1CON3bits.SAMC = 3;   // sample time - XXX may need adjustment
    AD1CON1bits.FORM = 0;   // unsigned int format
    AD1CON1bits.ADSIDL = 1; // sleep in idle
    AD1CON1bits.SIMSAM = 0; //no simultaneous sampling
    AD1CON2bits.ALTS = 0;   // Always use channel A
    AD1CON2bits.CSCNA = 0;  // no scanning */
    AD1CON1 = 0x04E0;
    AD1CON2 = 0x0000;
    AD1CON3 = 0x0301;
    AD1CON4 = 0x0000;
    AD1CSSL = 0x0000;
    IEC0bits.AD1IE = 1;     // Enable interrupt
    IPC3bits.AD1IP = 3;     // interrupt priority
    IFS0bits.AD1IF = 0;     // clear interrupt flag
    AD1CON1bits.ADON = 1;   // Turn on
}
/**
 * Called from ADC1 ISR. Service the sense input.
 */
void _ISR _ADC1Interrupt(void) {
    // get the ADC value and put into the readings buffer
    saveReading(thisThrottleDoingADC, ADC1BUF0);
    // take average of last few readings and take appropriate action
    unsigned int current = getAverageCurrent(thisThrottleDoingADC);
    
    if (current >= persist[FLASH_INDEX_ALARM_THRESHOLD]) {
        // fault condition
        faultOn(thisThrottleDoingADC);
        senseVars[thisThrottleDoingADC].faultCount = persist[FLASH_INDEX_ALARM_CUTOFF_TIME];
        vars[thisThrottleDoingADC].currentSpeed = 0;
        stop(thisThrottleDoingADC);
        cbusTransmit(5, persist[FLASH_INDEX_CANID], 11, OPC_TX_ACON, persist[FLASH_INDEX_NODEID], 
                    FAULT_EN+throttleDoingADC, 0,0,0);
    } else {
        if (current >= persist[FLASH_INDEX_TOTI_THRESHOLD]) {
            if (senseVars[thisThrottleDoingADC].toti) {
                // toti still present
            } else {
                // send toti on
                senseVars[thisThrottleDoingADC].toti = 1;
                cbusTransmit(5, persist[FLASH_INDEX_CANID], 11, OPC_TX_ACON, persist[FLASH_INDEX_NODEID], 
                    TOTI_EN+thisThrottleDoingADC, 0,0,0);
            }
        } else {
            if (senseVars[thisThrottleDoingADC].toti) {
                // send toti off
                senseVars[thisThrottleDoingADC].toti = 0;
                cbusTransmit(5, persist[FLASH_INDEX_CANID], 11, OPC_TX_ACOF, persist[FLASH_INDEX_NODEID], 
                    TOTI_EN+thisThrottleDoingADC, 0,0,0);
            } else {
                // toti still off
            }
            
        }
    }
    IFS0bits.AD1IF = 0;  // clear flag
}

/**
 * Get the current (pun intended) fault state.
 */
inline unsigned char getFaultState(unsigned char no) {
    return senseVars[no].faultCount;
}

/**
 * Get the current (pun intended) Toti state.
 */
inline unsigned char getTotiState(unsigned char no) {
    return senseVars[no].toti;
}

/**
 * Save a sense reading so that they can be averaged.
 */
inline void saveReading(unsigned char no, unsigned int r) {
    senseVars[no].readings[senseVars[no].nextReadingOffset] = r;
    senseVars[no].nextReadingOffset = (senseVars[no].nextReadingOffset + 1) % CURRENT_READINGS_BUF_SIZE;
}

/**
 * Get the average.
 */
inline unsigned int getAverageCurrent(unsigned char no) {
    unsigned int sum = 0;
    
    int i;
    for (i=0; i<persist[FLASH_INDEX_NUMBER_TO_AVERAGE]; i++) {
        int idx = (CURRENT_READINGS_BUF_SIZE +senseVars[no].nextReadingOffset - 1 - i) % CURRENT_READINGS_BUF_SIZE;
        sum += senseVars[no].readings[idx];
    }
    return sum/persist[FLASH_INDEX_NUMBER_TO_AVERAGE];
}
