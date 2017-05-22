/*
 * File:   pwm.c
 * Author: Ian Hogg
 *
 * Created on 20 January 2016, 20:01
 */

/**
 * Functionality for controlling the Timer 2/3 and PWM.
 */

#include "xc.h"
#include "frequencyConfig.h"
#include "persist.h"
#include "pwm.h"
#include "cbusdefs8j.h"

// prescalar is determined by Timer TCKPS:
// 0 = 1:1
// 1 = 1:8
// 2 = 1:64
// 3 = 1:256
const FrequencyConfig frequencyConfig[] = { 
    // frequency, multiplier, prescalar
    {62992, 2,	0}, //0
    {4199,  3,	0}, //1
    {3149,	4,	0}, //2
    {2519,	5,	0}, //3
    {2099,	6,	0}, //4
    {1799,	7,	0}, //5
    {1574,	8,	0}, //6
    {1399,	9,	0}, //7
    {1259,	10,	0}, //8
    {1145,	11,	0}, //9
    {1049,	12,	0}, //10
    {9691,	13,	0}, //11
    {8998,	14,	0}, //12
    {8398,	15,	0}, //13
    {7874,	16,	0}, //14
    {5249,	3,	1}, //15
    {3937,	4,	1}, //16
    {3149,	5,	1}, //17
    {2624,	6,	1}, //18
    {2249,	7,	1}, //19
    {1968,	8,	1}, //20
    {1749,	9,	1}, //21
    {1574,	10,	1}, //22
    {1431,	11,	1}, //23
    {1312,	12,	1}, //24
    {1211,	13,	1}, //25
    {1124,	14,	1}, //26
    {1049,	15,	1}, //27
    {984,	16,	1}, //28
    {656,	3,	2},    //29
    {492,	4,	2},    //30
    {393,	5,	2},    //31
    {328,	6,	2},    //32
    {281,	7,	2},    //33   
    {246,	8,	2},    //34
    {218,	9,	2},    //35
    {196,	10,	2},    //36
    {178,	11,	2},    //37
    {164,	3,	3},   //38
    {123,	4,	3},   //39
    {98,	5,	3},   //40
    {82,	6,	3},   //41
    {70,	7,	3},   //42
    {61,	8,	3}    //43
    /* don't go to frequencies below this as otherwise the T5 interrupt
     * could increment the throttle index twice between T2/T3 interrupts
     * and this would mean that a throttle's sense might not get serviced
     * and fault condition would't be handled.
     */
};
#define MAX_FREQUENCY_INDEX 43


int coredFrequencyIndex;
int corelessFrequencyIndex;

// forward declarations
inline unsigned int getFrequencyIndex(unsigned int cored);

/**
 * Initialise the PWM Output Compare peripheral.
 * @param no throttle number (0-3) which is the same as the OC peripheral number
 */
inline void initPWM(unsigned char no) {
    unsigned int * p;
    
    initPWMindexes();
    
    p = (unsigned int *)&OC1CON;
    p += (3 * no);      //OC1RS, OC1R, OCCON
    *p = 0x2000;        //OC1CONbits.OCM = OCM_DISABLED;
                        //OC1CONbits.OCSIDL = 1;      // idle when asleep
}

/**
 * Lookup the index usining the persisted frequencies so that prescalar and multiplier 
 * can be used elsewhere.
 */
inline void initPWMindexes() {
    coredFrequencyIndex = getFrequencyIndex(persist[FLASH_INDEX_CORED_FREQUENCY]);
    corelessFrequencyIndex = getFrequencyIndex(persist[FLASH_INDEX_CORELESS_FREQUENCY]);
}

/**
 * Start the PWM running by connecting the OC to the correct timer
 * @param no the throttle number = OC number
 * @param cored boolean for which timer to use as clock
 */
inline void startPWM(unsigned char no, char cored) {
    unsigned int * p;
    
    p = (unsigned int *)&OC1CON;
    p += (3 * no);
    // OC1CONbits.OCM = OCM_DISABLED;
    // OC1CONbits.OCTSEL = 0;  // Timer 2 (cored) or 1 for Timer 3 (coreless))
    if (cored) {
        *p = 0x2000;
    } else {
        *p = 0x2008;
    }
    *p |= 0x0006; // OC1CONbits.OCM = OCM_PWM_NO_FAULT;     // PWM with fault pin disabled
}

/**
 * Turn off the PWM OC.
 * @param no the throttle number
 */
inline void stopPWM(unsigned char no) {
    unsigned int * p;
    
    p = (unsigned int *)&OC1CON;
    p += (3 * no);
    *p = 0x2000;    // OC1CONbits.OCM = OCM_DISABLED;
    
    p = (unsigned int *)&OC1R;
    p += (3 * no);
    *p = 0;         // OC1R = 0;
    
    p = (unsigned int *)&OC1RS;
    p += (3 * no);
    *p = 0;         // OC1RS = 0;
}

/**
 * Set the PWM pulse width and hence actually change the loco speed.
 * @param no throttle number (0-3)
 * @param s speed (0-127)
 */
inline void setPWMwidth(unsigned char no, char s, unsigned int cored) {
    unsigned int * p;
    
    p = (unsigned int *)&OC1RS;
    p += (3 * no);
    if (cored) {
        *p = s*getCoredMultiplier();
    } else {
        *p = s*getCorelessMultiplier();
    }
    //cbusTransmit(8, persist.canId,0x11, OPC_DDRS,*p,0, s,getCoredMultiplier(),getCorelessMultiplier());
}

/**
 * Given a frequency we return the index into the conig
 */
inline unsigned int getFrequencyIndex(unsigned int frequency) {
    int i;
    for (i=0; i<MAX_FREQUENCY_INDEX; i++) {
        if (frequency >= frequencyConfig[i].frequency) {
            return i;
        }
    }
    return MAX_FREQUENCY_INDEX;
}

/**
 * Get the Cored prescalar value
 * @return 
 */
inline unsigned int getCoredPrescalar() {
    return frequencyConfig[coredFrequencyIndex].prescalar;
}

/**
 * Get the coreless prescalar value
 * @return 
 */
inline unsigned int getCorelessPrescalar() {
    return frequencyConfig[corelessFrequencyIndex].prescalar;
}

/**
 * Get the cored multiplier setting
 * @return 
 */
inline unsigned int getCoredMultiplier() {
    return frequencyConfig[coredFrequencyIndex].multiplier;
}

    
/**
 * Get the coreless multiplier setting
 * @return 
 */
inline unsigned int getCorelessMultiplier() {
    return frequencyConfig[corelessFrequencyIndex].multiplier;
}

