/*
 * File:   nodeVariables.c
 * Author: Ian Hogg
 *
 * Created on 05 March 2016, 18:14
 */

/**
 * Node variables contain global parameters that need to be persisted to Flash.
 */

#include "xc.h"
#include "can.h"
#include "nodeVariables.h"
#include "persist.h"
#include "pwm.h"
#include "throttle.h"
#include "sense.h"

/**
 * Get a node variable.
 */
inline unsigned int getNodeVar(unsigned int index) {
    switch (index) {
        case NV_CORELESS_FREQ_PARAM_HI:
            return (persist[FLASH_INDEX_CORELESS_FREQUENCY]>>8)&0xFF; // 2 bytes
        case NV_CORELESS_FREQ_PARAM_LO:
            return persist[FLASH_INDEX_CORELESS_FREQUENCY]&0xFF; // 2 bytes
        case NV_CORED_FREQ_PARAM_HI:
            return (persist[FLASH_INDEX_CORED_FREQUENCY]>>8)&0xFF;
        case NV_CORED_FREQ_PARAM_LO:
            return persist[FLASH_INDEX_CORED_FREQUENCY]&0xFF;
        case NV_ALARM_THRESHOLD_HI:
            return (persist[FLASH_INDEX_ALARM_THRESHOLD]>>8)&0xFF;
        case NV_ALARM_THRESHOLD_LO:
            return persist[FLASH_INDEX_ALARM_THRESHOLD]&0xFF;
        case NV_ALARM_CUTOFF_TIME:
            return persist[FLASH_INDEX_ALARM_CUTOFF_TIME]; // 1 byte
        case NV_TOTI_THRESHOLD_HI:
            return (persist[FLASH_INDEX_TOTI_THRESHOLD]>>8)&0xFF;
        case NV_TOTI_THRESHOLD_LO:
            return persist[FLASH_INDEX_TOTI_THRESHOLD]&0xFF;
        case NV_NUMBER_TO_AVERAGE:
            return persist[FLASH_INDEX_NUMBER_TO_AVERAGE]&0xFF;
        case NV_MIN_CORELESS_MEASURE:
            return persist[FLASH_INDEX_MIN_CORELESS];  // 1 byte
        case NV_MIN_CORED_MEASURE:
            return persist[FLASH_INDEX_MIN_CORED];  // 1 byte
    }
    return 0;
}

/**
 * Set and save a node variable.
 */
inline void setNodeVar(unsigned int index, unsigned int value) {
    unsigned int v;
    switch (index) {
        case NV_CORELESS_FREQ_PARAM_HI:
            v = (value << 8) | (persist[FLASH_INDEX_CORELESS_FREQUENCY]&0xFF);
            persist[FLASH_INDEX_CORELESS_FREQUENCY] = v;
            initPWMindexes();
            initTimer3();
            break;
        case NV_CORELESS_FREQ_PARAM_LO:
            v = (value & 0xFF) | (persist[FLASH_INDEX_CORELESS_FREQUENCY]&0xFF00);
            persist[FLASH_INDEX_CORELESS_FREQUENCY] = v;
            initPWMindexes();
            initTimer3();
            break;
        case NV_CORED_FREQ_PARAM_HI:
            v = (value << 8) | (persist[FLASH_INDEX_CORED_FREQUENCY]&0xFF);
            persist[FLASH_INDEX_CORED_FREQUENCY] = v;
            initPWMindexes();
            initTimer2();
            break;
        case NV_CORED_FREQ_PARAM_LO:
            v = (value & 0xFF) | (persist[FLASH_INDEX_CORED_FREQUENCY]&0xFF00);
            persist[FLASH_INDEX_CORED_FREQUENCY] = v;
            initPWMindexes();
            initTimer2();
            break;
        case NV_ALARM_THRESHOLD_HI:
            v = (value << 8) | (persist[FLASH_INDEX_ALARM_THRESHOLD]&0xFF);
            persist[FLASH_INDEX_ALARM_THRESHOLD] = v;
            break;
        case NV_ALARM_THRESHOLD_LO:
            v = (value & 0xFF) | (persist[FLASH_INDEX_ALARM_THRESHOLD]&0xFF00);
            persist[FLASH_INDEX_ALARM_THRESHOLD] = v;
            break;
        case NV_ALARM_CUTOFF_TIME:
            persist[FLASH_INDEX_ALARM_CUTOFF_TIME] = value &0x7F; // 1 byte
            break;
        case NV_TOTI_THRESHOLD_HI:
            v = (value << 8) | (persist[FLASH_INDEX_TOTI_THRESHOLD]&0xFF);
            persist[FLASH_INDEX_TOTI_THRESHOLD] = v;
            break;
        case NV_TOTI_THRESHOLD_LO:
            v = (value & 0xFF) | (persist[FLASH_INDEX_TOTI_THRESHOLD]&0xFF00);
            persist[FLASH_INDEX_TOTI_THRESHOLD] = v;
            break;
        case NV_NUMBER_TO_AVERAGE:
            persist[FLASH_INDEX_NUMBER_TO_AVERAGE] = value & 0xFF;
            if (persist[FLASH_INDEX_NUMBER_TO_AVERAGE] > CURRENT_READINGS_BUF_SIZE) {
                persist[FLASH_INDEX_NUMBER_TO_AVERAGE] = CURRENT_READINGS_BUF_SIZE;
            }
            break;
        case NV_MIN_CORELESS_MEASURE:
            persist[FLASH_INDEX_MIN_CORELESS] = value & 0x7F;  // 1 byte
            break;
        case NV_MIN_CORED_MEASURE:
            persist[FLASH_INDEX_MIN_CORED] = value & 0x7F;  // 1 byte
            break;
    }
    writeFlash();
}
