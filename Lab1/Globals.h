/*
 * Globals.h
 *
 *  Created on: Mar 23, 2019
 *      Author: doate
 */

#ifndef GLOBALS_H_
#define GLOBALS_H_

#include <stdint.h>

#define ADC_BUFFER_SIZE 2048 // size must be a power of 2

extern volatile int32_t gADCBufferIndex;
extern volatile uint16_t gADCBuffer[ADC_BUFFER_SIZE];
extern volatile uint32_t gSystemClock;
extern volatile uint32_t gTime;
extern volatile uint32_t gADCErrors;

#endif /* GLOBALS_H_ */
