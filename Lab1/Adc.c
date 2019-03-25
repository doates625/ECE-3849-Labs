/*
 * Adc.c
 *
 *  Created on: Mar 23, 2019
 *      Author: Daniel Oates
 *              Matthew Hagan
 */

#include "Adc.h"
#include "Globals.h"
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "inc/tm4c1294ncpdt.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"
#include "driverlib/adc.h"
#include "sysctl_pll.h"
#include <stdint.h>
#include <stdbool.h>

#define ADC_BUFFER_WRAP(i) ((i) & (ADC_BUFFER_SIZE - 1)) // index wrapping macro

volatile int32_t gADCBufferIndex = 0;
volatile uint16_t gADCBuffer[ADC_BUFFER_SIZE];
volatile uint32_t gADCErrors = 0;

// ADC ISR
void ADC_ISR(void)
{
    // Clear ADC interrupt flag
    ADC1_ISC_R |= ADC_ISC_IN0;



    // Read ADC into FIFO
    if (ADC1_OSTAT_R & ADC_OSTAT_OV0) { // check for ADC FIFO overflow
        gADCErrors++;                   // count errors
        ADC1_OSTAT_R = ADC_OSTAT_OV0;   // clear overflow condition
    }
    gADCBuffer[
               gADCBufferIndex = ADC_BUFFER_WRAP(gADCBufferIndex + 1)
               ] = ADC1_SSFIFO0_R & ADC_SSFIFO0_DATA_M;
}
