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
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"
#include "driverlib/adc.h"
#include "sysctl_pll.h"
#include <stdint.h>
#include <stdbool.h>

// ADC ISR
void ADC_ISR(void)
{
    ADCIntClear(ADC1_BASE, 0);
    if (ADC1_OSTAT_R & ADC_OSTAT_OV0) { // check for ADC FIFO overflow
        gADCErrors++;                   // count errors
        ADC1_OSTAT_R = ADC_OSTAT_OV0;   // clear overflow condition
    }
    gADCBuffer[
               gADCBufferIndex = ADC_BUFFER_WRAP(gADCBufferIndex + 1)
               ] = <...>;               // read sample from the ADC1 sequence 0 FIFO
    gAdcReading++;
}
