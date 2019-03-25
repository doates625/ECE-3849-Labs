/*
 * Adc.c
 *
 *  Created on: Mar 23, 2019
 *      Author: doate
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
void AdcIsr(void)
{
    ADCIntClear(ADC1_BASE, 0);
    // TODO: Read ADC into global variable
    gAdcReading++;
}
