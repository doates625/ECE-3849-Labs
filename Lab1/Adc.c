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

// ADC Initialization
void AdcInit(void)
{
    // Enable ADC control
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC1);

    // ADC clock configuration
    uint32_t pll_frequency = SysCtlFrequencyGet(CRYSTAL_FREQUENCY);
    uint32_t pll_divisor = (pll_frequency - 1) / (16 * ADC_SAMPLING_RATE) + 1; // Round up
    ADCClockConfigSet(ADC0_BASE, ADC_CLOCK_SRC_PLL | ADC_CLOCK_RATE_FULL, pll_divisor);
    ADCClockConfigSet(ADC1_BASE, ADC_CLOCK_SRC_PLL | ADC_CLOCK_RATE_FULL, pll_divisor);

    // ADC interrupt configuration
    ADCSequenceDisable(ADC1_BASE, 0);       // Disable before configuring
    ADCSequenceConfigure(ADC1_BASE, 0, ADC_TRIGGER_ALWAYS, 0);
    ADCSequenceStepConfigure(ADC1_BASE, 0, 0, ADC_CTL_CH3 | ADC_CTL_IE | ADC_CTL_END);
    ADCSequenceEnable(ADC1_BASE, 0);        // Enable after configuring
    ADCIntEnable(ADC1_BASE, 0);             // Enable sequence 0 interrupt in the ADC1 peripheral
    IntPrioritySet(INT_ADC1SS0_TM4C123, 0); // Set ADC1 sequence 0 interrupt priority
    IntEnable(INT_ADC1SS0_TM4C123);         // Enable ADC1 sequence 0 interrupt
}

// ADC ISR
void AdcIsr(void)
{
    ADCIntClear(ADC1_BASE, 0);
    // TODO: Read ADC into global variable
    gAdcReading++;
}
