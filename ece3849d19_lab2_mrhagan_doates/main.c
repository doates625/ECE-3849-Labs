/*
 * ECE 3849 Lab2 starter project
 *
 * Gene Bogdanov    9/13/2017
 */
/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <xdc/cfg/global.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>

#include <stdint.h>
#include <stdbool.h>
#include "driverlib/interrupt.h"

// Dependencies
#include "sysctl_pll.h"
#include "driverlib/sysctl.h"
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/adc.h"
#include "inc/tm4c1294ncpdt.h"

// Macros
#define ADC_BUFFER_SIZE 2048
#define BUTTON_FIFO_SIZE 11

// User defined constants
const uint8_t ADC_INT_PRIORITY = 0;             // ADC ISR priority

//Volatile Global Variables
volatile int32_t gADCBufferIndex = 0;           // ADC FIFO read index
volatile uint16_t gADCBuffer[ADC_BUFFER_SIZE];  // ADC readings FIFO
volatile uint32_t gADCErrors = 0;               // ADC overflow count

// Function Prototype
void ADCIndexWrap(volatile int32_t* index);

// Main Function
int main(void)
{
    IntMasterDisable();

    // Configure ADC1 for sampler ISR
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_0);
    ADCSequenceDisable(ADC1_BASE, 0);
    ADCSequenceConfigure(ADC1_BASE, 0, ADC_TRIGGER_ALWAYS, 0);
    ADCSequenceStepConfigure(ADC1_BASE, 0, 0, ADC_CTL_CH3 | ADC_CTL_IE | ADC_CTL_END);
    ADCSequenceEnable(ADC1_BASE, 0);
    ADCIntEnable(ADC1_BASE, 0);
    IntPrioritySet(INT_ADC1SS0, ADC_INT_PRIORITY);
    IntEnable(INT_ADC1SS0);

    /* Start BIOS */
    BIOS_start();

    return (0);
}

void m3Hwi0_func(UArg arg1)
{
    // Clear ADC interrupt flag
    ADC1_ISC_R |= ADC_ISC_IN0;

    // Count ADC FIFO overflows
    if (ADC1_OSTAT_R & ADC_OSTAT_OV0) {
        gADCErrors++;
        ADC1_OSTAT_R = ADC_OSTAT_OV0;
    }

    // Increment ADC buffer index and read into buffer
    gADCBufferIndex++;
    ADCIndexWrap(&gADCBufferIndex);
    gADCBuffer[gADCBufferIndex] = ADC1_SSFIFO0_R & ADC_SSFIFO0_DATA_M;
}

void task0_func(UArg arg1, UArg arg2)
{
    IntMasterEnable();

    while (true) {
        // do nothing
    }
}


void ADCIndexWrap(volatile int32_t* index)
{
    (*index) &= (ADC_BUFFER_SIZE - 1);
}

