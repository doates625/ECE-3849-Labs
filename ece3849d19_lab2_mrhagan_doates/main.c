/**
 * main.c
 *
 * ECE-3849 Lab 2 code submission
 * Submitted X-XX-2019
 *
 * Hardware:
 * - EK-TM4C1294XL LaunchPad
 * - BOOSTXL-EDUMKII BoosterPack
 *
 * Submitted by:
 * - Daniel Oates
 * - Matthew Hagan
 */

// XDCtools Header files
#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <xdc/cfg/global.h>

// BIOS Header files
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>

// General Header Files
#include <stdint.h>
#include <stdbool.h>
#include "sysctl_pll.h"
#include "inc/hw_memmap.h"
#include "inc/tm4c1294ncpdt.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/adc.h"

// Macros
#define ADC_BUFFER_SIZE 2048
#define BUTTON_FIFO_SIZE 11

// Constants
const uint32_t CRYSTAL_FREQUENCY = 25000000; // Crystal oscillator frequency [Hz]
const uint32_t ADC_INT_FREQUENCY = 1000000;     // ADC sample frequency [Hz]
const uint32_t JOYSTICK_POS_THRESHOLD = 3595;   // JoyStick ADC positive threshold
const uint32_t JOYSTICK_NEG_THRESHOLD = 500;    // JoyStick ADC negative threshold

// Shared Global Variables
volatile uint32_t gSystemClockFrequency = 0;    // System clock frequency [Hz]
volatile uint32_t gTimeMilliseconds = 0;        // System time counter [ms]
volatile int32_t gADCBufferIndex = 0;           // ADC FIFO read index
volatile uint16_t gADCBuffer[ADC_BUFFER_SIZE];  // ADC readings FIFO
volatile uint32_t gADCErrors = 0;               // ADC overflow count
volatile uint32_t gButtonPressCount = 0;        // Button press count

// Function Prototypes
void ADCIndexWrap(volatile int32_t* index);

// Main Function
int main(void)
{
    // Disable interrupts during configuration
    IntMasterDisable();

    // Initialize system clock to 120 MHz
    gSystemClockFrequency = SysCtlClockFreqSet(SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480, 120000000);

    // ADC Clock Configuration
    uint32_t pll_frequency = SysCtlFrequencyGet(CRYSTAL_FREQUENCY);
    uint32_t pll_divisor = (pll_frequency - 1) / (16 * ADC_INT_FREQUENCY) + 1;
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC1);
    ADCClockConfigSet(ADC0_BASE, ADC_CLOCK_SRC_PLL | ADC_CLOCK_RATE_FULL, pll_divisor);
    ADCClockConfigSet(ADC1_BASE, ADC_CLOCK_SRC_PLL | ADC_CLOCK_RATE_FULL, pll_divisor);

    // Configure BoosterPack GPIO

    // BoosterPack buttons 1 and 2 (PH1 and PK6)
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOH);
    GPIOPinTypeGPIOInput(GPIO_PORTH_BASE, GPIO_PIN_1);
    GPIOPadConfigSet(GPIO_PORTH_BASE, GPIO_PIN_1, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);
    GPIOPinTypeGPIOInput(GPIO_PORTK_BASE, GPIO_PIN_6);
    GPIOPadConfigSet(GPIO_PORTK_BASE, GPIO_PIN_6, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    // JoyStick Horizontal-X (Analog AIN13, GPIO PD2)
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_2);

    // JoyStick Vertical-Y (Analog AIN17, GPIO PK1)
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);
    GPIOPinTypeADC(GPIO_PORTK_BASE, GPIO_PIN_1);

    // Configure ADC modules

    // Configure ADC0 for JoyStick
    ADCSequenceDisable(ADC0_BASE, 0);
    ADCSequenceConfigure(ADC0_BASE, 0, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceStepConfigure(ADC0_BASE, 0, 0, ADC_CTL_CH13);
    ADCSequenceStepConfigure(ADC0_BASE, 0, 1, ADC_CTL_CH17 | ADC_CTL_IE | ADC_CTL_END);
    ADCSequenceEnable(ADC0_BASE, 0);

    // Configure ADC1 for sampler ISR
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_0);
    ADCSequenceDisable(ADC1_BASE, 0);
    ADCSequenceConfigure(ADC1_BASE, 0, ADC_TRIGGER_ALWAYS, 0);
    ADCSequenceStepConfigure(ADC1_BASE, 0, 0, ADC_CTL_CH3 | ADC_CTL_IE | ADC_CTL_END);
    ADCSequenceEnable(ADC1_BASE, 0);
    ADCIntEnable(ADC1_BASE, 0);

    // Start BIOS
    BIOS_start();

    // Code should never get here
    return 0;
}

/**
 * ADC sampling ISR
 */
void AdcHwiFunc(UArg arg1)
{
    // Clear ADC interrupt flag
    ADC1_ISC_R |= ADC_ISC_IN0;

    // Count ADC FIFO overflows
    if (ADC1_OSTAT_R & ADC_OSTAT_OV0)
    {
        gADCErrors++;
        ADC1_OSTAT_R = ADC_OSTAT_OV0;
    }

    // Increment ADC buffer index and read into buffer
    gADCBufferIndex++;
    ADCIndexWrap(&gADCBufferIndex);
    gADCBuffer[gADCBufferIndex] = ADC1_SSFIFO0_R & ADC_SSFIFO0_DATA_M;
}

/**
 * Button clock callback
 * - Triggers button scanning task
 */
void ButtonClockFunc()
{
    Semaphore_post(semTriggerButtonScan);
}

/*
 * The Waveform Task (highest priority) should also block on a Semaphore. When signaled,
 it should search for trigger in the ADC buffer, copy the triggered waveform into the waveform
 buffer, signal the Processing Task, and block again. This is the highest priority task to make sure
 the trigger search completes before the ADC overwrites the buffer. Protect access to the shared
 waveform buffer, if necessary
 * - Globally enables interrupts once
 */
void WaveformTaskFunc(UArg arg1, UArg arg2)
{
    // Enable interrupts
    IntMasterEnable();
    gADCErrors = 0;

    while (1)
    {
        Semaphore_pend(semTriggerWaveformTask, BIOS_WAIT_FOREVER);
    }

    /*
     Semaphore_pend();//TODO : Add processing task that the semaphore pends on
     <search for trigger in ADC buffer>
     <copy waveform>
     Semaphore_post();//TODO : Add processingTask that the semaphore posts on
     */
}

/**
 * Button scanning tasl
 * - Triggered by button clock
 * - Posts to button mailbox
 */
void ButtonTaskFunc(UArg arg1, UArg arg2)
{
    while (1)
    {
        // Wait for trigger from button clock
        Semaphore_pend(semTriggerButtonScan, BIOS_WAIT_FOREVER);

        // Buttons BitMap:
        // 0 -> BoosterPack Button 1
        // 1 -> BoosterPack Button 2
        // 2 -> JoyStick Right
        // 3 -> JoyStick Left
        // 4 -> JoyStick Up
        // 5 -> JoyStick Down
        // 6-7 (empty)

        // Read GPIO buttons
        uint8_t buttons = 0;
        buttons |= (~GPIOPinRead(GPIO_PORTH_BASE, 0xff) & GPIO_PIN_1) >> 1; // BoosterPack Button 1
        buttons |= (~GPIOPinRead(GPIO_PORTK_BASE, 0xff) & GPIO_PIN_6) >> 5; // BoosterPack Button 2

        // Read JoyStick ADCs
        uint32_t joystickADCs[2];
        ADCProcessorTrigger(ADC0_BASE, 0);
        while (!ADCIntStatus(ADC0_BASE, 0, false));
        ADCSequenceDataGet(ADC0_BASE, 0, joystickADCs);
        ADCIntClear(ADC0_BASE, 0);

        // Map JoyStick to 'buttons'
        if (joystickADCs[0] > JOYSTICK_POS_THRESHOLD)
        {
            buttons |= (1 << 2);
        }
        else if (joystickADCs[0] < JOYSTICK_NEG_THRESHOLD)
        {
            buttons |= (1 << 3);
        }
        if (joystickADCs[1] > JOYSTICK_POS_THRESHOLD)
        {
            buttons |= (1 << 4);
        }
        else if (joystickADCs[1] < JOYSTICK_NEG_THRESHOLD)
        {
            buttons |= (1 << 5);
        }

        // Push buttons to FIFO
        static uint8_t oldButtons = 0;
        if (buttons != 0 && buttons != oldButtons)
        {
            Mailbox_post(ButtonMailbox, &buttons, BIOS_NO_WAIT);
            gButtonPressCount++;
        }
        oldButtons = buttons;

        // Increment millisecond timer
        gTimeMilliseconds += 5;
    }
}

/**
 * Wraps index of ADC buffer
 */
void ADCIndexWrap(volatile int32_t* index)
{
    (*index) &= (ADC_BUFFER_SIZE - 1);
}
