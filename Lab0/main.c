/**
 * main.c
 *
 * ECE-3849 Lab 0 code submission
 * Submitted XX-XX-2019
 *
 * Hardware:
 * - EK-TM4C1294XL LaunchPad
 * - BOOSTXL-EDUMKII BoosterPack
 *
 * Submitted by:
 * - Daniel Oates
 * - Matthew Hagan
 */

// Dependencies
#include <stdint.h>
#include <stdbool.h>
#include "driverlib/fpu.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "buttons.h"
#include "Crystalfontz128x128_ST7735.h"
#include <stdio.h>

// Global Variables
uint32_t gSystemClock;          // System clock frequency [Hz]
volatile uint32_t gTime = 0;    // Time [units of 0.01s]

// Main Function
int main(void)
{
    IntMasterDisable();

    // ADC Configuration
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIO?);
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC1);

    // ADC Clock Configuration
    uint32_t pll_frequency = SysCtlFrequencyGet(CRYSTAL_FREQUENCY);
    uint32_t pll_divisor = (pll_frequency - 1) / (16 * ADC_SAMPLING_RATE) + 1; // Round up
    ADCClockConfigSet(ADC0_BASE, ADC_CLOCK_SRC_PLL | ADC_CLOCK_RATE_FULL, pll_divisor);
    ADCClockConfigSet(ADC1_BASE, ADC_CLOCK_SRC_PLL | ADC_CLOCK_RATE_FULL, pll_divisor);
    ADCSequenceDisable(ADC1_BASE, 0);       // Disable before configuring
    ADCSequenceConfigure(ADC1_BASE, 0, ADC_TRIGGER_ALWAYS, 0);
    ADCSequenceStepConfigure(ADC1_BASE, 0, 0, ADC_CTRL_CH3 | ADC_CTL_IE | ADC_CTL_END);
    ADCSequenceEnable(ADC1_BASE, 0);        // Enable after configuring
    ADCIntEnable(ADC1_BASE, 0);             // Enable sequence 0 interrupt in the ADC1 peripheral
    IntPrioritySet(INT_ADC1SS0_TM4C123, 0); // Set ADC1 sequence 0 interrupt priority
    IntEnable(INT_ADC1SS0_TM4C123);         // Enable ADC1 sequence 0 interrupt

    // Enable FPU and permit ISRs to use it
    FPUEnable();
    FPULazyStackingEnable();

    // Init system clock to 120 MHz
    gSystemClock = SysCtlClockFreqSet(SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480, 120000000);

    // Init LCD driver and set screen orientation
    Crystalfontz128x128_Init();
    Crystalfontz128x128_SetOrientation(LCD_ORIENTATION_UP);

    // Init grlib graphics context and select font
    tContext sContext;
    GrContextInit(&sContext, &g_sCrystalfontz128x128);
    GrContextFontSet(&sContext, &g_sFontFixed6x8);

    // Initialize buttons
    ButtonInit();
    IntMasterEnable();

    // Local variables
    uint32_t time;      // Copy of gTime
    uint32_t buttons;   // Copy of gButtons
    char str_time[50];  // String buffer for time
    char str_butt[50];  // String buffer for buttons
    tRectangle rectFullScreen = {0, 0, GrContextDpyWidthGet(&sContext)-1, GrContextDpyHeightGet(&sContext)-1};

    // Graphics loop
    while (true)
    {
        // Fill screen with black
        GrContextForegroundSet(&sContext, ClrBlack);
        GrRectFill(&sContext, &rectFullScreen);

        // Get time and convert to string
        time = gTime;
        uint8_t ff = time % 100;
        uint8_t ss = (time / 100) % 60;
        uint8_t mm = (time / 6000);
        snprintf(str_time, sizeof(str_time), "Time = %02u:%02u:%02u", mm, ss, ff);

        // Print time string as yellow text to LCD
        GrContextForegroundSet(&sContext, ClrYellow);
        GrStringDraw(&sContext, str_time, /*length*/ -1, /*x*/ 0, /*y*/ 0, /*opaque*/ false);

        // Display gButtons bitmap
        buttons = gButtons;
        // snprintf(str_butt, sizeof(str_butt), "Buttons = %04u", buttons);
        snprintf(str_butt, sizeof(str_butt), "Buttons = ");
        uint8_t i;
        for(i = 0; i < 9; i++)
        {
            if(buttons & (1 << i))
            {
                str_butt[18 - i] = '1';
            }
            else
            {
                str_butt[18 - i] = '0';
            }
        }

        // Print buttons bitmap as yellow text to LCD
        GrContextForegroundSet(&sContext, ClrYellow);
        GrStringDraw(&sContext, str_butt, /*length*/ -1, /*x*/ 0, /*y*/ 20, /*opaque*/ false);

        // Flush LCD frame buffer
        GrFlush(&sContext);
    }
}