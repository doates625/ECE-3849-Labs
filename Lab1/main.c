/**
 * main.c
 *
 * ECE-3849 Lab 1 code submission
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
// #include "Globals.h"
#include <stdint.h>
#include <stdbool.h>
#include "Globals.h"
#include "Adc.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/fpu.h"
#include "Crystalfontz128x128_ST7735.h"

// Global variables
volatile uint32_t gSystemClock = 0;
volatile uint32_t gAdcReading = 0;

// Main Function
int main(void)
{
    // Disable interrupts during configuration
    IntMasterDisable();

    // Initialize ADC
    AdcInit();

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
    // ButtonInit(); // NO BUTTONS YET***
    IntMasterEnable();

    // Local variables
    tRectangle rectFullScreen = {0, 0, GrContextDpyWidthGet(&sContext)-1, GrContextDpyHeightGet(&sContext)-1};

    // Graphics loop
    while (1)
    {
        // Fill screen with black
        GrContextForegroundSet(&sContext, ClrBlack);
        GrRectFill(&sContext, &rectFullScreen);

        // Write more things to LCD...

        // Flush LCD frame buffer
        GrFlush(&sContext);
    }
}
