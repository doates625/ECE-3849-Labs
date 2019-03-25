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
#include <stdint.h>
#include <stdbool.h>
#include "sysctl_pll.h"
#include "inc/hw_memmap.h"
#include "inc/tm4c1294ncpdt.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "driverlib/adc.h"
#include "Crystalfontz128x128_ST7735.h"

// Macros
#define ADC_BUFFER_SIZE 2048

// Constants
const uint8_t ADC_INT_PRIORITY = 0;             // ADC ISR priority
const uint8_t BUTTON_INT_PRIORITY = 32;         // Button ISR priority
const uint32_t CRYSTAL_FREQUENCY = 25000000;    // Crystal oscillator frequency [Hz]
const uint32_t ADC_INT_FREQUENCY = 1000000;     // ADC sample frequency [Hz]
const uint32_t BUTTON_INT_FREQUENCY = 200;      // Button sample frequency [Hz]
const uint32_t ADC_OFFSET = 2048;               // ADC 0V offset
const float VIN_RANGE = 3.3f;                   // Input voltage range [V]
const uint32_t PIXELS_PER_DIV = 20;             // Scope pixels per division
const uint32_t ADC_BITS = 12;                   // ADC bit count

// Global Variables
volatile uint32_t gSystemClockFrequency = 0;    // System clock frequency [Hz]
volatile uint32_t gTimeMilliseconds = 0;        // System time counter [ms]
volatile int32_t gADCBufferIndex = 0;           // ADC buffer read index
volatile uint16_t gADCBuffer[ADC_BUFFER_SIZE];  // ADC readings buffer
volatile uint32_t gADCErrors = 0;               // ADC overflow count
uint32_t gPixelYCoords[LCD_HORIZONTAL_MAX];     // Oscilloscope pixel y-coordinates
int32_t gTrigAdcIndex = 0;                      // Trigger ADC buffer index

// Circularly wraps ADC buffer index to valid range
void ADCIndexWrap(volatile int32_t* index)
{
    (*index) &= (ADC_BUFFER_SIZE - 1);
}

// ADC ISR
void ADC_ISR(void)
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

// Button ISR
void Button_ISR(void)
{
    // Clear Timer interrupt flag
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    // Increment millisecond timer
    gTimeMilliseconds += 5;
}

// Main Function
int main(void)
{
    // Disable interrupts during configuration
    IntMasterDisable();

    // Enable FPU and permit ISRs to use it
    FPUEnable();
    FPULazyStackingEnable();

    // Initialize system clock to 120 MHz
    gSystemClockFrequency = SysCtlClockFreqSet(SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480, 120000000);

    // Initialize LCD driver
    Crystalfontz128x128_Init();
    Crystalfontz128x128_SetOrientation(LCD_ORIENTATION_UP);
    tContext sContext;
    GrContextInit(&sContext, &g_sCrystalfontz128x128);
    GrContextFontSet(&sContext, &g_sFontFixed6x8);

    // Configure Timer0A for button ISR
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    TimerDisable(TIMER0_BASE, TIMER_BOTH);
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER0_BASE, TIMER_A, (float)gSystemClockFrequency / BUTTON_INT_FREQUENCY - 0.5f);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    TimerEnable(TIMER0_BASE, TIMER_BOTH);
    IntPrioritySet(INT_TIMER0A, BUTTON_INT_PRIORITY);
    IntEnable(INT_TIMER0A);

    // Initialize ADC
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC1);

    // ADC clock configuration
    uint32_t pll_frequency = SysCtlFrequencyGet(CRYSTAL_FREQUENCY);
    uint32_t pll_divisor = (pll_frequency - 1) / (16 * ADC_INT_FREQUENCY) + 1;
    ADCClockConfigSet(ADC0_BASE, ADC_CLOCK_SRC_PLL | ADC_CLOCK_RATE_FULL, pll_divisor);
    ADCClockConfigSet(ADC1_BASE, ADC_CLOCK_SRC_PLL | ADC_CLOCK_RATE_FULL, pll_divisor);

    // Configure ADC for ADC ISR
    ADCSequenceDisable(ADC1_BASE, 0);
    ADCSequenceConfigure(ADC1_BASE, 0, ADC_TRIGGER_ALWAYS, 0);
    ADCSequenceStepConfigure(ADC1_BASE, 0, 0, ADC_CTL_CH3 | ADC_CTL_IE | ADC_CTL_END);
    ADCSequenceEnable(ADC1_BASE, 0);
    ADCIntEnable(ADC1_BASE, 0);
    IntPrioritySet(INT_ADC1SS0, ADC_INT_PRIORITY);
    IntEnable(INT_ADC1SS0);

    // Enable interrupts after configuration
    IntMasterEnable();

    // Local variables
    tRectangle rectFullScreen = {0, 0, GrContextDpyWidthGet(&sContext)-1, GrContextDpyHeightGet(&sContext)-1};

    // Graphics loop
    while (1)
    {

        // Find trigger location
        int32_t startIndex = gADCBufferIndex - (LCD_HORIZONTAL_MAX / 2);
        ADCIndexWrap(&startIndex);
        int32_t endIndex = startIndex - (ADC_BUFFER_SIZE / 2);
        ADCIndexWrap(&startIndex);
        int32_t testIndex = startIndex;
        bool prevHigh = false;
        while (true)
        {
            // Check for trigger condition
            bool currentLow = gADCBuffer[testIndex] < ADC_OFFSET;
            if (currentLow & prevHigh) break;
            prevHigh = !currentLow;

            // Decrement trigger index
            testIndex--;
            ADCIndexWrap(&testIndex);
            if (testIndex == endIndex)
            {
                testIndex = startIndex;
                break;
            }
        }
        gTrigAdcIndex = testIndex;

        // Calculate voltage scale
        float voltsPerDiv = 1.0f;
        float pixelPerAdc = (VIN_RANGE * PIXELS_PER_DIV)/((1 << ADC_BITS) * voltsPerDiv);

        // Copy into local pixel coordinate buffer
        startIndex = gTrigAdcIndex - (LCD_HORIZONTAL_MAX / 2);
        ADCIndexWrap(&startIndex);
        endIndex = gTrigAdcIndex + (LCD_HORIZONTAL_MAX / 2);
        ADCIndexWrap(&endIndex);
        int32_t copyIndex = startIndex;
        while (true)
        {
            // Calculate pixel coordinate
            int32_t pixelIndex = copyIndex - startIndex;
            ADCIndexWrap(&pixelIndex);
            gPixelYCoords[pixelIndex] = (int)(LCD_VERTICAL_MAX / 2) - (int)(pixelPerAdc * ((int)gADCBuffer[copyIndex] - (int)ADC_OFFSET));

            // Increment index
            copyIndex++;
            ADCIndexWrap(&copyIndex);
            if (copyIndex == endIndex) break;
        }

        // Fill screen with black
        GrContextForegroundSet(&sContext, ClrBlack);
        GrRectFill(&sContext, &rectFullScreen);

        // Draw sample pixels to LCD
        uint32_t pixelIndex;
        for(pixelIndex = 0; pixelIndex < LCD_HORIZONTAL_MAX-1; pixelIndex++)
        {
            uint32_t nextIndex = pixelIndex+1;
            GrContextForegroundSet(&sContext, ClrYellow);
            // GrPixelDraw(&sContext, pixelIndex, gPixelYCoords[pixelIndex]);
            GrLineDraw(&sContext, pixelIndex, gPixelYCoords[pixelIndex], nextIndex, gPixelYCoords[nextIndex]);
        }

        // Flush LCD frame buffer
        GrFlush(&sContext);
    }
}
