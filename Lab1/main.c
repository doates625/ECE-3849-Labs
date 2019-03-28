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
#include <stdio.h>
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
#define BUTTON_FIFO_SIZE 11

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
const uint32_t JOYSTICK_POS_THRESHOLD = 3595;   // JoyStick ADC positive threshold
const uint32_t JOYSTICK_NEG_THRESHOLD = 500;    // JoyStick ADC negative threshold

// Volatile Global Variables
volatile uint32_t gSystemClockFrequency = 0;    // System clock frequency [Hz]
volatile uint32_t gTimeMilliseconds = 0;        // System time counter [ms]
volatile int32_t gADCBufferIndex = 0;           // ADC FIFO read index
volatile uint16_t gADCBuffer[ADC_BUFFER_SIZE];  // ADC readings FIFO
volatile uint32_t gADCErrors = 0;               // ADC overflow count
volatile int8_t gButtonFifoHead = 0;            // Button FIFO head
volatile int8_t gButtonFifoTail = 0;            // Button FIFO tail
volatile uint8_t gButtonFifo[BUTTON_FIFO_SIZE]; // Button presses FIFO
volatile uint32_t gButtonPressCount = 0;        // Button press count

// Non-volatile Global Variables
uint8_t gTrigState = 0;                         // Trigger state (0 = Rising, 1 = Falling)
uint32_t gPixelYCoords[LCD_HORIZONTAL_MAX];     // Oscilloscope pixel y-coordinates
int32_t gTrigAdcIndex = 0;                      // Trigger ADC buffer index
uint8_t gVoltScaleState = 3;                    // Voltage scale state (0 = 100mV, 1 = 200mV, 2 = 500mV, 3 = 1V)
uint32_t gCpuCountUnloaded = 0;                 // CPU unloaded count (CPU load estimation)
uint32_t gCpuCountLoaded = 0;                   // CPU loaded count (CPU load estimation)

// Function Templates
void ADCIndexWrap(volatile int32_t* index);
void ADC_ISR(void);
void Button_ISR(void);
uint8_t ButtonFifoPut(uint8_t val);
uint8_t ButtonFifoGet(uint8_t *val);
uint32_t CpuLoadCount();

// Main Function
int main(void)
{
    // Disable interrupts during configuration
    IntMasterDisable();

    // Enable FPU and permit ISRs to use it
    FPUEnable();
    FPULazyStackingEnable();

    // Initialize LCD driver
    Crystalfontz128x128_Init();
    Crystalfontz128x128_SetOrientation(LCD_ORIENTATION_UP);
    tContext sContext;
    GrContextInit(&sContext, &g_sCrystalfontz128x128);
    GrContextFontSet(&sContext, &g_sFontFixed6x8);

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

    // Configure clocks

    // Initialize system clock to 120 MHz
    gSystemClockFrequency = SysCtlClockFreqSet(SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480, 120000000);

    // ADC Clock Configuration
    uint32_t pll_frequency = SysCtlFrequencyGet(CRYSTAL_FREQUENCY);
    uint32_t pll_divisor = (pll_frequency - 1) / (16 * ADC_INT_FREQUENCY) + 1;
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC1);
    ADCClockConfigSet(ADC0_BASE, ADC_CLOCK_SRC_PLL | ADC_CLOCK_RATE_FULL, pll_divisor);
    ADCClockConfigSet(ADC1_BASE, ADC_CLOCK_SRC_PLL | ADC_CLOCK_RATE_FULL, pll_divisor);

    // Configure interrupts

    // Configure Timer3A for CPU load estimation
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER3);
    TimerDisable(TIMER3_BASE, TIMER_BOTH);
    TimerConfigure(TIMER3_BASE, TIMER_CFG_ONE_SHOT);
    TimerLoadSet(TIMER3_BASE, TIMER_A, gSystemClockFrequency / 100);

    // Get unloaded CPU count
    gCpuCountUnloaded = CpuLoadCount();

    // Configure Timer0A for button ISR
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    TimerDisable(TIMER0_BASE, TIMER_BOTH);
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER0_BASE, TIMER_A, (float)gSystemClockFrequency / BUTTON_INT_FREQUENCY - 0.5f);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    TimerEnable(TIMER0_BASE, TIMER_BOTH);
    IntPrioritySet(INT_TIMER0A, BUTTON_INT_PRIORITY);
    IntEnable(INT_TIMER0A);

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
    IntPrioritySet(INT_ADC1SS0, ADC_INT_PRIORITY);
    IntEnable(INT_ADC1SS0);

    // Enable interrupts after configuration
    IntMasterEnable();

    // Graphics loop
    while (1)
    {
        // Read button commands
        uint8_t buttons;
        while (ButtonFifoGet(&buttons))
        {
            // Handle trigger state
            if (buttons & (1 << 0))
            {
                // BoosterPack Button 1 -> Rising Trigger
                gTrigState = 0;
            }
            else if (buttons & (1 << 1))
            {
                // BoosterPack Button 2 -> Falling Trigger
                gTrigState = 1;
            }

            // Handle voltage scale
            if (buttons & (1 << 4))
            {
                // JoyStick Up -> Increase Voltage scale
                if (gVoltScaleState < 3)
                {
                    gVoltScaleState++;
                }
            }
            if (buttons & (1 << 5))
            {
                // JoyStick Down -> Decrease Voltage scale
                if (gVoltScaleState > 0)
                {
                    gVoltScaleState--;
                }
            }
        }

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
            bool currentHigh = gADCBuffer[testIndex] > ADC_OFFSET;
            if(gTrigState == 0 && prevHigh && !currentHigh) break;
            if(gTrigState == 1 && !prevHigh && currentHigh) break;
            prevHigh = currentHigh;

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
        float voltsPerDiv;
        switch (gVoltScaleState)
        {
        case 0:
            voltsPerDiv = 0.1f;
            break;
        case 1:
            voltsPerDiv = 0.2f;
            break;
        case 2:
            voltsPerDiv = 0.5f;
            break;
        case 3:
        default:
            voltsPerDiv = 1.0f;
            break;
        }
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
        tRectangle rectFullScreen = {0, 0, GrContextDpyWidthGet(&sContext)-1, GrContextDpyHeightGet(&sContext)-1};
        GrRectFill(&sContext, &rectFullScreen);

        // Draw division lines
        uint32_t divIndex;
        for(divIndex = 4; divIndex < LCD_HORIZONTAL_MAX; divIndex += 20)
        {
            GrContextForegroundSet(&sContext, ClrDimGray);
            GrLineDrawH(&sContext, 0, LCD_HORIZONTAL_MAX-1, divIndex);
            GrLineDrawV(&sContext, divIndex, 0, LCD_HORIZONTAL_MAX-1);
        }

        // Draw sample pixels to LCD
        uint32_t pixelIndex;
        for(pixelIndex = 0; pixelIndex < LCD_HORIZONTAL_MAX-1; pixelIndex++)
        {
            uint32_t nextIndex = pixelIndex+1;
            GrContextForegroundSet(&sContext, ClrYellow);
            GrLineDraw(&sContext, pixelIndex, gPixelYCoords[pixelIndex], nextIndex, gPixelYCoords[nextIndex]);
        }

        // Print text to LCD
        GrContextForegroundSet(&sContext, ClrWhite);

        // Estimate and print CPU load
        gCpuCountLoaded = CpuLoadCount();
        float cpuLoad = 1.0f - ((float)gCpuCountLoaded)/((float)gCpuCountUnloaded);
        char cpuLoadStr[20];
        snprintf(cpuLoadStr, sizeof(cpuLoadStr), "CPU Load: %.1f%%", cpuLoad * 100.0f);
        GrStringDraw(&sContext, cpuLoadStr, sizeof(cpuLoadStr), /*x*/ 9, /*y*/ 113, /*opaque*/ false);

        // Flush LCD frame buffer
        GrFlush(&sContext);
    }
}

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
    while(!ADCIntStatus(ADC0_BASE, 0, false));
    ADCSequenceDataGet(ADC0_BASE, 0, joystickADCs);
    ADCIntClear(ADC0_BASE, 0);

    // Map JoyStick to 'buttons'
    if(joystickADCs[0] > JOYSTICK_POS_THRESHOLD)
    {
        buttons |= (1 << 2);
    }
    else if(joystickADCs[0] < JOYSTICK_NEG_THRESHOLD)
    {
        buttons |= (1 << 3);
    }
    if(joystickADCs[1] > JOYSTICK_POS_THRESHOLD)
    {
        buttons |= (1 << 4);
    }
    else if(joystickADCs[1] < JOYSTICK_NEG_THRESHOLD)
    {
        buttons |= (1 << 5);
    }

    // Push buttons to FIFO
    static uint8_t oldButtons = 0;
    if (buttons != 0 && buttons != oldButtons)
    {
        ButtonFifoPut(buttons);
        gButtonPressCount++;
    }
    oldButtons = buttons;

    // Increment millisecond timer
    gTimeMilliseconds += 5;
}

// Puts value into FIFO (returns 1 if not full)
uint8_t ButtonFifoPut(uint8_t val)
{
    int8_t newTail = gButtonFifoTail + 1;
    if (newTail >= BUTTON_FIFO_SIZE)
    {
        newTail = 0;
    }
    if (gButtonFifoHead != newTail)
    {
        gButtonFifo[gButtonFifoTail] = val;
        gButtonFifoTail = newTail;
        return 1;
    }
    return 0;
}

// Gets value from FIFO (returns 1 if not empty)
uint8_t ButtonFifoGet(uint8_t *val)
{
    if (gButtonFifoHead != gButtonFifoTail)
    {
        *val = gButtonFifo[gButtonFifoHead];
        gButtonFifoHead++;
        if (gButtonFifoHead >= BUTTON_FIFO_SIZE)
        {
            gButtonFifoHead = 0;
        }
        return 1;
    }
    return 0;
}

// Counter for CPU load estimation using TIMER3A
uint32_t CpuLoadCount()
{
    uint32_t count = 0;
    TimerIntClear(TIMER3_BASE, TIMER_TIMA_TIMEOUT);
    TimerEnable(TIMER3_BASE, TIMER_A);
    while (!(TimerIntStatus(TIMER3_BASE, false) & TIMER_TIMA_TIMEOUT))
    {
        count++;
    }
    return count;
}
