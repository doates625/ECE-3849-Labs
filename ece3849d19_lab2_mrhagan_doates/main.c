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
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "sysctl_pll.h"
#include "inc/hw_memmap.h"
#include "inc/tm4c1294ncpdt.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/adc.h"
#include "Crystalfontz128x128_ST7735.h"

// Macros
#define ADC_BUFFER_SIZE 2048
#define BUTTON_FIFO_SIZE 11

// Constants
const uint32_t CRYSTAL_FREQUENCY = 25000000;    // Crystal oscillator frequency [Hz]
const uint32_t ADC_INT_FREQUENCY = 1000000;     // ADC sample frequency [Hz]
const uint32_t ADC_OFFSET = 2048;               // ADC 0V offset
const uint32_t ADC_BITS = 12;                   // ADC bit count
const float VIN_RANGE = 3.3f;                   // Input voltage range [V]
const uint32_t PIXELS_PER_DIV = 20;             // Scope pixels per division
const uint32_t JOYSTICK_POS_THRESHOLD = 3595;   // JoyStick ADC positive threshold
const uint32_t JOYSTICK_NEG_THRESHOLD = 500;    // JoyStick ADC negative threshold

// Shared Global Variables
volatile uint32_t gSystemClockFrequency = 0;            // System clock frequency [Hz]
volatile uint32_t gTimeMilliseconds = 0;                // System time counter [ms]
volatile int32_t gADCBufferIndex = 0;                   // ADC FIFO read index
volatile uint16_t gADCBuffer[ADC_BUFFER_SIZE];          // ADC readings FIFO
volatile uint32_t gADCErrors = 0;                       // ADC overflow count
volatile uint32_t gWaveformBuffer[LCD_HORIZONTAL_MAX];  // ADC waveform buffer
volatile uint32_t gPixelBuffer[LCD_HORIZONTAL_MAX];     // Pixel coordinates of waveform
volatile uint32_t gButtonPressCount = 0;                // Button press count
volatile uint8_t gTrigState = 0;                        // Trigger state (0 = Rising, 1 = Falling)
volatile uint8_t gVoltScaleState = 3;                   // Voltage scale state (0 = 100mV, 1 = 200mV, 2 = 500mV, 3 = 1V)
tContext gContext;                                      // Graphics context handle

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

    // Initialize LCD driver
    Crystalfontz128x128_Init();
    Crystalfontz128x128_SetOrientation(LCD_ORIENTATION_UP);
    GrContextInit(&gContext, &g_sCrystalfontz128x128);
    GrContextFontSet(&gContext, &g_sFontFixed6x8);

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

/**
 * Waveform task
 * - Detects trigger in ADC buffer
 * - Copies trigger-centered waveform to gWaveformBuffer
 * - Triggers processing task
 */
void WaveformTaskFunc(UArg arg1, UArg arg2)
{
    // Enable interrupts
    IntMasterEnable();
    gADCErrors = 0;

    // Waveform copying loop
    while (1)
    {
        // Wait for trigger from processing task
        Semaphore_pend(semTriggerWaveformTask, BIOS_WAIT_FOREVER);

        // Find trigger location
        int32_t startIndex = gADCBufferIndex - (LCD_HORIZONTAL_MAX / 2);
        ADCIndexWrap(&startIndex);
        int32_t endIndex = startIndex - (ADC_BUFFER_SIZE / 2);
        ADCIndexWrap(&startIndex);
        int32_t trigIndex = startIndex;
        bool prevHigh = false;
        uint8_t trigState = gTrigState;
        while (true)
        {
            // Check for trigger condition
            bool currentHigh = gADCBuffer[trigIndex] > ADC_OFFSET;
            if(trigState == 0 && prevHigh && !currentHigh) break;
            if(trigState == 1 && !prevHigh && currentHigh) break;
            prevHigh = currentHigh;

            // Decrement trigger index
            trigIndex--;
            ADCIndexWrap(&trigIndex);
            if (trigIndex == endIndex)
            {
                trigIndex = startIndex;
                break;
            }
        }

        // Copy into waveform buffer
        startIndex = trigIndex - (LCD_HORIZONTAL_MAX / 2);
        ADCIndexWrap(&startIndex);
        endIndex = trigIndex + (LCD_HORIZONTAL_MAX / 2);
        ADCIndexWrap(&endIndex);
        int32_t copyIndex = startIndex;
        while (true)
        {
            // Copy single reading to waveform buffer
            int32_t pixelIndex = copyIndex - startIndex;
            ADCIndexWrap(&pixelIndex);
            gWaveformBuffer[pixelIndex] = gADCBuffer[copyIndex];

            // Increment index
            copyIndex++;
            ADCIndexWrap(&copyIndex);
            if (copyIndex == endIndex) break;
        }

        // Trigger processing task
        Semaphore_post(semTriggerProcessingTask);
    }
}

/**
 * Button scanning task
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
 * User input task
 * - Updates state variables from button Bailbox input
 */
void UserInputTaskFunc(UArg arg1, UArg arg2)
{
    uint8_t buttons;

    while (1)
    {
        // Pend on button mailbox
        Mailbox_pend(ButtonMailbox, &buttons, BIOS_WAIT_FOREVER);

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
}

/**
 * Display task
 * - Writes data from gPixelBuffer to LCD display
 */
void DisplayTaskFunc(UArg arg1, UArg arg2)
{
    while (1)
    {
        // Wait for trigger from ProcessingTask
        Semaphore_pend(semTriggerDisplayTask, BIOS_WAIT_FOREVER);

        // Fill screen with black
        GrContextForegroundSet(&gContext, ClrBlack);
        tRectangle rectFullScreen = {0, 0, GrContextDpyWidthGet(&gContext)-1, GrContextDpyHeightGet(&gContext)-1};
        GrRectFill(&gContext, &rectFullScreen);

        // Draw division lines
        uint32_t divIndex;
        for(divIndex = 4; divIndex < LCD_HORIZONTAL_MAX; divIndex += 20)
        {
            GrContextForegroundSet(&gContext, ClrDimGray);
            GrLineDrawH(&gContext, 0, LCD_HORIZONTAL_MAX-1, divIndex);
            GrLineDrawV(&gContext, divIndex, 0, LCD_HORIZONTAL_MAX-1);
        }

        // Draw sample pixels to LCD
        uint32_t pixelIndex;
        for(pixelIndex = 0; pixelIndex < LCD_HORIZONTAL_MAX-1; pixelIndex++)
        {
            uint32_t nextIndex = pixelIndex+1;
            GrContextForegroundSet(&gContext, ClrYellow);
            GrLineDraw(&gContext,
                       pixelIndex,
                       gPixelBuffer[pixelIndex],
                       nextIndex, gPixelBuffer[nextIndex]);
        }

        // Print text to LCD
        GrContextForegroundSet(&gContext, ClrWhite);

        // Print Voltage Scale
        char voltScaleStr[10];
        uint8_t voltScaleState = gVoltScaleState;
        switch (voltScaleState)
        {
        case 0:
            snprintf(voltScaleStr, sizeof(voltScaleStr), "100mV");
            break;
        case 1:
            snprintf(voltScaleStr, sizeof(voltScaleStr), "200mV");
            break;
        case 2:
            snprintf(voltScaleStr, sizeof(voltScaleStr), "500mV");
            break;
        case 3:
        default:
            snprintf(voltScaleStr, sizeof(voltScaleStr), "1.00V");
            break;
        }
        GrStringDraw(&gContext, voltScaleStr, sizeof(voltScaleStr), /*x*/ 9, /*y*/ 8, /*opaque*/ false);

        // Flush LCD frame buffer
        GrFlush(&gContext);
    }
}

/**
 * Waveform processing task
 * - Converts gWaveformBuffer to pixel coordinates
 * - Posts pixel coordinates in gPixelBuffer
 * - Triggers display task and processing task
 */
void ProcessingTaskFunc(UArg arg1, UArg arg2)
{
    while (1)
    {
        // Wait for trigger from Waveform task
        Semaphore_pend(semTriggerProcessingTask, BIOS_WAIT_FOREVER);

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
        float pixelPerAdc =
                (VIN_RANGE * PIXELS_PER_DIV)/((1 << ADC_BITS) *
                        voltsPerDiv);

        // Convert gWaveformBuffer to gPixelBuffer
        int32_t pixelIndex;
        for (pixelIndex = 0; pixelIndex < LCD_HORIZONTAL_MAX; pixelIndex++)
        {
            gPixelBuffer[pixelIndex] =
                    (int)(LCD_VERTICAL_MAX / 2) -
                    (int)(pixelPerAdc * ((int)gWaveformBuffer[pixelIndex] - (int)ADC_OFFSET));
        }

        // Trigger display and waveform tasks
        Semaphore_post(semTriggerWaveformTask);
        Semaphore_post(semTriggerDisplayTask);
    }
}

/**
 * Wraps index of ADC buffer
 */
void ADCIndexWrap(volatile int32_t* index)
{
    (*index) &= (ADC_BUFFER_SIZE - 1);
}
