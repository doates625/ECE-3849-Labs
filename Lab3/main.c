/**
 * main.c
 *
 * ECE-3849 Lab 3 code submission
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
#include "inc/tm4c1294ncpdt.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/adc.h"
#include "driverlib/timer.h"
#include "driverlib/udma.h"
#include "driverlib/comp.h"
#include "driverlib/pin_map.h"
#include "driverlib/pwm.h"
#include "Crystalfontz128x128_ST7735.h"

// Kiss FFT Library
#include <math.h>
#include "kiss_fft.h"
#include "_kiss_fft_guts.h"

// DMA Settings
#pragma DATA_ALIGN(gDMAControlTable, 1024) // Address alignment required
tDMAControlTable gDMAControlTable[64];     // uDMA control table (global)

// Macros
#define ADC_BUFFER_SIZE 2048
#define BUTTON_FIFO_SIZE 11
#define FFT_BUFFER_SIZE 1024
#define PI 3.14159265358979f
#define KISS_FFT_CFG_SIZE (sizeof(struct kiss_fft_state) + sizeof(kiss_fft_cpx)*(FFT_BUFFER_SIZE - 1))
#define PWM_PERIOD 258                          // PWM period = 2^8 + 2 system clock cycles
#define PWM_WAVEFORM_INDEX_BITS 10
#define PWM_WAVEFORM_TABLE_SIZE (1 << PWM_WAVEFORM_INDEX_BITS)

// Constants
const uint32_t CRYSTAL_FREQUENCY = 25000000;    // Crystal oscillator frequency [Hz]
const uint32_t ADC_INT_FREQUENCY = 2000000;     // ADC sample frequency [Hz]
const uint32_t ADC_OFFSET = 2048;               // ADC 0V offset
const uint32_t ADC_BITS = 12;                   // ADC bit count
const float VIN_RANGE = 3.3f;                   // Input voltage range [V]
const uint32_t PIXELS_PER_DIV = 20;             // Scope pixels per division
const uint32_t JOYSTICK_POS_THRESHOLD = 3595;   // JoyStick ADC positive threshold
const uint32_t JOYSTICK_NEG_THRESHOLD = 500;    // JoyStick ADC negative threshold
const uint32_t gPhaseIncrement = 26;             // phase increment for 18 kHz
//TODO: the Exact number was 26.4, so either use 26 or 27


// Shared Global Variables
volatile uint32_t gSystemClockFrequency = 0;            // System clock frequency [Hz]
volatile uint32_t gTimeMilliseconds = 0;                // System time counter [ms]
volatile uint16_t gADCBuffer[ADC_BUFFER_SIZE];          // ADC readings buffer
volatile uint32_t gWaveformBuffer[LCD_HORIZONTAL_MAX];  // ADC waveform buffer
volatile uint32_t gFFTBuffer[FFT_BUFFER_SIZE];          // FFT copy buffer
volatile uint32_t gPixelBuffer[LCD_HORIZONTAL_MAX];     // Pixel coordinates of waveform
volatile uint32_t gButtonPressCount = 0;                // Button press count
volatile uint8_t gTrigState = 0;                        // Trigger state (0 = Rising, 1 = Falling)
volatile uint8_t gVoltScaleState = 3;                   // Voltage scale state (0 = 100mV, 1 = 200mV, 2 = 500mV, 3 = 1V)
volatile uint8_t gDisplayState = 0;                     // Display state (0 = waveform, 1 = FFT)
volatile bool gDMAPrimary = true;                       // Flag for DMA occurring in primary channel (false = 2nd channel)
volatile uint32_t gPeriodSum = 0;                       // Input signal period sum since last reset [clocks]
volatile uint32_t gPeriodCount = 0;                     // Number of input periods since last reset
volatile float gInputFrequency = 0.0f;                  // Input signal frequency estimate [Hz]
volatile uint32_t gPhase = 0;                      // phase accumulator
volatile uint8_t gPWMWaveformTable[PWM_WAVEFORM_TABLE_SIZE] = {0}; //PWM output table with a given table size
// Non-Shared global variables
tContext gContext;              // Graphics context handle
uint32_t gCpuCountUnloaded = 0; // CPU unloaded count (CPU load estimation)
uint32_t gCpuCountLoaded = 0;   // CPU loaded count (CPU load estimation)

// FFT Variables
static char kiss_fft_cfg_buffer[KISS_FFT_CFG_SIZE]; // FFT config memory
size_t buffer_size = KISS_FFT_CFG_SIZE;             // Config buffer size
kiss_fft_cfg cfg;                                   // Config buffer object
static kiss_fft_cpx in[FFT_BUFFER_SIZE];            // FFT input
static kiss_fft_cpx out[FFT_BUFFER_SIZE];           // FFT output

// Function Prototypes
void ADCIndexWrap(volatile int32_t* index);
int32_t getADCBufferIndex(void);
uint32_t cpuLoadCount(void);

/**
 * Main Function
 */

int main(void)
{
    /**
     * Disable Interrupts
     */

    IntMasterDisable();

    /**
     * Clock Configuration
     */

    // Initialize system clock to 120 MHz
    gSystemClockFrequency = SysCtlClockFreqSet(SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480, 120000000);

    // ADC Clock Configuration
    uint32_t pll_frequency = SysCtlFrequencyGet(CRYSTAL_FREQUENCY);
    uint32_t pll_divisor = (pll_frequency - 1) / (16 * ADC_INT_FREQUENCY) + 1;
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC1);
    ADCClockConfigSet(ADC0_BASE, ADC_CLOCK_SRC_PLL | ADC_CLOCK_RATE_FULL, pll_divisor);
    ADCClockConfigSet(ADC1_BASE, ADC_CLOCK_SRC_PLL | ADC_CLOCK_RATE_FULL, pll_divisor);

    /**
     * Button and Joystick GPIO
     */

    // BoosterPack buttons 1 and 2 (PH1 and PK6)
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOH);
    GPIOPinTypeGPIOInput(GPIO_PORTH_BASE, GPIO_PIN_1);
    GPIOPadConfigSet(GPIO_PORTH_BASE, GPIO_PIN_1, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);
    GPIOPinTypeGPIOInput(GPIO_PORTK_BASE, GPIO_PIN_6);
    GPIOPadConfigSet(GPIO_PORTK_BASE, GPIO_PIN_6, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    // Board button 1 (PJ0)
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);
    GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE, GPIO_PIN_0);
    GPIOPadConfigSet(GPIO_PORTJ_BASE, GPIO_PIN_0, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    // JoyStick Horizontal-X (Analog AIN13, GPIO PD2)
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_2);

    // JoyStick Vertical-Y (Analog AIN17, GPIO PK1)
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);
    GPIOPinTypeADC(GPIO_PORTK_BASE, GPIO_PIN_1);

    // Configure FFT module
    cfg = kiss_fft_alloc(FFT_BUFFER_SIZE, 0, kiss_fft_cfg_buffer, &buffer_size);

    /**
     * ADC Modules
     */

    // Configure ADC0 for JoyStick
    ADCSequenceDisable(ADC0_BASE, 0);
    ADCSequenceConfigure(ADC0_BASE, 0, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceStepConfigure(ADC0_BASE, 0, 0, ADC_CTL_CH13);
    ADCSequenceStepConfigure(ADC0_BASE, 0, 1, ADC_CTL_CH17 | ADC_CTL_IE | ADC_CTL_END);
    ADCSequenceEnable(ADC0_BASE, 0);

    // Configure ADC1 for DMA sampler
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_0);
    ADCSequenceDisable(ADC1_BASE, 0);
    ADCSequenceConfigure(ADC1_BASE, 0, ADC_TRIGGER_ALWAYS, 0);
    ADCSequenceStepConfigure(ADC1_BASE, 0, 0, ADC_CTL_CH3 | ADC_CTL_IE | ADC_CTL_END);
    ADCSequenceDMAEnable(ADC1_BASE, 0);         // Enable DMA for ADC1 sequence 0
    ADCIntEnableEx(ADC1_BASE, ADC_INT_DMA_SS0); // Enable ADC1 sequence 0 DMA interrupt
    ADCSequenceEnable(ADC1_BASE, 0);

    /**
     * DMA Controller
     */

    // Initialize and configure DMA
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);
    uDMAEnable();
    uDMAControlBaseSet(gDMAControlTable);
    uDMAChannelAssign(UDMA_CH24_ADC1_0);
    uDMAChannelAttributeDisable(UDMA_SEC_CHANNEL_ADC10, UDMA_ATTR_ALL);

    // Primary DMA channel = first half of the ADC buffer
    uDMAChannelControlSet(
        UDMA_SEC_CHANNEL_ADC10 | UDMA_PRI_SELECT,
        UDMA_SIZE_16 | UDMA_SRC_INC_NONE | UDMA_DST_INC_16 | UDMA_ARB_4);
    uDMAChannelTransferSet(
        UDMA_SEC_CHANNEL_ADC10 | UDMA_PRI_SELECT,
        UDMA_MODE_PINGPONG, (void*)&ADC1_SSFIFO0_R,
        (void*)&gADCBuffer[0], ADC_BUFFER_SIZE/2);

    // Alternate DMA channel = second half of the ADC buffer
    uDMAChannelControlSet(UDMA_SEC_CHANNEL_ADC10 | UDMA_ALT_SELECT,
        UDMA_SIZE_16 | UDMA_SRC_INC_NONE | UDMA_DST_INC_16 | UDMA_ARB_4);
    uDMAChannelTransferSet(UDMA_SEC_CHANNEL_ADC10 | UDMA_ALT_SELECT,
        UDMA_MODE_PINGPONG, (void*)&ADC1_SSFIFO0_R,
        (void*)&gADCBuffer[ADC_BUFFER_SIZE/2], ADC_BUFFER_SIZE/2);

    // Enable DMA Channel
    uDMAChannelEnable(UDMA_SEC_CHANNEL_ADC10);

    /**
     * Analog Comparator
     */

    // Configure Analog Comparator
    SysCtlPeripheralEnable(SYSCTL_PERIPH_COMP0);
    ComparatorRefSet(COMP_BASE, COMP_REF_1_65V);
    ComparatorConfigure(COMP_BASE, 1,
        COMP_TRIG_NONE |    // Do not trigger ADC
        COMP_INT_RISE |     // Rising edge interrupt (not handled yet...)
        COMP_ASRCP_REF |    // Use internal reference for C1+
        COMP_OUTPUT_INVERT  // Invert output (input is C1-)
    );

    // Comparator input C1- = BoosterPack Connector #1 pin 3 (PC4)
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    GPIOPinTypeComparator(GPIO_PORTC_BASE, GPIO_PIN_4);

    // Comparator output C1o = BoosterPack Connector #1 pin 15 (PD1)
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    GPIOPinTypeComparatorOutput(GPIO_PORTD_BASE, GPIO_PIN_1);
    GPIOPinConfigure(GPIO_PD1_C1O);

    /**
     * Period Counter Timer
     */

    // Configure GPIO PD0 as timer input T0CCP0 at PD0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    GPIOPinTypeTimer(GPIO_PORTD_BASE, GPIO_PIN_0);
    GPIOPinConfigure(GPIO_PD0_T0CCP0);

    // Configure Timer0A for Edge Time Capture Mode
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    TimerDisable(TIMER0_BASE, TIMER_BOTH);
    TimerConfigure(TIMER0_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_CAP_TIME_UP);
    TimerControlEvent(TIMER0_BASE, TIMER_A, TIMER_EVENT_POS_EDGE);
    TimerLoadSet(TIMER0_BASE, TIMER_A, 0xffff);     // Use maximum load value
    TimerPrescaleSet(TIMER0_BASE, TIMER_A, 0xff);   // Use maximum prescale value
    TimerIntEnable(TIMER0_BASE, TIMER_CAPA_EVENT);
    TimerEnable(TIMER0_BASE, TIMER_A);

    /**
     * CPU Load Estimation Timer
     */

    // Configure Timer3A for CPU load estimation
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER3);
    TimerDisable(TIMER3_BASE, TIMER_BOTH);
    TimerConfigure(TIMER3_BASE, TIMER_CFG_ONE_SHOT);
    TimerLoadSet(TIMER3_BASE, TIMER_A, gSystemClockFrequency / 100);

    // Get unloaded CPU count
    gCpuCountUnloaded = cpuLoadCount();

    /**
     * PWM peripheral initialization
     */

    // use M0PWM1, at GPIO PF1, which is BoosterPack Connector #1 pin 40
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_1); // PF1 = M0PWM1
    GPIOPinConfigure(GPIO_PF1_M0PWM1);
    GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_STRENGTH_8MA,
    GPIO_PIN_TYPE_STD);
    // configure the PWM0 peripheral
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    PWMClockSet(PWM0_BASE, PWM_SYSCLK_DIV_1); // use system clock
    PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, PWM_PERIOD);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, PWM_PERIOD/2); // initial 50% duty cycle
    PWMOutputInvert(PWM0_BASE, PWM_OUT_1_BIT, true); // invert PWM output
    PWMOutputState(PWM0_BASE, PWM_OUT_1_BIT, true); // enable PWM output
    PWMGenEnable(PWM0_BASE, PWM_GEN_0); // enable PWM generator
    // enable PWM interrupt in the PWM peripheral
    PWMGenIntTrigEnable(PWM0_BASE, PWM_GEN_0, PWM_INT_CNT_ZERO);
    PWMIntEnable(PWM0_BASE, PWM_INT_GEN_0);

    uint32_t i;
    const float freq = 2.0f * PI / 1024.0f;
    for (i = 0; i < PWM_WAVEFORM_TABLE_SIZE; i++)
    {
        gPWMWaveformTable[i] = 127 + (int8_t)(120.0f * sinf(freq * (float)i));
    }

    /**
     * LCD Driver
     */

    Crystalfontz128x128_Init();
    Crystalfontz128x128_SetOrientation(LCD_ORIENTATION_UP);
    GrContextInit(&gContext, &g_sCrystalfontz128x128);
    GrContextFontSet(&gContext, &g_sFontFixed6x8);

    /**
     * RTOS Initialization
     */

    BIOS_start();
    return 0;
}

/**
 * PWM hardware interrupt
 */
void PwmHwiFunc(void)
{
    PWMGenIntClear(PWM0_BASE, PWM_GEN_0, PWM_INT_CNT_ZERO); // clear PWM interrupt flag
    gPhase += gPhaseIncrement;
    // write directly to the Compare B register that determines the duty cycle
    PWM0_0_CMPB_R = 1 + gPWMWaveformTable[gPhase >> (32 - PWM_WAVEFORM_INDEX_BITS)];
}




/**
 * ADC sampling ISR
 */
void AdcHwiFunc(UArg arg1)
{
    // Clear interrupt flag
    ADCIntClearEx(ADC1_BASE, ADC_INT_DMA_SS0);

    // Check the primary DMA channel for end of transfer, and restart if needed.
    if (uDMAChannelModeGet(UDMA_SEC_CHANNEL_ADC10 | UDMA_PRI_SELECT) == UDMA_MODE_STOP)
    {
        // Restart primary channel (same as setup)
        uDMAChannelTransferSet(
            UDMA_SEC_CHANNEL_ADC10 | UDMA_PRI_SELECT,
            UDMA_MODE_PINGPONG, (void*)&ADC1_SSFIFO0_R,
            (void*)&gADCBuffer[0], ADC_BUFFER_SIZE/2);

        // DMA is currently occurring in the alternate buffer
        gDMAPrimary = false;
    }


    // Check the alternate DMA channel for end of transfer, and restart if needed.
    if (uDMAChannelModeGet(UDMA_SEC_CHANNEL_ADC10 | UDMA_ALT_SELECT) == UDMA_MODE_STOP)
    {
        // Restart alternate channel (same as setup)
        uDMAChannelTransferSet(
            UDMA_SEC_CHANNEL_ADC10 | UDMA_ALT_SELECT,
            UDMA_MODE_PINGPONG, (void*)&ADC1_SSFIFO0_R,
            (void*)&gADCBuffer[ADC_BUFFER_SIZE/2], ADC_BUFFER_SIZE/2);

        // DMA is currently occurring in the primary buffer
        gDMAPrimary = true;
    }

    // The DMA channel may be disabled if the CPU is paused by the debugger.
    if (!uDMAChannelIsEnabled(UDMA_SEC_CHANNEL_ADC10))
    {
        uDMAChannelEnable(UDMA_SEC_CHANNEL_ADC10);  // Re-enable the DMA channel
    }
}

/**
 * Timer0A capture ISR
 */
void TimerCaptureHwiFunc()
{
    // Clear interrupt flag
    TimerIntClear(TIMER0_BASE, TIMER_CAPA_EVENT);

    // Previous time variable
    static uint32_t sPrevTime = 0;

    // Add to global sum
    uint32_t time = TimerValueGet(TIMER0_BASE, TIMER_A);
    gPeriodSum += (time - sPrevTime) & 0xffff;
    sPrevTime = time;

    // Increment period counter
    gPeriodCount++;
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
 * Frequency clock callback
 * - Triggers frequency measurement task
 */
void FrequencyClockFunc()
{
    Semaphore_post(semTriggerFrequencyTask);
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

    // Waveform copying loop
    while (1)
    {
        // Wait for trigger from processing task
        Semaphore_pend(semTriggerWaveformTask, BIOS_WAIT_FOREVER);

        // Capture buffer
        uint8_t displayState = gDisplayState;
        if (displayState == 0)
        {
            // Capture waveform buffer

            // Find trigger location
            int32_t startIndex = getADCBufferIndex() - (LCD_HORIZONTAL_MAX / 2);
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
        }
        else
        {
            // Capture FFT buffer

            // Copy into waveform buffer
            int32_t endIndex = getADCBufferIndex();
            int32_t startIndex = endIndex - FFT_BUFFER_SIZE;
            ADCIndexWrap(&startIndex);
            int32_t copyIndex = startIndex;
            while (true)
            {
                // Copy single reading to FFT buffer
                int32_t fftIndex = copyIndex - startIndex;
                ADCIndexWrap(&fftIndex);
                gFFTBuffer[fftIndex] = gADCBuffer[copyIndex];

                // Increment index
                copyIndex++;
                ADCIndexWrap(&copyIndex);
                if (copyIndex == endIndex) break;
            }
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
        // 6 -> Board Button 1
        // 7 -> Empty

        // Read GPIO buttons
        uint8_t buttons = 0;
        buttons |= (~GPIOPinRead(GPIO_PORTH_BASE, 0xff) & GPIO_PIN_1) >> 1; // BoosterPack Button 1
        buttons |= (~GPIOPinRead(GPIO_PORTK_BASE, 0xff) & GPIO_PIN_6) >> 5; // BoosterPack Button 2
        buttons |= (~GPIOPinRead(GPIO_PORTJ_BASE, 0xff) & GPIO_PIN_0) << 6; // Board Button 1

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
 * Frequency measurement task
 * - Estimates frequency of input signal
 * - Saves frequency to global gFrequency (in Hz)
 */
void FrequencyTaskFunc(UArg arg1, UArg arg2)
{
    while (1)
    {
        // Pend on triggering semaphore
        Semaphore_pend(semTriggerFrequencyTask, BIOS_WAIT_FOREVER);

        // Disable Hwis to access global data
        IArg key = GateHwi_enter(gateHwi1);
        uint32_t periodSum = gPeriodSum;
        uint32_t periodCount = gPeriodCount;
        gPeriodSum = 0;
        gPeriodCount = 0;
        GateHwi_leave(gateHwi1, key);

        // Compute frequency
        gInputFrequency = (float)periodCount / (float)periodSum * gSystemClockFrequency;
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
        if (buttons & (1 << 0)) gTrigState = 0; // BoosterPack Button 1 -> Rising Trigger
        if (buttons & (1 << 1)) gTrigState = 1; // BoosterPack Button 2 -> Falling Trigger

        // Swap display states on Board Button 1 press
        if (buttons & (1 << 6)) gDisplayState = 1 - gDisplayState;

        // Handle voltage scale
        if (buttons & (1 << 4) && gVoltScaleState < 3) gVoltScaleState++;   // JoyStick Up -> Increase Voltage scale
        if (buttons & (1 << 5) && gVoltScaleState > 0) gVoltScaleState--;   // JoyStick Down -> Decrease Voltage scale

        // Trigger display task
        Semaphore_post(semTriggerDisplayTask);
    }
}

/**
 * Display task
 * - Writes data from gPixelBuffer to LCD display
 * - Displays vertical and horizontal division scales
 * - Estimates and prints the CPU load
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
        GrContextForegroundSet(&gContext, ClrDimGray);
        uint8_t displayState = gDisplayState;
        if (displayState == 0)
        {
            // Display lines for waveform
            for(divIndex = 4; divIndex < LCD_HORIZONTAL_MAX; divIndex += 20)
            {
                GrLineDrawH(&gContext, 0, LCD_HORIZONTAL_MAX-1, divIndex);
                GrLineDrawV(&gContext, divIndex, 0, LCD_HORIZONTAL_MAX-1);
            }
        }
        else
        {
            // Display lines for FFT
            for(divIndex = 4; divIndex < LCD_HORIZONTAL_MAX; divIndex += 20)
            {
                GrLineDrawH(&gContext, 0, LCD_HORIZONTAL_MAX-1, divIndex);
                GrLineDrawV(&gContext, divIndex - 4, 0, LCD_HORIZONTAL_MAX-1);
            }
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

        // Print division scales
        if (displayState == 0)
        {
            // Display waveform scales

            // Print time scale
            char timeScaleStr[10];
            snprintf(timeScaleStr, sizeof(timeScaleStr), "10us");
            GrStringDraw(&gContext, timeScaleStr, sizeof(timeScaleStr), /*x*/ 9, /*y*/ 8, /*opaque*/ false);

            // Print voltage scale
            char voltScaleStr[10];
            uint8_t voltScaleState = gVoltScaleState;
            switch (voltScaleState)
            {
                case 0: snprintf(voltScaleStr, sizeof(voltScaleStr), "100mV"); break;
                case 1: snprintf(voltScaleStr, sizeof(voltScaleStr), "200mV"); break;
                case 2: snprintf(voltScaleStr, sizeof(voltScaleStr), "500mV"); break;
                case 3: snprintf(voltScaleStr, sizeof(voltScaleStr), "1.00V"); break;
            }
            GrStringDraw(&gContext, voltScaleStr, sizeof(voltScaleStr), /*x*/ 72, /*y*/ 8, /*opaque*/ false);

            // Print frequency estimate
            float inputFrequency = gInputFrequency;
            char freqStr[20];
            snprintf(freqStr, sizeof(freqStr), "Freq: %.3fHz", inputFrequency);
            GrStringDraw(&gContext, freqStr, sizeof(freqStr), /*x*/ 9, /*y*/ 102, /*opaque*/ false);
        }
        else
        {
            // Display FFT scales

            // Print frequency scale
            char freqScaleStr[10];
            snprintf(freqScaleStr, sizeof(freqScaleStr), "40KHz");
            GrStringDraw(&gContext, freqScaleStr, sizeof(freqScaleStr), /*x*/ 9, /*y*/ 8, /*opaque*/ false);

            // Print DB scale
            char dbScaleStr[10];
            snprintf(dbScaleStr, sizeof(dbScaleStr), "20dB");
            GrStringDraw(&gContext, dbScaleStr, sizeof(dbScaleStr), /*x*/ 72, /*y*/ 8, /*opaque*/ false);
        }

        // Estimate and print CPU load
        gCpuCountLoaded = cpuLoadCount();
        float cpuLoad = 1.0f - ((float)gCpuCountLoaded)/((float)gCpuCountUnloaded);
        char cpuLoadStr[20];
        snprintf(cpuLoadStr, sizeof(cpuLoadStr), "CPU Load: %.1f%%", cpuLoad * 100.0f);
        GrStringDraw(&gContext, cpuLoadStr, sizeof(cpuLoadStr), /*x*/ 9, /*y*/ 112, /*opaque*/ false);

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

        // Calculate the pixel positions
        uint8_t displayState = gDisplayState;
        if (displayState == 0)
        {
            // Calculate waveform pixels

            // Calculate voltage scale
            float voltsPerDiv;
            uint8_t voltScaleState = gVoltScaleState;
            switch (voltScaleState)
            {
                case 0: voltsPerDiv = 0.1f; break;
                case 1: voltsPerDiv = 0.2f; break;
                case 2: voltsPerDiv = 0.5f; break;
                case 3: voltsPerDiv = 1.0f; break;
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
                        (int)(pixelPerAdc *
                                ((int)gWaveformBuffer[pixelIndex] - (int)ADC_OFFSET));
            }
        }
        else
        {
            // Calculate FFT pixels

            // Compute FFT
            int ifft;
            for (ifft = 0; ifft < FFT_BUFFER_SIZE; ifft++) {
                in[ifft].r = gFFTBuffer[ifft];      // Input is real
                in[ifft].i = 0.0f;                  // Imaginary is 0
            }
            kiss_fft(cfg, in, out);

            // Convert to pixels in pixel buffer
            uint32_t pix;    // Pixel index
            for(pix = 0; pix < LCD_HORIZONTAL_MAX; pix++)
            {
                float mag = hypotf(out[pix].r, out[pix].i);
                float db = 20.0f * log10f(mag);
                gPixelBuffer[pix] = 164 - (uint32_t)db;
            }
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

/**
 * Returns most up-to-date ADC buffer index.
 */
int32_t getADCBufferIndex(void)
{
    int32_t index;
    IArg key = GateHwi_enter(gateHwi0);
    if (gDMAPrimary)
    {
        // DMA is currently in the primary channel
        index = ADC_BUFFER_SIZE/2 - 1 -
                uDMAChannelSizeGet(UDMA_SEC_CHANNEL_ADC10 | UDMA_PRI_SELECT);
    }
    else
    {
        // DMA is currently in the alternate channel
        index = ADC_BUFFER_SIZE - 1 -
                uDMAChannelSizeGet(UDMA_SEC_CHANNEL_ADC10 | UDMA_ALT_SELECT);
    }
    GateHwi_leave(gateHwi0, key);
    return index;
}

/**
 * Returns count from Timer3A for CPU load estimation.
 */
uint32_t cpuLoadCount(void)
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
