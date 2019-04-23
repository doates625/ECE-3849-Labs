/*
 *  Do not modify this file; it is automatically 
 *  generated and any modifications will be overwritten.
 *
 * @(#) xdc-B06
 */

#include <xdc/std.h>

#include <ti/sysbios/knl/Clock.h>
extern const ti_sysbios_knl_Clock_Handle ButtonClock;

#include <ti/sysbios/knl/Semaphore.h>
extern const ti_sysbios_knl_Semaphore_Handle semTriggerButtonScan;

#include <ti/sysbios/knl/Task.h>
extern const ti_sysbios_knl_Task_Handle ButtonTask;

#include <ti/sysbios/knl/Task.h>
extern const ti_sysbios_knl_Task_Handle WaveformTask;

#include <ti/sysbios/knl/Semaphore.h>
extern const ti_sysbios_knl_Semaphore_Handle semTriggerWaveformTask;

#include <ti/sysbios/knl/Mailbox.h>
extern const ti_sysbios_knl_Mailbox_Handle ButtonMailbox;

#include <ti/sysbios/knl/Semaphore.h>
extern const ti_sysbios_knl_Semaphore_Handle semTriggerProcessingTask;

#include <ti/sysbios/knl/Task.h>
extern const ti_sysbios_knl_Task_Handle ProcessingTask;

#include <ti/sysbios/knl/Semaphore.h>
extern const ti_sysbios_knl_Semaphore_Handle semTriggerDisplayTask;

#include <ti/sysbios/knl/Task.h>
extern const ti_sysbios_knl_Task_Handle DisplayTask;

#include <ti/sysbios/knl/Task.h>
extern const ti_sysbios_knl_Task_Handle UserInputTask;

#include <ti/sysbios/family/arm/m3/Hwi.h>
extern const ti_sysbios_family_arm_m3_Hwi_Handle AdcHwi;

#include <ti/sysbios/gates/GateHwi.h>
extern const ti_sysbios_gates_GateHwi_Handle gateHwi0;

#include <ti/sysbios/family/arm/m3/Hwi.h>
extern const ti_sysbios_family_arm_m3_Hwi_Handle TimerCaptureHwi;

extern int xdc_runtime_Startup__EXECFXN__C;

extern int xdc_runtime_Startup__RESETFXN__C;

