/**
 * ----------------------------------------------------------------------------
 * main.c
 * Author: Carl Larsson
 * Description: Lab6 4.3, print ADC values on UART
 * Date: 2023-10-01
 * ----------------------------------------------------------------------------
 */

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#include <stdint.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include "utils/uartstdio.c"
#include "driverlib/gpio.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/adc.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#define WAIT_TIME 5
// If below 50 then joystick-x and microphone becomes correlated (legacy issue)
#define BLOCK_TIME 50

QueueHandle_t queue1_handle, queue2_handle, queue3_handle;
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
struct joystick_values
{
    uint32_t joy_x;
    uint32_t joy_y;
};
struct accelerometer_values
{
    uint32_t acc_x;
    uint32_t acc_y;
    uint32_t acc_z;
};
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//=============================================================================
// The error routine that is called if the driver library
// encounters an error.
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
    while(1);
}
#endif
//=============================================================================
// Configure the UART.
void ConfigureUART(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
    UARTStdioConfig(0, 115200, 16000000);
}
//=============================================================================
// Initialize ADC0 for accelerometer (gyroscope)
void initialize_adc0(void)
{
    // Enable the ADC0 module.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0))
    {
    }

    // Must disable sequence before configuring it
    ADCSequenceDisable(ADC0_BASE, 0);
    // Enables trigger from ADC on the GPIO pin.
    // Enable the first sample sequencer to capture the value of the channel when
    // the processor trigger occurs.
    // Channel 2 is X-axis, channel 1 is Y-axis, channel 3 is Z-axis.
    // However, channel 3 should be X-axis, channel 2 should be Y-axis and channel 1 should be Z-axis.
    ADCSequenceConfigure(ADC0_BASE, 0, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceStepConfigure(ADC0_BASE, 0, 0, ADC_CTL_IE | ADC_CTL_END | ADC_CTL_CH2);
    ADCSequenceEnable(ADC0_BASE, 0);
}
//=============================================================================
// Initialize ADC1 for joystick and microphone
void initialize_adc1(void)
{
    // Enable the ADC1 module.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC1);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC1))
    {
    }

    // Must disable sequence before configuring it
    ADCSequenceDisable(ADC1_BASE, 0);
    // Enables trigger from ADC on the GPIO pin.
    // Enable the first sample sequencer to capture the value of the channel when
    // the processor trigger occurs.
    // Channel 9 is microphone, channel 0 is joystick horizontal, channel 8 is joystick vertical.
    // However, should be channel 8 is microphone, channel 9 is joystick horizontal, channel 0 is joystick vertical.
    ADCSequenceConfigure(ADC1_BASE, 0, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceStepConfigure(ADC1_BASE, 0, 0, ADC_CTL_IE | ADC_CTL_END | ADC_CTL_CH9);
    ADCSequenceEnable(ADC1_BASE, 0);
}
//=============================================================================
void initialize_pins(void)
{
    // Enable port for all the sensors
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

    /* These might be incorrectly labeled, unsure which goes to which */
    // Enable microphone on PE3.
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3);
    // Enable Joystick horizontal/X on PE4.
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_4);
    // Enable Joystick vertical/Y on PE5.
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_5);
    // Enable Accelerometer X-axis on PE1.
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_1);
    // Enable Accelerometer Y-axis on PE2.
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_2);
    // Enable Accelerometer Z-axis on PE0.
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_0);
}
//=============================================================================
// Takes microphone values, send on message queue 1
void microphone_task(void *temp)
{
    //-----------------------------------------------------------------------------
    uint32_t microphone_value = 0;

    TickType_t xLastWakeTime;
    // Initialize the xLastWakeTime variable with the current time.
    xLastWakeTime = xTaskGetTickCount();
    // ms to wait, 1/8 period of gatekeeper
    const int period = WAIT_TIME*1;
    const TickType_t wait_time = pdMS_TO_TICKS(period);
    //-----------------------------------------------------------------------------

    for (;;)
    {
        // Create periodicity
        vTaskDelayUntil(&xLastWakeTime, wait_time);

        //-----------------------------------------------------------------------------
        // Must disable sequence before configuring it
        ADCSequenceDisable(ADC1_BASE, 0);
        // Microphone
        // Switch ADC1 to channel 8 for microphone.
        ADCSequenceStepConfigure(ADC1_BASE, 0, 0, ADC_CTL_IE | ADC_CTL_END | ADC_CTL_CH8);
        // Enable sequence again
        ADCSequenceEnable(ADC1_BASE, 0);
        // Wait for microphone.
        ADCProcessorTrigger(ADC1_BASE, 0);
        while (!ADCIntStatus(ADC1_BASE, 0, false))
        {
        }
        ADCSequenceDataGet(ADC1_BASE, 0, &microphone_value);
        // Required to not cause mapping issues between different ADC and correlations
        // Because there is a write buffer in the Cortex-M processor, it may take several clock cycles before the interrupt source is actually cleared.
        ADCIntClear(ADC1_BASE, 0);
        //-----------------------------------------------------------------------------

        // Send microphone value on queue 1
        xQueueSend(queue1_handle, (void* )&microphone_value, BLOCK_TIME);
    }
}
//=============================================================================
// Takes joystick values and send on message queue 2
void joystick_task(void *temp)
{
    //-----------------------------------------------------------------------------
    struct joystick_values joy_val;

    TickType_t xLastWakeTime;
    // Initialize the xLastWakeTime variable with the current time.
    xLastWakeTime = xTaskGetTickCount();
    // ms to wait, 1/4 period of gatekeeper
    const int period = WAIT_TIME*2;
    const TickType_t wait_time = pdMS_TO_TICKS(period);
    //-----------------------------------------------------------------------------

    for (;;)
    {
        // Create periodicity
        vTaskDelayUntil(&xLastWakeTime, wait_time);

        //-----------------------------------------------------------------------------
        // Joystick horizontal
        // Must disable sequence before configuring it
        ADCSequenceDisable(ADC1_BASE, 0);
        // Switch ADC1 to channel 9 for joystick horizontal.
        ADCSequenceStepConfigure(ADC1_BASE, 0, 0, ADC_CTL_IE | ADC_CTL_END | ADC_CTL_CH9);
        // Enable sequence again
        ADCSequenceEnable(ADC1_BASE, 0);
        // Wait for joystick horizontal.
        ADCProcessorTrigger(ADC1_BASE, 0);
        while (!ADCIntStatus(ADC1_BASE, 0, false))
        {
        }
        ADCSequenceDataGet(ADC1_BASE, 0, &joy_val.joy_x);
        // Required to not cause mapping issues between different ADC and correlations
        // Because there is a write buffer in the Cortex-M processor, it may take several clock cycles before the interrupt source is actually cleared.
        ADCIntClear(ADC1_BASE, 0);
        //-----------------------------------------------------------------------------
        // Joystick vertical
        // Must disable sequence before configuring it
        ADCSequenceDisable(ADC1_BASE, 0);
        // Switch ADC1 to channel 0 for joystick vertical.
        ADCSequenceStepConfigure(ADC1_BASE, 0, 0, ADC_CTL_IE | ADC_CTL_END | ADC_CTL_CH0);
        // Enable sequence again
        ADCSequenceEnable(ADC1_BASE, 0);
        // Wait for joystick vertical.
        ADCProcessorTrigger(ADC1_BASE, 0);
        while (!ADCIntStatus(ADC1_BASE, 0, false))
        {
        }
        ADCSequenceDataGet(ADC1_BASE, 0, &joy_val.joy_y);
        // Required to not cause mapping issues between different ADC and correlations
        // Because there is a write buffer in the Cortex-M processor, it may take several clock cycles before the interrupt source is actually cleared.
        ADCIntClear(ADC1_BASE, 0);
        //-----------------------------------------------------------------------------

        // Send joystick values on queue 2
        xQueueSend(queue2_handle, (void*)&joy_val, BLOCK_TIME);
    }
}
//=============================================================================
// Takes accelerometer values and send on message queue 3
void accelerometer_task(void *temp)
{
    //-----------------------------------------------------------------------------
    struct accelerometer_values acc_val;

    TickType_t xLastWakeTime;
    // Initialize the xLastWakeTime variable with the current time.
    xLastWakeTime = xTaskGetTickCount();
    // ms to wait, 1/2 period of gatekeeper
    const int period = WAIT_TIME*4;
    const TickType_t wait_time = pdMS_TO_TICKS(period);
    //-----------------------------------------------------------------------------

    for (;;)
    {
        // Create periodicity
        vTaskDelayUntil(&xLastWakeTime, wait_time);

        //-----------------------------------------------------------------------------
        // Accelerometer (Gyroscope?) X-axis
        // Must disable sequence before configuring it
        ADCSequenceDisable(ADC0_BASE, 0);
        // Switch ADC0 to channel 3 for Accelerometer (gyroscope?) X-axis.
        ADCSequenceStepConfigure(ADC0_BASE, 0, 0, ADC_CTL_IE | ADC_CTL_END | ADC_CTL_CH3);
        // Enable sequence again
        ADCSequenceEnable(ADC0_BASE, 0);
        // Wait for Accelerometer (gyroscope) X-axis (PE1) value.
        ADCProcessorTrigger(ADC0_BASE, 0);
        while (!ADCIntStatus(ADC0_BASE, 0, false))
        {
        }
        ADCSequenceDataGet(ADC0_BASE, 0, &acc_val.acc_x);
        // Required to not cause mapping issues between different ADC and correlations
        // Because there is a write buffer in the Cortex-M processor, it may take several clock cycles before the interrupt source is actually cleared.
        ADCIntClear(ADC0_BASE, 0);
        //-----------------------------------------------------------------------------
        // Accelerometer (Gyroscope?) Y-axis
        // Must disable sequence before configuring it
        ADCSequenceDisable(ADC0_BASE, 0);
        // Switch ADC0 to channel 2 for Accelerometer (gyroscope?) Y-axis.
        ADCSequenceStepConfigure(ADC0_BASE, 0, 0, ADC_CTL_IE | ADC_CTL_END | ADC_CTL_CH2);
        // Enable sequence again
        ADCSequenceEnable(ADC0_BASE, 0);
        // Wait for Accelerometer (gyroscope) Y-axis (PE2) value.
        ADCProcessorTrigger(ADC0_BASE, 0);
        while (!ADCIntStatus(ADC0_BASE, 0, false))
        {
        }
        ADCSequenceDataGet(ADC0_BASE, 0, &acc_val.acc_y);
        // Required to not cause mapping issues between different ADC and correlations
        // Because there is a write buffer in the Cortex-M processor, it may take several clock cycles before the interrupt source is actually cleared.
        ADCIntClear(ADC0_BASE, 0);
        //-----------------------------------------------------------------------------
        // Accelerometer (Gyroscope?) Z-axis
        // Must disable sequence before configuring it
        ADCSequenceDisable(ADC0_BASE, 0);
        // Switch ADC0 to channel 1 for Accelerometer (gyroscope?) Z-axis.
        ADCSequenceStepConfigure(ADC0_BASE, 0, 0, ADC_CTL_IE | ADC_CTL_END | ADC_CTL_CH1);
        // Enable sequence again
        ADCSequenceEnable(ADC0_BASE, 0);
        // Wait for Accelerometer (gyroscope) Z-axis (PE0) value.
        ADCProcessorTrigger(ADC0_BASE, 0);
        while (!ADCIntStatus(ADC0_BASE, 0, false))
        {
        }
        ADCSequenceDataGet(ADC0_BASE, 0, &acc_val.acc_z);
        // Required to not cause mapping issues between different ADC and correlations
        // Because there is a write buffer in the Cortex-M processor, it may take several clock cycles before the interrupt source is actually cleared.
        ADCIntClear(ADC0_BASE, 0);
        //-----------------------------------------------------------------------------

        // Send accelerometer values on queue 3
        xQueueSend(queue3_handle, (void*)&acc_val, BLOCK_TIME);
    }
}
//=============================================================================
// Receives all values from message queues and prints them on serial port (UART)
void gatekeeper_task(void *temp)
{
    //-----------------------------------------------------------------------------
    // Microphone
    uint32_t mic_val = 0;
    uint32_t total_mic_val = 0;
    uint32_t avg_mic_val = 0;
    uint32_t mic_samples = 0;
    // Joystick
    struct joystick_values joy_val = {0, 0};
    struct joystick_values total_joy_val = {0, 0};
    struct joystick_values avg_joy_val = {0, 0};
    uint32_t joy_samples = 0;
    // Accelerometer
    struct accelerometer_values acc_val = {0, 0, 0};
    struct accelerometer_values total_acc_val = {0, 0, 0};
    struct accelerometer_values avg_acc_val = {0, 0, 0};
    uint32_t acc_samples = 0;

    TickType_t xLastWakeTime;
    // Initialize the xLastWakeTime variable with the current time.
    xLastWakeTime = xTaskGetTickCount();
    // ms to wait
    const int period = WAIT_TIME*8;
    const TickType_t wait_time = pdMS_TO_TICKS(period);
    //-----------------------------------------------------------------------------

    for (;;)
    {
        // Create periodicity
        vTaskDelayUntil(&xLastWakeTime, wait_time);

        //-----------------------------------------------------------------------------
        // Receive microphone value from queue 1
        while(xQueueReceive(queue1_handle, &mic_val, 0) == pdTRUE)
        {
            total_mic_val = total_mic_val + mic_val;
            mic_samples++;
        }
        // Calculate average using 8 samples
        if(mic_samples >= 8)
        {
            // Convert to dB
            avg_mic_val = round(20.0*log10(total_mic_val / mic_samples));
            total_mic_val = 0;
            mic_samples = 0;
        }
        //-----------------------------------------------------------------------------

        //-----------------------------------------------------------------------------
        // Receive joystick values from queue 2
        while(xQueueReceive(queue2_handle, &joy_val, 0) == pdTRUE)
        {
            total_joy_val.joy_x = total_joy_val.joy_x + joy_val.joy_x;
            total_joy_val.joy_y = total_joy_val.joy_y + joy_val.joy_y;
            joy_samples++;
        }
        if(joy_samples >= 4)
        {
            avg_joy_val.joy_x = total_joy_val.joy_x / joy_samples;
            avg_joy_val.joy_y = total_joy_val.joy_y / joy_samples;
            total_joy_val.joy_x = 0;
            total_joy_val.joy_y = 0;
            joy_samples = 0;
        }
        //-----------------------------------------------------------------------------

        //-----------------------------------------------------------------------------
        // Receive accelerometer values from queue 3
        while(xQueueReceive(queue3_handle, &acc_val, 0) == pdTRUE)
        {
            total_acc_val.acc_x = total_acc_val.acc_x + acc_val.acc_x;
            total_acc_val.acc_y = total_acc_val.acc_y + acc_val.acc_y;
            total_acc_val.acc_z = total_acc_val.acc_z + acc_val.acc_z;
            acc_samples++;
        }
        if(acc_samples >= 2)
        {
            avg_acc_val.acc_x = total_acc_val.acc_x / acc_samples;
            avg_acc_val.acc_y = total_acc_val.acc_y / acc_samples;
            avg_acc_val.acc_z = total_acc_val.acc_z / acc_samples;
            total_acc_val.acc_x = 0;
            total_acc_val.acc_y = 0;
            total_acc_val.acc_z = 0;
            acc_samples = 0;
        }
        //-----------------------------------------------------------------------------

        // Prevent terminal scrolling
        UARTprintf("\033[2J");
        // Print out ADC values
        UARTprintf("\n Microphone    : %d dB \n", avg_mic_val);
        UARTprintf("\n Joystick      : %d, %d \n", avg_joy_val.joy_x, avg_joy_val.joy_y);
        UARTprintf("\n Accelerometer : %d, %d, %d \n", avg_acc_val.acc_x, avg_acc_val.acc_y, avg_acc_val.acc_z);
    }
}
//=============================================================================
// Main Function
int main(void)
{
    //-----------------------------------------------------------------------------
    /* Variable declarations etc */
    // Run from the PLL at 120 MHz (because this is what FreeRTOS sets in it's config file).
    uint32_t systemClock;
    systemClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480), 120000000);

    // Handles for the different tasks
    TaskHandle_t microphone_task_handle = NULL;
    TaskHandle_t joystick_task_handle = NULL;
    TaskHandle_t accelerometer_task_handle = NULL;
    TaskHandle_t gatekeeper_task_handle = NULL;

    // Return values for the different tasks
    BaseType_t microphone_task_return;
    BaseType_t joystick_task_return;
    BaseType_t accelerometer_task_return;
    BaseType_t gatekeeper_task_return;

    // Priorities of different tasks
    UBaseType_t sensor_priority = 2;
    UBaseType_t gatekeeper_priority = 1;
    //-----------------------------------------------------------------------------

    //-----------------------------------------------------------------------------
    /* Initialization */
    // Initialize and start UART
    ConfigureUART();

    UARTprintf("\n//-----------------------------------------------------------------------------\n");

    // Initialize ADC0 for accelerometer
    initialize_adc0();
    // Initialize ADC1 for joystick and microphone
    initialize_adc1();
    // Initialize all the pins for the sensors
    initialize_pins();
    //-----------------------------------------------------------------------------

    //-----------------------------------------------------------------------------
    /* Create message queues */
    // Message queue 1 for microphone
    queue1_handle = xQueueCreate(8, sizeof(uint32_t));
    // Message queue 2 for joystick
    queue2_handle = xQueueCreate(4, sizeof(struct joystick_values));
    // Message queue 3 for accelerometer
    queue3_handle = xQueueCreate(2, sizeof(struct accelerometer_values));

    if((queue1_handle == NULL) || (queue2_handle == NULL) || (queue3_handle == NULL))
    {
        UARTprintf("\n !!COULD NOT ALLOCATE REQUIRED MEMORY FOR MESSAGE QUEUE!! \n");
    }
    //-----------------------------------------------------------------------------

    //-----------------------------------------------------------------------------
    // Create the different tasks
    microphone_task_return = xTaskCreate(microphone_task, "microphone", 256, (void*) 1, sensor_priority, &microphone_task_handle);
    joystick_task_return = xTaskCreate(joystick_task, "joystick", 256, (void*) 1, sensor_priority, &joystick_task_handle);
    accelerometer_task_return = xTaskCreate(accelerometer_task, "accelerometer", 256, (void*) 1, sensor_priority, &accelerometer_task_handle);
    gatekeeper_task_return = xTaskCreate(gatekeeper_task, "gatekeeper", 256, (void*) 1, gatekeeper_priority, &gatekeeper_task_handle);

    // Make sure all tasks could be created successfully
    if ((microphone_task_return == pdPASS) && (joystick_task_return == pdPASS) &&
        (accelerometer_task_return == pdPASS) && (gatekeeper_task_return  == pdPASS))
    {
        // Start the scheduler
        vTaskStartScheduler();

        // vTaskStartScheduler() will only return if there is insufficient RTOS heap available to create the idle or timer daemon tasks.
        // Will not get here unless there is insufficient RAM.
        UARTprintf("\n !!INSUFFICENT RAM FOR SCHEDULER!! \n");
    }
    // One or more tasks could not be created successfully because of memory
    UARTprintf("\n !!COULD NOT ALLOCATE REQUIRED MEMORY FOR TASK!! \n");
    //-----------------------------------------------------------------------------
}
//=============================================================================
