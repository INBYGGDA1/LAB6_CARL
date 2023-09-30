/**
 * ----------------------------------------------------------------------------
 * main.c
 * Author: Carl Larsson
 * Description: Lab 6 4.1a) Producer/Consumer problem
 * Date: 2023-09-30
 * ----------------------------------------------------------------------------
 */

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#include <stdint.h>
#include <stdlib.h>
#include <time.h>
#include "utils/uartstdio.c"
#include "driverlib/gpio.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#define BUFFER_SIZE 5
volatile int g_byte_count = 0;
volatile int buffer[BUFFER_SIZE];

int g_producer_sleep = 0;
int g_consumer_sleep = 0;

// Handles for the different tasks
TaskHandle_t g_producer_handle = NULL;
TaskHandle_t g_consumer_handle = NULL;
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
// Timer callback function
static void timer_callback(TimerHandle_t xTimer)
{
    if((g_producer_sleep == 1) && (g_consumer_sleep == 1))
    {
        UARTprintf("\n Both suspended \n");
    }
    /*
    else
    {
        // Print task status
        char task_buffer[40];
        vTaskList(task_buffer);
        UARTprintf("\n %s \n", task_buffer);
    }
    */
}
//=============================================================================
void producer_task(void *temp)
{
    //-----------------------------------------------------------------------------
    int byte;
    int position;
    //-----------------------------------------------------------------------------

    for (;;)
    {
        // Produce byte
        byte = g_byte_count;
        // If buffer is full then we can't produce any more, sleep
        if (g_byte_count == BUFFER_SIZE)
        {
            g_producer_sleep = 1;
            vTaskSuspend(g_producer_handle);
        }
        // Put produced byte in buffer
        position = g_byte_count;
        buffer[position] = byte;
        if(byte != position)
        {
            UARTprintf("\n P %d : %d \n", byte, position);
        }
        g_byte_count++;
        // Wake up consumer if there is bytes to consume
        // (FLAWED, should be > 0)
        if (g_byte_count == 1)
        {
            g_consumer_sleep = 0;
            vTaskResume(g_consumer_handle);
        }
    }
}
//=============================================================================
void consumer_task(void *temp)
{
    //-----------------------------------------------------------------------------
    int byte;
    int position;
    //-----------------------------------------------------------------------------

    for (;;)
    {
        // If there are no bytes to consume, sleep
        if (g_byte_count == 0)
        {
            g_consumer_sleep = 1;
            vTaskSuspend(g_consumer_handle);
        }
        // Get byte from buffer
        position = g_byte_count-1;
        byte = buffer[position];
        if(byte != position)
        {
            UARTprintf("\n C %d : %d \n", byte, position);
        }
        g_byte_count--;
        // Wake up producer if buffer isn't full
        // (FLAWED, should be < BUFFER_SIZE-1)
        if (g_byte_count == BUFFER_SIZE - 1)
        {
            g_producer_sleep = 0;
            vTaskResume(g_producer_handle);
        }
        // Consume byte
        // nom nom nom
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

    // Return values for the different tasks
    BaseType_t producer_return;
    BaseType_t consumer_return;

    // Priorities of different tasks
    UBaseType_t task_priority = 1;
    //-----------------------------------------------------------------------------

    //-----------------------------------------------------------------------------
    /* Initialization */
    // Initialize and start UART
    ConfigureUART();

    UARTprintf("\n//-----------------------------------------------------------------------------\n");
    //-----------------------------------------------------------------------------

    //-----------------------------------------------------------------------------
    // ms to wait, 10s
    const TickType_t wait_time = pdMS_TO_TICKS(10000);
    TimerHandle_t timer_handle;
    // Create 10s timer
    // pdTRUE makes the timer periodic
    timer_handle = xTimerCreate((const char*) "timer", wait_time, pdTRUE, (void*) 0, timer_callback);
    xTimerStart(timer_handle, 0);
    //-----------------------------------------------------------------------------

    //-----------------------------------------------------------------------------
    // Create the different tasks
    producer_return = xTaskCreate(producer_task, "Producer", 128, (void* ) 1, task_priority, &g_producer_handle);
    consumer_return = xTaskCreate(consumer_task, "Consumer", 128, (void* ) 1, task_priority, &g_consumer_handle);

    // Make sure all tasks could be created successfully
    if ((producer_return == pdPASS) && (consumer_return == pdPASS))
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
