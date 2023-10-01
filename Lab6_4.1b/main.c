/**
 * ----------------------------------------------------------------------------
 * main.c
 * Author: Carl Larsson
 * Description: Lab 6 4.1b) Producer/Consumer problem, (FIXED)
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

// Handles for the different tasks
TaskHandle_t g_producer_handle = NULL;
TaskHandle_t g_consumer_handle = NULL;

SemaphoreHandle_t bin_sem_handle = NULL;
SemaphoreHandle_t empty_slots_handle = NULL;
SemaphoreHandle_t full_slots_handle = NULL;
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
int insert_byte(int byte)
{
    // Insert the byte into the buffer if it isn't full
    if(g_byte_count < BUFFER_SIZE)
    {
        buffer[g_byte_count] = byte;
        g_byte_count++;
        return 0;
    }
    // Error, the buffer is full
    else
    {
        return -1;
    }
}
//=============================================================================
void producer_task(void *producer_number)
{
    //-----------------------------------------------------------------------------
    int byte;
    srand(time(NULL));
    int random_time;
    //-----------------------------------------------------------------------------

    for (;;)
    {
        // Sleep random amount of time
        random_time = rand() / 10;
        vTaskDelay(random_time);

        // Decrement the number of empty slots, "puts a producer to sleep"
        if(xSemaphoreTake(empty_slots_handle, portMAX_DELAY) == pdTRUE)
        {
            // Lock binary semaphore, block until we can take it
            if(xSemaphoreTake(bin_sem_handle, portMAX_DELAY ) == pdTRUE)
            {
                // Produce byte
                byte = g_byte_count;
                if(insert_byte(byte) != 0)
                {
                    UARTprintf("\n !!(Producer %d) ERROR inserting byte into buffer!! \n", (int) producer_number);
                }
                else
                {
                    UARTprintf("\n Producer %d produced %d \n", (int) producer_number, byte);
                }

                // Unlock binary semaphore
                while (xSemaphoreGive(bin_sem_handle) != pdTRUE)
                {
                    ;
                }
                // Increment the number of full slots, "wakes up a consumer"
                while (xSemaphoreGive(full_slots_handle) != pdTRUE)
                {
                    ;
                }
            }
        }
    }
}
//=============================================================================
int remove_byte(int *byte)
{
    // Remove item from buffer unless it is empty
    if(g_byte_count > 0)
    {
        *byte = buffer[g_byte_count-1];
        g_byte_count--;
        return 0;
    }
    // Error, the buffer is empty
    else
    {
        return -1;
    }
}
//=============================================================================
void consumer_task(void *consumer_number)
{
    //-----------------------------------------------------------------------------
    int byte;
    srand(time(NULL));
    int random_time;
    //-----------------------------------------------------------------------------

    for (;;)
    {
        // Sleep random amount of time
        random_time = rand() / 10;
        vTaskDelay(random_time);

        // Decrement number of full slots, "puts a consumer to sleep"
        if (xSemaphoreTake(full_slots_handle, portMAX_DELAY) == pdTRUE)
        {
            // Lock binary semaphore, block until we can take it
            if (xSemaphoreTake(bin_sem_handle, portMAX_DELAY ) == pdTRUE)
            {
                if(remove_byte(&byte) != 0)
                {
                    UARTprintf("\n !!(Consumer %d) ERROR removing byte from buffer!! \n", (int) consumer_number);
                }
                else
                {
                    UARTprintf("\n Consumer %d consumed %d \n", (int) consumer_number, byte);
                }

                // Unlock binary semaphore
                while (xSemaphoreGive(bin_sem_handle) != pdTRUE)
                {
                    ;
                }
                // Increment the number of empty slots, "wakes up a producer"
                while (xSemaphoreGive(empty_slots_handle) != pdTRUE)
                {
                    ;
                }
            }
        }
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
    // Create two counting semaphores and one binary semaphore
    // Create binary semaphore, handle is global
    bin_sem_handle = xSemaphoreCreateBinary();
    // Create counting semaphore for number of empty slots in buffer, handle is global
    // Initially there's BUFFER_SIZE number of empty slots, BUFFER_SIZE is the max number
    empty_slots_handle = xSemaphoreCreateCounting(BUFFER_SIZE, BUFFER_SIZE);
    // Create counting semaphore for number of full slots in buffer, handle is global
    // Initially there's 0 full slots, BUFFER_SIZE is the max number
    full_slots_handle = xSemaphoreCreateCounting(BUFFER_SIZE, 0);
    // Make sure semaphore could be created successfully
    if ((bin_sem_handle == NULL) && (empty_slots_handle == NULL) && (full_slots_handle == NULL))
    {
        UARTprintf(" !!SEMAPHORE COULD NOT BE ALLOCATED!! ");
    }
    // Binary semaphores need to be given instantly upon creation, they start locked
    while (xSemaphoreGive(bin_sem_handle) != pdTRUE)
    {
        ;
    }
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
