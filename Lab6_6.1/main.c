/**
 * ----------------------------------------------------------------------------
 * main.c
 * Author: Carl Larsson
 * Description: Lab 6 6.1 Dining philosophers
 * Date: 2023-10-01
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
SemaphoreHandle_t chopstick[5];
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
// Dining philosopher task
// Problem solved by putting a number on each chopstick, 0-4, then when each philosopher wants to eat he tries to take the lowest valued chopstick first.
// Down side of this solution is that chopstick 0 has higher demand then the other chopsticks, and chopstick 4 has lower demand than the other chopsticks, leading to unfairness for the processes.
void dining_philosophers(void *temp)
{
    //-----------------------------------------------------------------------------
    int i = (int) temp;

    int random_time;
    srand(time(NULL));
    // ms to wait, 1s
    const TickType_t wait_time = pdMS_TO_TICKS(1000);
    //-----------------------------------------------------------------------------

    for (;;)
    {
        // Think for 1s
        vTaskDelay(wait_time);

        // Take lowest valued chopstick first
        // For philosopher 4 this means taking chopstick 0 first, then chopstick 4
        if ((i + 1) % 5 < i)
        {
            if (xSemaphoreTake(chopstick[(i + 1) % 5], portMAX_DELAY) != pdTRUE)
            {
                UARTprintf("\n !!ERROR Philosopher %d : Chopstick %d!! \n", i, (i + 1) % 5);
            }
            UARTprintf("\n Philosopher %d takes chopstick %d (first) \n", i, (i + 1) % 5);

            if (xSemaphoreTake(chopstick[i], portMAX_DELAY) != pdTRUE)
            {
                UARTprintf("\n !!ERROR Philosopher %d : Chopstick %d \n", i, i);
            }
            UARTprintf("\n Philosopher %d takes chopstick %d (second) \n", i, i);
        }
        // for every other philosopher this means, philosopher x takes chopstick x first, then chopstick x+1
        else
        {
            if (xSemaphoreTake(chopstick[i], portMAX_DELAY) != pdTRUE)
            {
                UARTprintf("\n !!ERROR Philosopher %d : Chopstick %d \n", i, i);
            }
            UARTprintf("\n Philosopher %d takes chopstick %d (first) \n", i, i);

            if (xSemaphoreTake(chopstick[(i + 1) % 5], portMAX_DELAY) != pdTRUE)
            {
                UARTprintf("\n !!ERROR Philosopher %d : Chopstick %d!! \n", i, (i + 1) % 5);
            }
            UARTprintf("\n Philosopher %d takes chopstick %d (second) \n", i, (i + 1) % 5);
        }

        // Eat for a random amount of time
        random_time = rand();
        vTaskDelay(random_time);
        UARTprintf("\n Philosopher %d has eaten \n", i);

        UARTprintf("\n Philosopher %d puts down chopsticks \n", i);
        // Always return latest taken mutex first, see documentation
        if ((i + 1) % 5 < i)
        {
            while (xSemaphoreGive(chopstick[i]) != pdTRUE)
            {
                ;
            }

            while (xSemaphoreGive(chopstick[(i + 1) % 5]) != pdTRUE)
            {
                ;
            }
        }
        else
        {
            while (xSemaphoreGive(chopstick[(i + 1) % 5]) != pdTRUE)
            {
                ;
            }

            while (xSemaphoreGive(chopstick[i]) != pdTRUE)
            {
                ;
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

    // Handles for different tasks
    TaskHandle_t philosopher_handles[5];

    // Return values for the different tasks
    BaseType_t philosopher_returns[5];

    // Priorities of different tasks
    UBaseType_t task_priority = 1;

    int i;
    //-----------------------------------------------------------------------------

    //-----------------------------------------------------------------------------
    /* Initialization */
    // Initialize and start UART
    ConfigureUART();

    UARTprintf("\n//-----------------------------------------------------------------------------\n");
    //-----------------------------------------------------------------------------

    //-----------------------------------------------------------------------------
    // Create five mutexes (chopsticks), handles are global
    for(i = 0; i<5; i++)
    {
        chopstick[i] = xSemaphoreCreateMutex();
    }
    if((chopstick[0] == NULL) || (chopstick[1] == NULL) || (chopstick[2] == NULL) || (chopstick[3] == NULL) || (chopstick[4] == NULL))
    {
        UARTprintf(" !!SEMAPHORE COULD NOT BE ALLOCATED!! ");
    }
    //-----------------------------------------------------------------------------

    //-----------------------------------------------------------------------------
    // Create the different tasks
    for(i = 0; i<5; i++)
    {
        philosopher_returns[i] = xTaskCreate(dining_philosophers, "philosopher", 128, (void* ) i, task_priority, &philosopher_handles[i]);
    }

    // Make sure all tasks could be created successfully
    if ((philosopher_returns[0] == pdPASS) && (philosopher_returns[1] == pdPASS) && (philosopher_returns[2] == pdPASS) && (philosopher_returns[3] == pdPASS) && (philosopher_returns[4] == pdPASS))
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
