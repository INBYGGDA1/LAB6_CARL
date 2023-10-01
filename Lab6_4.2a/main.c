/**
 * ----------------------------------------------------------------------------
 * main.c
 * Author: Carl Larsson
 * Description: Lab6 4.2a) UART
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
#define BUFFER_SIZE 50
#define ECHO_SIZE 15
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
void UART_task(void *temp)
{
    //-----------------------------------------------------------------------------
    // Buffer for user input string
    char UART_buffer[BUFFER_SIZE];
    int characters_read = 0;

    char echo_buffer[ECHO_SIZE];
    int total_characters_read = 0;

    int i;
    int index = 0;
    int num_of_shifts;
    int temp_num_of_shifts;
    //-----------------------------------------------------------------------------

    for (;;)
    {
        // Prevent terminal scrolling
        UARTprintf("\033[2J");
        // Printing should only happen once something has been sent
        if(total_characters_read > 0)
        {
            // Echo the input back
            UARTprintf("\n %s \n", echo_buffer);
            //UARTprintf("\n %d \n", total_characters_read);
        }
        // Receive user input
        characters_read = UARTgets(UART_buffer, BUFFER_SIZE);
        total_characters_read = total_characters_read + characters_read;
        // How many shifts left are needed
        num_of_shifts = (characters_read - (ECHO_SIZE - index));
        temp_num_of_shifts = num_of_shifts;

        // Only shift if buffer is full or string read exceeds echo buffer size
        if(total_characters_read > ECHO_SIZE)
        {
            // Shift everything left as many steps as there are new characters which need to be added
            while(temp_num_of_shifts > 0)
            {
                for(i = 0; i<(index); i++)
                {
                    echo_buffer[i] = echo_buffer[i+1];
                }
                temp_num_of_shifts--;
            }

            // Find index where we are gonna put new characters in echo_buffer
            index = index - num_of_shifts;
        }

        // Add in characters read at the end of the echo_buffer
        for(i = 0; i<characters_read; i++)
        {
            echo_buffer[index+i] = UART_buffer[i];
        }
        index = index + i;
        // Add end of string character
        echo_buffer[index] = '\0';
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
    TaskHandle_t UART_task_handle = NULL;

    // Return values for the different tasks
    BaseType_t UART_task_return;

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
    // Create the different tasks
    UART_task_return = xTaskCreate(UART_task, "UART", 128, (void* ) 1, task_priority, &UART_task_handle);

    // Make sure all tasks could be created successfully
    if ((UART_task_return == pdPASS))
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
