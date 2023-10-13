/**
 * ----------------------------------------------------------------------------
 * main.c
 * Author: Carl Larsson
 * Description: Lab6 4.2b) UART with total characters received
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
#include "drivers/buttons.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#define BUFFER_SIZE 50
#define ECHO_SIZE 15

char echo_buffer[ECHO_SIZE];
int g_total_characters_read = 0;
int g_print_total_characters = 0;
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
// Disables printing of total characters
static void timer_callback(TimerHandle_t xTimer)
{
    // Disable printing of total characters
    g_print_total_characters = 0;

    // Update screen with just echo
    UARTprintf("\033[2J");
    UARTprintf("\n %s \n", echo_buffer);
    UARTprintf("\n \n");
}
//=============================================================================
// If button is pressed then total characters is printed for 10s
void button_task(void *sys_clock)
{
    //-----------------------------------------------------------------------------
    unsigned char ucDelta, ucState;
    // ms to wait, 10s
    const TickType_t wait_time = pdMS_TO_TICKS(10000);
    TimerHandle_t timer_handle;
    // Create 10s timer
    // pdFALSE makes the timer non periodic
    timer_handle = xTimerCreate(( const char * ) "timer", wait_time, pdFALSE, ( void * ) 0, timer_callback);
    //-----------------------------------------------------------------------------

    for( ;; )
    {
        // Poll the buttons.
        ucState = ButtonsPoll(&ucDelta, 0);

        // LEFT button has been pressed
        if (BUTTON_PRESSED(LEFT_BUTTON, ucState, ucDelta) || BUTTON_RELEASED(LEFT_BUTTON, ucState, ucDelta))
        {
            // Enable printing of total characters
            g_print_total_characters = 1;
            // Start 10s timer
            xTimerStart(timer_handle, 0 );
            // Only print if something has been sent
            if(g_total_characters_read > 0)
            {
                UARTprintf("\033[2J");
                UARTprintf("\n %s \n", echo_buffer);
                UARTprintf("\n %d \n", g_total_characters_read);
            }
        }
    }
}
//=============================================================================
// Task for printing on UART
void UART_task(void *temp)
{
    //-----------------------------------------------------------------------------
    // Buffer for user input string
    char character_read;

    int i;
    int index = 0;
    //-----------------------------------------------------------------------------

    for (;;)
    {
        // Prevent terminal scrolling
        UARTprintf("\033[2J");
        // Printing should only happen once something has been sent
        if(g_total_characters_read > 0)
        {
            if(g_print_total_characters == 1)
            {
                // Echo the input back and total characters read
                UARTprintf("\n %s \n", echo_buffer);
                UARTprintf("\n %d \n", g_total_characters_read);
            }
            else
            {
                // Echo the input back
                UARTprintf("\n %s \n", echo_buffer);
                UARTprintf("\n \n");
            }
        }
        // Receive user input
        character_read = (char)UARTCharGet(UART0_BASE);
        g_total_characters_read++;

        // Only shift if buffer is full
        if (g_total_characters_read > ECHO_SIZE)
        {
            // Shift everything one step to the left
            for (i = 0; i < (index); i++)
            {
                echo_buffer[i] = echo_buffer[i + 1];
            }

            // Find index where we are gonna put new character in echo_buffer
            index = index - 1;
        }

        // Add in character read at the end of the echo_buffer
        echo_buffer[index] = character_read;
        index++;
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
    TaskHandle_t button_task_handle = NULL;

    // Return values for the different tasks
    BaseType_t UART_task_return;
    BaseType_t button_task_return;

    // Priorities of different tasks
    UBaseType_t task_priority = 1;
    //-----------------------------------------------------------------------------

    //-----------------------------------------------------------------------------
    /* Initialization */
    // Initialize and start UART
    ConfigureUART();

    UARTprintf("\n//-----------------------------------------------------------------------------\n");

    // Initialize the button driver.
    ButtonsInit();
    //-----------------------------------------------------------------------------

    //-----------------------------------------------------------------------------
    // Create the different tasks
    UART_task_return = xTaskCreate(UART_task, "UART", 128, (void* ) 1, task_priority, &UART_task_handle);
    button_task_return = xTaskCreate(button_task, "button", 128, (void* ) 1, task_priority, &button_task_handle);

    // Make sure all tasks could be created successfully
    if ((UART_task_return == pdPASS) && (button_task_return == pdPASS))
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
