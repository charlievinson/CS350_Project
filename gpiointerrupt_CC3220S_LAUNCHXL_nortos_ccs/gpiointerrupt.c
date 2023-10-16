/*
 * Copyright (c) 2015-2020, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== gpiointerrupt.c ========
 */
#include <stdint.h>
#include <stddef.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>

/* Driver configuration */
#include "ti_drivers_config.h"


// Driver headers
#include <ti/drivers/I2C.h>
#include <ti/drivers/UART.h>
#include <ti/drivers/Timer.h>

/* ======== Macro Definitions ======== */
#define DISPLAY(x) UART_write(uart, &output, x);

#define ON 1
#define OFF 0

#define TEMPERATURE_INCREMENT 1
#define TEMPERATURE_RESET 0
#define SET_TEMPERATURE_START 25

#define CLOCK_MAX 10
#define CLOCK_MIN 0
#define CLOCK_INCREMENT 1
#define CLOCK_200_MS 2
#define CLOCK_500_MS 5

#define TASKS_MIN 0
#define TASKS_MAX 3

// Global variables
volatile unsigned char check_button_flag = OFF; // flags
volatile unsigned char check_temperature_flag = OFF;
volatile unsigned char report_server_flag = OFF;
volatile unsigned char up_button_flag = OFF;
volatile unsigned char down_button_flag = OFF;

int16_t clock = OFF; // counters and temperatures
int16_t online_seconds = OFF;
int16_t recorded_temperature = TEMPERATURE_RESET;
int16_t set_temperature = SET_TEMPERATURE_START;

// Task process declarations
void checkButton();
void checkTemperature();
void reportServer();

/*
 * Tasks data structure consisting of flag and function
 */
struct task {
    char flag;
    void (*f)();
};

/*
 * Task list consisting of 3 tasks checkButton, checkTemperature, and reportServer
 */
struct task tasks[TASKS_MAX] = {
                        {OFF, &checkButton},
                        {OFF, &checkTemperature},
                        {OFF, &reportServer}
};

/*
 *
 * ========= UART DRIVER CODE FROM LAB GUIDE ===========
 *
 * */

// UART Global Variables
char output[64];
int bytesToSend;

// Driver Handles - Global variables
UART_Handle uart;

void initUART(void)
{
    UART_Params uartParams;

    // Init the driver
    UART_init();

    // Configure the driver
    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_BINARY;
    uartParams.readDataMode = UART_DATA_BINARY;
    uartParams.readReturnMode = UART_RETURN_FULL;
    uartParams.baudRate = 115200;

    // Open the driver
    uart = UART_open(CONFIG_UART_0, &uartParams);
    if (uart == NULL) {
        /* UART_open() failed */
        while (1);
    }
}

/*
 *
 * ========== TIMER DRIVER CODE FROM LAB GUIDE ===========
 *
 * */

// Driver Handles - Global variables
Timer_Handle timer0;
volatile unsigned char TimerFlag = 0;

/*
 *
 * Timer callback function handles task scheduling logic
 * Raises checkButton flag every 200ms, checkTemperature flag every 500ms, and reportServer flag every 1000ms
 *
*/
void timerCallback(Timer_Handle myHandle, int_fast16_t status)
{
    if (clock < CLOCK_MAX) { // loop from 0 to 9 in 100ms intervals (0ms-900ms)
        clock += CLOCK_INCREMENT;
    } else {
        clock = CLOCK_MIN;
    }

    if (!(clock % CLOCK_200_MS)) { // handle 200ms interval
        check_button_flag = ON;
        tasks[0].flag = ON;
    }

    if (!(clock % CLOCK_500_MS)) { // handle 500ms interval
        check_temperature_flag = ON;
        tasks[1].flag = ON;
    }

    if (!clock) { // handle 1000ms interval
        report_server_flag = ON;
        tasks[2].flag = ON;
        online_seconds += CLOCK_INCREMENT;
    }
}

void initTimer(void)
{
    Timer_Params params;

    // Init the driver
    Timer_init();

    // Configure the driver
    Timer_Params_init(&params);
    params.period = 100000;
    params.periodUnits = Timer_PERIOD_US;
    params.timerMode = Timer_CONTINUOUS_CALLBACK;
    params.timerCallback = timerCallback;

    // Open the driver
    timer0 = Timer_open(CONFIG_TIMER_0, &params);
    if (timer0 == NULL) {
        /* Failed to initialized timer */
        while (1) {}
    }
    if (Timer_start(timer0) == Timer_STATUS_ERROR) {
        /* Failed to start timer */
        while (1) {}
    }
}

/*
 *
 *  =========== I2C DRIVER CODE FROM LAB GUIDE ===========
 *
 * */

// I2C Global Variables
static const struct {
    uint8_t address;
    uint8_t resultReg;
    char *id;
}

sensors[3] = {
              { 0x48, 0x0000, "11X" },
              { 0x49, 0x0000, "116" },
              { 0x41, 0x0001, "006" }
};

uint8_t txBuffer[1];
uint8_t rxBuffer[2];
I2C_Transaction i2cTransaction;

// Driver Handles - Global variables
I2C_Handle i2c;

// Make sure you call initUART() before calling this function.
void initI2C(void)
{
    int8_t i, found;
    I2C_Params i2cParams;
    DISPLAY(snprintf(output, 64, "Initializing I2C Driver - "))

    // Init the driver
    I2C_init();

    // Configure the driver
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;

    // Open the driver
    i2c = I2C_open(CONFIG_I2C_0, &i2cParams);
    if (i2c == NULL)
    {
        DISPLAY(snprintf(output, 64, "Failed\n\r"))
        while (1);
    }

    DISPLAY(snprintf(output, 32, "Passed\n\r"))

    // Boards were shipped with different sensors.
    // Welcome to the world of embedded systems.
    // Try to determine which sensor we have.
    // Scan through the possible sensor addresses
    /* Common I2C transaction setup */
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 0;
    found = false;

    for (i=0; i<3; ++i)
    {
        i2cTransaction.slaveAddress = sensors[i].address;
        txBuffer[0] = sensors[i].resultReg;
        DISPLAY(snprintf(output, 64, "Is this %s? ", sensors[i].id))

        if (I2C_transfer(i2c, &i2cTransaction))
        {
            DISPLAY(snprintf(output, 64, "Found\n\r"))
            found = true;
            break;
        }

        DISPLAY(snprintf(output, 64, "No\n\r"))
    }
}

int16_t readTemp(void)
{
    int16_t j;
    int16_t temperature = 0;
    i2cTransaction.readCount = 2;

    if (I2C_transfer(i2c, &i2cTransaction))
    {
        /*
         * Extract degrees C from the received data;
         * see TMP sensor datasheet
         */
        temperature = (rxBuffer[0] << 8) | (rxBuffer[1]);
        temperature *= 0.0078125;

        /*
         * If the MSB is set '1', then we have a 2's complement
         * negative value which needs to be sign extended
         */
        if (rxBuffer[0] & 0x80)
        {
            temperature |= 0xF000;
        }
    }

    return temperature;
}

/*
 *  ======== gpioButtonFxn0 ========
 *  Callback function for the GPIO interrupt on CONFIG_GPIO_BUTTON_0.
 *
 *  Note: GPIO interrupts are cleared prior to invoking callbacks.
 */
void gpioButtonFxn0(uint_least8_t index)
{
    // Raises flag indicating set_temperature should increase on next 200ms interval
    down_button_flag = ON;

}

/*
 *  ======== gpioButtonFxn1 ========
 *  Callback function for the GPIO interrupt on CONFIG_GPIO_BUTTON_1.
 *  This may not be used for all boards.
 *
 *  Note: GPIO interrupts are cleared prior to invoking callbacks.
 */
void gpioButtonFxn1(uint_least8_t index)
{
    // Raises flag indicating set_temperature should decrease on next 200ms interval
    up_button_flag = ON;

}

/*
 *
 * Function that is called every 200ms when check_button_flag is raised
 * Determines whether to increase or decrease set_temperature, then lowers flags
 *
 */
void checkButton() {
    if (check_button_flag) {
        if (up_button_flag) { // handle increase set_temperature
            set_temperature += TEMPERATURE_INCREMENT; // increase set_temperature
            up_button_flag = OFF; // lower increase flag
        }

        if (down_button_flag) { // handle decrease set_temperature
            set_temperature -= TEMPERATURE_INCREMENT; // decrease set_temperature
            down_button_flag = OFF; // lower decrease flag
        }
        check_button_flag = OFF; // lower check_button_flag
    }
}


/*
 *
 * Function that is called every 500ms when check_temperature_flag is raised
 * Sets global variable recorded_temperature equal to current temperature reading from sensor, then lowers flag
 *
 */
void checkTemperature() {
    if (check_temperature_flag) {
        recorded_temperature = TEMPERATURE_RESET; // reset recorded_temperature to zero
        recorded_temperature = recorded_temperature + readTemp(); // add temperature reading to recorded_temperature
        check_temperature_flag = OFF; // lower flag
    }
}

/*
 *
 * Function that is called every 1000ms when report_server_flag is raised
 * Checks if current temperature is lower than set_temperature, then lowers flag
 *
 * If so, the LED is turned on indicating a heater has been activated and the data is sent to the server via UART
 * Otherwise, the LED is turned off indicating a heater is not activated and the data is sent to the server via UART
 *
 */
void reportServer() {
    if (report_server_flag) {
        if (recorded_temperature < set_temperature) { // if current temperature is lower than set_temperature
            GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON); // turn LED on
            DISPLAY( snprintf(output, 64, "<%02d,%02d,%d,%04d>\n\r", recorded_temperature, set_temperature, ON, online_seconds)) // send data to server
        } else { // if current temperature is not lower than set_temperature
            GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF); // turn LED off
            DISPLAY( snprintf(output, 64, "<%02d,%02d,%d,%04d>\n\r", recorded_temperature, set_temperature, OFF, online_seconds)) // send data to server
        }
        report_server_flag = OFF; // lower flag
    }
}


/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{
    /* Call driver init functions */
    GPIO_init();

    /* Configure the LED and button pins */
    GPIO_setConfig(CONFIG_GPIO_LED_0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);

    GPIO_setConfig(CONFIG_GPIO_BUTTON_0, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

    /* Turn on user LED */
    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);

    /* Install Button callback */
    GPIO_setCallback(CONFIG_GPIO_BUTTON_0, gpioButtonFxn0);

    /* Enable interrupts */
    GPIO_enableInt(CONFIG_GPIO_BUTTON_0);

    /*
     *  If more than one input pin is available for your device, interrupts
     *  will be enabled on CONFIG_GPIO_BUTTON1.
     */
    if (CONFIG_GPIO_BUTTON_0 != CONFIG_GPIO_BUTTON_1) {
        /* Configure BUTTON1 pin */
        GPIO_setConfig(CONFIG_GPIO_BUTTON_1, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

        /* Install Button callback */
        GPIO_setCallback(CONFIG_GPIO_BUTTON_1, gpioButtonFxn1);
        GPIO_enableInt(CONFIG_GPIO_BUTTON_1);
    }


    initUART();
    initI2C();
    initTimer();

    while(1){ // repeat while device is connected and program is running

        // loop through tasks, process task if flag is raised, then reset flags
        int i = TASKS_MIN;
        for (i = TASKS_MIN; i < TASKS_MAX; i++) { // loop through tasks
            if (tasks[i].flag) { // check if flag is raised
                tasks[i].f(); // process task
                tasks[i].flag = OFF; // reset flag
            }
        }
    }

    return (NULL);
}
