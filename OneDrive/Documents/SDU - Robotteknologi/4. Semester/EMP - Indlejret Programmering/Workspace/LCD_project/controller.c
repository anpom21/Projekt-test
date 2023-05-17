/*****************************************************************************
 * University of Southern Denmark
 *
 *
 * MODULENAME.: PID_controller.c
 *
 * PROJECT....: Pan Tilt Project
 *
 * DESCRIPTION: See module specification file (.h-file).
 *
 * Change Log:
 *****************************************************************************
 * Date    Id    Change
 * YYMMDD
 * --------------------
 * 230323  MoH   Module created.
 *
 *****************************************************************************/

/***************************** Include files *******************************/
#include <stdint.h>
// #include <stdio.h>
#include "tm4c123gh6pm.h"
#include "FreeRTOS.h"
#include "Task.h"
#include "queue.h"
#include "semphr.h"
#include "emp_type.h"

#include "file.h"
// #include "emp_type.h"

/*****************************    Defines    *******************************/

// Sampling rate
// -------------
#define T 0.05

// Controller constants
// --------------------
#define KP_PAN 0.3070
#define TI_PAN 14.0183
#define TD_PAN 0.2870
#define N_PAN 8.6961

#define KP_TILT 0.3
#define TI_TILT 20
#define TD_TILT 0.001
#define N_TILT 2

// Max and min pan angles
// ----------------------
#define PAN_MAX 140
#define PAN_MIN 20

// Max and min duty cycle
// --------------
#define VOLTAGE_MAX_TILT 12
#define VOLTAGE_MIN_TILT 1

#define VOLTAGE_MAX_PAN 12
#define VOLTAGE_MIN_PAN 1

/*****************************   Constants   *******************************/

/*****************************   Variables   *******************************/
extern QueueHandle_t q_pan_SPI;
extern QueueHandle_t q_model_pan_enc;
extern QueueHandle_t q_pan_con;

extern QueueHandle_t q_tilt_SPI;
extern QueueHandle_t q_model_tilt_enc;
extern QueueHandle_t q_tilt_con;

/******************************  SEMAPHORE ************************************/
extern SemaphoreHandle_t mutex_model_pan_enc;
extern SemaphoreHandle_t mutex_model_tilt_enc;

extern SemaphoreHandle_t mutex_pan_con;
extern SemaphoreHandle_t mutex_tilt_con;

extern SemaphoreHandle_t mutex_pan_SPI;
extern SemaphoreHandle_t mutex_tilt_SPI;
/*****************************   Functions   *******************************/

INT16S pan_controller(INT16U sensor_value, INT16U setpoint)
/*****************************************************************************
*   Input    : INT16S sensor_value: Current position in degrees.
               INT16S setpoint:     Target position in degrees.
*   Output   : PWM control signal for FPGA.
*   Function : Takes the sensor value and setpoint to generate a PWM signal for the motor.
******************************************************************************/
{
    // Initialize variables
    static INT16S integral = 0;
    static INT16S prev_integral = 0;
    static INT16S prev_derivative = 0;
    static INT16S prev_error = 0;
    static BOOLEAN anti_wind_up = 1;
    INT16S error, derivative, output;

    //////////////////   FOR PAN ONLY    //////////////////

    // Limit the setpoint to not hit physical stop
    if (setpoint < PAN_MIN)
    {
        setpoint = PAN_MIN;
    }
    else if (setpoint > PAN_MAX)
    {
        setpoint = PAN_MAX;
    }
    //////////////////   FOR PAN ONLY    //////////////////

    // Calculate error
    error = setpoint - sensor_value; // May have to change sign if controller doesnt work

    // Calculate integral term
    // integral += error * DT;
    integral = prev_integral + KP_PAN / TI_PAN * T / 2 * (error + prev_error);

    // Calculate derivative term
    derivative = (KP_PAN * TD_PAN / (T / 2 + TD_PAN / N_PAN)) * (error - prev_error) - ((T / 2 - TD_PAN / N_PAN) / (T / 2 + TD_PAN / N_PAN)) * prev_derivative;

    // Calculate output
    output = KP_PAN * error + anti_wind_up * integral + derivative;

    // Saturate MAX signal and anti wind up
    if (output > VOLTAGE_MAX_PAN)
    {
        output = VOLTAGE_MAX_PAN;
        anti_wind_up = 0;
    }
    else if (output < -VOLTAGE_MAX_PAN)
    {
        output = -VOLTAGE_MAX_PAN;
        anti_wind_up = 0;
    }
    else
    {
        anti_wind_up = 1;
    }

    // Saturate MIN signal
    if (output < VOLTAGE_MIN_PAN && output > 0)
    {
        output = VOLTAGE_MIN_PAN;
    }
    else if (output > -VOLTAGE_MIN_PAN && output < 0)
    {
        output = -VOLTAGE_MIN_PAN;
    }

    // Save the previous error
    prev_error = error;
    prev_derivative = derivative;
    prev_integral = integral;
    output = output * 254 / 12;
    if (output < 0)
    {
        output = ~output;
        output |= (1 << 8);
        output++; // change from signed to the protocol for SPI
    }

    return output;
}

INT16S tilt_controller(INT16U sensor_value, INT16U setpoint)
/*****************************************************************************
*   Input    : INT16S sensor_value: Current position in degrees.
               INT16S setpoint:     Target position in degrees.
*   Output   : PWM control signal for FPGA.
*   Function : Takes the sensor value and setpoint to generate a PWM signal for the motor.
******************************************************************************/
{

    // Initialize static variables
    static INT16S integral = 0;
    static INT16S prev_integral = 0;
    static INT16S prev_derivative = 0;
    static INT16S prev_error = 0;
    static BOOLEAN anti_wind_up = 1;
    INT16S error, derivative, output;

    // Calculate error
    error = setpoint - sensor_value; // May have to change sign if controller doesnt work

    // Determine shortest direction
    if (error > 180)
    {
        error = -1 * (360 - error);
    }
    else if (error < -180)
    {
        error = 360 + error;
    }

    ///// CONTROLLER /////

    // Calculate integral term
    if (anti_wind_up)
        integral = prev_integral + KP_TILT / TI_TILT * T / 2 * (error + prev_error);

    // Calculate derivative term
    derivative = (KP_TILT * TD_TILT / (T / 2 + TD_TILT / N_TILT)) * (error - prev_error) - ((T / 2 - TD_TILT / N_TILT) / (T / 2 + TD_TILT / N_TILT)) * prev_derivative;

    // Calculate output
    output = KP_TILT * error + anti_wind_up * integral + derivative;

    ///// CONTROLLER /////

    // Saturate MAX signal and anti wind up
    if (output > VOLTAGE_MAX_TILT)
    {
        output = VOLTAGE_MAX_TILT;
        anti_wind_up = 0;
    }
    else if (output < -VOLTAGE_MAX_TILT)
    {
        output = -VOLTAGE_MAX_TILT;
        anti_wind_up = 0;
    }
    else
    {
        anti_wind_up = 1;
    }

    // Saturate MIN signal
    if (output < VOLTAGE_MIN_TILT && output > 0)
    {
        output = VOLTAGE_MIN_TILT;
    }
    else if (output > -VOLTAGE_MIN_TILT && output < 0)
    {
        output = -VOLTAGE_MIN_TILT;
    }

    // Save the previous error
    prev_error = error;
    prev_derivative = derivative;
    prev_integral = integral;
    output = output * 254 / 12;
    // Change from signed to the protocol for SPI
    if (output < 0)
    {
        output = ~output;
        output |= (1 << 8);
        output++;
    }

    return output;
}

void pan_contr_task(void *pvParameters)
{
    INT16U machine_enc;
    INT16U model_enc;
    INT16S PWM;
    while (1)
    {
        if (xSemaphoreTake(mutex_model_pan_enc, (TickType_t)10) == pdTRUE)
        {
            if (xSemaphoreTake(mutex_pan_SPI, (TickType_t)10) == pdTRUE)
            {
                if (xQueueReceive(q_model_pan_enc, &model_enc, 0) == pdTRUE && xQueueReceive(q_pan_SPI, &machine_enc, 0) == pdTRUE)
                {
                    PWM = pan_controller(machine_enc, model_enc); // need to seample 200 times every second
                    xQueueOverwrite(q_pan_con, &PWM);
                }
                xSemaphoreGive(mutex_pan_SPI);
            }
            xSemaphoreGive(mutex_model_pan_enc);
            vTaskDelay(5 / portTICK_RATE_MS);
        }
    }
}

void tilt_contr_task(void *pvParameters)
{
    INT16U machine_enc;
    INT16U model_enc;
    INT16S PWM;
    while (1)
    {
        if (xSemaphoreTake(mutex_model_tilt_enc, (TickType_t)10) == pdTRUE)
        {
            if (xSemaphoreTake(mutex_tilt_SPI, (TickType_t)10) == pdTRUE)
            {
                if (xQueueReceive(q_model_tilt_enc, &model_enc, 0) == pdTRUE && xQueueReceive(q_tilt_SPI, &machine_enc, 0) == pdTRUE)
                {
                    PWM = tilt_controller(machine_enc, model_enc);
                    xQueueOverwrite(q_tilt_con, &PWM);
                }
                xSemaphoreGive(mutex_tilt_SPI);
            }
            xSemaphoreGive(mutex_model_tilt_enc);
            vTaskDelay(5 / portTICK_RATE_MS); // needs to sample 200 times every second
        }
    }
}
