/*****************************************************************************
 * University of Southern Denmark
 * Embedded C Programming (ECP)
 *
 * MODULENAME.: SPI.h
 *
 * PROJECT....: ECP
 *
 * DESCRIPTION:
 * functions and setup for SPI communication
 *
 * Change Log:
 ******************************************************************************
 * Date    Id    Change
 * YYMMDD
 * --------------------
 * 050128  KA    Module created.
 *
 *****************************************************************************/

/* ******************************** INCLUDES ******************************** */
#include <stdint.h>
#include "tm4c123gh6pm.h"
#include "FreeRTOS.h"
#include "Task.h"
#include "queue.h"
#include "semphr.h"
#include "emp_type.h"

#include "gpio.h"
#include "tmodel.h"
#include "SPI.h"
#include "uart0.h"
#include "file.h"
#include "string.h"

/*****************************    Defines    *******************************/

#define SPI_BITS 10
#define ENCODER_BIT 9

#define SS_PIN 3

#define TILT 0
#define PAN 1

#define PRESCALER 10

/* **************************** GENERAL VARIABLES *************************** */

// data for 9 bits data
// INT16U data_send = 0b1111111111; // random value used for sending
INT16U data_send = 767; // random value used for sending
int counter = 0;        // used for counting through sending a string
INT16U data;
BOOLEAN pan_or_tilt = 0; // used for indicating pan or tilt
INT8U counter_prescale = 0;
INT8U sample_frequency = 50;

INT16U model_encoder_angle = 90; // for testing

/* ********************************* QUEUES ********************************* */
extern QueueHandle_t q_pan_SPI;
extern QueueHandle_t q_tilt_SPI;

extern QueueHandle_t q_pan_con;
extern QueueHandle_t q_tilt_con;

extern QueueHandle_t q_model_pan_enc;
extern QueueHandle_t q_model_tilt_enc;

/* ******************************** SEMAPHORE ******************************* */
extern SemaphoreHandle_t mutex_model_pan_enc;
extern SemaphoreHandle_t mutex_model_tilt_enc;

extern SemaphoreHandle_t mutex_pan_con;
extern SemaphoreHandle_t mutex_tilt_con;

extern SemaphoreHandle_t mutex_pan_SPI;
extern SemaphoreHandle_t mutex_tilt_SPI;

/* ******************************** FUNCTIONS ******************************* */

void SPI_init()
{

    // pins used 2,3,4,5 on port A
    // STEP 1
    SYSCTL_RCGCSSI_R |= (1 << 0); /*set clock enabling bit for SPI0 */
    // SYSCTL_RCGCGPIO_R |= (1<<3); /* enable clock to GPIOD for SPI1 */

    // STEP 2
    SYSCTL_RCGCGPIO_R |= (1 << 0); /* enable clock to GPIOF for slave select */

    // STEP 3
    GPIO_PORTA_AFSEL_R |= (1 << 2) | (1 << 3) | (1 << 4) | (1 << 5); /* enable alternate function of the pins*/

    //
    // GPIO_PORTA_PCTL_R &= ~0x0000F00F; /* assign RD0 and RD3 pins to SPI1 */

    // STEP 4
    GPIO_PORTA_PCTL_R &= ~(0x00FFFF00); // clear register before assigning, not sure if reset value is zero
    GPIO_PORTA_PCTL_R |= 0x00222200;    /* assign pins to SPI1  */

    // STEP 5
    GPIO_PORTA_AMSEL_R &= ~0b00111100; /* disable analog functionality pins, not sure if necessary??? */
    GPIO_PORTA_DIR_R |= 0b00101100;    // set direction
    GPIO_PORTA_DEN_R |= 0b00111100;    /* Set PINS as digital pins */
    GPIO_PORTA_DATA_R |= (1 << 3);     // set ss as high when idle

    // STEP 6
    /* Select SPI1 as a Master, POL = 0, PHA = 0, clock = 4 MHz, 8 bit data */
    SSI0_CR1_R &= ~(1 << 1); /* disable SPI1 and configure it as a Master */
    SSI0_CR1_R = 0x00000000;
    SSI0_CC_R = 0;          /* Enable System clock Option */
    SSI0_CPSR_R = 1;        /* Select prescaler value of 4 .i.e 16MHz/100 = 0.16MHz */
    SSI0_CR0_R |= 0 << 8;   // set the serial clock rate; BR=SysClk/(CPSDVSR * (1 + SCR))
    SSI0_CR0_R = SPI_BITS;  /* 4MHz SPI1 clock, SPI mode, 10 bit data */
    SSI0_CR0_R |= (1 << 6); // set clock to inverted mode
    SSI0_CR0_R |= (1 << 7); // set SPH to high, this makes the first transmission happen on the first falling edge.
    SSI0_CR1_R |= 2;        /* enable SPI1 */
}

/* Function to get parity of number n. It returns 0
   if n has odd parity, and returns 1 if n has even
   parity */
BOOLEAN getParity(INT16U data)
{
    BOOLEAN parity = 0;
    while (data)
    {
        parity = !parity;
        data = data & (data - 1);
    }
    return parity;
}

INT16U insertParity(INT16U data)
{
    data = data << 1; // shift data to make room for parity bit
    if (!getParity(data))
    {              // check if number of bits is odd, we use even parity.
        data |= 1; // if set the last bit to make number of bits even
    }
    return data;
}

INT16U removeParity(INT16U data)
{
    data = data >> 1;
    return data;
}

BOOLEAN checkEncoderBit(INT16U data)
{
    if (data & (1 << ENCODER_BIT))
        pan_or_tilt = PAN;
    else
        pan_or_tilt = TILT;
    return data;
}

INT16U removeEncoderBit(INT16U data)
{
    data &= ~(1 << ENCODER_BIT);
    return data;
}

void SPI_receive()
{
    /* **************************** read data on MISO *************************** */
    if (!((SSI0_SR_R & 4) == 1))
    { // should check to see if the receive FIFO is not empty,
      // currently it just acts as if it empty
        INT16U r_data = SSI0_DR_R;
        r_data &= 0b1111111111;                           // clear everything except the 10 bits. This is done to also clear the MSB of the MISO
        INT16U r_data_without_enc = r_data & 0b111111111; // clear the encoder bit
        /* -------------------------------------------------------------------------- */

        /* ************************* send data to controller ************************ */
        if (r_data & (1 << 9)) // check if the ninth bit is set
        {
            r_data_without_enc = r_data_without_enc * 4 / 3; // convert to degrees
            gfprintf(COM1, "P:%u", r_data_without_enc);
            if (xSemaphoreTake(mutex_pan_SPI, (TickType_t)10) == pdTRUE)
            {
                xQueueOverwrite(q_pan_SPI, &r_data_without_enc);
                xSemaphoreGive(mutex_pan_SPI);
            }
        }
        else
        {
            r_data_without_enc = r_data_without_enc * 4 / 3; // convert to degrees
            gfprintf(COM1, "T:%u\r\n", r_data_without_enc);
            if (xSemaphoreTake(mutex_tilt_SPI, (TickType_t)10) == pdTRUE)
            {
                xQueueOverwrite(q_tilt_SPI, &r_data_without_enc);
                xSemaphoreGive(mutex_tilt_SPI);
            }
        }
    }
}

/* ********************************** TASKS ********************************* */
void SPI_test_task(void *pvParameters)
{
    INT16U pos_neg;
    INT16U pwm;
    while (1)
    {
        if (xSemaphoreTake(mutex_model_pan_enc, (TickType_t)10) == pdTRUE)
        {
            xQueueOverwrite(q_model_pan_enc, &model_encoder_angle);
            xSemaphoreGive(mutex_model_pan_enc);
        }

        if (xSemaphoreTake(mutex_model_tilt_enc, (TickType_t)10) == pdTRUE)
        {
            xQueueOverwrite(q_model_tilt_enc, &model_encoder_angle);
            xSemaphoreGive(mutex_model_tilt_enc);
        }
        pan_or_tilt = !pan_or_tilt;

        /* ************************ get data from controller ************************ */
        if (pan_or_tilt == PAN)
        {
            if (xSemaphoreTake(mutex_pan_con, (TickType_t)10) == pdTRUE)
            {
                if (xQueuePeek(q_pan_con, &data, 0) == pdFALSE)
                {
                    data = 0;
                }
                pos_neg = data & (1 << 8);
                pwm = data & ~(1 << 8);
                if (pos_neg)
                {
                    gfprintf(COM1, " PANCON:- %u  ", pwm);
                }
                else
                {
                    gfprintf(COM1, " PANCON:+ %u  ", pwm);
                }
                xSemaphoreGive(mutex_pan_con);
            }
        }
        else
        {
            if (xSemaphoreTake(mutex_tilt_con, (TickType_t)10) == pdTRUE)
            {
                if (xQueuePeek(q_tilt_con, &data, 0) == pdFALSE)
                {
                    data = 0;
                }
                pos_neg = data & (1 << 8);
                pwm = data & ~(1 << 8);
                if (pos_neg)
                {
                    gfprintf(COM1, " TILTCON:- %u  ", pwm);
                }
                else
                {
                    gfprintf(COM1, " TILTCON:+ %u  ", pwm);
                }
                xSemaphoreGive(mutex_tilt_con);
            }
        }
        gfprintf(COM1, " hej:");
        vTaskDelay(5 / portTICK_RATE_MS);
    }
}
void SPI_task(void *pvParameters)
/*****************************************************************************
 *   Input    :
 *   Output   :
 *   Function : Sends data on the transmit ´pin
 ******************************************************************************/
{
    INT16U pos_neg;
    INT16U pwm;
    while (1)
    {
        /* ********************* temporarily give fake reference ******************** */
        if (xSemaphoreTake(mutex_model_pan_enc, (TickType_t)10) == pdTRUE)
        {
            xQueueOverwrite(q_model_pan_enc, &model_encoder_angle);
            xSemaphoreGive(mutex_model_pan_enc);
        }

        if (xSemaphoreTake(mutex_model_tilt_enc, (TickType_t)10) == pdTRUE)
        {
            xQueueOverwrite(q_model_tilt_enc, &model_encoder_angle);
            xSemaphoreGive(mutex_model_tilt_enc);
        }
        /* -------------------------------------------------------------------------- */

        // data_send++;
        // data_send = data_send % 255;
        // data = data_send;
        // if (data % 2 == 0)
        //     data |= 1 << 9;

        pan_or_tilt = !pan_or_tilt;

        /* ************************ get data from controller ************************ */
        if (pan_or_tilt == PAN)
        {
            if (xSemaphoreTake(mutex_pan_con, (TickType_t)10) == pdTRUE)
            {
                if (xQueuePeek(q_pan_con, &data, 0) == pdFALSE)
                {
                    data = 0;
                }
                pos_neg = data & (1 << 8);
                pwm = data & ~(1 << 8);
                if (pos_neg)
                {
                    gfprintf(COM1, "PC:-%u", pwm);
                }
                else
                {
                    gfprintf(COM1, "PC:+%u", pwm);
                }
                xSemaphoreGive(mutex_pan_con);
            }
        }
        else
        {
            if (xSemaphoreTake(mutex_tilt_con, (TickType_t)10) == pdTRUE)
            {
                if (xQueuePeek(q_tilt_con, &data, 0) == pdFALSE)
                {
                    data = 0;
                }
                pos_neg = data & (1 << 8);
                pwm = data & ~(1 << 8);
                if (pos_neg)
                {
                    gfprintf(COM1, "TC:-%u", pwm);
                }
                else
                {
                    gfprintf(COM1, "TC:+%u", pwm);
                }
                xSemaphoreGive(mutex_tilt_con);
            }
        }
        // gfprintf(COM1, "data%u", data);

        data |= 1 << 10; // make LSB redundant

        /* *************************** send data over spi *************************** */
        GPIO_PORTA_DATA_R &= ~(1 << SS_PIN); // set ss as low to indicate communication
        while ((SSI0_SR_R & 2) == 0)
            ;
        SSI0_DR_R = data;
        while ((SSI0_SR_R & 0x10))
            ;
        GPIO_PORTA_DATA_R |= (1 << SS_PIN); // set ss as high to end communication
        /* -------------------------------------------------------------------------- */

        SPI_receive();
        vTaskDelay(5 / portTICK_RATE_MS);
        // vTaskDelay(1000 / sample_frequency / portTICK_RATE_MS);
    }
}

/* -------------------------------------------------------------------------- */
