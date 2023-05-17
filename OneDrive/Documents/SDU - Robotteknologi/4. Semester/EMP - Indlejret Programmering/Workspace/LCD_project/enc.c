/*
 * enc.c
 *
 *  Created on: 25. apr. 2023
 *      Author: jense
 */

#include <stdint.h>
#include "tm4c123gh6pm.h"
#include "FreeRTOS.h"
#include "Task.h"
#include "queue.h"
#include "semphr.h"
#include "emp_type.h"
#include "enc.h"
#include "uart0.h"
#include "string.h"
#include "file.h"

// Pan
// ---
// * SW to PC7
// * A  to PC6
// * B  to PC5

// Tilt
// ---
// * SW to PE3
// * A  to PE2
// * B  to PE1



/*****************************    Defines    *******************************/
#define start_state 0
#define zero_state  1
#define read_state  2


#define PAN_AB_PORT    GPIO_PORTC_DATA_R
#define PAN_AB_check   0x60
#define PAN_AB_A       0x40    // PE6
#define PAN_AB_B       0x20    // PE5

#define TILT_AB_PORT    GPIO_PORTA_DATA_R
#define TILT_AB_check   0xC0
#define TILT_AB_A       0x80  // PE2
#define TILT_AB_B       0x40  // PE1

#define DEBOUNCE_LENGTH    3
#define DEBOUNCE_ACTIVATED 1

/*****************************   Constants   *******************************/

/*****************************   Variables   *******************************/
extern SemaphoreHandle_t mutex_model_pan_enc;
extern QueueHandle_t q_pan_angle;
INT16S pan_angle;

extern SemaphoreHandle_t mutex_model_tilt_enc;
extern QueueHandle_t q_tilt_angle;
INT16S tilt_angle;

/*****************************   Functions   *******************************/


void enc_pan_init()
{ 
    
    SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOC;
    // Digital enable
    GPIO_PORTC_DEN_R |= 0xE0; // Pan to PIN PC5 PC6 PC7
    // Set direction
    GPIO_PORTC_DIR_R &= ~(0xE0);

    mutex_pan_angle = xSemaphoreCreateMutex();
    q_pan_angle = xQueueCreate(1, sizeof(INT16S));
}

void enc_tilt_init()
{
        
    //SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOE;
    //
    //// Digital enable
    //GPIO_PORTE_DEN_R |= 0x0E; // Tilt to PIN PE1 PE2 PE3
    //// Set direction
    //GPIO_PORTE_DIR_R &= ~(0x0E);

    SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOA;
    
    // Digital enable
    GPIO_PORTA_DEN_R |= 0xE0; // Tilt to PIN PE1 PE2 PE3
    // Set direction
    GPIO_PORTA_DIR_R &= ~(0xE0);

    mutex_tilt_angle = xSemaphoreCreateMutex();
    q_tilt_angle = xQueueCreate(1, sizeof(INT16S));
}

//
//INT16S get_pan_position(){
//    if (xSemaphoreTake(mutex_pan_angle, (TickType_t)10) == pdTRUE)
//    {
//
//
//        xSemaphoreGive(mutex_pan_angle);
//    }
//}
//
//INT16S get_tilt_position(){
//    if (xSemaphoreTake(mutex_tilt_angle, (TickType_t)10) == pdTRUE)
//    {
//
//
//        xSemaphoreGive(mutex_tilt_angle);
//    }
//}
//
BOOLEAN debounce_pan(BOOLEAN strobe){
    static INT8U prev_strobe = 0;

	static INT8U debounce_arr[DEBOUNCE_LENGTH];
	static INT8U fill = 0;
    INT8U check, i;
    BOOLEAN result;

    check = (strobe == prev_strobe);
    result = check;

	for (i = DEBOUNCE_LENGTH-1; i > 0; i--)
	{
		debounce_arr[i] = debounce_arr[ i-1 ];
		
		if (debounce_arr[i] != strobe)
		{
			result = 0;
		}
		
	}
	debounce_arr[0] = strobe;
	
    prev_strobe = strobe;
	return result;
}

BOOLEAN debounce_tilt(BOOLEAN strobe){
    static INT8U prev_strobe = 0;
	static INT8U debounce_arr[DEBOUNCE_LENGTH];
	static INT8U fill = 0;
    INT8U check, i;
    BOOLEAN result;

    check = (strobe == prev_strobe);
    result = check;

	for (i = DEBOUNCE_LENGTH-1; i > 0; i--)
	{
		debounce_arr[i] = debounce_arr[ i-1 ];

		if (debounce_arr[i] != strobe)
		{
			result = 0;
		}

	}
	debounce_arr[0] = strobe;

    prev_strobe = strobe;
	return result;
}


void angle_pan_task(void *pvParameters)
{
    INT16S enc_ticks;
    INT8U AB, AB_prev, YY;
    INT16S deg_pr_tick = 6;
    INT8U state = start_state;


    while (1)
    {
        switch (state)
        {
        case start_state:
            // Initialize encoder
            if (xSemaphoreTake(mutex_pan_angle, (TickType_t)10) == pdTRUE)
            {
                pan_angle = 0;
                enc_ticks = 0;
                AB = PAN_AB_PORT & PAN_AB_check;
                AB_prev = AB;
                YY = 0x00;
                state = read_state;
                xQueueOverwrite(q_pan_angle, pan_angle);
                xSemaphoreGive(mutex_pan_angle);
                uart0_put_string("Starting pan encoder");
            }
            break;
        case read_state:
            
            if (xSemaphoreTake(mutex_pan_angle, (TickType_t)10) == pdTRUE)
            {
                // Check current encoder position
                AB = PAN_AB_PORT & PAN_AB_check; 

                // Debounce signal and check if it is different from the previous position
                if ( debounce_pan(AB) && AB != AB_prev)
                {
                    // Calculate direction and inc / dec enc_ticks
                    YY = AB ^ AB_prev;

                    if (AB == PAN_AB_check || AB == 0x00)
                    {
                        if (YY == PAN_AB_A)
                            enc_ticks++;
                        else if (YY == PAN_AB_B)
                            enc_ticks--;
                    }

                    else
                    {
                        if (YY == PAN_AB_A)
                            enc_ticks--;
                        else if (YY == PAN_AB_B)
                            enc_ticks++;
                    }

                    AB_prev = AB;
                }

                // Calculate current angle
                pan_angle = enc_ticks * deg_pr_tick;

                // Save in queue
                xQueueOverwrite(q_pan_angle, &pan_angle);
                xSemaphoreGive(mutex_pan_angle);
            }
            vTaskDelay(1 / portTICK_RATE_MS);
            break;
        }
    }
}

void angle_tilt_task(void *pvParameters)
{
    INT16S enc_ticks;
    INT8U AB, AB_prev, YY;
    INT16S deg_pr_tick = 6;
    INT8U state = start_state;


    while (1)
    {
        switch (state)
        {
        case start_state:
            // Initialize encoder
            if (xSemaphoreTake(mutex_tilt_angle, (TickType_t)10) == pdTRUE)
            {
                tilt_angle = 0;
                enc_ticks = 0;
                AB = TILT_AB_PORT & TILT_AB_check;
                AB_prev = AB;
                YY = 0x00;
                state = read_state;
                xQueueOverwrite(q_tilt_angle, tilt_angle);
                xSemaphoreGive(mutex_tilt_angle);
                uart0_put_string("Starting tilt encoder");
            }
            break;
        case read_state:
            
            if (xSemaphoreTake(mutex_tilt_angle, (TickType_t)10) == pdTRUE)
            {
                // Check current encoder position
                AB = TILT_AB_PORT & TILT_AB_check; 

                // Debounce signal and check if it is different from the previous position
                if ( debounce_tilt(AB)  && AB != AB_prev)
                {
                    // Calculate direction and inc / dec enc_ticks
                    YY = AB ^ AB_prev;

                    if (AB == TILT_AB_check || AB == 0x00)
                    {
                        if (YY == TILT_AB_A)
                            enc_ticks++;
                        else if (YY == TILT_AB_B)
                            enc_ticks--;
                    }

                    else
                    {
                        if (YY == TILT_AB_A)
                            enc_ticks--;
                        else if (YY == TILT_AB_B)
                            enc_ticks++;
                    }

                    AB_prev = AB;
                }

                // Calculate current angle
                tilt_angle = enc_ticks * deg_pr_tick;

                // Save in queue 
                xQueueOverwrite(q_tilt_angle, &tilt_angle);
                xSemaphoreGive(mutex_tilt_angle);
            }
            vTaskDelay(1 / portTICK_RATE_MS);
            break;
        }
    }
}
