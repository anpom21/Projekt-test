/*****************************************************************************
* University of Southern Denmark
* Embedded C Programming (ECP)
*
* MODULENAME.: leds.h
*
* PROJECT....: ECP
*
* DESCRIPTION: Test.
*
* Change Log:
******************************************************************************
* Date    Id    Change
* YYMMDD
* --------------------
* 050128  KA    Module created.
*
*****************************************************************************/

#ifndef _LCD_H
  #define _LCD_H

/***************************** Include files *******************************/
#include "emp_type.h"
/*****************************    Defines    *******************************/
// Special ASCII characters
// ------------------------

#define LF		0x0A
#define FF		0x0C
#define CR		0x0D

#define ESC		0x1B


/*****************************   Constants   *******************************/

/*****************************   Functions   *******************************/
void move_LCD( INT8U, INT8U );
INT8U wr_ch_LCD( INT8U );
void wr_str_LCD( INT8U* string );

// Variables for main:
//--------------------
// QueueHandle_t q_lcd;
// SemaphoreHandle_t lcd_mutex;



void lcd_write(INT8U* string, INT8U x, INT8U y);
/*****************************************************************************
*   Input    : Strings can be put directly in as the string input.
*              Chars need to be passed by address ex lcd_write(&ch,0,0)
*              For x and y the desired LCD position should be used
*   Output   : -
*   Function : Initialization
******************************************************************************/

void lcd_init();
/*****************************************************************************
*   Input    : -
*   Output   : -
*   Function : Initialization
******************************************************************************/
void clr_LCD();
/*****************************************************************************
*   Input    : -
*   Output   : -
*   Function : Clear display
******************************************************************************/






extern void lcd_task(void *pvParameters );
/*****************************************************************************
*   Input    : -
*   Output   : -
*   Function : Should be run by freeRTOS
******************************************************************************/
extern void lcd_example(void *pvParameters );
/*****************************************************************************
*   Input    : -
*   Output   : -
*   Function : Example use of LCD screen
******************************************************************************/

/****************************** End Of Module *******************************/
#endif

