/*****************************************************************************
* University of Southern Denmark
* Embedded C Programming (ECP)
*
* MODULENAME.: leds.c
*
* PROJECT....: ECP
*
* DESCRIPTION: See module specification file (.h-file).
*
* Change Log:
******************************************************************************
* Date    Id    Change
* YYMMDD
* --------------------
* 050128  KA    Module created.
*
*****************************************************************************/

/***************************** Include files *******************************/

#include <stdint.h>
#include "tm4c123gh6pm.h"
#include "emp_type.h"
#include "lcd.h"

#include "glob_def.h"
#include "tmodel.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"

#include "projdefs.h"
#include "portmacro.h"


/*****************************    Defines    *******************************/

#define QUEUE_LEN   128

enum LCD_states
{
  LCD_POWER_UP,
  LCD_INIT,
  LCD_READY,
  LCD_ESC_RECEIVED,
};

/*****************************   Constants   *******************************/

const INT8U LCD_init_sequense[]= 
{
  0x30,		// Reset
  0x30,		// Reset
  0x30,		// Reset
  0x20,		// Set 4bit interface
  0x28,		// 2 lines Display
  0x0C,		// Display ON, Cursor OFF, Blink OFF
  0x06,		// Cursor Increment
  0x01,		// Clear Display
  0x02,     // Home
  0xFF		// stop
 
  
}; 

/*****************************   Variables   *******************************/

extern QueueHandle_t q_lcd;
extern SemaphoreHandle_t lcd_mutex;
extern SemaphoreHandle_t lcd_write_mutex;

enum LCD_states LCD_state = LCD_POWER_UP;
INT8U LCD_init;

/*****************************   Functions   *******************************/

void lcd_init()
/*****************************************************************************
*   Input    :
*   Output   :
*   Function : Initialize queues and mutexes
******************************************************************************/
{

    q_lcd = xQueueCreate(128, sizeof(INT8U));
    lcd_mutex = xSemaphoreCreateMutex();
    lcd_write_mutex = xSemaphoreCreateMutex();
    clr_LCD();
    //lcd_write("Keypad er klar!",0,0);
}


INT8U wr_ch_LCD( INT8U Ch )
/*****************************************************************************
*   OBSERVE  : LCD_PROC NEEDS 20 mS TO PRINT OUT ONE CHARACTER 
*   Function : See module specification (.h-file).
*****************************************************************************/
{
  xQueueSend( q_lcd, &Ch, portMAX_DELAY );
  vTaskDelay(5 / portTICK_RATE_MS);
  return ( 1 );
}

void wr_str_LCD( INT8U *string )
/*****************************************************************************
*   Function : See module specification (.h-file).
*****************************************************************************/
{
  while( *string )
  {
    wr_ch_LCD( *string );
    string++;
  }
}

void move_LCD( INT8U x, INT8U y )
/*****************************************************************************
*   Function : See module specification (.h-file).
*****************************************************************************/
{
  INT8U Pos;

  Pos = y*0x40 + x;
  Pos |= 0x80;
  wr_ch_LCD( ESC );
  wr_ch_LCD( Pos );


}
//----------------------------

void wr_ctrl_LCD_low( INT8U Ch )
/*****************************************************************************
*   Input    : -
*   Output   : -
*   Function : Write low part of control data to LCD.
******************************************************************************/
{
  INT8U temp;
  volatile int i;
  
  temp = GPIO_PORTC_DATA_R & 0x0F;
  temp  = temp | ((Ch & 0x0F) << 4);
  GPIO_PORTC_DATA_R  = temp;
  for( i=0; i<1000; i )
	  i++;
  GPIO_PORTD_DATA_R &= 0xFB;        // Select Control mode, write
  for( i=0; i<1000; i )
	  i++;
  GPIO_PORTD_DATA_R |= 0x08;		// Set E High

  for( i=0; i<1000; i )
	  i++;

  GPIO_PORTD_DATA_R &= 0xF7;		// Set E Low

  for( i=0; i<1000; i )
	  i++;
}

void wr_ctrl_LCD_high( INT8U Ch )
/*****************************************************************************
*   Input    : -
*   Output   : -
*   Function : Write high part of control data to LCD.
******************************************************************************/
{
  wr_ctrl_LCD_low(( Ch & 0xF0 ) >> 4 );
}

void out_LCD_low( INT8U Ch )
/*****************************************************************************
*   Input    : Mask
*   Output   : -
*   Function : Send low part of character to LCD. 
*              This function works only in 4 bit data mode.
******************************************************************************/
{
  INT8U temp;
	  
  temp = GPIO_PORTC_DATA_R & 0x0F;
  GPIO_PORTC_DATA_R  = temp | ((Ch & 0x0F) << 4);
  GPIO_PORTD_DATA_R |= 0x04;        // Select data mode
  GPIO_PORTD_DATA_R |= 0x08;		// Set E High
  GPIO_PORTD_DATA_R &= 0xF7;		// Set E Low
}

void out_LCD_high( INT8U Ch )
/*****************************************************************************
*   Input    : Mask
*   Output   : -
*   Function : Send high part of character to LCD. 
*              This function works only in 4 bit data mode.
******************************************************************************/
{
  out_LCD_low((Ch & 0xF0) >> 4);
}

void wr_ctrl_LCD( INT8U Ch )
/*****************************************************************************
*   Input    : -
*   Output   : -
*   Function : Write control data to LCD.
******************************************************************************/
{
  static INT8U Mode4bit = FALSE;
  INT16U i;

  wr_ctrl_LCD_high( Ch );
  if( Mode4bit )
  {
	for(i=0; i<1000; i++);
	wr_ctrl_LCD_low( Ch );
  }
  else
  {
	if( (Ch & 0x30) == 0x20 )
	  Mode4bit = TRUE;
  }
}

void clr_LCD()
/*****************************************************************************
*   Input    : -
*   Output   : -
*   Function : Clear LCD.
******************************************************************************/
{
  
  if( xSemaphoreTake( lcd_write_mutex, ( TickType_t ) 10 ) == pdTRUE ){
   // wait 500 ms.
  wr_ctrl_LCD( 0x01 );
  
  xSemaphoreGive( lcd_write_mutex );
  }
    // 4: Give back mutex
        
        
}


void home_LCD()
/*****************************************************************************
*   Input    : -
*   Output   : -
*   Function : Return cursor to the home position.
******************************************************************************/
{
  wr_ctrl_LCD( 0x02 );
}

void Set_cursor( INT8U Ch )
/*****************************************************************************
*   Input    : New Cursor position
*   Output   : -
*   Function : Place cursor at given position.
******************************************************************************/
{
  wr_ctrl_LCD( Ch );
}


void out_LCD( INT8U Ch )
/*****************************************************************************
*   Input    : -
*   Output   : -
*   Function : Write control data to LCD.
******************************************************************************/
{
  INT16U i;

  out_LCD_high( Ch );
  for(i=0; i<1000; i++);
  out_LCD_low( Ch );
}




extern void lcd_task(void *pvParameters )

/*****************************************************************************
*   Input    :
*   Output   :
*   Function :
******************************************************************************/
{
  INT8U ch;
  
  
  while(1){

  
  switch( LCD_state )
  {
      case LCD_POWER_UP:
        LCD_init = 0;
        LCD_state = LCD_INIT;
        q_lcd = xQueueCreate(128, sizeof(INT8U));
        break;

      case LCD_INIT:
        if( LCD_init_sequense[LCD_init] != 0xFF )
          wr_ctrl_LCD( LCD_init_sequense[LCD_init++] );
        else
        { 
          LCD_state = LCD_READY;
        }
      break;

      case LCD_READY:
        if( uxQueueMessagesWaiting(q_lcd)){
        if( xSemaphoreTake( lcd_mutex, ( TickType_t ) 10 ) == pdTRUE ){
          if( xQueueReceive( q_lcd, &ch, 0 ) == pdTRUE  )
          {
            switch( ch )
            {
            case 0xFF:
              clr_LCD();
              break;
            case ESC:
              LCD_state = LCD_ESC_RECEIVED;
            break;
            default:
            out_LCD( ch );
            }
          }
        }
        xSemaphoreGive( lcd_mutex );
        }
      break;

    case LCD_ESC_RECEIVED:
    if( uxQueueMessagesWaiting(q_lcd)){
      if( xSemaphoreTake( lcd_mutex, ( TickType_t ) 10 ) == pdTRUE ){
        if( xQueueReceive( q_lcd, &ch, 0 ) == pdTRUE  )
        {
        if( ch & 0x80 )
        {
          Set_cursor( ch );
        }
        else
        {
          switch( ch )
          {
            case '@':
              home_LCD();
            break;
          }
        }
        LCD_state = LCD_READY;
        }
      }
      xSemaphoreGive( lcd_mutex );
    }
    break;
      
    
    
    }
  }
}



void lcd_write(INT8U* string, INT8U x, INT8U y){
  
  if( xSemaphoreTake( lcd_write_mutex, ( TickType_t ) 10 ) == pdTRUE ){
    
    move_LCD(x,y);
    wr_str_LCD(string);
    xSemaphoreGive( lcd_write_mutex );
    vTaskDelay(5 / portTICK_RATE_MS);
  }



 
}




/****************************** End Of Module *******************************/




