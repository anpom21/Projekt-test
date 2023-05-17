#include <stdint.h>
#include "tm4c123gh6pm.h"
#include "emp_type.h"
#include "gpio.h"
#include "tmodel.h"
#include "SPI.h"
#include "uart0.h"
#include "debug.h"
#include "status_led.h"
#include "systick_frt.h"

#include "FreeRTOS.h"
#include "task.h"
#include "file.h"
#include "queue.h"
#include "semphr.h"
#include "enc.h"

#define USERTASK_STACK_SIZE configMINIMAL_STACK_SIZE
#define IDLE_PRIO 0
#define LOW_PRIO 1
#define MED_PRIO 2
#define HIGH_PRIO 3

#define UART_BITS 10

/******************************  QUEUES  **************************************/
QueueHandle_t q_uart_tx;
QueueHandle_t q_uart_rx;

// QueueHandle_t q_pan_angle;
// QueueHandle_t q_tilt_angle;
//
// QueueHandle_t q_pan_con;
// QueueHandle_t q_tilt_con;

/******************************  SEMAPHORE ************************************/
// SemaphoreHandle_t mutex_pan_angle;
// SemaphoreHandle_t mutex_tilt_angle;
SemaphoreHandle_t mutex_uart_tx;
SemaphoreHandle_t mutex_uart_rx;
/******************************  VARIABLES ************************************/
INT8U adc_value = 0;
INT8U scalefactor = 1;
INT8U offset = 0;

static void setupHardware(void)
/*****************************************************************************
 *   Input    :  -
 *   Output   :  -
 *   Function :
 *****************************************************************************/
{
    // TODO: Put hardware configuration and initialisation in here

    // Warning: If you do not initialize the hardware clock, the timings will be inaccurate
    SPI_init();
    // enc_pan_init();
    // enc_tilt_init();
    init_files();
    init_gpio();
    init_systick();
    uart0_init(9600, UART_BITS, 1, 'n');

    // queues
    // q_pan_con = xQueueCreate(128, sizeof(INT8U));
    // q_tilt_con = xQueueCreate(128, sizeof(INT8U));
}

int main(void)
{
    setupHardware();
    xTaskCreate(SPI_task, "SPI_task", USERTASK_STACK_SIZE, NULL, LOW_PRIO, NULL);
    //  xTaskCreate(angle_pan_task, "angle_pan_task", USERTASK_STACK_SIZE, NULL, LOW_PRIO, NULL);
    //  xTaskCreate(angle_tilt_task, "angle_tilt_task", USERTASK_STACK_SIZE, NULL, LOW_PRIO, NULL);
    xTaskCreate(status_led_task, "alive_task", USERTASK_STACK_SIZE, NULL, LOW_PRIO, NULL);
    // xTaskCreate(uart_rx_task, "uart_rx_task", USERTASK_STACK_SIZE, NULL, LOW_PRIO, NULL);
    xTaskCreate(uart_tx_task, "uart_tx_task", USERTASK_STACK_SIZE, NULL, LOW_PRIO, NULL);
    vTaskStartScheduler();
    return 0;
}
