#include <stdint.h>
#include "tm4c123gh6pm.h"
#include "emp_type.h"
#include "controller.h"
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

QueueHandle_t q_pan_con;
QueueHandle_t q_tilt_con;

// QueueHandle_t q_pan_angle;
// QueueHandle_t q_tilt_angle;

QueueHandle_t q_pan_SPI;
QueueHandle_t q_tilt_SPI;

QueueHandle_t q_model_tilt_enc;
QueueHandle_t q_model_pan_enc;

/******************************  SEMAPHORE ************************************/
SemaphoreHandle_t mutex_model_pan_enc;
SemaphoreHandle_t mutex_model_tilt_enc;

SemaphoreHandle_t mutex_pan_con;
SemaphoreHandle_t mutex_tilt_con;

SemaphoreHandle_t mutex_pan_SPI;
SemaphoreHandle_t mutex_tilt_SPI;

SemaphoreHandle_t mutex_uart_tx;
SemaphoreHandle_t mutex_uart_rx;
/******************************  VARIABLES ************************************/
INT16S angle;
INT8U adc_value = 0;
INT8U scalefactor = 1;
INT8U offset = 0;

/* ******************************** FUNCTIONS ******************************* */

static void setupHardware(void)
/*****************************************************************************
 *   Input    :  -
 *   Output   :  -
 *   Function :
 *****************************************************************************/
{
    // Warning: If you do not initialize the hardware clock, the timings will be inaccurate
    init_systick();
    init_gpio();
    uart0_init(115200, UART_BITS, 1, 'n');
    init_files();
    SPI_init();
    enc_pan_init();
    enc_tilt_init();

    /* -------------------------------------------------------------------------- */
    q_pan_con = xQueueCreate(1, sizeof(INT16S));
    q_tilt_con = xQueueCreate(1, sizeof(INT16S));

    q_pan_SPI = xQueueCreate(1, sizeof(INT16U));
    q_tilt_SPI = xQueueCreate(1, sizeof(INT16U));

    q_model_tilt_enc = xQueueCreate(1, sizeof(INT16S));
    q_model_pan_enc = xQueueCreate(1, sizeof(INT16S));

    /* -------------------------------------------------------------------------- */
    mutex_model_pan_enc = xSemaphoreCreateMutex();
    mutex_model_tilt_enc = xSemaphoreCreateMutex();
    mutex_pan_con = xSemaphoreCreateMutex();
    mutex_tilt_con = xSemaphoreCreateMutex();
    mutex_pan_SPI = xSemaphoreCreateMutex();
    mutex_tilt_SPI = xSemaphoreCreateMutex();
}

int main(void)
{
    setupHardware();

    // xTaskCreate(angle_task, "angle_task", USERTASK_STACK_SIZE, NULL, LOW_PRIO, NULL);
    xTaskCreate(SPI_task, "SPI_task", USERTASK_STACK_SIZE, NULL, HIGH_PRIO, NULL);
    xTaskCreate(status_led_task, "alive_task", USERTASK_STACK_SIZE, NULL, LOW_PRIO, NULL);
    xTaskCreate(tilt_contr_task, "tilt_contr_task", USERTASK_STACK_SIZE, NULL, MED_PRIO, NULL);
    xTaskCreate(pan_contr_task, "pan_contr_task", USERTASK_STACK_SIZE, NULL, MED_PRIO, NULL);
    //   xTaskCreate(uart_rx_task, "uart_rx_task", USERTASK_STACK_SIZE, NULL, LOW_PRIO, NULL);
    xTaskCreate(uart_tx_task, "uart_tx_task", USERTASK_STACK_SIZE, NULL, MED_PRIO, NULL);
    vTaskStartScheduler();
    return 0;
}
