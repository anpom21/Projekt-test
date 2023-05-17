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

/***************************** Include files *******************************/
#ifndef _DEBUG_H
#define _DEBUG_H
#include "emp_type.h"

/*****************************    Defines    *******************************/

/********************** External declaration of Variables ******************/

/*****************************   Constants   *******************************/

/*************************  Function interfaces ****************************/

void debug_task(INT8U my_id, INT8U my_state, INT8U event, INT8U data);

#endif
/****************************** End Of Module *******************************/
