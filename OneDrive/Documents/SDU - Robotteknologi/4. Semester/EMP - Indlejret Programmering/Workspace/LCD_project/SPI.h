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
#ifndef _SPI_H
#define _SPI_H
#include "emp_type.h"

/*****************************    Defines    *******************************/

/********************** External declaration of Variables ******************/

/*****************************   Constants   *******************************/

/*************************  Function interfaces ****************************/

void SPI_init();

void SPI_task(void *pvParameters);

void SPI_receive();

#endif
/****************************** End Of Module *******************************/
