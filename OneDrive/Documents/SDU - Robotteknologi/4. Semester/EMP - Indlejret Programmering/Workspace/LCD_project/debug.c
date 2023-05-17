///*****************************************************************************
//* University of Southern Denmark
//* Embedded C Programming (ECP)
//*
//* MODULENAME.: DEBUG.c
//*
//* PROJECT....: ECP
//*
//* DESCRIPTION:
//* functions and setup for SPI communication
//*
//* Change Log:
//******************************************************************************
//* Date    Id    Change
//* YYMMDD
//* --------------------
//* 050128  KA    Module created.
//*
//*****************************************************************************/
//
///***************************** Include files *******************************/
// #include <stdint.h>
// #include "tm4c123gh6pm.h"
// #include "emp_type.h"
// #include "debug.h"
// #include "uart0.h"
// #include "file.h"
// #include "string.h"
// #include "tmodel.h"
///*****************************    Defines    *******************************/
//
///********************** External declaration of Variables ******************/
// extern q_uart_tx;
//
///*****************************   Constants   *******************************/
//
///*************************  Function interfaces ****************************/
// void print_debug(tcb task){
// gfprintf(COM1, "TASK: %d, CONDITION: %s, SEM: %d, TIM: %d, STATE: %d\r\n", task.name, task.condition, task.sem, task.timer, task.state);
// }
//
//
// void debug_task(INT8U my_id, INT8U my_state, INT8U event, INT8U data){
//     int j;
//     if(queue_empty(q_uart_tx)){
//     for(j = 0; j<MAX_TASKS;j++){
//     print_debug(pot[j]);
//     }
//     }
//
// }
//
///****************************** End Of Module *******************************/
//