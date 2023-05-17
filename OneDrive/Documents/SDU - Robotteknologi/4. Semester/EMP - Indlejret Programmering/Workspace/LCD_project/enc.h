/*
 * enc.h
 *
 *  Created on: 25. apr. 2023
 *      Author: jense
 */

#ifndef ENC_H_

void enc_pan_init();
/*****************************************************************************
 *   Input    : -
 *   Output   : -
 *   Function : Initialization of the encoder pins (PIN 5, 6, 7 of PORT A)
 ******************************************************************************/

void enc_tilt_init();
/*****************************************************************************
 *   Input    : -
 *   Output   : -
 *   Function : Initialization of the encoder pins (PIN 5, 6, 7 of PORT A)
 ******************************************************************************/

void angle_pan_task();
/*****************************************************************************
 *   Input    : -
 *   Output   : -
 *   Function : Task for reading the angle of the rotary encoder
 ******************************************************************************/

void angle_tilt_task();
/*****************************************************************************
 *   Input    : -
 *   Output   : -
 *   Function : Task for reading the angle of the rotary encoder
 ******************************************************************************/

#define ENC_H_

#endif /* ENC_H_ */
