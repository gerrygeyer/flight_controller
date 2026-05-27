/*
 * encoder.h
 *
 *  Created on: Sep 13, 2025
 *      Author: gerrygeyer
 */

#ifndef INC_ENCODER_H_
#define INC_ENCODER_H_

#include <stdio.h>
#include <stdbool.h>
#include <main.h>
#include <parameter.h>
#include <settings.h>
#include <sys_math.h>

void set_encoder_to_zero(void);
int16_t read_encoder_value(void);
int16_t read_encoder_rotational_speed(const int16_t frequency);
void set_encoder_to_value(int16_t value);

#endif /* INC_ENCODER_H_ */
