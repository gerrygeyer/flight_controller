/*
 * encoder.c
 *
 *  Created on: Sep 13, 2025
 *      Author: gerrygeyer
 */
#include <encoder.h>

extern TIM_HandleTypeDef htim4;

int32_t encoder_angle_Q15;

void set_encoder_to_zero(void){
	uint32_t counter_value;
	counter_value = 0;
	__HAL_TIM_SET_COUNTER(&htim4,counter_value);
}

void set_encoder_to_value(int16_t value){
	uint32_t counter_value;
	counter_value = (uint16_t)(TEST_STATION_ENCODER_SIGN * (value/16));
	__HAL_TIM_SET_COUNTER(&htim4,counter_value);
}

int16_t read_encoder_value(void){
//	 return (int16_t)(((uint32_t)__HAL_TIM_GET_COUNTER(&htim4) * (uint32_t)(1<<16)) >> 12);
	return (TEST_STATION_ENCODER_SIGN * (int16_t)((uint32_t)__HAL_TIM_GET_COUNTER(&htim4) << 4));
}

int16_t read_encoder_rotational_speed(const int16_t frequency){
	const int16_t max_degree = 2000;
	int16_t rot_speed;
	static int16_t old_encoder_value = 0;
	int16_t encoder_count_value = ((int32_t)__HAL_TIM_GET_COUNTER(&htim4) << 4);

	rot_speed = CLAMP_INT32_TO_INT16((((int32_t)encoder_count_value - (int32_t)old_encoder_value) * frequency * 180)/max_degree);

	old_encoder_value = encoder_count_value;

	return rot_speed;
}
