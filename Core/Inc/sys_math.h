/*
 * sys_math.h
 *
 *  Created on: Nov 19, 2024
 *      Author: Gerry
 */

#ifndef INC_SYS_MATH_H_
#define INC_SYS_MATH_H_

#include <parameter.h>
#include <math.h>

#define CLAMP_INT32_TO_INT16(x) ((x) > INT16_MAX ? INT16_MAX : ((x) < INT16_MIN ? INT16_MIN : (int16_t)(x)))

#define INT16_HALF_VALUE	0x3FFF
#define INT16_MAX_VALUE		0x7FFF
#define Q15_ONE				0x7FFF
#define PI					(float)3.14159265358979
#define PI_MULTIPLY_2		(float)6.283185307179586
#define RAD_TO_RPM			(float)9.549296585513721
#define RPM_TO_RAD			(float)(1.0f/RAD_TO_RPM)
#define RAD_TO_RPM_SQRT		(float)3.090193616185517
#define DEGREE_TO_RAD		(float)0.017453292519943
#define RAD_TO_DEGREE		(float)57.29577951308232

int16_t sin_i(int16_t y);
int16_t cos_i(int16_t y);

void multiply_matrix_with_scalar(float scalar, float in_matrix[4][4], float out_matrix[4][4]);
void inverse_matrix_3x3_f(float in_matrix[3][3], float out_matrix[3][3]);
void inverse_matrix_4x4_f(float in_matrix[4][4], float out_matrix[4][4]);
void multiply_4x4_with_vector(float in_matrix[4][4], float in_vector[4][1], float out_vector[4][1]);
at_angl_f degree_to_rad(at_angl_f input);

void start_time_measurement(void);
uint32_t stopp_time_measurement(void);

// Functions for sensor fusion
uint32_t sqrt_fast_uint(uint32_t n);
void q_t_conj_function_in_out_q15(int16_t *q_in, int16_t *q_out);
void norm_3d_vector(int16_t *input, int16_t *norm_out);
void Normalize4DvectorQ15(int16_t *in, int16_t *out);
void NormalizeQuaternionQ15(const int16_t *q, int16_t *q_out);
void q_t_conj_function(int16_t *q);
void multiplicateQuaternionQ15(const int16_t *q1, const int16_t *q2, int16_t *q_out);
void quat_to_euler_q15(const int16_t q[4], int16_t euler[3]);
#endif /* INC_SYS_MATH_H_ */
