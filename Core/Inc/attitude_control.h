/*
 * attitude_control.h
 *
 *  Created on: Nov 2, 2024
 *      Author: Gerry Geyer
 */

#ifndef INC_ATTITUDE_CONTROL_H_
#define INC_ATTITUDE_CONTROL_H_
#include <parameter.h>

void init_attitude_control(void);
void run_attitude_control(motor_t *pHandle_motor, at_control_f *pHandle_control);
void generate_matrix_A(float A[4][4], float k, float l, float b);
void generate_u_vector(control_output_f in, float u_out[4][1]);
void generate_RPM_commands(float w[4][1], motor_t *pHandle_motor);


void attitude_control_quaternion_lqr_q15(const int16_t *q,const int16_t *q_ref, const int16_t *w_gyro_t, int16_t *tau);
void transform_u2_motorSpeed(const int16_t *u, motor_t *pHandle_motor);
void get_motor_speed_from_u(const int32_t *u, int16_t *w_rpm, motor_t *pHandle_motor);

void set_init_yaw_position(const int16_t *q, int16_t *q_yaw_corr, int16_t *q_axis_corr);
void correct_q_axis(int16_t *q, int16_t *w);
void attitude_control_quaternion_nonlinear_q15(const int16_t *q,const int16_t *q_ref, const int16_t *w_gyro_t, int16_t *tau);
void filter_SLERP_EMA_quaternion_Q15(const int16_t *q_in, int16_t *q_out);
//#define RAD_MAX_16		4
#define RAD_MAX_32		5
//#define RAD_MAX_64		6


// Werte aus MATLAB (bereits auf int16_t skaliert)
//const int16_t A_inv_Q21[4][4] = {
//    {   186,  -1163,   1163,  20581 },
//    {   186,   1163,   1163, -20581 },
//    {   186,   1163,  -1163,  20581 },
//    {   186,  -1163,  -1163, -20581 }
//};


#endif /* INC_ATTITUDE_CONTROL_H_ */
