/*
 * control_functions.h
 *
 *  Created on: Nov 21, 2024
 *      Author: Gerry Geyer
 */

#ifndef INC_CONTROL_FUNCTIONS_H_
#define INC_CONTROL_FUNCTIONS_H_

#include <parameter.h>
#include <sys_math.h>

void init_control_functions(void);
void clear_control_functions(void);
control_output_f attitude_PID_control(at_angl_f *pHandle_angle, at_control_f *pHandle_control);
void attitude_LQR_control(float u[4][1], at_angl_f *pHandle_angle, at_angl_f *pHandle_angular_rate,at_control_f *pHandle_control);
void get_x_for_LQR(float x_in[6][1],float e_in[3][1],at_angl_f *pHandle_angle,at_angl_f *pHandle_angular_rate, at_control_f *pHandle_control);
void multiply_x_K_LQR(float u[3][1], float x[6][1]);
void generate_LQR_output(float u_out[4][1],float u_in[3][1], float e_in[3][1]);
#endif /* INC_CONTROL_FUNCTIONS_H_ */
