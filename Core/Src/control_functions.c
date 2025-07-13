/*
 * control_function.c
 *
 *  Created on: Nov 21, 2024
 *      Author: Gerry Geyer
 */

#include <control_functions.h>
#include <parameter.h>

// PID
PID_f c_pitch, c_roll, c_yaw, c_T;

float z_axis;
// LQR
float Kr, Kr_yaw;
LQR_f lqr_x;

at_angl_f e_int;
uint8_t yaw_count;

float u2_debug, u3_debug, u4_debug;
float debug_pitch;


float K_gain[3][6] = {
    {3.162277660168383, -0.000000000000002, 0.000000000000000, 0.357241189459577, -0.000000000000000, 0.000000000000000},
    {0.000000000000001, 3.162277660168377, 0.000000000000000, 0.000000000000000, 0.359473754177601, 0.000000000000000},
    {-0.000000000000001, -0.000000000000001, 0.300000000000000, -0.000000000000000, 0.000000000000000, 0.112168623063671}
};

void init_control_functions(void){

	c_pitch.kp = 3.0;
	c_pitch.ki = 1.6;
	c_pitch.kd = 0.8;

	c_roll.kp = 3.0;
	c_roll.ki = 1.6;
	c_roll.kd = 0.8;

	c_yaw.kp = 0.45;
	c_yaw.ki = 0.7;
	c_yaw.kd = 0.006;

	z_axis = 6.0;

	c_pitch.I_mem = 0.0;
	c_pitch.control_frequency = SYSTEM_FREQUENCY;

	c_roll.I_mem = 0.0;
	c_roll.control_frequency = SYSTEM_FREQUENCY;

	c_yaw.I_mem = 0.0;
	c_yaw.control_frequency = SYSTEM_FREQUENCY;

	// LQR

	Kr = 4.3f;
	Kr_yaw = 0.2f;
	lqr_x.int_e_pitch = 0.0;
	lqr_x.int_e_roll = 0.0;
	lqr_x.int_e_yaw = 0.0;
	lqr_x.p = 0.0;
	lqr_x.pitch = 0.0;
	lqr_x.q = 0.0;
	lqr_x.r = 0.0;
	lqr_x.roll = 0.0;
	lqr_x.yaw = 0.0;

	e_int.pitch = 0.0;
	e_int.roll = 0.0;
	e_int.yaw = 0.0;


	//debug_pitch = 0.0f; 

}

void clear_control_functions(void){

	//for PID
	c_pitch.I_mem = 0.0;
	c_roll.I_mem = 0.0;
	c_yaw.I_mem = 0.0;

	//for LQR
	lqr_x.int_e_pitch = 0.0;
	lqr_x.int_e_roll = 0.0;
	lqr_x.int_e_yaw = 0.0;
	lqr_x.p = 0.0;
	lqr_x.pitch = 0.0;
	lqr_x.q = 0.0;
	lqr_x.r = 0.0;
	lqr_x.roll = 0.0;
	lqr_x.yaw = 0.0;

	e_int.pitch = 0.0;
	e_int.roll = 0.0;
	e_int.yaw = 0.0;


	//debug_pitch = 0;


}
control_output_f attitude_PID_control(at_angl_f *pHandle_angle, at_control_f *pHandle_control){
	control_output_f Output;
	float P,D;

	c_pitch.error = (pHandle_control->pitch * DEGREE_TO_RAD) - pHandle_angle->pitch;
	c_roll.error = (pHandle_control->roll * DEGREE_TO_RAD) - pHandle_angle->roll;
	c_yaw.error = (pHandle_control->yaw * DEGREE_TO_RAD) - pHandle_angle->yaw;

	Output.u1 = z_axis;

	P = c_pitch.error * c_pitch.kp;
	c_pitch.I_mem += c_pitch.error * c_pitch.ki / c_pitch.control_frequency;
	D = c_pitch.kd * (c_pitch.error - c_pitch.D_last) * c_pitch.control_frequency;
	c_pitch.D_last = c_pitch.error;

	Output.u2 = P + c_pitch.I_mem + D;

	P = c_roll.error * c_roll.kp;
	c_roll.I_mem += c_roll.error * c_roll.ki / c_roll.control_frequency;
	D = c_roll.kd * (c_roll.error - c_roll.D_last) * c_roll.control_frequency;
	c_roll.D_last = c_roll.error;

	Output.u3 = P + c_roll.I_mem + D;

	P = c_yaw.error * c_yaw.kp;
	c_yaw.I_mem += c_yaw.error * c_yaw.ki / c_yaw.control_frequency;
	D = c_yaw.kd * (c_yaw.error - c_yaw.D_last) * c_yaw.control_frequency;
	c_yaw.D_last = c_yaw.error;

	Output.u4 = P + c_yaw.I_mem + D;

	return (Output);
}


void attitude_LQR_control(float u[4][1], at_angl_f *pHandle_angle, at_angl_f *pHandle_angular_rate,at_control_f *pHandle_control){

	float x[6][1];
	float e[3][1];
	float u_lqr[3][1];

	get_x_for_LQR(x,e, pHandle_angle,pHandle_angular_rate, pHandle_control);
	multiply_x_K_LQR(u_lqr, x);
	generate_LQR_output(u, u_lqr,e);


	lqr_x.pitch = x[0][0];
		lqr_x.roll = x[1][0];
		lqr_x.yaw = x[2][0];
		lqr_x.p = x[3][0];
		lqr_x.q = x[4][0];
		lqr_x.r = x[5][0];
		lqr_x.int_e_pitch = e[0][0];
		lqr_x.int_e_roll = e[1][0];
		lqr_x.int_e_yaw = e[2][0];

}

void get_x_for_LQR(float x_in[6][1],float e_in[3][1],at_angl_f *pHandle_angle,at_angl_f *pHandle_angular_rate, at_control_f *pHandle_control){
	x_in[0][0] = pHandle_angle->pitch;
	x_in[1][0] = pHandle_angle->roll;
	x_in[2][0] = pHandle_angle->yaw;

	x_in[3][0] = pHandle_angular_rate->pitch * DEGREE_TO_RAD;
	x_in[4][0] = pHandle_angular_rate->roll * DEGREE_TO_RAD;
	x_in[5][0] = pHandle_angular_rate->yaw * DEGREE_TO_RAD;

	debug_pitch += x_in[3][0]*SYSTEM_TS;



	e_int.pitch += ((pHandle_control->pitch * DEGREE_TO_RAD) - pHandle_angle->pitch) * SYSTEM_TS;
	e_int.roll += ((pHandle_control->roll * DEGREE_TO_RAD) - pHandle_angle->roll) * SYSTEM_TS;
	e_int.yaw += ((pHandle_control->yaw * DEGREE_TO_RAD)- pHandle_angle->yaw) * SYSTEM_TS;

	e_in[0][0] = e_int.pitch * Kr;
	e_in[1][0] = e_int.roll * Kr;
	e_in[2][0] = e_int.yaw * Kr_yaw;

//	e_in[0][0] = ((pHandle_control->pitch *DEGREE_TO_RAD - pHandle_angle->pitch))* Kr;
//	e_in[1][0] = ((pHandle_control->roll *DEGREE_TO_RAD - pHandle_angle->roll))* Kr;
//	e_in[2][0] = ((pHandle_control->yaw *DEGREE_TO_RAD - pHandle_angle->yaw))* Kr;
}

void multiply_x_K_LQR(float u[3][1], float x[6][1]) {
    // calculation w = -K * x
    for (int i = 0; i < 3; i++) {
        u[i][0] = 0.0f;
        for (int j = 0; j < 6; j++) {
            u[i][0] -= K_gain[i][j] * x[j][0];
        }
    }
}

void generate_LQR_output(float u_out[4][1],float u_in[3][1], float e_in[3][1]){

	u_out[0][0] = 6; // const out
	u_out[1][0] = u_in[0][0] + e_in[0][0];
	u_out[2][0] = u_in[1][0] + e_in[1][0];
	u_out[3][0] = u_in[2][0] + e_in[2][0];

	u2_debug = u_out[1][0];
	u3_debug = u_out[2][0];
	u4_debug = u_out[3][0];

}


