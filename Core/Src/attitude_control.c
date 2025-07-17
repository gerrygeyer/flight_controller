/*
 * attitude_control.c
 *
 *  Created on: Nov 2, 2024
 *      Author: Gerry Geyer
 */

#include <attitude_control.h>
#include <control_functions.h>
#include <orientation.h>
#include <parameter.h>
#include <sys_math.h>
#include <settings.h>
#include <task.h>

float inv_A[4][4];
at_angl_f debug_angle;
void init_attitude_control(void){
	float A[4][4];
	generate_matrix_A(A, DRONE_PARAM_K_E6, DRONE_PARAM_KL_E6, DRONE_PARAM_B_E6);
	inverse_matrix_4x4_f(A,inv_A);
}


void run_attitude_control(motor_t *pHandle_motor, at_control_f *pHandle_control){
	at_angl_f pitch_roll_yaw, p_q_r;
	control_output_f u_control;
	float u[4][1], w_2[4][1];

	get_orientation(&pitch_roll_yaw);
	get_angular_rate(&p_q_r);
	pitch_roll_yaw = degree_to_rad(pitch_roll_yaw);

	debug_angle = pitch_roll_yaw;

	switch (ATTITUDE_CONTROL){

		case PID:
			u_control = attitude_PID_control(&pitch_roll_yaw, pHandle_control);
			generate_u_vector(u_control,u);
			break;
		case LQR:
			attitude_LQR_control(u, &pitch_roll_yaw,&p_q_r, pHandle_control);
			break;
		default:
			system_stop_function();
			break;
	}

	multiply_4x4_with_vector(inv_A, u, w_2);
	generate_RPM_commands(w_2, pHandle_motor);
}

/*
 * probleme bei genauigkeit k * l
 */
void generate_matrix_A(float A[4][4], float k, float kl, float b) {
    A[0][0] = k;    A[0][1] = k;    A[0][2] = k;    A[0][3] = k;
    A[1][0] = -kl; A[1][1] = kl;  A[1][2] = kl;  A[1][3] = -kl;
    A[2][0] = kl;  A[2][1] = kl;  A[2][2] = -kl; A[2][3] = -kl;
    A[3][0] = b;    A[3][1] = -b;   A[3][2] = b;    A[3][3] = -b;
}


void generate_RPM_commands(float w[4][1], motor_t *pHandle_motor){

	int32_t x;

	x = (int32_t)(w[0][0] * (float)E6 * RAD_TO_RPM * RAD_TO_RPM);
 	x = (x<0)? -sqrt(-x): sqrt(x);
	x = (x < -MAX_SPEED_MOTOR_RPM)? -MAX_SPEED_MOTOR_RPM:x;
	x = (x > MAX_SPEED_MOTOR_RPM)? MAX_SPEED_MOTOR_RPM:x;
	pHandle_motor->m_1 = x;

	x = (int32_t)(w[1][0] * (float)E6) * RAD_TO_RPM * RAD_TO_RPM;
	x = (x<0)? -sqrt(-x): sqrt(x);
	x = (x < -MAX_SPEED_MOTOR_RPM)? -MAX_SPEED_MOTOR_RPM:x;
	x = (x > MAX_SPEED_MOTOR_RPM)? MAX_SPEED_MOTOR_RPM:x;
	pHandle_motor->m_2 = -x;

	x = (int32_t)(w[2][0] * (float)E6 * RAD_TO_RPM * RAD_TO_RPM);
	x = (x<0)? -sqrt(-x): sqrt(x);
	x = (x < -MAX_SPEED_MOTOR_RPM)? -MAX_SPEED_MOTOR_RPM:x;
	x = (x > MAX_SPEED_MOTOR_RPM)? MAX_SPEED_MOTOR_RPM:x;
	pHandle_motor->m_3 = x;

	x = (int32_t)(w[3][0] * (float)E6 * RAD_TO_RPM * RAD_TO_RPM);
	x = (x<0)? -sqrt(-x): sqrt(x);
	x = (x < -MAX_SPEED_MOTOR_RPM)? -MAX_SPEED_MOTOR_RPM:x;
	x = (x > MAX_SPEED_MOTOR_RPM)? MAX_SPEED_MOTOR_RPM:x;
	pHandle_motor->m_4 = -x;
}

void generate_u_vector(control_output_f in, float u_out[4][1]){
	u_out[0][0] = in.u1;
	u_out[1][0] = in.u2;
	u_out[2][0] = in.u3;
	u_out[3][0] = in.u4;

}


void attitude_control_quaternion_lqr_q15(void){




}
