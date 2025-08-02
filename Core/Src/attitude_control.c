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

#include <string.h>

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








/**
 * @brief       Computes the discrete time derivative of a Q15 vector (e.g., quaternion or sensor signal).
 *
 * @details     Approximates the derivative using finite differences:
 *              \f$ \dot{x} = (x_{\text{new}} - x_{\text{old}}) \cdot f_s \f$,
 *              where `f_s` is the sampling frequency.
 *              The input and output vectors are assumed to be in Q15 format.
 *
 * @param[in]   val         Current value vector (int16_t[4]).
 * @param[in]   val_old     Previous value vector (int16_t[4]).
 * @param[in]   frequency   Sampling frequency in Hz (Q0 format).
 * @param[out]  diff_out    Output differential (int16_t[4], Q15).
 *
 * @note
 * - This function uses integer arithmetic; overflow is clamped to int16_t range.
 * - Suitable for fixed-point implementations such as sensor fusion or quaternion rates.
 *
 * @see         CLAMP_INT32_TO_INT16
 */
static void differential_q15(const int16_t *val, const int16_t *val_old,const int16_t frequency, int16_t *diff_out){
	int32_t x;
	x = ((int32_t)val[0] - val_old[0]) * frequency;
	diff_out[0] = CLAMP_INT32_TO_INT16(x);
	x = ((int32_t)val[1] - val_old[1]) * frequency;
	diff_out[1] = CLAMP_INT32_TO_INT16(x);
	x = ((int32_t)val[2] - val_old[2]) * frequency;
	diff_out[2] = CLAMP_INT32_TO_INT16(x);
	x = ((int32_t)val[3] - val_old[3]) * frequency;
	diff_out[3] = CLAMP_INT32_TO_INT16(x);
}

void attitude_control_quaternion_lqr_q15(int16_t *q){

int16_t q_inv[4], q_err[4], q_err_inv[4], ln_q[3], w_err[4], diff_err[4], x_err[6], u_lqr[3];
static int16_t q_err_last_value[4];

memcpy(q_inv, q, sizeof(q));
// generate linearized quaternion error: theta_
q_t_conj_function(q_inv);
multiplicateQuaternionQ15(q_inv, q, q_err);
ln_q15_unit_quaternions_multiplicate_2(q_err, ln_q);

// calculate angular velocity error: w_err
memcpy(q_err_inv, q_err, sizeof(q_err));
differential_q15(q_err, q_err_last_value, ATTITUDE_FREQUENCY, diff_err);
multiplicateQuaternionQ15(q_err_inv,diff_err,w_err);

x_err[0] = ln_q[0];
x_err[1] = ln_q[1];
x_err[2] = ln_q[2];
x_err[3] = w_err[1];
x_err[4] = w_err[2];
x_err[5] = w_err[3];

lqr_q15(x_err,u_lqr);


memcpy(q_err_last_value, q_err, sizeof(q_err));
}



const int16_t K_q10[3][6] = {
    {25679,     0,     0, 16537,     0,     0},
    {    0, 25679,     0,     0, 16537,     0},
    {    0,     0, 25679,     0,     0, 16537}
};

//const int16_t J = [0.012273 0         0;
//0         0.012526 0;
//0         0         0.020953];


void lqr_q15(const int16_t *x_error,int16_t *u_out){
	int32_t x, sum;

	for (int i = 0; i < 3; i++) {
		sum = 0;

		x = (int32_t)K_q10[i][0] * x_error[0];
		sum += (x >> 2); // Q28

		x = (int32_t)K_q10[i][1] * x_error[1];
		sum += (x >> 2); // Q28

		x = (int32_t)K_q10[i][2] * x_error[2];
		sum += (x >> 2); // Q28

		x = (int32_t)K_q10[i][3] * x_error[3];
		sum += (x >> 2); // Q28

		x = (int32_t)K_q10[i][4] * x_error[4];
		sum += (x >> 2); // Q28

		x = (int32_t)K_q10[i][5] * x_error[5];
		sum += (x >> 2); // Q28

		sum = ((sum + (1 << 14)) >> 13); // back to Q15
		u_out[i] = CLAMP_INT32_TO_INT16(sum);
	}
}



//void feedback_linearisation(const int16_t *x_err, const int16_t rpm, ){
//
//}
