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


/**
 * @brief       Multiplies two Q15 fixed-point numbers with rounding.
 *
 * @details     Performs a 16-bit × 16-bit multiplication with intermediate 32-bit precision,
 *              adds 0.5 (1 << 14) for rounding, and shifts the result back to Q15 format.
 *
 * @param       a       First Q15 operand.
 * @param       b       Second Q15 operand.
 *
 * @return      Rounded Q15 result of the multiplication.
 *
 * @note        This function is `static inline` for performance and can be used in tight control loops.
 *              Result is clamped implicitly by casting to int16_t.
 */
static inline int16_t q15_mul(int16_t a, int16_t b);


/**
 * @brief		Calculate u = -Kx in Q15 representation
 *
 * @details 	The K are pre-calculatet in MATLAB
 */
static void lqr_q15(const int16_t *x_error,int16_t *u_out);

/**
 * @brief       Computes the torque vector using feedback linearization in Q15 format.
 *
 * @details     This function implements feedback linearization for a rigid-body rotational system.
 *              The torque output @p tau is computed based on the control input @p u, the current
 *              angular velocity @p w, and the system's inertia parameters from @p pHandle.
 *
 *              It calculates:
 *              \f[
 *                  \tau = J \cdot u + \omega \times (J \cdot \omega)
 *              \f]
 *
 *              where:
 *              - \f$ J \f$ is the diagonal inertia matrix (with Jxx, Jyy, Jzz),
 *              - \f$ u \f$ is the control input (desired angular acceleration),
 *              - \f$ \omega \f$ is the current angular velocity,
 *              - \f$ \tau \f$ is the resulting control torque.
 *
 *              All computations are performed using Q15 fixed-point arithmetic.
 *              The cross product is calculated in Q15 using intermediate 32-bit values.
 *
 * @param[in]   u        Pointer to the control input vector (3x1, Q15 format).
 * @param[in]   w        Pointer to the angular velocity vector (3x1, Q15 format).
 * @param[in]   pHandle  Pointer to system parameters containing Jxx, Jyy, Jzz (Q15).
 * @param[out]  tau      Pointer to the resulting torque vector (3x1, Q15 format).
 *
 * @note        Assumes a diagonal inertia matrix. All inputs and outputs must be in Q15 format.
 *              Intermediate values use 32-bit precision to preserve accuracy.
 *
 * @warning     Inputs must be properly initialized.
 *
 * @see         crossproduct_3x3_Q15(), q15_mul()
 */
static void feedback_linearisation(const int16_t *u,const int16_t *w,system_parameter *pHandle, int16_t *tau);



system_parameter drone_parameter;


float inv_A[4][4];

// debg variable
at_angl_f debug_angle;
int16_t debug_x_err_1, debug_x_err_2,debug_x_err_3,debug_x_err_4,debug_x_err_5,debug_x_err_6;


void init_attitude_control(void){
	float A[4][4];
	generate_matrix_A(A, DRONE_PARAM_K_E6, DRONE_PARAM_KL_E6, DRONE_PARAM_B_E6);
	inverse_matrix_4x4_f(A,inv_A);


	drone_parameter.Jxx = 402; //0.012273 * Q15
	drone_parameter.Jyy = 410; //0.012526 * Q15
	drone_parameter.Jzz = 687; //0.020953 * Q15
}


void run_attitude_control(motor_t *pHandle_motor, at_control_f *pHandle_control){
	at_angl_f pitch_roll_yaw, p_q_r;
	control_output_f u_control;
	float u[4][1], w_2[4][1];

	get_orientation(&pitch_roll_yaw);
	get_angular_rate(&p_q_r);
	pitch_roll_yaw = degree_to_rad(pitch_roll_yaw);

	debug_angle = pitch_roll_yaw;

//	switch (ATTITUDE_CONTROL){
//
//		case PID:
//			u_control = attitude_PID_control(&pitch_roll_yaw, pHandle_control);
//			generate_u_vector(u_control,u);
//			break;
//		case LQR:
//			attitude_LQR_control(u, &pitch_roll_yaw,&p_q_r, pHandle_control);
//			break;
//		default:
//			system_stop_function();
//			break;
//	}

	multiply_4x4_with_vector(inv_A, u, w_2);
	generate_RPM_commands(w_2, pHandle_motor);
}

void transform_u2_motorSpeed(const int16_t *u, motor_t *pHandle_motor){
	float u_f[4][1], w_2[4][1];

	u_f[0][0] = 0.0f;
	u_f[1][0] = (float)u[0]/(float)Q15;
	u_f[2][0] = (float)u[1]/(float)Q15;
	u_f[3][0] = (float)u[2]/(float)Q15;
	multiply_4x4_with_vector(inv_A, u_f, w_2);
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



// ############

void generate_matrix_A_Q15(int16_t A[4][4], float k_f, float l, float k_b) {

	int16_t k, kl, b;

	k 	= CLAMP_INT32_TO_INT16((int32_t)(k_f * (float)Q15));
	b 	= CLAMP_INT32_TO_INT16((int32_t)(k_b * (float)Q15));
	kl 	=  CLAMP_INT32_TO_INT16((int32_t)(k_f * l *  (float)Q15));


    A[0][0] = k;    A[0][1] = k;    A[0][2] = k;    A[0][3] = k;
    A[1][0] = -kl; A[1][1] = kl;  A[1][2] = kl;  A[1][3] = -kl;
    A[2][0] = kl;  A[2][1] = kl;  A[2][2] = -kl; A[2][3] = -kl;
    A[3][0] = b;    A[3][1] = -b;   A[3][2] = b;    A[3][3] = -b;
}

void generate_RPM_commands_Q15(int16_t *w, motor_t *pHandle_motor){

	int32_t x;

	const int16_t rad_to_rpm_2_q7 = 11672; // 9.5493^2 * Q7

	x = (int32_t)(w[0] * rad_to_rpm_2_q7);
 	x = (x<0)? -sqrt(-x): sqrt(x);
	x = (x < -MAX_SPEED_MOTOR_RPM)? -MAX_SPEED_MOTOR_RPM:x;
	x = (x > MAX_SPEED_MOTOR_RPM)? MAX_SPEED_MOTOR_RPM:x;
	pHandle_motor->m_1 = x;

	x = (int32_t)(w[1] * (float)E6) * RAD_TO_RPM * RAD_TO_RPM;
	x = (x<0)? -sqrt(-x): sqrt(x);
	x = (x < -MAX_SPEED_MOTOR_RPM)? -MAX_SPEED_MOTOR_RPM:x;
	x = (x > MAX_SPEED_MOTOR_RPM)? MAX_SPEED_MOTOR_RPM:x;
	pHandle_motor->m_2 = -x;

	x = (int32_t)(w[2] * (float)E6 * RAD_TO_RPM * RAD_TO_RPM);
	x = (x<0)? -sqrt(-x): sqrt(x);
	x = (x < -MAX_SPEED_MOTOR_RPM)? -MAX_SPEED_MOTOR_RPM:x;
	x = (x > MAX_SPEED_MOTOR_RPM)? MAX_SPEED_MOTOR_RPM:x;
	pHandle_motor->m_3 = x;

	x = (int32_t)(w[3] * (float)E6 * RAD_TO_RPM * RAD_TO_RPM);
	x = (x<0)? -sqrt(-x): sqrt(x);
	x = (x < -MAX_SPEED_MOTOR_RPM)? -MAX_SPEED_MOTOR_RPM:x;
	x = (x > MAX_SPEED_MOTOR_RPM)? MAX_SPEED_MOTOR_RPM:x;
	pHandle_motor->m_4 = -x;
}



/**
 * @brief       Computes the discrete-time derivative of a Q15 vector (e.g., quaternion or sensor signal)
 *              with additional scaling.
 *
 * @details     Approximates the derivative using finite differences:
 *              \f[
 *                  \dot{x} = \frac{(x_{\text{new}} - x_{\text{old}}) \cdot f_s}{2^{\text{scaling\_shift}}}
 *              \f]
 *              where:
 *              - `f_s` is the sampling frequency in Hz (Q0 format)
 *              - `scaling_shift` is a right-shift amount applied after multiplication for scaling.
 *
 *              The input and output vectors are in Q15 format.
 *
 * @param[in]   val             Current value vector (int16_t[4], Q15 format).
 * @param[in]   val_old         Previous value vector (int16_t[4], Q15 format).
 * @param[in]   frequency       Sampling frequency in Hz (Q0 format).
 * @param[in]   scaling_shift   Right-shift applied to the intermediate 32-bit result
 *                              for scaling (equivalent to division by 2^scaling_shift).
 * @param[out]  diff_out        Output differential (int16_t[4], Q15 format).
 *
 * @note
 * - Uses integer arithmetic; overflow is clamped to int16_t range.
 * - The additional `scaling_shift` can be used to adjust for fixed-point format changes
 *   or to normalize the derivative magnitude.
 * - Suitable for fixed-point implementations such as sensor fusion or quaternion rate estimation.
 *
 * @see         CLAMP_INT32_TO_INT16
 */
static void differential_with_scaling_q15(const int16_t *val, const int16_t *val_old,const int16_t frequency, const int16_t scaling_shift, int16_t *diff_out){
	int32_t x;
	x = (((int32_t)val[0] - val_old[0]) * frequency) >> scaling_shift;
	diff_out[0] = CLAMP_INT32_TO_INT16(x);
	x = (((int32_t)val[1] - val_old[1]) * frequency) >> scaling_shift;
	diff_out[1] = CLAMP_INT32_TO_INT16(x);
	x = (((int32_t)val[2] - val_old[2]) * frequency) >> scaling_shift;
	diff_out[2] = CLAMP_INT32_TO_INT16(x);
	x = (((int32_t)val[3] - val_old[3]) * frequency) >> scaling_shift;
	diff_out[3] = CLAMP_INT32_TO_INT16(x);
}

void attitude_control_quaternion_lqr_q15(const int16_t *q,const int16_t *q_ref, int16_t *tau){

int16_t q_inv[4],q_ref_inv[4], q_err[4], q_err_inv[4], ln_q[3],w[4], w_err[4], diff_err[4], diff_q[4], x_err[6], u_lqr[3];
static int16_t q_err_last_value[4], q_last_value[4];

memcpy(q_ref_inv, q_ref, 4 * sizeof(int16_t));
// generate linearized quaternion error: theta_
q_t_conj_function(q_ref_inv);
multiplicateQuaternionQ15(q_ref_inv, q, q_err);
ln_q15_unit_quaternions_multiplicate_2(q_err, ln_q);

// calculate angular velocity error: w_err
memcpy(q_err_inv, q_err, sizeof(q_err));
q_t_conj_function(q_err_inv);
differential_with_scaling_q15(q_err, q_err_last_value,RAD_MAX_32, ATTITUDE_FREQUENCY, diff_err);
multiplicateQuaternionQ15(q_err_inv,diff_err,w_err);

x_err[0] = ln_q[0];
x_err[1] = ln_q[1];
x_err[2] = ln_q[2];
x_err[3] = w_err[1];
x_err[4] = w_err[2];
x_err[5] = w_err[3];

debug_x_err_1 = ln_q[0];
debug_x_err_2 = ln_q[1];
debug_x_err_3 = ln_q[2];
debug_x_err_4 = w_err[1];
debug_x_err_5 = w_err[2];
debug_x_err_6 = w_err[3];



lqr_q15(x_err,u_lqr);

// calculate angular velocity w for feedback linearisation
memcpy(q_inv, q, 4 * sizeof(int16_t));
q_t_conj_function(q_inv);
differential_with_scaling_q15(q, q_last_value,RAD_MAX_32,ATTITUDE_FREQUENCY, diff_q);
multiplicateQuaternionQ15(q_inv,diff_q,w);

// feedback linearisation
feedback_linearisation(u_lqr, w, &drone_parameter, tau);

memcpy(q_err_last_value, q_err, sizeof(q_err)); // Necessary for differential_q15();
memcpy(q_last_value, q, 4 * sizeof(int16_t)); // Necessary for differential_q15();
}



const int16_t K_q10[3][6] = {
    {25679,     0,     0, 16537,     0,     0},
    {    0, 25679,     0,     0, 16537,     0},
    {    0,     0, 25679,     0,     0, 16537}
};

//const int16_t J = [0.012273 0         0;
//0         0.012526 0;
//0         0         0.020953];


static void lqr_q15(const int16_t *x_error,int16_t *u_out){
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

		sum = ((sum + (1 << 12)) >> 13); // back to Q15
		u_out[i] = CLAMP_INT32_TO_INT16(sum);
	}
}



/**
 * @brief       Computes the cross product of two 3D vectors in Q15 fixed-point format.
 *
 * @details     This function calculates the cross product of two input vectors @p v1 and @p v2,
 *              both represented in Q15 fixed-point format. Intermediate results are calculated
 *              using 32-bit integers to prevent overflow and then scaled back to Q15 with rounding.
 *
 * @param[in]   v1     Pointer to the first vector (array of 3 Q15 values).
 * @param[in]   v2     Pointer to the second vector (array of 3 Q15 values).
 * @param[out]  cross  Pointer to the output vector (array of 3 Q15 values).
 *
 * @note        The function uses 32-bit intermediate arithmetic and rounding
 *              with `(1 << 13)` before shifting back to Q15.
 * @warning     Input values must be within the valid Q15 range ([-1, 1) scaled to int16_t).
 *
 * @see         dotproduct_3x3_Q15()
 */
static void crossproduct_3x3_Q15(const int16_t *v1, const int16_t *v2, int16_t *cross){
	int32_t x;

	x = (((int32_t)v1[1] * (int32_t)v2[2]) >> 1) - (((int32_t)v1[2] * (int32_t)v2[1]) >> 1);
	x = ((x + ( 1 << 13 )) >> 14); // back to Q15
	cross[0] = CLAMP_INT32_TO_INT16(x);

	x = (((int32_t)v1[2] * (int32_t)v2[0]) >> 1) - (((int32_t)v1[0] * (int32_t)v2[2]) >> 1);
	x = ((x + ( 1 << 13 )) >> 14); // back to Q15
	cross[1] = CLAMP_INT32_TO_INT16(x);

	x = (((int32_t)v1[0] * (int32_t)v2[1]) >> 1) - (((int32_t)v1[1] * (int32_t)v2[0]) >> 1);
	x = ((x + ( 1 << 13 )) >> 14); // back to Q15
	cross[2] = CLAMP_INT32_TO_INT16(x);

}


static void feedback_linearisation(const int16_t *u,const int16_t *w,system_parameter *pHandle, int16_t *tau){
	int16_t Jw[3], Ju[3], cross_product[3];

	Jw[0] = q15_mul(pHandle->Jxx, w[0]);
	Jw[1] = q15_mul(pHandle->Jyy, w[1]);
	Jw[2] = q15_mul(pHandle->Jzz, w[2]);

	Ju[0] = q15_mul(pHandle->Jxx, u[0]);
	Ju[1] = q15_mul(pHandle->Jyy, u[1]);
	Ju[2] = q15_mul(pHandle->Jzz, u[2]);

	crossproduct_3x3_Q15(w, Jw,cross_product);

	tau[0] = CLAMP_INT32_TO_INT16(((int32_t)Ju[0] + (int32_t)cross_product[0]));
	tau[1] = CLAMP_INT32_TO_INT16(((int32_t)Ju[1] + (int32_t)cross_product[1]));
	tau[2] = CLAMP_INT32_TO_INT16(((int32_t)Ju[2] + (int32_t)cross_product[2]));

}







static inline int16_t q15_mul(int16_t a, int16_t b) {
    return (int16_t)(((int32_t)a * b + (1 << 14)) >> 15); // mit Rundung
}
