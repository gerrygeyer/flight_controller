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
#include <sensor_fusion.h>

#include <string.h>
#include <stdlib.h>

float debug_u_f1, debug_u_f2, debug_u_f3,debug_u_f4;



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
//static inline int16_t q15_mul(int16_t a, int16_t b);


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

static void compute_quat_err_p2_control(const int16_t *q_m, const int16_t *q_d, int16_t *q_out);

static void rotate_plus_to_x(const int16_t *u_plus, int16_t *u_x);
static void rotate_plus_to_x_lqr(const int16_t *u_plus, int16_t *u_x);
static void iir_filter_tau_Q15(const int16_t *tau_raw, int16_t *tau_filter, const float fc, const float Ts);
static float iir_lp_hight_control(float input);

system_parameter drone_parameter;
volatile bool correction_ready;

P2_attitude_control gain;



float inv_A[4][4];

// debg variable
at_angl_f debug_angle;
int16_t debug_x_err_1, debug_x_err_2,debug_x_err_3,debug_x_err_4,debug_x_err_5,debug_x_err_6;
int16_t debug_diff_err1, debug_diff_err2, debug_diff_err3, debug_diff_err4;
wxyz_16t debug_q_err, debug_q_err_old;
int16_t debug_u_lqr1, debug_u_lqr2,debug_u_lqr3;
int16_t debug_gyro_x, debug_gyro_y,debug_gyro_z;


void init_attitude_control(void){
	float A[4][4];
	generate_matrix_A(A, DRONE_PARAM_K_E6, DRONE_PARAM_KL_E6, DRONE_PARAM_B_E6);
	inverse_matrix_4x4_f(A,inv_A);

	correction_ready = false;


	drone_parameter.Jxx = (int16_t)(0.012273f * (float)Q15);
	drone_parameter.Jyy = (int16_t)(0.012526f * (float)Q15);
	drone_parameter.Jzz = (int16_t)(0.020953f * (float)Q15);

//	drone_parameter.Jxx = 25738; //0.012273 * Q21
//	drone_parameter.Jyy = 26269; //0.012526 * Q21
//	drone_parameter.Jzz = 43942; //0.020953 * Q21

//	drone_parameter.Jz_y_div_x = drone_parameter.Jzz - drone_parameter.Jyy

//	gain.P1.pitch = 4.2f;
//	gain.P1.roll = 4.2f;
//	gain.P1.yaw = 0.15f;
//
//
//	gain.P2.pitch = 0.32;
//	gain.P2.roll = 0.32;
//	gain.P2.yaw = 0.008;

	gain.P1.pitch = 7.0f;//7.4f;
	gain.P1.roll = 7.0f;//11.5f;//7.4f;
	gain.P1.yaw = 0.25;//0.264f;


	gain.P2.pitch = 0.75f; //0.8;//0.95;
	gain.P2.roll = 0.80f; //0.8;//0.95;
	gain.P2.yaw = 0.08;//0.016;


}

float hight_value = 9.0f;
void run_attitude_control(motor_t *pHandle_motor, at_control_f *pHandle_control){
	at_angl_f pitch_roll_yaw, p_q_r;
	control_output_f u_control;
	float u[4][1], w_2[4][1];

	get_orientation(&pitch_roll_yaw);
	get_angular_rate(&p_q_r);
	pitch_roll_yaw = degree_to_rad(pitch_roll_yaw);

	debug_angle = pitch_roll_yaw;


	multiply_4x4_with_vector(inv_A, u, w_2);
	generate_RPM_commands(w_2, pHandle_motor);
}
float debug_force_up = 10.5f;
void transform_u2_motorSpeed(const int16_t *u, motor_t *pHandle_motor){
	float u_f[4][1], w_2[4][1];



//	u_f[0][0] = debug_force_up;// = 11.0f; // 12.5f;//(float)u[0]/(float)Q10;//10.5f;//(float)u[0]/(float)Q15;
	u_f[0][0] = (float)u[0]/(float)Q10;//10.5f;//
	u_f[1][0] = (float)u[1]/(float)Q10;
	u_f[2][0] = (float)u[2]/(float)Q10;
	u_f[3][0] = (float)u[3]/(float)Q10;

	if(u_f[0][0] < 11.0f) u_f[0][0] = 11.0f;
	if(u_f[0][0] > 20.0f) u_f[0][0] = 20.0f;

	if(POSITION_CONTROL == OFF) u_f[0][0] = hight_value;//10.0f; //12.0f;
//	u_f[0][0] = iir_lp_hight_control(u_f[0][0]);
//	u_f[0][0] = hight_value;//10.0f; //12.0f;
//	u_f[0][0] = 9.0f;
	debug_u_f1 = u_f[0][0];
	debug_u_f2 = u_f[1][0];
	debug_u_f3 = u_f[2][0];
	debug_u_f4 = u_f[3][0];

	multiply_4x4_with_vector(inv_A, u_f, w_2);
	generate_RPM_commands(w_2, pHandle_motor);

}

static float iir_lp_hight_control(float input){
	static float old_value = 0;
	const float a =  0.2391f;
	old_value = a* input + (1.0f -a)*old_value;
	return old_value;
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
//	pHandle_motor->m_1 = x;
	pHandle_motor->m_4 = x;

	x = (int32_t)(w[1][0] * (float)E6) * RAD_TO_RPM * RAD_TO_RPM;
	x = (x<0)? -sqrt(-x): sqrt(x);
	x = (x < -MAX_SPEED_MOTOR_RPM)? -MAX_SPEED_MOTOR_RPM:x;
	x = (x > MAX_SPEED_MOTOR_RPM)? MAX_SPEED_MOTOR_RPM:x;
//	pHandle_motor->m_2 = -x;
	pHandle_motor->m_3 = x;

	x = (int32_t)(w[2][0] * (float)E6 * RAD_TO_RPM * RAD_TO_RPM);
	x = (x<0)? -sqrt(-x): sqrt(x);
	x = (x < -MAX_SPEED_MOTOR_RPM)? -MAX_SPEED_MOTOR_RPM:x;
	x = (x > MAX_SPEED_MOTOR_RPM)? MAX_SPEED_MOTOR_RPM:x;
//	pHandle_motor->m_3 = x;
	pHandle_motor->m_2 = x;

	x = (int32_t)(w[3][0] * (float)E6 * RAD_TO_RPM * RAD_TO_RPM);
	x = (x<0)? -sqrt(-x): sqrt(x);
	x = (x < -MAX_SPEED_MOTOR_RPM)? -MAX_SPEED_MOTOR_RPM:x;
	x = (x > MAX_SPEED_MOTOR_RPM)? MAX_SPEED_MOTOR_RPM:x;
//	pHandle_motor->m_4 = -x;
	pHandle_motor->m_1 = x;
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

	x = (int32_t)(w[0] * (float)E6) * RAD_TO_RPM * RAD_TO_RPM;
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
 * @brief		Copy q_in in q_copy
 *
 * @details 	[Optional: Ausführlichere Beschreibung, ggf. Verhalten, Nebenwirkungen, Einschränkungen]
 *
 * @param  		param [Beschreibung des Eingabeparameters 1]
 *
 * @note     	[Optional: Zusatzhinweis, z. B. nicht thread-safe]
 * @warning		[Optional: z. B. darf nur nach Init aufgerufen werden]
 * @see			andere_funktion() [Optionaler Verweis]
 */
static inline void copy_q(const int16_t *q_in, int16_t *q_copy){
	q_copy[0] = q_in[0];
	q_copy[1] = q_in[1];
	q_copy[2] = q_in[2];
	q_copy[3] = q_in[3];
}

static inline void neg_q_Q15(int16_t *q){
	q[0] = -q[0];
	q[1] = -q[1];
	q[2] = -q[2];
	q[3] = -q[3];
}

static inline void quat_div_2_Q15(int16_t *q){
	q[0] = Q1_SHIFT_ROUND(q[0]);
	q[1] = Q1_SHIFT_ROUND(q[1]);
	q[2] = Q1_SHIFT_ROUND(q[2]);
	q[3] = Q1_SHIFT_ROUND(q[3]);
}

static inline void multQuatwithConstQ15(int16_t* q, const int16_t x){
	q[0] = q15_mul(q[0], x);
	q[1] = q15_mul(q[1], x);
	q[2] = q15_mul(q[2], x);
	q[3] = q15_mul(q[3], x);
}



static void speed_IIR_LP_filter_Q15(const int16_t * w, int16_t *w_out){
//	static int32_t a = (int16_t)(0.6013f * (float)Q15);
	static int32_t w_old[4] = {0};
	const int32_t a =  6581; // Ts = 1/500 , fc = 2Hz

	int32_t x;
	x = Q15_SHIFT_ROUND(a* w[0]) + (Q15_SHIFT_ROUND((int32_t)(Q15-a) * w_old[0]));
	w_out[0] = w_old[0] = CLAMP_INT32_TO_INT16(x);


	x = Q15_SHIFT_ROUND(a* w[1]) + Q15_SHIFT_ROUND((int32_t)(Q15-a) * w_old[1]);
	w_out[1] = w_old[1] = CLAMP_INT32_TO_INT16(x);

	x = Q15_SHIFT_ROUND(a* w[2]) + Q15_SHIFT_ROUND((int32_t)(Q15-a) * w_old[2]);
	w_out[2] = w_old[2] = CLAMP_INT32_TO_INT16(x);

	x = Q15_SHIFT_ROUND(a * w[3]) + Q15_SHIFT_ROUND((int32_t)(Q15-a) * w_old[3]);
	w_out[3] = w_old[3] = CLAMP_INT32_TO_INT16(x);
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

void attitude_control_quaternion_lqr_q15(const int16_t *q,const int16_t *q_ref, const int16_t *w_gyro_t, int16_t *tau){

int16_t q_inv[4],q_ref_inv[4], q_err[4], q_err_inv[4], ln_q[3],w[4], w_err[4], diff_err[4],diff_err_filt[4], diff_q[4], x_err[6], u_lqr[3];
static int16_t q_err_last_value[4], q_last_value[4];
static uint8_t q_count= 0;

//memcpy(q_ref_inv, q_ref,sizeof(q_inv));
copy_q(q_ref, q_ref_inv);
// generate linearized quaternion error: theta_
q_t_conj_function(q_ref_inv);
//multiplicateQuaternionQ15(q_ref_inv, q, q_err);
multiplicateQuaternionQ15(q_ref_inv, q, q_err); // stimmt
ln_q15_unit_quaternions_multiplicate_2(q_err, ln_q);

// calculate angular velocity error: w_err
//memcpy(q_err_inv, q_err, sizeof(q_err));
copy_q(q_err, q_err_inv);
q_t_conj_function(q_err_inv);
debug_q_err.w = q_err[0];
debug_q_err.x = q_err[1];
debug_q_err.y = q_err[2];
debug_q_err.z = q_err[3];

debug_q_err_old.w = q_err_last_value[0];
debug_q_err_old.x = q_err_last_value[1];
debug_q_err_old.y = q_err_last_value[2];
debug_q_err_old.z = q_err_last_value[3];

differential_with_scaling_q15(q_err, q_err_last_value,(ATTITUDE_FREQUENCY),LQR_DIFF_SCALE_SHIFT, diff_err); // RAD_MAX_32
speed_IIR_LP_filter_Q15(diff_err,diff_err_filt);
multiplicateQuaternionQ15(q_err_inv,diff_err_filt,w_err);
// scale w to Q5
//for(uint8_t i = 0;i<4;i++){
//	w_err[i] = Q5_SHIFT_ROUND(w_err[i]);
//}


debug_diff_err1 = diff_err_filt[0];
debug_diff_err2 = diff_err_filt[1];
debug_diff_err3 = diff_err_filt[2];
debug_diff_err4 = diff_err_filt[3];

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
debug_u_lqr1 = u_lqr[0];
debug_u_lqr2 = u_lqr[1];
debug_u_lqr3 = u_lqr[2];

// calculate angular velocity w for feedback linearisation
//memcpy(q_inv, q, 4 * sizeof(int16_t));
copy_q(q, q_inv);
q_t_conj_function(q_inv);
differential_with_scaling_q15(q, q_last_value,(ATTITUDE_FREQUENCY),LQR_DIFF_SCALE_SHIFT, diff_q); // RAD_MAX_32

multiplicateQuaternionQ15(q_inv,diff_q,w);
feedback_linearisation(u_lqr, w_gyro_t, &drone_parameter, tau);
//rotate_plus_to_x_lqr(tau,tau);

//if(q_count++ > 10){
q_err_last_value[0] = q_err[0];
q_err_last_value[1] = q_err[1];
q_err_last_value[2] = q_err[2];
q_err_last_value[3] = q_err[3];
//copy_q(q, q_last_value);


q_last_value[0] = q[0];
q_last_value[1] = q[1];
q_last_value[2] = q[2];
q_last_value[3] = q[3];
//q_count = 0;
//}

}



//const int16_t K_q10[3][6] = {
//    {25679,     0,     0, 16537,     0,     0},
//    {    0, 25679,     0,     0, 16537,     0},
//    {    0,     0, 25679,     0,     0, 16537}
//};

/*
 * K =

    7.7460    0.0000    0.0000   23.4270   -0.0000   -0.0000
    0.0000    7.7460   -0.0000    0.0000   12.1994    0.0000
   -0.0000    0.0000    3.6515   -0.0000    0.0000   14.3980

>>
 */

//const int16_t K_q10[3][6] = {
//    {24919,     0,     0,  23989,     0,     0},
//    {    0, 24919,     0,     0, 12492,     0},
//    {    0,     0, 11747,     0,     0, 14744}
//};
//
//const int16_t K_q10[3][6] = {
//    {  5871,      0,      0,  26666,      0,      0},
//    {     0,   5609,      0,      0,  19000,      0},
//    {     0,      0,   1322,      0,      0,  10371}
//};

//const int16_t K_q10[3][6] = {
//    {  4347,      0,      0,  11691,      0,      0},
//    {     0,   4347,      0,      0,  11691,      0},
//    {     0,      0,      4,      0,      0,     97}
//};
// OLD LQR
//const int16_t K_q10[3][6] = {
//    {  6422,      0,      0,  22896,      0,      0},
//    {     0,   6422,      0,      0,  22896,      0},
//    {     0,      0,    189,      0,      0,    625}
//};
// Hat gerade so funktioniert:
//const int16_t K_q10[3][6] = {
//    {   1723,       0,       0,     384,       0,       0},
//    {      0,    1758,       0,       0,     392,       0},
//    {      0,       0,      19,       0,       0,      31}
//};
// Ganz ok
//const int16_t K_q10[3][6] = {
//    {   2615,       0,       0,     521,       0,       0},
//    {      0,    2669,       0,       0,     531,       0},
//    {      0,       0,      97,       0,       0,      80}
//};
//const int16_t K_q10[3][6] = {
//    {   2701,       0,       0,     600,       0,       0},
//    {      0,    2756,       0,       0,     612,       0},
//    {      0,       0,      97,       0,       0,      80}
//};
// attitude
//const int16_t K_q10[3][6] = {
//    {   3180,       0,       0,     696,       0,       0},
//    {      0,    3245,       0,       0,     710,       0},
//    {      0,       0,     121,       0,       0,      87}
//};
// ############## flight #############
//const int16_t K_q10[3][6] = {
//    {   3180,       0,       0,     696,       0,       0},
//    {      0,    3245,       0,       0,     710,       0},
//    {      0,       0,      15,       0,       0,      26}
//};
// works better
const int16_t K_q10[3][6] = {
    {   3180,       0,       0,     696,       0,       0},
    {      0,    3245,       0,       0,     710,       0},
    {      0,       0,      30,       0,       0,      38}
};
// reduced force
//const int16_t K_q10[3][6] = {
//    {   2985,       0,       0,     657,       0,       0},
//    {      0,    3046,       0,       0,     670,       0},
//    {      0,       0,      28,       0,       0,      37}
//};
//// reduced yaw maximum
//const int16_t K_q10[3][6] = {
//    {   3180,       0,       0,     696,       0,       0},
//    {      0,    3245,       0,       0,     710,       0},
//    {      0,       0,       2,       0,       0,       8}
//};
//const int16_t J = [0.012273 0         0;
//0         0.012526 0;
//0         0         0.020953];


static void lqr_q15(const int16_t *x_error,int16_t *u_out){
	int32_t x, sum;

	for (uint8_t i = 0; i < 3; i++) {
		sum = 0;

		x = (int32_t)K_q10[i][0] * x_error[0]; // Q10 * Q15 = Q25
		sum += (x >> 2); // Q23

		x = (int32_t)K_q10[i][1] * x_error[1];
		sum += (x >> 2); // Q23

		x = (int32_t)K_q10[i][2] * x_error[2];
		sum += (x >> 2); // Q23

		x = (int32_t)K_q10[i][3] * x_error[3];
		sum += (x >> 2); // Q23

		x = (int32_t)K_q10[i][4] * x_error[4];
		sum += (x >> 2); // Q23

		x = (int32_t)K_q10[i][5] * x_error[5];
		sum += (x >> 2); // Q23

		sum = Q13_SHIFT_ROUND(sum); // back to Q15 //-> u_max = Q5
		u_out[i] = -CLAMP_INT32_TO_INT16(sum);
	}
}






//static void feedback_linearisation(const int16_t *u,const int16_t *w,system_parameter *pHandle, int16_t *tau){
//	int16_t Jw[3], Ju[3], cross_product[3];
//
//	Jw[0] = q15_mul(pHandle->Jxx, w[0]);
//	Jw[1] = q15_mul(pHandle->Jyy, w[1]);
//	Jw[2] = q15_mul(pHandle->Jzz, w[2]);
//
//	Ju[0] = q15_mul(pHandle->Jxx, u[0]);
//	Ju[1] = q15_mul(pHandle->Jyy, u[1]);
//	Ju[2] = q15_mul(pHandle->Jzz, u[2]);
//
//	crossproduct_3x3_Q15(w, Jw,cross_product);
//
//	tau[0] = CLAMP_INT32_TO_INT16(((int32_t)Ju[0] + (int32_t)cross_product[0]));
//	tau[1] = CLAMP_INT32_TO_INT16(((int32_t)Ju[1] + (int32_t)cross_product[1]));
//	tau[2] = CLAMP_INT32_TO_INT16(((int32_t)Ju[2] + (int32_t)cross_product[2]));
//
//}
static void feedback_linearisation(const int16_t *u,const int16_t *w,system_parameter *pHandle, int16_t *tau){

	int32_t x, y;

	// input u, max torque is Q5 = 16 N
	// input w, max speed ist 34,9 rad/s
	// scaling the result with Q5

//	x = (pHandle->Jxx) * (int32_t)u[0]; // Q15 * Q15 = Q30
	x = (int32_t)u[0] << 10; // Q15 * Q15 = Q30
	y = (((int32_t)w[2] * (int32_t)w[1])/38); // [Q15^2 / 34.9^2] -> [Q15/32] => (34.9^2 / 32) = 38.0628
	y = (y * (int32_t)(pHandle->Jzz - pHandle->Jyy)) >> 5;
//	y = (y >> 5); //unit of y is torque, we scale torque as Q15/max_torque and max_torque is 32 (Q5)
	tau[0] = CLAMP((x + y) >> 10, -Q15,Q15);
// ab hier
//	x = (pHandle->Jyy) * (int32_t)u[1]; // Q20 * Q10 = Q30
	x = (int32_t)u[1] << 10; // Q20 * Q10 = Q30
	y = (((int32_t)w[2] * (int32_t)w[0])/38); // [Q15^2 / 34.9^2] -> [Q15/32] => (34.9^2 / 32) = 38.0628
	y = (y * (int32_t)(pHandle->Jxx - pHandle->Jzz)) >> 5;
//	y = (y >> 5); //unit of y is torque, we scale torque as Q15/max_torque and max_torque is 32 (Q5)
	tau[1] = CLAMP((x + y) >> 10, -Q15,Q15);

//	x = (pHandle->Jzz) * (int32_t)u[2]; // Q20 * Q10 = Q30
	x = (int32_t)u[2] << 10; // Q20 * Q10 = Q30
	y = (((int32_t)w[0] * (int32_t)w[1])/38); // [Q15^2 / 34.9^2] -> [Q15/32] => (34.9^2 / 32) = 38.0628
	y = (y * (int32_t)(pHandle->Jyy - pHandle->Jxx)) >> 5;
//	y = (y >> 5); //unit of y is torque, we scale torque as Q15/max_torque and max_torque is 32 (Q5)
	tau[2] = CLAMP((x + y) >> 10, -Q15,Q15);

	// unit of tau is Q15/max_torque(Q5)
}


const int16_t A_inv_q9[4][4] = {
    {   186,  -1163,   1163,  20581 },
    {   186,   1163,   1163, -20581 },
    {   186,   1163,  -1163,  20581 },
    {   186,  -1163,  -1163, -20581 }
};



void get_motor_speed_from_u(const int32_t *u, int16_t *w_rpm, motor_t *pHandle_motor) {

    for (int i = 0; i < 4; i++) {
        int32_t acc = 0;
        int32_t acc_sqrt;
        for (int j = 0; j < 4; j++) {
            acc += (((int32_t)A_inv_q9[i][j] * (int32_t)u[j]) + (1 << 4)) >> 5; // Q30/Q10
        }
         if(acc >0){
         	acc_sqrt = sqrt_fast_uint(acc);
         }else{
         	acc_sqrt = -sqrt_fast_uint(-acc);
         }
         // acc_sqrt = Q15/Q5 * w


         w_rpm[i] = CLAMP_INT32_TO_INT16(((acc_sqrt * RADQ5_TO_RPMQ13_Q15 + (1 << 14)) >> 15));
     }
//        acc = CLAMP_INT32_TO_INT16(acc);
//        acc = (acc * FACTOR_TAU2RPM_Q15) << 13;
//        if(acc >0){
//        	acc_sqrt = sqrt_fast_uint(acc);
//        }else{
//        	acc_sqrt = -sqrt_fast_uint(-acc);
//        }
//
//        w_rpm[i] = CLAMP_INT32_TO_INT16(acc_sqrt);
//    }

    pHandle_motor->m_1 = (w_rpm[0] << 2); // w_rpm = (speed * Q15 /speed_max) so get speed wrpm * speed_max/Q15 -> wrpm /Q2
    pHandle_motor->m_2 = (w_rpm[1] << 2);
    pHandle_motor->m_3 = (w_rpm[2] << 2);
    pHandle_motor->m_4 = (w_rpm[3] << 2);

}





void set_init_yaw_position(const int16_t *q, int16_t *q_yaw_corr, int16_t *q_axis_corr){
	int16_t w[4], theta, q_corr_0, q_corr_3;
	int32_t q_03, q_12, q_22,q_33,x,y;

	// calculation of yaw with \psi = \operatorname{atan2}\!\big( 2(q_0 q_3 + q_1 q_2), \; 1 - 2(q_2^2 + q_3^2) \big)
	q_03 = ((int32_t)q[0] * (int32_t)q[3]) >> 1; // Q29
	q_12 = ((int32_t)q[1] * (int32_t)q[2]) >> 1; // Q29
	x = (q_03 + q_12 + (1 << 12)) >> 13; // 2*Q15
	x = CLAMP_INT32_TO_INT16(x);

	q_22 = ((int32_t)q[2] * (int32_t)q[2]) >> 1; // Q29
	q_33 = ((int32_t)q[3] * (int32_t)q[3]) >> 1; // Q29
	y = (q_22 + q_33 + (1<<12)) >> 13;	// 2*Q15
	y = CLAMP_INT32_TO_INT16(y);
	y = (Q15 - y);

	theta = q15_atan2(x,y);
	theta = (theta >> 1); // theta/2
//	q_correct
	q_corr_0 = cos_i(theta);
	q_corr_3 = -sin_i(theta);

	q_yaw_corr[0] = q_corr_0;
	q_yaw_corr[1] = 0;
	q_yaw_corr[2] = 0;
	q_yaw_corr[3] = q_corr_3;

	q_axis_corr[0]= cos_i((Q15 >> 2));
	q_axis_corr[1]= 0;
	q_axis_corr[2]= 0;
	q_axis_corr[3]= sin_i((Q15 >> 2));

	NormalizeQuaternionQ15(q_yaw_corr, q_yaw_corr);
	NormalizeQuaternionQ15(q_axis_corr, q_axis_corr);
	correction_ready = 1;

}

void correct_q_axis(int16_t *q, int16_t *w){
	static int16_t q_axis_rot[4] = {SQRT_2_OVER_2_Q15,0,0,-SQRT_2_OVER_2_Q15};

		multiplicateQuaternionQ15(q, q_axis_rot, q);
		multiplicateQuaternionQ15(q, q_axis_rot, q);
		rotate_vector_Q15(q_axis_rot, w, w);
		rotate_vector_Q15(q_axis_rot, w, w);
		NormalizeQuaternionQ15(q, q);

}

static void iir_filter_gyro(const int16_t *gyro_raw, int16_t *gyro_filter){
	static int16_t a = (int16_t)(0.1116f * (float)Q15);
	static int32_t gyro_last_value[3] = {0};

	gyro_last_value[0] = (((int32_t)a * (int32_t)gyro_raw[0]) >> 15) + ((((int32_t)Q15 - a)*(int32_t)gyro_last_value[0]) >> 15);
	gyro_last_value[1] = (((int32_t)a * (int32_t)gyro_raw[1]) >> 15) + ((((int32_t)Q15 - a)*(int32_t)gyro_last_value[1]) >> 15);
	gyro_last_value[2] = (((int32_t)a * (int32_t)gyro_raw[2]) >> 15) + ((((int32_t)Q15 - a)*(int32_t)gyro_last_value[2]) >> 15);

	gyro_last_value[0] = CLAMP_INT32_TO_INT16(gyro_last_value[0]);
	gyro_last_value[1] = CLAMP_INT32_TO_INT16(gyro_last_value[1]);
	gyro_last_value[2] = CLAMP_INT32_TO_INT16(gyro_last_value[2]);

	gyro_filter[0] = gyro_last_value[0];
	gyro_filter[1] = gyro_last_value[1];
	gyro_filter[2] = gyro_last_value[2];

}
static void normalize_to_canonical(int16_t *q){
    if (q[0] < 0) q_t_flipp(q);
}


xyz_16t debug_control_out;
wxyz_16t debut_q_err_control;
void attitude_control_quaternion_nonlinear_q15(const int16_t *q,const int16_t *q_ref, const int16_t *w_gyro_t, int16_t *tau){
	int16_t q_reff[4], q_con[4], q_err[4], gyro_q5[3], gyro_small[3];
	int32_t out[3];

	if(ATT_CONT_HYBRID_LIFT == OFF){
//		copy_q(q_ref,q_reff);
//		copy_q(q,q_con);
//		q_t_conj_function(q_con);
//		q_t_flipp(q_reff);
//		normalize_to_canonical(q_reff);
//		normalize_to_canonical(q_con);
//		multiplicateQuaternionQ15(q_reff,q_con, q_err);
//		normalize_to_canonical(q_err); // take the shortes path
		compute_quat_err_p2_control(q,q_ref,q_err);
	}else{
		q_error_hybrid_lifting(q,q_ref,q_err,ATT_CONT_HYBRID_LIFT_ALPHA,0);
	}
	debut_q_err_control.w = q_err[0];
	debut_q_err_control.x = q_err[1];
	debut_q_err_control.y = q_err[2];
	debut_q_err_control.z = q_err[3];

//	iir_filter_gyro(w_gyro_t,gyro_filter);
	gyro_q5[0] = q15_mul(w_gyro_t[0], GRAD2RAD_GYRO_MAX_Q15); // gyro_q5 = Q15 / w_max -> w_max = 32 rad/s
	gyro_q5[1] = q15_mul(w_gyro_t[1], GRAD2RAD_GYRO_MAX_Q15);
	gyro_q5[2] = q15_mul(w_gyro_t[2], GRAD2RAD_GYRO_MAX_Q15);


	out[0] = q15_mul(q_err[1], (int16_t)(gain.P1.pitch * (float)Q10));
	out[1] = q15_mul(q_err[2],(int16_t)(gain.P1.roll * (float)Q10));
	out[2] = q15_mul(q_err[3],(int16_t)(gain.P1.yaw * (float)Q10));



//	out[0] = CLAMP_INT32_TO_INT16(out[0] - q15_mul(gyro_q5[0],(int32_t)(gain.P2.pitch * (float)Q15)));
//	out[1] = CLAMP_INT32_TO_INT16(out[1] - q15_mul(gyro_q5[1],(int32_t)(gain.P2.roll * (float)Q15)));
//	out[2] = CLAMP_INT32_TO_INT16(out[2] - q15_mul(gyro_q5[2],(int32_t)(gain.P2.yaw * (float)Q15)));


	int16_t P2[3];
	P2[0] = CLAMP_INT32_TO_INT16((int32_t)(gain.P2.pitch * (float)Q10));
	P2[1] = CLAMP_INT32_TO_INT16((int32_t)(gain.P2.roll * (float)Q10));
	P2[2] = CLAMP_INT32_TO_INT16((int32_t)(gain.P2.yaw * (float)Q10));


	for(uint8_t i = 0; i<3; i++){
		int32_t x = ((int32_t)out[i]) - Q10_SHIFT_ROUND((int32_t)gyro_q5[i] * P2[i]);
		out[i] = CLAMP_INT32_TO_INT16(x);
	}
	tau[0] = out[0];
	tau[1] = out[1];
	tau[2] = out[2];
	rotate_plus_to_x(tau,tau);
	iir_filter_tau_Q15(tau,tau,20.0f,(1.0f/500.0f));
	debug_control_out.x = tau[0];
	debug_control_out.y = tau[1];
	debug_control_out.z = tau[2];
}

static void iir_filter_tau_Q15(const int16_t *tau_raw, int16_t *tau_filter, const float fc, const float Ts){

//	static int32_t a = CLAMP_INT32_TO_INT16((int32_t)((PI_MULTIPLY_2 * (float)fc * (1.0f/(float)SENSOR_FUSION_FREQUENCY_IMU))/(PI_MULTIPLY_2 * (float)fc * (1.0f/(float)SENSOR_FUSION_FREQUENCY_IMU) + 1.0f)));
	static int16_t a;
	static bool init = 0;
	if(!init){
		init = 1;
		a = CLAMP_INT32_TO_INT16((int32_t)((PI_MULTIPLY_2 * Ts * fc)/((PI_MULTIPLY_2 * Ts * fc + 1.0f)) * (float)Q15)); // ~0.3859 * Q15 // fc = 10 Hz Ts = 1/100
	}
	static int32_t acc_old[3] = {0};

	for(uint8_t i = 0; i<3;i++){
		acc_old[i] = ((a * (int32_t)tau_raw[i]) >> 15) + ((((int32_t)Q15 - a) * acc_old[i]) >> 15);
		tau_filter[i] = acc_old[i] = CLAMP_INT32_TO_INT16(acc_old[i]);
	}

}



void filter_SLERP_EMA_quaternion_Q15(const int16_t *q_in, int16_t *q_out){

	const int16_t a_div2 = 2814; // fc = 15 Hz, Ts = 1/500, a_div2 = a/2 ... \alpha = 1 - e^{-2\pi f_c T_s}
	static int16_t q_last[4] = {0};
	int16_t dot, q[4], q_last_con[4], q_err[4], e[3], delta_q[4];
	bool sign_q;
	copy_q(q_in, q);
	if(q_in[0] < 0) sign_q = 1;

	dotporduct_4x4_Q15(q_last,q, &dot);
	if(dot < 0) neg_q_Q15(q);
	q_t_conj_function_in_out_q15(q_last,q_last_con);
	multiplicateQuaternionQ15(q_last_con, q, q_err);

	ln_q15_unit_quaternions_multiplicate_2(q_err,e);
//	quat_div_2_Q15(e);
	multQuatwithConstQ15(e,a_div2);

	exponential_mapping_error_Q15(e,delta_q);
	multiplicateQuaternionQ15(q_last, delta_q, q_out);

	NormalizeQuaternionQ15(q_out, q_out);

	copy_q(q_out, q_last);

	if((q_out[0] < 0) && sign_q){
		// you're correct
	}else{
		neg_q_Q15(q_out);
	}
}


void Safty_function_Motor_RPM_output(motor_t *pHandle_motor, uint8_t reset){
	if(!SAFETY_MOTOR_COMMAND) return;
	static int32_t last_motor_speed[4] = {0};
//	int32_t m1, m2, m3, m4;
	const int32_t a_q15 = 409; // IIR: fc = 1Hz, Ts = (1/500)
	const int32_t a_1_q = 32358; // Q15 - a_q15
//	m1 = abs(pHandle_motor->m_1); m2 = abs(pHandle_motor->m_2); m3 = abs(pHandle_motor->m_3); m4 = abs(pHandle_motor->m_4);

	if(reset > 0){
		pHandle_motor->m_1  = 0;
		pHandle_motor->m_2  = 0;
		pHandle_motor->m_3  = 0;
		pHandle_motor->m_4  = 0;
		for(uint8_t i = 0;i<4;i++){
			last_motor_speed[i] = 0;
		}
	}

	if((last_motor_speed[0] < SAFE_MIN_MOTORSPEED) || (labs((int32_t)pHandle_motor->m_1 - last_motor_speed[0]) > SAFT_MAX_MOTORSP_JUMP)){
		last_motor_speed[0] = CLAMP((Q15_SHIFT_ROUND((a_q15 * pHandle_motor->m_1) + (a_1_q *last_motor_speed[0]))),MIN_MOTORSPEED,MAX_SPEED_MOTOR_RPM);
		pHandle_motor->m_1 = last_motor_speed[0];
	}
	if((last_motor_speed[1] < SAFE_MIN_MOTORSPEED)|| (labs((int32_t)pHandle_motor->m_2 - last_motor_speed[1]) > SAFT_MAX_MOTORSP_JUMP)){
		last_motor_speed[1] = CLAMP((Q15_SHIFT_ROUND((a_q15 * pHandle_motor->m_2) + (a_1_q *last_motor_speed[1]))),MIN_MOTORSPEED,MAX_SPEED_MOTOR_RPM);
		pHandle_motor->m_2 = last_motor_speed[1];
	}
	if((last_motor_speed[2] < SAFE_MIN_MOTORSPEED) || (labs((int32_t)pHandle_motor->m_3 - last_motor_speed[2]) > SAFT_MAX_MOTORSP_JUMP)){
		last_motor_speed[2] = CLAMP((Q15_SHIFT_ROUND((a_q15 * pHandle_motor->m_3) + (a_1_q *last_motor_speed[2]))),MIN_MOTORSPEED,MAX_SPEED_MOTOR_RPM);
		pHandle_motor->m_3 = last_motor_speed[2];
	}
	if((last_motor_speed[3] < SAFE_MIN_MOTORSPEED)|| (labs((int32_t)pHandle_motor->m_4 - last_motor_speed[3]) > SAFT_MAX_MOTORSP_JUMP) ){
		last_motor_speed[3] = CLAMP((Q15_SHIFT_ROUND((a_q15 * pHandle_motor->m_4) + (a_1_q *last_motor_speed[3]))),MIN_MOTORSPEED,MAX_SPEED_MOTOR_RPM);
		pHandle_motor->m_4 = last_motor_speed[3];
	}
	if(pHandle_motor->m_1 > SAFE_MAX_MOTORSPEED) pHandle_motor->m_1 = SAFE_MAX_MOTORSPEED;
	if(pHandle_motor->m_2 > SAFE_MAX_MOTORSPEED) pHandle_motor->m_2 = SAFE_MAX_MOTORSPEED;
	if(pHandle_motor->m_3 > SAFE_MAX_MOTORSPEED) pHandle_motor->m_3 = SAFE_MAX_MOTORSPEED;
	if(pHandle_motor->m_4 > SAFE_MAX_MOTORSPEED) pHandle_motor->m_4 = SAFE_MAX_MOTORSPEED;
	last_motor_speed[0] = pHandle_motor->m_1;
	last_motor_speed[1] = pHandle_motor->m_2;
	last_motor_speed[2] = pHandle_motor->m_3;
	last_motor_speed[3] = pHandle_motor->m_4;


}





/**
 * @brief Hybrid-Lifting basierte Fehlerquaternion-Berechnung
 *
 * @param q_m      gemessene Quaternion (Messung)
 * @param q_d      Soll-Quaternion (Referenz)
 * @param q_output Ergebnis: stetiger Fehlerquaternion (geht an Regler)
 * @param alpha    Schwellwert für Sprungbedingung (z.B. 3276 ≈ 0.1)
 * @param reset    >0 setzt internen Zustand zurück
 */
void q_error_hybrid_lifting(const int16_t *q_m,
                            const int16_t *q_d,
                            int16_t *q_output,
                            const int16_t alpha,
                            const uint8_t reset)
{
    static int16_t q_err_old[4] = {Q15, 0, 0, 0};
    int16_t q_d_conj[4];
    int16_t q_err_raw[4];
    int16_t q_err_alt[4];
    int16_t q_selected[4];
    int16_t s1, s2, d;

    // === Reset ===
    if (reset > 0) {
        q_err_old[0] = Q15; q_err_old[1] = 0; q_err_old[2] = 0; q_err_old[3] = 0;
        q_output[0]  = Q15; q_output[1]  = 0; q_output[2]  = 0; q_output[3]  = 0;
        return;
    }

    // === 1) Rohfehler: q_err_raw = conj(q_d) ⊗ q_m ===
    q_t_conj_function_in_out_q15(q_d, q_d_conj);
    multiplicateQuaternionQ15(q_d_conj, q_m, q_err_raw);
    // NICHT hier normalisieren!

    // === 2) Antipodales Gegenstück ===
    q_err_alt[0] = -q_err_raw[0];
    q_err_alt[1] = -q_err_raw[1];
    q_err_alt[2] = -q_err_raw[2];
    q_err_alt[3] = -q_err_raw[3];

    // === 3) Ähnlichkeit (dot-product) mit dem gespeicherten Zustand ===
    dotporduct_4x4_Q15(q_err_old, q_err_raw, &s1);
    dotporduct_4x4_Q15(q_err_old, q_err_alt, &s2);

    // === 4) Wähle den näheren Kandidaten (größeres Skalarprodukt) ===
    if (s1 >= s2) {
        copy_q(q_err_raw, q_selected);
    } else {
        copy_q(q_err_alt, q_selected);
    }

    // === 5) Hybrid-Memory-Regel ===
    dotporduct_4x4_Q15(q_err_old, q_selected, &d);

    // Wenn Ähnlichkeit zu gering -> update memory (jump)
    if (abs(d) < alpha) {  // Geänderte Bedingung
        copy_q(q_selected, q_err_old);
    }

    // === 6) Ausgabe und Normalisierung (erst jetzt!) ===
    NormalizeQuaternionQ15(q_selected, q_selected);
    copy_q(q_selected, q_output);
}


static void compute_quat_err_p2_control(const int16_t *q_m, const int16_t *q_d, int16_t *q_out){
	int16_t d, q_meas[4];
	copy_q(q_m, q_meas);
	dotporduct_4x4_Q15(q_d, q_meas, &d);
	if(d<0) q_t_flipp(q_meas);
	q_t_conj_function(q_meas);
	multiplicateQuaternionQ15(q_d, q_meas, q_out);
	normalize_to_canonical(q_out);
}

#include <stdint.h>

#define INV_SQRT2_Q15 23170   // ≈ 1/sqrt(2) * 32768

// u_plus[0] = tau_x_plus (Q15)
// u_plus[1] = tau_y_plus (Q15)
// u_plus[2] = tau_z_plus (Q15)
// u_x[0]    = tau_x_x    (Q15)
// u_x[1]    = tau_y_x    (Q15)
// u_x[2]    = tau_z_x    (Q15)

static void rotate_plus_to_x(const int16_t *u_plus, int16_t *u_x)
{
    // temporäre 32-Bit Akkus zur Vermeidung von Überläufen
    int32_t sum, diff;

    // tau_x_x = (tau_x_plus + tau_y_plus) * 1/sqrt(2)
    sum  = (int32_t)u_plus[0] + (int32_t)u_plus[1];
    sum  = (sum * INV_SQRT2_Q15) >> 15;

    // tau_y_x = (-tau_x_plus + tau_y_plus) * 1/sqrt(2)
    diff = (int32_t)(-u_plus[0]) + (int32_t)u_plus[1];
    diff = (diff * INV_SQRT2_Q15) >> 15;

    // Ergebnisse zuweisen (clamping optional)
    if (sum  >  32767) sum  =  32767;
    if (sum  < -32768) sum  = -32768;
    if (diff >  32767) diff =  32767;
    if (diff < -32768) diff = -32768;

    u_x[0] = (int16_t)sum;             // τx_X
    u_x[1] = (int16_t)diff;            // τy_X
    u_x[2] = u_plus[2];                // τz bleibt unverändert
}


static void rotate_plus_to_x_lqr(const int16_t *u_plus, int16_t *u_x)
{
    int32_t sum, diff;

    // τx_X = ( τx⁺ - τy⁺ ) * 1/√2
    sum  = (int32_t)u_plus[0] - (int32_t)u_plus[1];
    sum  = (sum * INV_SQRT2_Q15) >> 15;

    // τy_X = ( τx⁺ + τy⁺ ) * 1/√2
    diff = (int32_t)u_plus[0] + (int32_t)u_plus[1];
    diff = (diff * INV_SQRT2_Q15) >> 15;

    // Clamping
    if (sum  >  32767) sum  =  32767;
    if (sum  < -32768) sum  = -32768;
    if (diff >  32767) diff =  32767;
    if (diff < -32768) diff = -32768;

    u_x[0] = (int16_t)sum;   // τx_X
    u_x[1] = (int16_t)diff;  // τy_X
    u_x[2] = u_plus[2];      // τz unverändert
}


