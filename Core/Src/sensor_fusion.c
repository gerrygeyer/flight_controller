/*
 * sensor_fusion.c
 *
 *  Created on: May 16, 2025
 *      Author: gerrygeyer
 */


#include <sensor_fusion.h>
//#include <imu.h>
#include "VL53L1X_api.h"
#include "bno085.h"
#include <mpu600.h>
#include "lis3mdl.h"
#include <main.h>
#include <stdbool.h>
#include <sys_math.h>
#include <settings.h>
#include <log_data.h>

#include <communication.h>
#include <Distance/distance_sensor.h>


extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;

//###### PARAMETER STRUCKT ######
sensor_fusion sf_values;
// ####### DOUBLE BUFFER ######
wxyz_16t quaternion_buffer;
xyz_16t gyro_rad_buffer;
volatile bool sf_writing;
// ##### SETINGS #####
float beta_f;
int16_t drift_gain;
int16_t q_drift[4];

int16_t ax, ay, az, gx, gy, gz, mx, my, mz, dt_q15, beta_t;
int16_t q_madgwick_out[4], q_compl_out[4];
int16_t gyro_grad2rad_delta_t_q15;
// Flags
volatile bool mag_ready_flag;
// Output (debug)
float beta_yaw = 0.005;
int16_t beta_yaw_t;
int16_t euler_debug[3];
float euler_debug_pitch,euler_debug_roll,euler_debug_yaw;
xyz_16t debug_gyro, debug_omega, debug_mag_norm, debug_mag_hard_iron, debug_mag_raw, debug_mag_soft;
wxyz_16t debug_qDot, debug_q_out, debug_q_out_norm, debug_h;
int16_t debug_bx, debug_bz;
int16_t debug_grad1, debug_grad2,debug_grad3,debug_grad4;
int16_t debug_h1, debug_h2, debug_h3, debug_h4;
wxyz_16t debug_q_mult;
wxyz_16t debug_gyro_drift;
uint8_t sensor_fusion_takeout;

//#define CLAMP_INT32_TO_INT16(x) ((x) > INT16_MAX ? INT16_MAX : ((x) < INT16_MIN ? INT16_MIN : (int16_t)(x)))
static inline int16_t q15_mul(int16_t a, int16_t b);
static inline void divideQuaternionBy2(int16_t *q);
static inline void multQuaternionWith2(int16_t *q);
static inline void multQuatwithConstQ15(int16_t* q, const int16_t x);
static inline void add2QuaternionQ15(const int16_t *q1, const int16_t *q2, int16_t *q_out);
static void error_function_small(int16_t *accel, int16_t *q, int16_t *f_error);
static void error_function_bigQ15(const int16_t *accel, const int16_t *mag,const int16_t *q, int16_t *f_error, const int16_t *bxz);
static void get_jacobi_small(int16_t J[3][4], int16_t *q);
static void get_jacobi_bigQ15(int16_t J[6][4], int16_t *q, int16_t *bxz);
static void compute_gradient(int16_t grad[4], int16_t J[3][4], int16_t f_error[3]);
static void compute_gradient_bigQ15(int16_t *grad, int16_t J[6][4], int16_t *f_error);
static void get_bx_bz_q15(const int16_t *h, int16_t *bxz);
static void Madgwick_filter_acc_gyro_mag(int16_t beta, sensor_fusion *pHandle_sf);
static void Madgwick_filter_acc_gyro(int16_t beta, sensor_fusion *pHandle_sf);
static void q15_qDot_mu_dt_with_rest(int16_t *q_in, const int16_t *qDot, const int16_t dt);
static void iir_filter_bx_bz_q15(int16_t *bxz);
static void gyro_drift_Q15(const int16_t *q_est, const int16_t *qDot, int16_t *q_drift);

void get_quaternion_Q15(int16_t *q, int16_t *w) {
	__disable_irq();
	if(!sf_writing){
		q[0] = sf_values.quaternion.w;
		q[1] = sf_values.quaternion.x;
		q[2] = sf_values.quaternion.y;
		q[3] = sf_values.quaternion.z;

		w[0] = sf_values.gyro_t_rad.x;
		w[1] = sf_values.gyro_t_rad.y;
		w[2] = sf_values.gyro_t_rad.z;
		__enable_irq();
		return q;
	}else{
		q[0] = quaternion_buffer.w;
		q[1] = quaternion_buffer.x;
		q[2] = quaternion_buffer.y;
		q[3] = quaternion_buffer.z;

		w[0] = gyro_rad_buffer.x;
		w[1] = gyro_rad_buffer.y;
		w[2] = gyro_rad_buffer.z;

		__enable_irq();
		return q;
	}
}

sensor_fusion* read_sensorfusion_data(void){
    return &sf_values;
}


void init_sensors(void)
{
	sf_writing = false;
	sensor_fusion_takeout = 0;
	mag_ready_flag = false;

	q_drift[0] = Q15;
	q_drift[1] = 0;
	q_drift[2] = 0;
	q_drift[3] = 0;
//	dt_q15 = ((1UL <<15) * 35)/80;
	float x = ((float)((float)Q15 * GRAD2RAD_GYRO)/(float)SENSOR_FUSION_FREQUENCY_IMU);
	dt_q15 = CLAMP_INT32_TO_INT16((int32_t)x);

//	dt_q15 = (1UL <<15)/80;
	beta_f = 0.00015;
	beta_yaw = 0.009;

	q_madgwick_out[0] = Q15_ONE;
	q_madgwick_out[1] = 0;
	q_madgwick_out[2] = 0;
	q_madgwick_out[3] = 0;

	drift_gain = CLAMP_INT32_TO_INT16((int32_t)(0.01f * (float)Q15));

	if(COMMUNICATION_IMU_MAG){
		LIS3MDL_Init();
		MPU6000_Init();
	}


}

void task_imu_sensor_fusion(void){
//	 ignore the first interrupts during the initalisation

	if(sensor_fusion_takeout < 15){
		sensor_fusion_takeout ++;
	}else{

		MPU6000_Get_data_IT(&sf_values);
			// debug calc for beta
			beta_t = (int16_t)((float)INT16_MAX * beta_f * GRAD2RAD_GYRO);
			beta_yaw_t = CLAMP_INT32_TO_INT16((int32_t)((float)INT16_MAX * beta_yaw * GRAD2RAD_GYRO));


			switch(mag_ready_flag){
			case(true):

		  stopp_time_measurement();
		  start_time_measurement();

				read_data_mag(&sf_values);
				Madgwick_filter_acc_gyro_mag(beta_t, &sf_values);
				mag_ready_flag = false;
			break;
			default:
				Madgwick_filter_acc_gyro(beta_t, &sf_values);
			break;
			}


	}
}

void mag_ready(void){
	mag_ready_flag = true;
}

void read_bno085_values(void){
//	read_sensor_values(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);

}


void read_distance_sensor(void){

//	BNO085_GetAccel(&a);
//	BNO085_GetGyro(&g);
//	BNO085_GetMag(&m);
    // distance ist nun dein Messwert (in mm)
}


static void Madgwick_filter_acc_gyro_mag(int16_t beta, sensor_fusion *pHandle_sf){


	int16_t accel[3], accel_norm[3], gyro[3],mag_raw[3], mag_equalized[3], mag_norm[3],
			f_error[6], grad[4], grad_norm[4], omega[4], mag_quat[4], qDot[4], h[4],q_mul_w[4], q_con[4],
			bxz[2], J[6][4];


	//get values

	accel[0] = pHandle_sf->acc_t.x;
	accel[1] = pHandle_sf->acc_t.y;
	accel[2] = pHandle_sf->acc_t.z;
	gyro[0] = pHandle_sf->gyro_t.x;
	gyro[1] = pHandle_sf->gyro_t.y;
	gyro[2] = pHandle_sf->gyro_t.z;

//	gyro[0] -= pHandle_sf->gyro_drift_est.x;
//	gyro[1] -= pHandle_sf->gyro_drift_est.y;
//	gyro[2] -= pHandle_sf->gyro_drift_est.z;

	// compensate Drift
	gyro[0] -= q15_mul(q_drift[1],drift_gain);
	gyro[1] -= q15_mul(q_drift[2],drift_gain);
	gyro[2] -= q15_mul(q_drift[3],drift_gain);

	debug_gyro_drift.x = q_drift[1];
	debug_gyro_drift.y = q_drift[2];
	debug_gyro_drift.z = q_drift[3];

	mag_raw[0] = pHandle_sf->mag_t.x;
	mag_raw[1] = pHandle_sf->mag_t.y;
	mag_raw[2] = pHandle_sf->mag_t.z; // raw z-axis point up

	omega[0] = 0;
//	omega[1] = debug_omega.x = q15_mul(gyro[0],GYRO_GRAD_TO_RAD_Q15);
//	omega[2] = debug_omega.y = q15_mul(gyro[1],GYRO_GRAD_TO_RAD_Q15);
//	omega[3] = debug_omega.z = q15_mul(gyro[2],GYRO_GRAD_TO_RAD_Q15);

	omega[1] = debug_omega.x = gyro[0];
	omega[2] = debug_omega.y = gyro[1];
	omega[3] = debug_omega.z = gyro[2];



	debug_mag_raw.x = mag_raw[0];
	debug_mag_raw.y = mag_raw[1];
	debug_mag_raw.z = mag_raw[2];


	hardiron_apply_q15(mag_raw);
	debug_mag_hard_iron.x = mag_raw[0];
	debug_mag_hard_iron.y = mag_raw[1];
	debug_mag_hard_iron.z = mag_raw[2];
	softiron_apply_q15(mag_raw, mag_equalized);
	debug_mag_soft.x = mag_equalized[0];
	debug_mag_soft.y = mag_equalized[1];
	debug_mag_soft.z = mag_equalized[2];
	norm_3d_vector(accel, accel_norm);
	norm_3d_vector(mag_equalized, mag_norm);


	debug_mag_norm.x = mag_norm[1];
	debug_mag_norm.y = mag_norm[0];
	debug_mag_norm.z = mag_norm[2];

	mag_quat[0] = 0;
	mag_quat[1] = mag_norm[1];
	mag_quat[2] = mag_norm[0];
	mag_quat[3] = mag_norm[2];


	// h = q ⊗ [0; mag] ⊗ q*    // getestet und funkitoniert ✅
	q_t_conj_function_in_out_q15(q_madgwick_out, q_con); // getestet und funkitoniert ✅
	multiplicateQuaternionQ15(q_madgwick_out,mag_quat,q_mul_w); // getestet und funkitoniert ✅
	multiplicateQuaternionQ15(q_mul_w,q_con,h); // getestet und funkitoniert ✅
	get_bx_bz_q15(h, bxz); // getestet und funkitoniert ✅

	bxz[0] = - bxz[0]; // correct the yaw direction

	iir_filter_bx_bz_q15(bxz);

	debug_q_mult.w = q_mul_w[0];
	debug_q_mult.x = q_mul_w[1];
	debug_q_mult.y = q_mul_w[2];
	debug_q_mult.z = q_mul_w[3];

	debug_h1 = h[0];
	debug_h2 = h[1];
	debug_h3 = h[2];
	debug_h4 = h[3];

	debug_bx = bxz[0];
	debug_bz = bxz[1];

	error_function_bigQ15(accel_norm, mag_norm, q_madgwick_out, f_error, bxz); // getestet und funkitoniert ✅
	get_jacobi_bigQ15(J,q_madgwick_out, bxz); // getestet und funkitoniert ✅
	compute_gradient_bigQ15(grad, J, f_error); // getestet und funkitoniert ✅
	Normalize4DvectorQ15(grad, grad_norm);

	gyro_drift_Q15(q_madgwick_out,grad_norm, q_drift);

	multiplicateQuaternionQ15(q_madgwick_out,omega,qDot); // getestet und funkitoniert ✅
	divideQuaternionBy2(qDot); // getestet und funkitoniert ✅
	/*
	 * Matlab:
	 * qDot = qDot - beta * grad;
	 */



	qDot[0] = debug_qDot.w = CLAMP_INT32_TO_INT16((int32_t)qDot[0] - (int32_t)q15_mul(grad_norm[0], beta));
	qDot[1] = debug_qDot.x = CLAMP_INT32_TO_INT16((int32_t)qDot[1] - (int32_t)q15_mul(grad_norm[1], beta));
	qDot[2] = debug_qDot.y = CLAMP_INT32_TO_INT16((int32_t)qDot[2] - (int32_t)q15_mul(grad_norm[2], beta));
	qDot[3] = debug_qDot.z = CLAMP_INT32_TO_INT16((int32_t)qDot[3] - (int32_t)q15_mul(grad_norm[3], beta_yaw_t));




	q15_qDot_mu_dt_with_rest(q_madgwick_out,qDot, dt_q15); // here in dt_q15 the calc gyro_grad_2_rad

	NormalizeQuaternionQ15(q_madgwick_out, q_madgwick_out);
	debug_q_out_norm.w = q_madgwick_out[0];
	debug_q_out_norm.x = q_madgwick_out[1];
	debug_q_out_norm.y = q_madgwick_out[2];
	debug_q_out_norm.z = q_madgwick_out[3];


	quat_to_euler_q15(q_madgwick_out, euler_debug);
	euler_debug_roll = (float)euler_debug[0];// * 360.0f / (float)INT16_MAX;
	euler_debug_pitch = (float)euler_debug[1];// * 360.0f / (float)INT16_MAX;
	euler_debug_yaw = (float)euler_debug[2];// * 360.0f / (float)INT16_MAX;

	// Safe quaternion / gyro two times
	// 1. Time
	sf_writing = true;
	pHandle_sf->quaternion.w = q_madgwick_out[0];
	pHandle_sf->quaternion.x = q_madgwick_out[1];
	pHandle_sf->quaternion.y = q_madgwick_out[2];
	pHandle_sf->quaternion.z = q_madgwick_out[3];

	pHandle_sf->gyro_t_rad.x =  q15_mul(gyro[0],GRAD2RAD_GYRO_MAX_Q15);
	pHandle_sf->gyro_t_rad.y =  q15_mul(gyro[1],GRAD2RAD_GYRO_MAX_Q15);
	pHandle_sf->gyro_t_rad.z =  q15_mul(gyro[2],GRAD2RAD_GYRO_MAX_Q15);
	sf_writing = false;
	// 2. Time
	quaternion_buffer.w = q_madgwick_out[0];
	quaternion_buffer.x = q_madgwick_out[1];
	quaternion_buffer.y = q_madgwick_out[2];
	quaternion_buffer.z = q_madgwick_out[3];

	gyro_rad_buffer.x =  q15_mul(gyro[0],GRAD2RAD_GYRO_MAX_Q15);
	gyro_rad_buffer.y =  q15_mul(gyro[1],GRAD2RAD_GYRO_MAX_Q15);
	gyro_rad_buffer.z =  q15_mul(gyro[2],GRAD2RAD_GYRO_MAX_Q15);
	// end
}

static void Madgwick_filter_acc_gyro(int16_t beta, sensor_fusion *pHandle_sf){

	int16_t accel[3], accel_norm[3], gyro[3], f_error[3], grad[4], grad_norm[4],omega[4], qDot[4], J[3][4];



	accel[0] = pHandle_sf->acc_t.x;
	accel[1] = pHandle_sf->acc_t.y;
	accel[2] = pHandle_sf->acc_t.z;

	gyro[0] = pHandle_sf->gyro_t.x;
	gyro[1] = pHandle_sf->gyro_t.y;
	gyro[2] = pHandle_sf->gyro_t.z;

	// compensate Drift
	gyro[0] -= q15_mul(q_drift[1],drift_gain);
	gyro[1] -= q15_mul(q_drift[2],drift_gain);
	gyro[2] -= q15_mul(q_drift[3],drift_gain);

	debug_gyro_drift.x = q_drift[1];
	debug_gyro_drift.y = q_drift[2];
	debug_gyro_drift.z = q_drift[3];


	omega[0] = 0;
//	omega[1] = debug_gyro.x = q15_mul(gyro[0],GYRO_GRAD_TO_RAD_Q15);
//	omega[2] = debug_gyro.y = q15_mul(gyro[1],GYRO_GRAD_TO_RAD_Q15);
//	omega[3] = debug_gyro.z = q15_mul(gyro[2],GYRO_GRAD_TO_RAD_Q15);
	omega[1] = debug_omega.x = gyro[0];
	omega[2] = debug_omega.y = gyro[1];
	omega[3] = debug_omega.z = gyro[2];


	norm_3d_vector(accel, accel_norm); // getestet und funkitoniert ✅
	error_function_small(accel_norm, q_madgwick_out, f_error); // getestet und funkitoniert ✅
	get_jacobi_small(J, q_madgwick_out);// getestet und funkitoniert ✅
	compute_gradient(grad, J, f_error);// getestet und funkitoniert ✅
	Normalize4DvectorQ15(grad, grad_norm);

	gyro_drift_Q15(q_madgwick_out,grad_norm, q_drift);
	// Benötige Testfunktion ab hier:
	multiplicateQuaternionQ15(q_madgwick_out,omega,qDot); // getestet und funkitoniert ✅
	divideQuaternionBy2(qDot); // getestet und funkitoniert ✅
	/*
	 * Matlab:
	 * qDot = qDot - beta * grad;
	 */
	qDot[0] = debug_qDot.w = CLAMP_INT32_TO_INT16((int32_t)qDot[0] - (int32_t)q15_mul(grad_norm[0], beta));
	qDot[1] = debug_qDot.x = CLAMP_INT32_TO_INT16((int32_t)qDot[1] - (int32_t)q15_mul(grad_norm[1], beta));
	qDot[2] = debug_qDot.y = CLAMP_INT32_TO_INT16((int32_t)qDot[2] - (int32_t)q15_mul(grad_norm[2], beta));
	qDot[3] = debug_qDot.z = CLAMP_INT32_TO_INT16((int32_t)qDot[3] - (int32_t)q15_mul(grad_norm[3], beta));

	q15_qDot_mu_dt_with_rest(q_madgwick_out,qDot, dt_q15);

	debug_q_out.w = q_madgwick_out[0];
	debug_q_out.x = q_madgwick_out[1];
	debug_q_out.y = q_madgwick_out[2];
	debug_q_out.z = q_madgwick_out[3];

	NormalizeQuaternionQ15(q_madgwick_out, q_madgwick_out);

	// Safe quaternion / gyro two times
	// 1. Time
	sf_writing = true;
	pHandle_sf->quaternion.w = q_madgwick_out[0];
	pHandle_sf->quaternion.x = q_madgwick_out[1];
	pHandle_sf->quaternion.y = q_madgwick_out[2];
	pHandle_sf->quaternion.z = q_madgwick_out[3];

	pHandle_sf->gyro_t_rad.x =  q15_mul(gyro[0],GRAD2RAD_GYRO_MAX_Q15);
	pHandle_sf->gyro_t_rad.y =  q15_mul(gyro[1],GRAD2RAD_GYRO_MAX_Q15);
	pHandle_sf->gyro_t_rad.z =  q15_mul(gyro[2],GRAD2RAD_GYRO_MAX_Q15);
	sf_writing = false;
	// 2. Time
	quaternion_buffer.w = q_madgwick_out[0];
	quaternion_buffer.x = q_madgwick_out[1];
	quaternion_buffer.y = q_madgwick_out[2];
	quaternion_buffer.z = q_madgwick_out[3];

	gyro_rad_buffer.x =  q15_mul(gyro[0],GRAD2RAD_GYRO_MAX_Q15);
	gyro_rad_buffer.y =  q15_mul(gyro[1],GRAD2RAD_GYRO_MAX_Q15);
	gyro_rad_buffer.z =  q15_mul(gyro[2],GRAD2RAD_GYRO_MAX_Q15);
	// end

	debug_q_out_norm.w = q_madgwick_out[0];
	debug_q_out_norm.x = q_madgwick_out[1];
	debug_q_out_norm.y = q_madgwick_out[2];
	debug_q_out_norm.z = q_madgwick_out[3];


	quat_to_euler_q15(q_madgwick_out, euler_debug);
	euler_debug_roll = (float)euler_debug[0];// * 360.0f / (float)INT16_MAX;
	euler_debug_pitch = (float)euler_debug[1];// * 360.0f / (float)INT16_MAX;
	euler_debug_yaw = (float)euler_debug[2];// * 360.0f / (float)INT16_MAX;
}
// ######## QUATERNION INLINE FUNCTIONS ############
static inline int16_t q15_mul(int16_t a, int16_t b) {
    return (int16_t)(((int32_t)a * b + (1 << 14)) >> 15); // mit Rundung
}

/**
 * @brief       Integrates a quaternion derivative using fixed-point arithmetic with residual correction.
 *
 * @details     Computes:
 *              \f$ q \leftarrow q + \dot{q} \cdot dt \f$
 *              using Q15 format and accumulates sub-Q15 residuals (`rest`) to preserve precision over time.
 *
 * @param[in,out]   q_in    Quaternion to be updated (int16_t[4], Q15).
 * @param[in]       qDot    Quaternion derivative (int16_t[4], Q15).
 * @param[in]       dt      Timestep in Q15 format (1.0 ≙ 32768).
 *
 * @note
 * - Uses static residual storage to improve integration accuracy.
 * - Residuals are retained between function calls.
 * - Not thread-safe or interrupt-safe due to static state.
 *
 * @warning
 * - Ensure `q_in` and `qDot` point to valid 4-element arrays.
 *
 * @see         CLAMP_INT32_TO_INT16
 */
static void q15_qDot_mu_dt_with_rest(int16_t *q_in, const int16_t *qDot, const int16_t dt){
	static wxyz_16t rest = {0, 0, 0, 0};
    int32_t prod, delta, sum;
    // w
    prod = (int32_t)qDot[0] * dt + rest.w;
    delta = prod >> 15;
    rest.w = prod - (delta << 15);
    sum = (int32_t)q_in[0] + delta;
    q_in[0] = CLAMP_INT32_TO_INT16(sum);

    // x
    prod = (int32_t)qDot[1] * dt + rest.x;
    delta = prod >> 15;
    rest.x = prod - (delta << 15);
    sum = (int32_t)q_in[1] + delta;
    q_in[1] = CLAMP_INT32_TO_INT16(sum);

    // y
    prod = (int32_t)qDot[2] * dt + rest.y;
    delta = prod >> 15;
    rest.y = prod - (delta << 15);
    sum = (int32_t)q_in[2] + delta;
    q_in[2] = CLAMP_INT32_TO_INT16(sum);

    // z
    prod = (int32_t)qDot[3] * dt + rest.z;
    delta = prod >> 15;
    rest.z = prod - (delta << 15);
    sum = (int32_t)q_in[3] + delta;
    q_in[3] = CLAMP_INT32_TO_INT16(sum);
}

static inline void divideQuaternionBy2(int16_t *q) {
    q[0] >>= 1;
    q[1] >>= 1;
    q[2] >>= 1;
    q[3] >>= 1;
}

static inline void multQuaternionWith2(int16_t *q){
	q[0] = CLAMP_INT32_TO_INT16(((int32_t)q[0] << 1));
	q[1] = CLAMP_INT32_TO_INT16(((int32_t)q[1] << 1));
	q[2] = CLAMP_INT32_TO_INT16(((int32_t)q[2] << 1));
	q[3] = CLAMP_INT32_TO_INT16(((int32_t)q[3] << 1));
}

static inline void multQuatwithConstQ15(int16_t* q, const int16_t x){
	q[0] = q15_mul(q[0], x);
	q[1] = q15_mul(q[1], x);
	q[2] = q15_mul(q[2], x);
	q[3] = q15_mul(q[3], x);
}

static inline void add2QuaternionQ15(const int16_t *q1, const int16_t *q2, int16_t *q_out){
	int32_t q_x[4];
	q_x[0] = (int32_t)q1[0] + (int32_t)q2[0];
	q_x[1] = (int32_t)q1[1] + (int32_t)q2[1];
	q_x[2] = (int32_t)q1[2] + (int32_t)q2[2];
	q_x[3] = (int32_t)q1[3] + (int32_t)q2[3];

	q_out[0] = CLAMP_INT32_TO_INT16(q_x[0]);
	q_out[1] = CLAMP_INT32_TO_INT16(q_x[1]);
	q_out[2] = CLAMP_INT32_TO_INT16(q_x[2]);
	q_out[3] = CLAMP_INT32_TO_INT16(q_x[3]);
}

// ######### END INLINE FUNCTIONS ###############

// ######## SENSOR FUSION STATIC FUNCTIONS ############



static void error_function_small(int16_t *accel, int16_t *q, int16_t *f_error){
	int32_t q_24, q_13, q_12, q_34;


	q_24 = q15_mul(q[1],q[3]);
	q_13 = q15_mul(q[0],q[2]);
	q_12 = q15_mul(q[0],q[1]);
	q_34 = q15_mul(q[2],q[3]);


	// f1 = 2*(q2*q4 - q1*q3) - ax
	int32_t f1 = ((q_24 - q_13) << 1) - accel[0];

	// f2 = 2*(q1*q2 + q3*q4) - ay
	int32_t f2 = ((q_12 + q_34) << 1) - accel[1];

	// f3 = 1 - 2*q2^2 - 2*q3^2 - az
	int32_t q2_sq = ((int32_t)q[1] * q[1]) >> 15;
	int32_t q3_sq = ((int32_t)q[2] * q[2]) >> 15;
	int32_t f3 = Q15_ONE - ((q2_sq + q3_sq) << 1) - accel[2];

	f_error[0] = CLAMP_INT32_TO_INT16(f1);
	f_error[1] = CLAMP_INT32_TO_INT16(f2);
	f_error[2] = CLAMP_INT32_TO_INT16(f3);
}

static void error_function_bigQ15(const int16_t *accel, const int16_t *mag,const int16_t *q, int16_t *f_error, const int16_t *bxz){
	int32_t q_24, q_13, q_12, q_34, q_23, q_14;
	uint32_t x_ut;
	int32_t x_t;

	q_24 = q15_mul(q[1],q[3]);
	q_13 = q15_mul(q[0],q[2]);
	q_12 = q15_mul(q[0],q[1]);
	q_34 = q15_mul(q[2],q[3]);
	q_23 = q15_mul(q[1],q[2]);
	q_14 = q15_mul(q[0],q[3]);


	// f1 = 2*(q2*q4 - q1*q3) - ax
	int32_t f1 = ((q_24 - q_13) << 1) - accel[0];

	// f2 = 2*(q1*q2 + q3*q4) - ay
	int32_t f2 = ((q_12 + q_34) << 1) - accel[1];

	// f3 = 1 - 2*q2^2 - 2*q3^2 - az
	int32_t q2_sq = ((int32_t)q[1] * q[1]) >> 15;
	int32_t q3_sq = ((int32_t)q[2] * q[2]) >> 15;
	int32_t f3 = Q15_ONE - ((q2_sq + q3_sq) << 1) - accel[2];

	f_error[0] = CLAMP_INT32_TO_INT16(f1);
	f_error[1] = CLAMP_INT32_TO_INT16(f2);
	f_error[2] = CLAMP_INT32_TO_INT16(f3);

	// 2.Part f_error[3] to f_error[5]

//   f4 =  2*bx*(0.5 - q3^2 - q4^2) + 2*bz*(q2*q4 - q1*q3) - mx;
//   f5 = 2*bx*(q2*q3 - q1*q4) + 2*bz*(q1*q2 + q3*q4) - my;
//   f6 = 2*bx*(q1*q3 + q2*q4) + 2*bz*(0.5 - q2^2 - q3^2) - mz];
	int32_t q3_sq_q30 = (int32_t)q[2] * (int32_t)q[2]; // Q30
	int32_t q4_sq_q30 = (int32_t)q[3] * (int32_t)q[3]; // Q30

	x_t = (q3_sq_q30 + q4_sq_q30) >> 14; // = Q16 -> 2*Q15
	x_t = (0x7FFF - x_t);

//	x_ut 		= ((0x7FFF << 15) - (((uint32_t)(q[2] * q[2])) << 1 ) - (((uint32_t)(q[3] * q[3])) << 1)); // 2*(0.5 - q3^2 - q4^2) all in q30
//	x_t 		= CLAMP_INT32_TO_INT16(x_ut >> 15);
	x_t 		= CLAMP_INT32_TO_INT16(x_t);
	x_t			= q15_mul(x_t, bxz[0]);
	x_t			+=(q15_mul((q_24 - q_13), bxz[1]) << 1);
	x_t			-= mag[0];
	f_error[3]	= CLAMP_INT32_TO_INT16(x_t);

	x_t 		= (q15_mul((q_23 - q_14), bxz[0]) << 1);
	x_t			+= (q15_mul((q_12 + q_34), bxz[1]) << 1);
	x_t			-= mag[1];
	f_error[4]	= CLAMP_INT32_TO_INT16(x_t);

	int32_t q2_sq_q30 = (int32_t)q[1] * (int32_t)q[1]; // Q30

	x_t = (q3_sq_q30 + q2_sq_q30) >> 14; // = Q16 -> 2*Q15

	x_t = (0x7FFF - x_t);
//	x_ut		= ((0x7FFF << 15) - ((uint32_t)(q[1] * q[1]) << 1 ) - ((uint32_t)(q[2] * q[2]) << 1));
	x_t			= CLAMP_INT32_TO_INT16(x_t);
	x_t			= q15_mul(x_t, bxz[1]);
	x_t 		+= (q15_mul((q_13 + q_24), bxz[0]) << 1);
	x_t			-= mag[2];
	f_error[5]	= CLAMP_INT32_TO_INT16(x_t);
}
//% Jacobian J (3x4)
//J = [-2*q3,  2*q4, -2*q1,  2*q2;
//      2*q2,  2*q1,  2*q4,  2*q3;
//         0, -4*q2, -4*q3,     0];
// int16_t J[3][4];
static void get_jacobi_small(int16_t J[3][4], int16_t *q){

    J[0][0] = CLAMP_INT32_TO_INT16(-((int32_t)q[2] << 1));
    J[0][1] = CLAMP_INT32_TO_INT16( ((int32_t)q[3] << 1));
    J[0][2] = CLAMP_INT32_TO_INT16(-((int32_t)q[0] << 1));
    J[0][3] = CLAMP_INT32_TO_INT16( ((int32_t)q[1] << 1));

    J[1][0] = CLAMP_INT32_TO_INT16( ((int32_t)q[1] << 1));
    J[1][1] = CLAMP_INT32_TO_INT16( ((int32_t)q[0] << 1));
    J[1][2] = CLAMP_INT32_TO_INT16( ((int32_t)q[3] << 1));
    J[1][3] = CLAMP_INT32_TO_INT16( ((int32_t)q[2] << 1));

    J[2][0] = 0;
    J[2][1] = CLAMP_INT32_TO_INT16(-((int32_t)q[1] << 2));
    J[2][2] = CLAMP_INT32_TO_INT16(-((int32_t)q[2] << 2));
    J[2][3] = 0;
}

//% Jacobian J (6x4)
//J = [-2*q3,            2*q4,           -2*q1,          2*q2;
//      2*q2,            2*q1,            2*q4,          2*q3;
//         0,           -4*q2,           -4*q3,             0;
//    -2*bz*q3,        2*bz*q4,       -4*bx*q3-2*bz*q1, -4*bx*q4+2*bz*q2;
//    -2*bx*q4+2*bz*q2, 2*bx*q3+2*bz*q1, 2*bx*q2+2*bz*q4, -2*bx*q1+2*bz*q3;
//     2*bx*q3,        2*bx*q4-4*bz*q2, 2*bx*q1-4*bz*q3, 2*bx*q2];

// Erwartet: q[0]=q0, q[1]=q1, q[2]=q2, q[3]=q3
//           bxz[0]=bx, bxz[1]=bz
// Ausgabe:  J[6][4] als Q15-Werte

static void get_jacobi_bigQ15(int16_t J[6][4], int16_t *q, int16_t *bxz)
{
    int16_t q0 = q[0];
    int16_t q1 = q[1];
    int16_t q2 = q[2];
    int16_t q3 = q[3];

    int16_t bx = bxz[0];
    int16_t bz = bxz[1];

    // Zeile 0
    J[0][0] = CLAMP_INT32_TO_INT16(-((int32_t)q3 << 1));              // -2*q3
    J[0][1] = CLAMP_INT32_TO_INT16( ((int32_t)q2 << 1));              //  2*q2
    J[0][2] = CLAMP_INT32_TO_INT16(-((int32_t)q0 << 1));              // -2*q0
    J[0][3] = CLAMP_INT32_TO_INT16( ((int32_t)q1 << 1));              //  2*q1

    // Zeile 1
    J[1][0] = CLAMP_INT32_TO_INT16( ((int32_t)q2 << 1));              //  2*q2
    J[1][1] = CLAMP_INT32_TO_INT16( ((int32_t)q0 << 1));              //  2*q0
    J[1][2] = CLAMP_INT32_TO_INT16( ((int32_t)q3 << 1));              //  2*q3
    J[1][3] = CLAMP_INT32_TO_INT16( ((int32_t)q1 << 1));              //  2*q1

    // Zeile 2
    J[2][0] = 0;
    J[2][1] = CLAMP_INT32_TO_INT16(-((int32_t)q1 << 2));              // -4*q1
    J[2][2] = CLAMP_INT32_TO_INT16(-((int32_t)q2 << 2));              // -4*q2
    J[2][3] = 0;

    // Zeile 3
    J[3][0] = CLAMP_INT32_TO_INT16(-(q15_mul(bz, q3) << 1));                                    // -2*bz*q3
    J[3][1] = CLAMP_INT32_TO_INT16( q15_mul(bz, q2) << 1);                                    //  2*bz*q2
    J[3][2] = CLAMP_INT32_TO_INT16((-((q15_mul(bx, q3) << 2)) - (q15_mul(bz, q0) << 1)));     // -4*bx*q3 - 2*bz*q0
    J[3][3] = CLAMP_INT32_TO_INT16((-((q15_mul(bx, q2) << 2)) + (q15_mul(bz, q1) << 1)));     // -4*bx*q2 + 2*bz*q1

    // Zeile 4
    J[4][0] = CLAMP_INT32_TO_INT16((-(q15_mul(bx, q3) << 1)) + (q15_mul(bz, q1) << 1));         // -2*bx*q3 + 2*bz*q1
    J[4][1] = CLAMP_INT32_TO_INT16(( q15_mul(bx, q2) << 1) + (q15_mul(bz, q0) << 1));         //  2*bx*q2 + 2*bz*q0
    J[4][2] = CLAMP_INT32_TO_INT16(( q15_mul(bx, q1) << 1) + (q15_mul(bz, q3) << 1));         //  2*bx*q1 + 2*bz*q3
    J[4][3] = CLAMP_INT32_TO_INT16((-(q15_mul(bx, q0) << 1)) + (q15_mul(bz, q2) << 1));         // -2*bx*q0 + 2*bz*q2

    // Zeile 5
    J[5][0] = CLAMP_INT32_TO_INT16( q15_mul(bx, q2) << 1);                                    // 2*bx*q2
    J[5][1] = CLAMP_INT32_TO_INT16(( q15_mul(bx, q3) << 1) - (q15_mul(bz, q1) << 2));         // 2*bx*q3 - 4*bz*q1
    J[5][2] = CLAMP_INT32_TO_INT16(( q15_mul(bx, q0) << 1) - (q15_mul(bz, q2) << 2));         // 2*bx*q0 - 4*bz*q2
    J[5][3] = CLAMP_INT32_TO_INT16( q15_mul(bx, q1) << 1);                                    // 2*bx*q1
}

static void compute_gradient(int16_t grad[4], int16_t J[3][4], int16_t f_error[3]) {
    for (uint8_t j = 0; j < 4; j++) {
        int32_t acc = 0;

        acc += (int32_t)J[0][j] * f_error[0]; // Q30
        acc += (int32_t)J[1][j] * f_error[1]; // Q30
        acc += (int32_t)J[2][j] * f_error[2]; // Q30

        // Rückskalieren auf Q15 (mit klassischer Rundung)
        int32_t result = (acc + (1 << 14)) >> 15; // Q30 → Q15

        grad[j] = CLAMP_INT32_TO_INT16(result);
    }
}
//grad = J' * f;
static void compute_gradient_bigQ15(int16_t *grad, int16_t J[6][4], int16_t *f_error)
{
    for (uint8_t j = 0; j < 4; j++) {
        int32_t acc = 0;

        for (uint8_t i = 0; i < 6; i++) {
            int32_t prod = ((int32_t)J[i][j] * (int32_t)f_error[i]) >> 3;  // Q30 → Q27 (because overrun)
            acc += prod;  // Q27
        }

        // Rückskalierung: Q27 → Q15 = >>12 mit Rundung
        int32_t result = (acc + (1 << 11)) >> 12;
        grad[j] = CLAMP_INT32_TO_INT16(result);
    }
}

static void get_bx_bz_q15(const int16_t *h, int16_t *bxz){
	uint32_t x = (uint32_t)((int32_t)h[1] * (int32_t)h[1]) + (uint32_t)((int32_t)h[2] * (int32_t)h[2]);
	bxz[0] = CLAMP_INT32_TO_INT16((int32_t)sqrt_fast_uint(x));
	bxz[1] = h[3];
}


static void gyro_drift_Q15(const int16_t *q_est, const int16_t *qDot, int16_t *q_drift){
	static int16_t delta_t = CLAMP_INT32_TO_INT16((int32_t)((GRAD2RAD_GYRO/(float)SENSOR_FUSION_FREQUENCY_IMU)*(float)Q15));
	int16_t q_est_con[4], q_rot[4];
	q_t_conj_function_in_out_q15(q_est,q_est_con);
	multiplicateQuaternionQ15(q_est_con,qDot,q_rot);
	multQuaternionWith2(q_rot);
	multQuatwithConstQ15(q_rot, delta_t);
	add2QuaternionQ15(q_rot, q_drift, q_drift);
	NormalizeQuaternionQ15(q_drift, q_drift);
}


static void iir_filter_bx_bz_q15(int16_t *bxz){
	static int16_t bxz_old[2] = {0,0};

	/*
	 * pre calculation of filter parameter with fc = 10 Hz
	 * 	-> a = (2*pi*Ts*fc)/(2*pi*Ts*fc + 1)
	 */
	static int16_t a_xz = CLAMP_INT32_TO_INT16((int32_t)((float)(PI_MULTIPLY_2 * (1.0f/(float)SENSOR_FUSION_FREQUENCY_MAG)*10)/(PI_MULTIPLY_2 * (1.0f/(float)SENSOR_FUSION_FREQUENCY_MAG)*10 + 1.0f) * (float)Q15));
	static int16_t one_minus_a_xz = (Q15 - CLAMP_INT32_TO_INT16((int32_t)((float)(PI_MULTIPLY_2 * (1.0f/(float)SENSOR_FUSION_FREQUENCY_MAG)*10)/(PI_MULTIPLY_2 * (1.0f/(float)SENSOR_FUSION_FREQUENCY_MAG)*10 + 1.0f) * (float)Q15)));

	int32_t x; // help variable

	x = (int32_t)q15_mul(bxz[0], a_xz) + (int32_t)q15_mul(bxz_old[0], one_minus_a_xz);
	bxz[0] = bxz_old[0] = CLAMP_INT32_TO_INT16(x);

	x = (int32_t)q15_mul(bxz[1], a_xz) + (int32_t)q15_mul(bxz_old[1], one_minus_a_xz);
	bxz[1] = bxz_old[1] = CLAMP_INT32_TO_INT16(x);

	norm_2d_vector_q15(bxz);

}




// ######## END STATIC FUNCTIONS ############

// ######## TEST #########

#define NUM_TESTS 5

typedef struct {
    int16_t q[4];            // Q15
    int16_t accel[3];        // Q15
    int16_t accel_norm[3];   // Q15
    int16_t f_error[3];      // Q15
    int16_t J[3][4];         // Q15
    int16_t grad[4];         // Q15
} TestResultQ15;

TestResultQ15 test_results[NUM_TESTS];

void run_test_vectors(void) {

	int16_t J[6][4] = {
	    { 20625, -14517,  29960,  19150 },
	    { 26593,   3072,   -959,  30113 },
	    {-24446,  29983,  19679,  10206 },
	    { 27091,  30466, -23470, -30428 },
	    {  8674, -22439,  -5128,  22880 },
	    {-26376,  30840,  27245,  28442 }
	};

	int16_t f_error[6] = {
	    11713,
	    16891,
	    15933,
	    -7064,
	    10189,
	   -21550
	};

	int16_t grad[4];

	compute_gradient_bigQ15(grad, J, f_error);

	HAL_Delay(1);


}


static void Complementary_filter_acc_gyro_mag(int16_t beta, sensor_fusion *pHandle_sf){
	int16_t accel[3], accel_norm[3], gyro[3],mag_raw[3], mag_equalized[3], mag_norm[3],
			f_error[6], grad[4], grad_norm[4], omega[4], mag_quat[4], qDot[4], h[4],q_mul_w[4], q_con[4],
			bxz[2];
	int16_t J[6][4];

	//get values

	accel[0] = pHandle_sf->acc_t.x;
	accel[1] = pHandle_sf->acc_t.y;
	accel[2] = pHandle_sf->acc_t.z;
	gyro[0] = pHandle_sf->gyro_t.x;
	gyro[1] = pHandle_sf->gyro_t.y;
	gyro[2] = pHandle_sf->gyro_t.z;

	mag_raw[0] = pHandle_sf->mag_t.x;
	mag_raw[1] = pHandle_sf->mag_t.y;
	mag_raw[2] = pHandle_sf->mag_t.z; // raw z-axis point up

	omega[0] = 0;
//	omega[1] = debug_omega.x = q15_mul(gyro[0],GYRO_GRAD_TO_RAD_Q15);
//	omega[2] = debug_omega.y = q15_mul(gyro[1],GYRO_GRAD_TO_RAD_Q15);
//	omega[3] = debug_omega.z = q15_mul(gyro[2],GYRO_GRAD_TO_RAD_Q15);

	omega[1] = gyro[0];
	omega[2] = gyro[1];
	omega[3] = gyro[2];


	hardiron_apply_q15(mag_raw);
	debug_mag_hard_iron.x = mag_raw[0];
	debug_mag_hard_iron.y = mag_raw[1];
	debug_mag_hard_iron.z = mag_raw[2];
	softiron_apply_q15(mag_raw, mag_equalized);
	debug_mag_soft.x = mag_equalized[0];
	debug_mag_soft.y = mag_equalized[1];
	debug_mag_soft.z = mag_equalized[2];
	norm_3d_vector(accel, accel_norm);
	norm_3d_vector(mag_equalized, mag_norm);


	debug_mag_norm.x = mag_norm[1];
	debug_mag_norm.y = mag_norm[0];
	debug_mag_norm.z = mag_norm[2];

	mag_quat[0] = 0;
	mag_quat[1] = mag_norm[1];
	mag_quat[2] = mag_norm[0];
	mag_quat[3] = mag_norm[2];

	multiplicateQuaternionQ15(q_compl_out,omega,qDot); // getestet und funkitoniert ✅
	divideQuaternionBy2(qDot); // getestet und funkitoniert ✅




//	q_compl_out
}





