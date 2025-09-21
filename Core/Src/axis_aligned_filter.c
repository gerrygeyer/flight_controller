/*
 * axis_aligned_filter.c
 *
 *  Created on: Sep 18, 2025
 *      Author: gerrygeyer
 */

#include <axis_aligned_filter.h>
#include <main.h>
#include <sys_math.h>
#include <settings.h>
#include <mpu600.h>
#include "lis3mdl.h"

static inline void divideQuaternionBy2(int16_t *q);
static inline void copy_quaternion(const int16_t *q_original, int16_t *q_copy);
static void iir_filter_acc_Q15(const int16_t *acc_raw, int16_t *acc_filter, const float fc, const float Frequency);
static void butterworth_lp20Hz_fs1k_Q30(const int16_t *acc_q15, int16_t *acc_filt_q15);
static void int_gyro_dot_Q15(const int16_t *q_est, const int16_t *q_dot, const int16_t delta_t, int16_t *q_int);

int16_t q_est[4], delta_t;
butterworth_coefficients bw;
adapt_coefficients beta;

float beta_max_f = 0.1;
float beta_min_f = 0.0;
float beta_c_f = 0.01;

void axis_aligned_init(void){
	q_est[0] = Q15;	q_est[1] = 0;	q_est[2] = 0;	q_est[3] = 0;
	delta_t = CLAMP_INT32_TO_INT16((int32_t)((float)(1.0f/(float)SENSOR_FUSION_FREQ) * (float)Q15));

	bw.a1_Q30 = -1957103774; 	// -1.82269493 * 2^30
	bw.a2_Q30 = 898916953; 		//  0.83718165 * 2^30
	bw.b0_Q30 = 3888751;   		// 0.00362168 * 2^30
	bw.b1_Q30 = 7777502;   		// 0.00724336 * 2^30
	bw.b2_Q30 = 3888751;   		// 0.00362168 * 2^30
}



xyz_16t debug_acc_filter,debug_euler_filter;
wxyz_16t q_out_debug,q_debug_acc_internal;
void axis_aligned_filter(sensor_fusion *pHandle_sf, const bool acc_on,const bool mag_on){
	int16_t acc[3], acc_norm[3], acc_filter[3], gyro[3],qDot[4],q_gyro[4],omega[4], mag_raw[3], mag_equalized[3], mag_norm[3],mag_q[3];
	int16_t q_acc_inertial[4], q_acc_inert_conj[4], q_err[4], q_est_[4],d;
	const int16_t q_0[4] = {Q15, 0,0,0};
	const int16_t gI[3] ={0,0,Q15};
	int32_t x;
//	static int16_t delta_t = CLAMP_INT32_TO_INT16((int32_t)((float)(34.9f/(float)SENSOR_FUSION_FREQUENCY_IMU) * (float)Q15));

	//get values

	acc[0] = pHandle_sf->acc_t.x;	acc[1] = pHandle_sf->acc_t.y;	acc[2] = pHandle_sf->acc_t.z;

	omega[0] = 0;
	omega[1] = gyro[0] = q15_mul(pHandle_sf->gyro_t.x, GRAD2RAD_GYRO_MAX_Q15);
	omega[2] = gyro[1] = q15_mul(pHandle_sf->gyro_t.y, GRAD2RAD_GYRO_MAX_Q15);
	omega[3] = gyro[2] = q15_mul(pHandle_sf->gyro_t.z, GRAD2RAD_GYRO_MAX_Q15);

	mag_raw[0] = pHandle_sf->mag_t.x;	mag_raw[1] = pHandle_sf->mag_t.y;	mag_raw[2] = pHandle_sf->mag_t.z; // raw z-axis point up

	hardiron_apply_q15(mag_raw);	softiron_apply_q15(mag_raw, mag_equalized);

	if(PREFILTER_SF){

		butterworth_lp20Hz_fs1k_Q30(acc,acc);
	}

	debug_acc_filter.x = acc_filter[0];
	debug_acc_filter.y = acc_filter[1];
	debug_acc_filter.z = acc_filter[2];

	norm_3d_vector(acc, acc_norm);
	norm_3d_vector(mag_equalized, mag_norm);


	mag_q[0] = 0;
	mag_q[1] = mag_norm[1];
	mag_q[2] = mag_norm[0];
	mag_q[3] = mag_norm[2];

	multiplicateQuaternionQ15(q_est,omega,qDot);
	divideQuaternionBy2(qDot);
	int_gyro_dot_Q15(q_est, qDot,delta_t,q_gyro);


//	minimal_rotation(acc_norm, gI,q_acc_inertial);
	minimal_rotation(gI,acc_norm,q_acc_inertial);
//	if(q_acc_inertial[0] < 0) q_t_flipp(q_acc_inertial);
	q_debug_acc_internal.w = q_acc_inertial[0];	q_debug_acc_internal.x = q_acc_inertial[1];
	q_debug_acc_internal.y = q_acc_inertial[2];	q_debug_acc_internal.z = q_acc_inertial[3];

	// ######## ADAPTIVE PART ###########
	q_t_conj_function_in_out_q15(q_acc_inertial,q_acc_inert_conj);
	dotporduct_4x4_Q15(q_acc_inertial,q_gyro,&d);
	if(d < 0){
		q_t_flipp(q_gyro);
	}
	multiplicateQuaternionQ15(q_acc_inert_conj, q_gyro, q_err);
//	if(q_err[0] < 0) q_t_flipp(q_est_);
	beta.theta = CLAMP_INT32_TO_INT16((int32_t)q15_acos(q_err[0])); // (pi, -pi) = (Q15, -Q15)

	beta.max = CLAMP_INT32_TO_INT16((int32_t)((float)Q15 * beta_max_f));
	beta.min = CLAMP_INT32_TO_INT16((int32_t)((float)Q15 * beta_min_f));
	beta.c = CLAMP_INT32_TO_INT16((int32_t)((float)Q15 * beta_c_f));

	x = ((int32_t)beta.theta << 15)/((int32_t)beta.theta + beta.c);
	x = beta.min + Q15_SHIFT_ROUND((beta.max - beta.min) * x);
	beta.result = CLAMP_INT32_TO_INT16(x);
	// ######## END ADAPTIVE PART ########
//	if(q_gyro[0] < 0) q_t_flipp(q_gyro);
//	if(q_acc_inertial[0] < 0) q_t_flipp(q_acc_inertial);
	nLERP_quaternion_Q15(q_gyro, q_acc_inertial, beta.result,q_est_);
//	if(q_est_[0] < 0) q_t_flipp(q_est_);
	copy_quaternion(q_est_, q_est);


	int16_t euler_debug[3];
	quat_to_euler_q15(q_est, euler_debug);

	debug_euler_filter.x = euler_debug[0];
	debug_euler_filter.y = euler_debug[1];
	debug_euler_filter.z = euler_debug[2];

	q_out_debug.w = q_est[0];
	q_out_debug.x = q_est[1];
	q_out_debug.y = q_est[2];
	q_out_debug.z = q_est[3];


}
static inline void divideQuaternionBy2(int16_t *q) {
    q[0] >>= 1;	q[1] >>= 1;	q[2] >>= 1;	q[3] >>= 1;
}

static inline void copy_quaternion(const int16_t *q_original, int16_t *q_copy){
	q_copy[0] = q_original[0];	q_copy[1] = q_original[1];	q_copy[2] = q_original[2];	q_copy[3] = q_original[3];
}


static void iir_filter_acc_Q15(const int16_t *acc_raw, int16_t *acc_filter, const float fc, const float Frequency){
	static bool init_flag = 0;
	static int32_t a;
	static int32_t acc_old[3] = {0};

	if (!init_flag){
		a = CLAMP_INT32_TO_INT16((int32_t)((PI_MULTIPLY_2 * (float)fc * (1.0f/(float)Frequency))/(PI_MULTIPLY_2 * (float)fc * (1.0f/(float)Frequency) + 1.0f)));
		init_flag = true;
	}

	for(uint8_t i = 0; i<3;i++){
		acc_old[i] = ((a * (int32_t)acc_raw[i]) >> 15) + ((((int32_t)Q15 - a) * acc_old[i]) >> 15);
		acc_filter[i] = acc_old[i] = CLAMP_INT32_TO_INT16(acc_old[i]);
	}
}

static void int_gyro_dot_Q15(const int16_t *q_est, const int16_t *q_dot, const int16_t delta_t, int16_t *q_int){

	q_int[0] = CLAMP_INT32_TO_INT16((int32_t)q15_mul(q_dot[0],delta_t) + q_est[0]);
	q_int[1] = CLAMP_INT32_TO_INT16((int32_t)q15_mul(q_dot[1],delta_t) + q_est[1]);
	q_int[2] = CLAMP_INT32_TO_INT16((int32_t)q15_mul(q_dot[2],delta_t) + q_est[2]);
	q_int[3] = CLAMP_INT32_TO_INT16((int32_t)q15_mul(q_dot[3],delta_t) + q_est[3]);

	NormalizeQuaternionQ15(q_int, q_int);
}

static void butterworth_lp20Hz_fs1k_Q30(const int16_t *acc_q15, int16_t *acc_filt_q15){
	static int32_t x1[3] = {0}, x2[3] = {0}; // Q30
	    static int32_t y1[3] = {0}, y2[3] = {0}; // Q30

	    for(int i=0;i<3;i++){
	        int32_t x0 = CLAMP(((acc_q15[i]) << 15),-Q30,Q30); // Q30

	        // y = b0*x0 + b1*x1 + b2*x2 - a1*y1 - a2*y2   (alles Q30)
	        int64_t y  = Q30_MUL(bw.b0_Q30, x0);
	        y         += Q30_MUL(bw.b1_Q30, x1[i]);
	        y         += Q30_MUL(bw.b2_Q30, x2[i]);
	        y         -= Q30_MUL(bw.a1_Q30, y1[i]); // a1 ist negativ -> korrektes Vorzeichen durch "-"
	        y         -= Q30_MUL(bw.a2_Q30, y2[i]);

	        // Output Q15 (mit Rundung) + Clamp
	        acc_filt_q15[i] = CLAMP_INT32_TO_INT16(Q15_SHIFT_ROUND(y));

	        // Zustände schieben (alles Q30)
	        x2[i] = x1[i]; x1[i] = x0;
	        y2[i] = y1[i]; y1[i] = (int32_t)y; // y in Q30
	    }
}
