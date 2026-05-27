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
#include <stdlib.h>
#include "lis3mdl.h"

static inline void divideQuaternionBy2(int16_t *q);
static inline void copy_quaternion(const int16_t *q_original, int16_t *q_copy);
static void iir_filter_acc_Q15(const int16_t *acc_raw, int16_t *acc_filter, const float fc, const float Frequency);
static void butterworth_lp20Hz_fs1k_Q30(const int16_t *acc_q15, int16_t *acc_filt_q15);
static void int_gyro_dot_Q15(const int16_t *q_est, const int16_t *q_dot, const int16_t delta_t, int16_t *q_int);
static void iir_filter_gyro_Q15(const int16_t *gyro_raw, int16_t *gyro_filter, const float fc, const float Frequency);
// VQF
static void vqf_butterworth_lp20Hz_fs1k_Q30(const int16_t *acc_q15, int16_t *acc_filt_q15);

int16_t q_est[4], delta_t;
butterworth_coefficients bw;
adapt_coefficients beta;



//float beta_max_f = 0.2;
float beta_max_f = 0.2;
float beta_min_f = 0.0;
//float beta_c_f = 0.01;
float beta_c_f = 0.001;

void axis_aligned_init(void){
	q_est[0] = Q15;	q_est[1] = 0;	q_est[2] = 0;	q_est[3] = 0;
	delta_t = CLAMP_INT32_TO_INT16((int32_t)((float)((1.0f * (float)Q4)/(float)SENSOR_FUSION_FREQ) * (float)Q15));
//	delta_t = CLAMP_INT32_TO_INT16((int32_t)((float)((1.0f)/(float)SENSOR_FUSION_FREQ) * (float)Q15));
	bw.a1_Q30 = -1957103774; 	// -1.82269493 * 2^30
	bw.a2_Q30 = 898916953; 		//  0.83718165 * 2^30
	bw.b0_Q30 = 3888751;   		// 0.00362168 * 2^30
	bw.b1_Q30 = 7777502;   		// 0.00724336 * 2^30
	bw.b2_Q30 = 3888751;   		// 0.00362168 * 2^30

	beta.max = CLAMP_INT32_TO_INT16((int32_t)((float)Q15 * beta_max_f));
	beta.min = CLAMP_INT32_TO_INT16((int32_t)((float)Q15 * beta_min_f));
	beta.c = CLAMP_INT32_TO_INT16((int32_t)((float)Q15 * beta_c_f));

}


int16_t debug_err;
xyz_16t debug_acc_filter,debug_euler_axis_aligned_filter, debug_filter_gyro;
wxyz_16t q_out_debug,q_debug_acc_internal, q_debug_gyro, debug_q_ax_al_gyro;
bool calibrate_encoder = 0;
void axis_aligned_filter(sensor_fusion *pHandle_sf, int16_t *q_out,const bool acc_on,const bool mag_on){
	int16_t acc[3], acc_norm[3], acc_filter[3], gyro[3],qDot[4],q_gyro[4],omega[4], mag_raw[3], mag_equalized[3], mag_norm[3];
	int16_t q_acc_inertial[4], q_acc_inert_conj[4], q_err[4], q_est_[4],d;
	const int16_t gI[3] ={0,0,Q15};
	int32_t x;
//	static int16_t delta_t = CLAMP_INT32_TO_INT16((int32_t)((float)(34.9f/(float)SENSOR_FUSION_FREQUENCY_IMU) * (float)Q15));

	//get values

	acc[0] = pHandle_sf->acc_t.x;	acc[1] = pHandle_sf->acc_t.y;	acc[2] = pHandle_sf->acc_t.z;


//	gyro[0] = pHandle_sf->gyro_t.x;
//	gyro[1] = pHandle_sf->gyro_t.y;
//	gyro[2] = pHandle_sf->gyro_t.z;

	gyro[0] = CLAMP_INT32_TO_INT16(q15_mul(pHandle_sf->gyro_t.x, GRAD2RAD_GYRO_MAX_VALUE_Q15));
	gyro[1] = CLAMP_INT32_TO_INT16(q15_mul(pHandle_sf->gyro_t.y, GRAD2RAD_GYRO_MAX_VALUE_Q15));
	gyro[2] = CLAMP_INT32_TO_INT16(q15_mul(pHandle_sf->gyro_t.z, GRAD2RAD_GYRO_MAX_VALUE_Q15));

//	int16_t max_gyro = MAX3(abs(acc[0]),abs(acc[1]),0);
//	gyro[0] = q15_mul(gyro[0],934);
//	gyro[1] = q15_mul(gyro[1],943);
//	gyro[2] = q15_mul(gyro[2],934);

	if(PREFILTER_SF){

//		if(PREF_METHODE == BUTTERWORTH)	butterworth_lp20Hz_fs1k_Q30(acc,acc);
//		if(PREF_METHODE == IIR)			iir_filter_acc_Q15(acc,acc,24.0f,1000.0f);
	}
//	iir_filter_gyro_Q15(gyro,gyro,24.0f,1000.0f);

	omega[0] = 0;
	omega[1] = gyro[0];// = CLAMP_INT32_TO_INT16(q15_mul(pHandle_sf->gyro_t.x, GRAD2RAD_GYRO_MAX_VALUE_Q15) << 1);
	omega[2] = gyro[1];// = CLAMP_INT32_TO_INT16(q15_mul(pHandle_sf->gyro_t.y, GRAD2RAD_GYRO_MAX_VALUE_Q15) << 1);
	omega[3] = gyro[2];
//	butterworth_lp20Hz_fs1k_Q30(acc,acc); // debug
//	iir_filter_acc_Q15(acc,acc,24.0f,1000.0f);



	norm_3d_vector(acc, acc_norm);



	multiplicateQuaternionQ15(q_est,omega,qDot);
//	divideQuaternionBy2(qDot);
	int_gyro_dot_Q15(q_est, qDot,delta_t,q_gyro);



// ####### REMOVE YAW ########
//	remove_yaw_component_q15(q_gyro,q_gyro);
	debug_q_ax_al_gyro.w = q_gyro[0];
	debug_q_ax_al_gyro.x = q_gyro[1];
	debug_q_ax_al_gyro.y = q_gyro[2];
	debug_q_ax_al_gyro.z = q_gyro[3];
// ###########################
//	minimal_rotation(acc_norm, gI,q_acc_inertial);
	minimal_rotation(gI,acc_norm,q_acc_inertial);
	q_t_conj_function(q_acc_inertial);
//	if(q_acc_inertial[0] < 0) q_t_flipp(q_acc_inertial);
	q_debug_acc_internal.w = q_acc_inertial[0];	q_debug_acc_internal.x = q_acc_inertial[1];
	q_debug_acc_internal.y = q_acc_inertial[2];	q_debug_acc_internal.z = q_acc_inertial[3];

	if(mag_on){
		mag_raw[0] = pHandle_sf->mag_t.x;	mag_raw[1] = pHandle_sf->mag_t.y;	mag_raw[2] = pHandle_sf->mag_t.z; // raw z-axis point up

		hardiron_apply_q15(mag_raw);	softiron_apply_q15(mag_raw, mag_equalized);
		norm_3d_vector(mag_equalized, mag_norm);

	}
//	remove_yaw_component_q15(q_acc_inertial,q_acc_inertial);
	// ######## ADAPTIVE PART ###########
	q_t_conj_function_in_out_q15(q_acc_inertial,q_acc_inert_conj);
	dotporduct_4x4_Q15(q_acc_inertial,q_gyro,&d);
	if(d < 0) q_t_flipp(q_gyro);
	multiplicateQuaternionQ15(q_acc_inert_conj, q_gyro, q_err);
//	if(q_err[0] < 0) q_t_flipp(q_est_);
	beta.theta = CLAMP_INT32_TO_INT16((int32_t)q15_acos(q_err[0]) << 1); // (pi, -pi) = (Q15, -Q15)


	if(DEBUG_MODE == ON){
		beta.max = CLAMP_INT32_TO_INT16((int32_t)((float)Q15 * beta_max_f));
		beta.min = CLAMP_INT32_TO_INT16((int32_t)((float)Q15 * beta_min_f));
		beta.c = CLAMP_INT32_TO_INT16((int32_t)((float)Q15 * beta_c_f));
	}
	x = debug_err =((int32_t)beta.theta << 15)/((int32_t)beta.theta + beta.c);
	x = beta.min + Q15_SHIFT_ROUND((beta.max - beta.min) * x);
	beta.result = CLAMP_INT32_TO_INT16(x);
	// ######## END ADAPTIVE PART ########
//	if(q_gyro[0] < 0) q_t_flipp(q_gyro);
//	if(q_acc_inertial[0] < 0) q_t_flipp(q_acc_inertial);




	int16_t euler_debug[3], q_output_angle[4];
	if(1){

//	q_t_conj_function(q_acc_inertial);
	nLERP_quaternion_Q15(q_gyro, q_acc_inertial, beta.result,q_est_);
//	if(q_est_[0] < 0) q_t_flipp(q_est_);
//	q_t_conj_function(q_est_);
	copy_quaternion(q_est_, q_est);
//	q_t_conj_function_in_out_q15(q_est_,q_output_angle);
	copy_quaternion(q_est, q_output_angle);
	}else{
//		q_t_conj_function(q_gyro);
		copy_quaternion(q_gyro, q_est);
//		q_t_conj_function_in_out_q15(q_est,q_output_angle);
		copy_quaternion(q_est, q_output_angle);
	}



//	q_t_conj_function_in_out_q15(q_output_angle,q_output_angle);
	if(DEBUG_MODE == ON){
	quat_to_euler_q15(q_output_angle, euler_debug);

	debug_euler_axis_aligned_filter.x = euler_debug[0];
	debug_euler_axis_aligned_filter.y = euler_debug[1];
	debug_euler_axis_aligned_filter.z = euler_debug[2];

	q_out_debug.w = q_est[0];
	q_out_debug.x = q_est[1];
	q_out_debug.y = q_est[2];
	q_out_debug.z = q_est[3];
	}

	copy_quaternion(q_output_angle, q_out);

//	if(calibrate_encoder){
//		calibrate_encoder = 0;
//		set_encoder_to_value(debug_euler_filter.x);
//	}
	// write, if this are the only filter
	if(SENSORFUSION_METHODE == SF_AXIS_ALIGN){
		pHandle_sf->quaternion.w = q_est[0];
		pHandle_sf->quaternion.x = q_est[1];
		pHandle_sf->quaternion.y = q_est[2];
		pHandle_sf->quaternion.z = q_est[3];
	}


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
static void iir_filter_gyro_Q15(const int16_t *gyro_raw, int16_t *gyro_filter, const float fc, const float Frequency){
	static bool init_flag = 0;
	static int32_t a;
	static int32_t gyro_old[3] = {0};

	if (!init_flag){
		a = CLAMP_INT32_TO_INT16((int32_t)((PI_MULTIPLY_2 * (float)fc * (1.0f/(float)Frequency))/(PI_MULTIPLY_2 * (float)fc * (1.0f/(float)Frequency) + 1.0f)));
		init_flag = true;
	}

	for(uint8_t i = 0; i<3;i++){
		gyro_old[i] = ((a * (int32_t)gyro_raw[i]) >> 15) + ((((int32_t)Q15 - a) * gyro_old[i]) >> 15);
		gyro_filter[i] = gyro_old[i] = CLAMP_INT32_TO_INT16(gyro_old[i]);
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




// ############ VQF ############
wxyz_16t debug_q_vqf_3;
int16_t q_est_vqf[4], delta_vqf_t, vqf_k_mag;
butterworth_coefficients bw;
float vqf_mag_corr = 6.0f;
void vqf_init(void){
	q_est_vqf[0] = Q15;	q_est_vqf[1] = 0;	q_est_vqf[2] = 0;	q_est_vqf[3] = 0;
	delta_vqf_t = CLAMP_INT32_TO_INT16((int32_t)((float)((1.0f * (float)Q4)/(float)SENSOR_FUSION_FREQ) * (float)Q15));
	vqf_k_mag = CLAMP_INT32_TO_INT16((int32_t)((float)((vqf_mag_corr)/(float)SENSOR_FUSION_FREQ) * (float)Q15));

	bw.a1_Q30 = -1957103774; 	// -1.82269493 * 2^30
	bw.a2_Q30 = 898916953; 		//  0.83718165 * 2^30
	bw.b0_Q30 = 3888751;   		// 0.00362168 * 2^30
	bw.b1_Q30 = 7777502;   		// 0.00724336 * 2^30
	bw.b2_Q30 = 3888751;   		// 0.00362168 * 2^30
}

wxyz_16t debug_vqf_q_out, debug_vqf_q_wxyz;
xyz_16t debug_vqf_acc_b,debug_vqf_acc_internal, debug_vqf_euler_3;
void vqf_filter(const sensor_fusion *pHandle_sf,int16_t *q_out, const bool mag_on, uint8_t reset){
	int16_t acc[3],acc_internal[3],acc_error_frame[3], gyro[3],qDot[4],omega[4], mag_raw[3], mag_equalized[3], mag_norm[3];
	int16_t q_est_vqf_conj[4],q_err2sens[4], q_wxyz[4];


	static int16_t q_acc_corr[4] = {Q15,0,0,0};
	static int16_t theta = 0;
	static int16_t q_mag_corr[4] ={Q15,0,0,0};

	if(reset > 0){
		q_acc_corr[0] = Q15;	q_acc_corr[1] = 0;	q_acc_corr[2] = 0;	q_acc_corr[3] = 0;
		q_mag_corr[0] = Q15;	q_mag_corr[1] = 0;	q_mag_corr[2] = 0;	q_mag_corr[3] = 0;
		theta = 0;
	}
	if(DEBUG_MODE == ON){
		vqf_k_mag = CLAMP_INT32_TO_INT16((int32_t)((float)((vqf_mag_corr)/(float)SENSOR_FUSION_FREQ) * (float)Q15));
	}


	acc[0] = pHandle_sf->acc_t.x;	acc[1] = pHandle_sf->acc_t.y;	acc[2] = pHandle_sf->acc_t.z;


	gyro[0] = CLAMP_INT32_TO_INT16(q15_mul(pHandle_sf->gyro_t.x, GRAD2RAD_GYRO_MAX_VALUE_Q15));
	gyro[1] = CLAMP_INT32_TO_INT16(q15_mul(pHandle_sf->gyro_t.y, GRAD2RAD_GYRO_MAX_VALUE_Q15));
	gyro[2] = CLAMP_INT32_TO_INT16(q15_mul(pHandle_sf->gyro_t.z, GRAD2RAD_GYRO_MAX_VALUE_Q15));

	mag_raw[0] = pHandle_sf->mag_t.x;	mag_raw[1] = pHandle_sf->mag_t.y;	mag_raw[2] = pHandle_sf->mag_t.z; // raw z-axis point up



	if(PREFILTER_SF){

//		if(PREF_METHODE == BUTTERWORTH)	butterworth_lp20Hz_fs1k_Q30(acc,acc);
//		if(PREF_METHODE == IIR)			iir_filter_acc_Q15(acc,acc,24.0f,1000.0f);
	}

	omega[0] = 0;	omega[1] = gyro[0];	omega[2] = gyro[1];	omega[3] = gyro[2];


	multiplicateQuaternionQ15(q_est_vqf,omega,qDot);
//	divideQuaternionBy2(qDot); // -> compensate this with delta_vqf_t -> Q4 instead Q5
	int_gyro_dot_Q15(q_est_vqf, qDot,delta_vqf_t,q_est_vqf);					// STEP 8

	// ###### DEBUG SECTION ########
	if(EULER_OUTPUT == ON){
		debug_q_vqf_3.w = q_est_vqf[0];
		debug_q_vqf_3.x = q_est_vqf[1];
		debug_q_vqf_3.y = q_est_vqf[2];
		debug_q_vqf_3.z = q_est_vqf[3];

		int16_t euler[3];
		quat_to_euler_q15(q_est_vqf,euler);
		debug_vqf_euler_3.x = euler[0];	debug_vqf_euler_3.y = euler[1];	debug_vqf_euler_3.z = euler[2];
	}
	// ####### DEBUG END #########

	rotate_vector_Q15(q_est_vqf,acc,acc_internal);
	if(PREFILTER_SF == ON) vqf_butterworth_lp20Hz_fs1k_Q30(acc_internal,acc_internal); 				// STEP 10
	debug_vqf_acc_b.x = acc_internal[0];	debug_vqf_acc_b.y = acc_internal[1];	debug_vqf_acc_b.z = acc_internal[2];
	rotate_vector_Q15(q_acc_corr,acc_internal,acc_error_frame); 				// STEP 11
	debug_vqf_acc_internal.x = acc_error_frame[0];	debug_vqf_acc_internal.y = acc_error_frame[1];	debug_vqf_acc_internal.z = acc_error_frame[2];
	norm_3d_vector(acc_error_frame, acc_error_frame); 							// STEP 12

	uint32_t x_u = CLAMP((((int32_t)acc_error_frame[2] + Q15) >> 1),0,Q30);
	q_wxyz[0] = sqrt_fast_uint((x_u << 15));											// STEP 13
	if(abs(acc_error_frame[0]) < 400 && abs(acc_error_frame[1]) < 400){
		q_wxyz[0] = Q15;
		q_wxyz[1] = Q1_SHIFT_ROUND(acc_error_frame[1]);
		q_wxyz[2] = Q1_SHIFT_ROUND(-acc_error_frame[0]);
		q_wxyz[3] = 0;
	}else{
		q_wxyz[1] = CLAMP_INT32_TO_INT16(((int32_t)acc_error_frame[1] << 14)/q_wxyz[0]);  // Q15 * (ay/2) / q0 -> Q14 * ay / q0
		q_wxyz[2] = CLAMP_INT32_TO_INT16(-((int32_t)acc_error_frame[0] << 14)/q_wxyz[0]);
		q_wxyz[3] = 0;
	}
		NormalizeQuaternionQ15(q_wxyz, q_wxyz);
		debug_vqf_q_wxyz.w = q_wxyz[0];
		debug_vqf_q_wxyz.x = q_wxyz[1];
		debug_vqf_q_wxyz.y = q_wxyz[2];
		debug_vqf_q_wxyz.z = q_wxyz[3];
		int16_t d;
		dotporduct_4x4_Q15(q_wxyz, q_acc_corr,&d);
		if (d < 0) {
		    q_wxyz[0] = -q_wxyz[0];
		    q_wxyz[1] = -q_wxyz[1];
		    q_wxyz[2] = -q_wxyz[2];
		    q_wxyz[3] = -q_wxyz[3];
		}


	multiplicateQuaternionQ15(q_wxyz, q_acc_corr, q_acc_corr);					// STEP 14
	multiplicateQuaternionQ15(q_acc_corr,q_est_vqf,q_err2sens);			// STEP 15

	if(mag_on){

//	hardiron_apply_q15(mag_raw);	softiron_apply_q15(mag_raw, mag_equalized);
//	norm_3d_vector(mag_equalized, mag_norm);
//
//	int16_t m_b[3];
//	rotate_vector_Q15(q_err2sens,mag_norm,m_b);					// STEP 17
//	theta = theta + q15_mul(vqf_k_mag, q15_atan2(m_b[0], m_b[1]));
//	q_mag_corr[0] = cos_Q15(theta>>1);
//	q_mag_corr[3] = sin_Q15(theta>>1);
//	NormalizeQuaternionQ15(q_mag_corr, q_mag_corr);
//	multiplicateQuaternionQ15(q_mag_corr, q_err2sens, q_err2sens);
	}

	NormalizeQuaternionQ15(q_acc_corr, q_acc_corr);
	NormalizeQuaternionQ15(q_err2sens, q_err2sens);

	if(q_err2sens[0]<0) q_t_flipp(q_err2sens);
	copy_quaternion(q_err2sens, q_out);

	// debug
	debug_vqf_q_out.w = q_out[0];	debug_vqf_q_out.x = q_out[1];
	debug_vqf_q_out.y = q_out[2];	debug_vqf_q_out.z = q_out[3];

}



static void vqf_butterworth_lp20Hz_fs1k_Q30(const int16_t *acc_q15, int16_t *acc_filt_q15){
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



// ############# MAHONY FILTER #############

static void quaternion_from_vector(const int16_t *v, int16_t *q){
	q[0] = 0; q[1] = v[0];	q[2] = v[1]; q[3] = v[2];
}
static void vector_from_quaternion(const int16_t *q, int16_t *v){
	v[0] = q[1];	v[1] = q[2];	v[2] = q[3];
}


float mahony_Ki_f = 0.01f;
float mahony_Kp_f = 1.5f;
int16_t mahony_Ki_q15;
int16_t mahony_delta_t,mahony_Kp_q15,mahony_delta_error_t;
int64_t mahony_int_x,mahony_int_y,mahony_int_z;
void mahony_filter_init(void){
	mahony_Ki_q15 = (int32_t)((float)Q10 * mahony_Ki_f); // here the 1/Q5 [rad/s] are included
	mahony_Kp_q15 = CLAMP_INT32_TO_INT16((int32_t)((float)Q10 * mahony_Kp_f)); // here the 1/Q5 [rad/s] are included
	mahony_delta_t = CLAMP_INT32_TO_INT16((int32_t)((float)((1.0f * (float)Q4)/(float)SENSOR_FUSION_FREQ) * (float)Q15));
	mahony_delta_error_t =  CLAMP_INT32_TO_INT16((int32_t)((float)((1.0f /(float)SENSOR_FUSION_FREQ) * (float)Q15)));
}
xyz_16t debug_mahony_vg, debug_mahony_error;
void mahony_filter(const sensor_fusion *pHandle_sf,int16_t *q_out, const bool mag_on, uint8_t reset){
	int16_t v_g[3], acc[3],acc_norm[3], acc_error[3], gyro[3],qDot[4],omega[4], mag_raw[3], mag_equalized[3], mag_norm[3],e[3];
	int16_t eIntq15[3], omega_corr[4];
	const int16_t gI[3] = {0,0,-Q15};

	static int16_t q_est[4] = {Q15,0,0,0};
	static int64_t eInt[3] = {0,0,0};

	if(reset >0){
		q_est[0] = Q15;	q_est[1] = 0;	q_est[2] = 0;	q_est[3] = 0;
		eInt[0] = 0;	eInt[1] = 0;	eInt[2] = 0;
	}
	if(DEBUG_MODE == ON){
		mahony_Ki_q15 = (int32_t)((float)Q10 * mahony_Ki_f); // here the 1/Q5 [rad/s] are included
		mahony_Kp_q15 = CLAMP_INT32_TO_INT16((int32_t)((float)Q10 * mahony_Kp_f)); // here the 1/Q5 [rad/s] are included
	}
//	static int16_t delta_t = CLAMP_INT32_TO_INT16((int32_t)((float)(34.9f/(float)SENSOR_FUSION_FREQUENCY_IMU) * (float)Q15));

	//get values

	acc[0] = pHandle_sf->acc_t.x;	acc[1] = pHandle_sf->acc_t.y;	acc[2] = pHandle_sf->acc_t.z;


	gyro[0] = CLAMP_INT32_TO_INT16(q15_mul(pHandle_sf->gyro_t.x, GRAD2RAD_GYRO_MAX_VALUE_Q15));
	gyro[1] = CLAMP_INT32_TO_INT16(q15_mul(pHandle_sf->gyro_t.y, GRAD2RAD_GYRO_MAX_VALUE_Q15));
	gyro[2] = CLAMP_INT32_TO_INT16(q15_mul(pHandle_sf->gyro_t.z, GRAD2RAD_GYRO_MAX_VALUE_Q15));



	if(PREFILTER_SF){

//		if(PREF_METHODE == BUTTERWORTH)	butterworth_lp20Hz_fs1k_Q30(acc,acc);
//		if(PREF_METHODE == IIR)			iir_filter_acc_Q15(acc,acc,24.0f,1000.0f);
	}
	omega[0] = 0;	omega[1] = gyro[0];	omega[2] = gyro[1];	omega[3] = gyro[2];
	norm_3d_vector(acc, acc_norm);

	int16_t q_est_[4];
	q_t_conj_function_in_out_q15(q_est, q_est_);
	rotate_vector_Q15(q_est_,gI,v_g);
	debug_mahony_vg.x = v_g[0];	debug_mahony_vg.y = v_g[1];	debug_mahony_vg.z = v_g[2];

	int16_t q_acc_norm[4], q_v_g_[4], q_err[4];


	quaternion_from_vector(acc_norm,q_acc_norm);
	quaternion_from_vector(v_g,q_v_g_);
	q_t_conj_function(q_v_g_);
//	q_t_conj_function(q_acc_norm);
	multiplicateQuaternionQ15(q_acc_norm,q_v_g_, q_err);
//	multiplicateQuaternionQ15(v_g,q_acc_norm, q_err);
//	if(q_err[0] < 0) q_t_flipp(q_err);
	if(q_err[0] > (Q15 - 100)){
		q_err[0] = Q15;
		q_err[1] = Q1_SHIFT_ROUND(q_err[1]);
		q_err[2] = Q1_SHIFT_ROUND(q_err[2]);
		q_err[3] = Q1_SHIFT_ROUND(q_err[3]);
		NormalizeQuaternionQ15(q_err, q_err);
	}

	vector_from_quaternion(q_err,acc_error);
//	int16_t d;
//	dotporduct_3x3_Q15(acc_norm, v_g, &d);
//	crossproduct_3x3_Q15(acc_norm, v_g, acc_error);
//	int16_t e_mag[3] ={0,0,0};
	debug_mahony_error.x = acc_error[0];
	debug_mahony_error.y = acc_error[1];
	debug_mahony_error.z = acc_error[2];
//	if(d<0){
//		acc_error[0] = -acc_error[0];
//		acc_error[1] = -acc_error[1];
//		acc_error[2] = -acc_error[2];
//	}

	if(mag_on){

		mag_raw[0] = pHandle_sf->mag_t.x;	mag_raw[1] = pHandle_sf->mag_t.y;	mag_raw[2] = pHandle_sf->mag_t.z; // raw z-axis point up

		hardiron_apply_q15(mag_raw);	softiron_apply_q15(mag_raw, mag_equalized);norm_3d_vector(mag_equalized, mag_norm);
		// mag auskommentiert um sicher zu sein dass es keine probleme gibt
//		int16_t snip = CLAMP_INT32_TO_INT16((((int32_t)q_est[0] * q_est[2]) - ((int32_t)q_est[3] * q_est[1])) << 1);
//		int16_t pitch_q = q15_asin(snip);
//		int16_t y = CLAMP_INT32_TO_INT16((((int32_t)q_est[0] * q_est[1]) - ((int32_t)q_est[2] * q_est[3])) << 1);
//		int16_t x = CLAMP_INT32_TO_INT16((((int32_t)q_est[1] * q_est[1]) - ((int32_t)q_est[2] * q_est[2])) << 1);
//		x = CLAMP_INT32_TO_INT16((int32_t)Q15 - x);
//		int16_t roll_q = q15_atan2(y, x);
//		// Tilt compensation
//		int16_t cx = cos_Q15(pitch_q);
//		int16_t sx = sin_Q15(pitch_q);
//		int16_t cy = cos_Q15(roll_q);
//		int16_t sy = sin_Q15(roll_q);
//		int16_t mx2 = q15_mul(mag_norm[0],cx);
//		int16_t my2 = CLAMP_INT32_TO_INT16((int32_t)q15_mul(q15_mul(mag_norm[0],sy),sx) + q15_mul(mag_norm[1],cy) - q15_mul(q15_mul(mag_norm[2],sy),cx));
//		int16_t	yaw_mag = q15_atan2(-my2, mx2);
//		y = CLAMP_INT32_TO_INT16((((int32_t)q_est[0] * q_est[3]) - ((int32_t)q_est[1] * q_est[2])) << 1);
//		x = CLAMP_INT32_TO_INT16((((int32_t)q_est[2] * q_est[2]) - ((int32_t)q_est[3] * q_est[3])) << 1);
//		x = CLAMP_INT32_TO_INT16((int32_t)Q15 - x);
//		int16_t yaw_q = q15_atan2(y, x);
//		e_mag[2] = yaw_q - yaw_mag;
	}else{
		acc_error[2] = 0;
	}

	omega_corr[0] = 0;
	for(uint8_t i = 0; i<3;i++){
		e[i]			= CLAMP_INT32_TO_INT16((int32_t)acc_error[i]);// + e_mag[i]);
		eInt[i] 		= eInt[i] + ((int64_t)e[i] * mahony_delta_error_t); // integration in Q30
		eInt[i] 		= CLAMP(eInt[i], -Q60, Q60);
		eIntq15[i] 		= CLAMP_INT32_TO_INT16(Q15_SHIFT_ROUND(eInt[i])); // back to Q15
		omega_corr[i+1] = CLAMP_INT32_TO_INT16((int32_t)gyro[i] + q15_mul(mahony_Kp_q15,e[i]) + q15_mul(mahony_Ki_q15, eIntq15[i])); // all components are in 1/Q5 rad
	}
	mahony_int_x = eInt[0];
	mahony_int_y = eInt[1];
	mahony_int_z = eInt[2];



	multiplicateQuaternionQ15(q_est,omega_corr,qDot);
//	divideQuaternionBy2(qDot); // -> compensate this with delta_vqf_t -> Q4 instead Q5
	int_gyro_dot_Q15(q_est, qDot,mahony_delta_t,q_est);

	copy_quaternion(q_est, q_out);

}
