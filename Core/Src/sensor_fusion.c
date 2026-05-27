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
#include <encoder.h>
// Optical_flow
#include <Optical_flow/uart_ring.h>
//#include <Optical_flow/mtf02.h>
#include "Optical_flow/mtf02_ring_buffer.h"
#include <EKF.h>
#include <axis_aligned_filter.h>

//
//extern I2C_HandleTypeDef hi2c1;
//extern I2C_HandleTypeDef hi2c2;
//
//extern UART_HandleTypeDef huart8;

//###### PARAMETER STRUCKT ######
sensor_fusion sf_values;
// ####### DOUBLE BUFFER ######
wxyz_16t quaternion_buffer;
xyz_16t gyro_t_buffer, acc_t_buffer;
volatile bool sf_writing;
// ##### SETINGS #####
float beta_f;
int16_t drift_gain;
int32_t w_drift[3];
uint8_t calibrate_encoder_ = 0;



// #### global sf variables #####
int16_t ax, ay, az, gx, gy, gz, mx, my, mz, dt_q15, beta_t;
int16_t q_madgwick_out[4], q_compl_out[4];
int16_t gyro_grad2rad_delta_t_q15;
butterworth_coefficients cf_bw;
// ##### Flags #####
volatile bool mag_ready_flag;

// Output (debug)
float beta_yaw = 0.005;
int16_t beta_yaw_t;
int16_t euler_debug[3];
int16_t euler_debug_madgwick_pitch,euler_debug_madgwick_roll,euler_debug_madgwick_yaw;
xyz_16t debug_gyro, debug_omega, debug_mag_norm,debug_mag_norm_ekf, debug_mag_hard_iron, debug_mag_raw, debug_mag_soft;
wxyz_16t debug_qDot, debug_q_out, debug_q_out_norm, debug_h;
int16_t debug_bx, debug_bz;
int16_t debug_grad1, debug_grad2,debug_grad3,debug_grad4;
int16_t debug_h1, debug_h2, debug_h3, debug_h4;
int16_t debug_cf_v_x, debug_cf_v_y, debug_cf_v_z;
wxyz_16t debug_q_mult;
wxyz_16t debug_gyro_drift;
uint8_t sensor_fusion_takeout;

//#define CLAMP_INT32_TO_INT16(x) ((x) > INT16_MAX ? INT16_MAX : ((x) < INT16_MIN ? INT16_MIN : (int16_t)(x)))
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
static void gyro_drift_Q15(const int16_t *q_est, const int16_t *qDot, int32_t *w_drift_grad);
static void butterworth_filter_acc_Q15(const int16_t *acc, int16_t *acc_filt);
static void madgwick_butterworth_lp20Hz_fs1k_Q30(const int16_t *acc_q15, int16_t *acc_filt_q15);
static void madgwick_iir_filter_acc_Q15(const int16_t *acc_raw, int16_t *acc_filter);
static void compl_iir_filter_acc_Q15(const int16_t *acc_raw, int16_t *acc_filter);
static void iir_filter_gyro_Q15(const int16_t *gyro_raw, int16_t *gyro_filter);
static void Complementary_filter_acc_gyro_mag(sensor_fusion *pHandle_sf, const bool acc_on,const bool mag_on);
static void ERROR_Blink_LED(void);
static void complementary_filter_init(void);
static void q15_qDot_mu_dt_q30(int16_t *q_out, const int16_t *qDot, const int16_t dt, const uint8_t reset);
static void madgwick_mag_butterworth_lp20Hz_fs1k_Q30(const int16_t *acc_q15, int16_t *acc_filt_q15);
static void magKF_Q15(int16_t *mag_norm);
static void acc_KF_Q15(int16_t *acc_q15);

void get_quaternion_Q15(int16_t *q, int16_t *w, int16_t *a) {
	__disable_irq();
	if(!sf_writing){
		q[0] = sf_values.quaternion.w; 	q[1] = sf_values.quaternion.x;
		q[2] = sf_values.quaternion.y; 	q[3] = sf_values.quaternion.z;

		w[0] = sf_values.gyro_t.x; 		w[1] = sf_values.gyro_t.y;
		w[2] = sf_values.gyro_t.z;

		a[0] = sf_values.acc_t.x;		a[1] = sf_values.acc_t.y;
		a[2] = sf_values.acc_t.z;
		__enable_irq();
		return;
	}else{
		q[0] = quaternion_buffer.w;		q[1] = quaternion_buffer.x;
		q[2] = quaternion_buffer.y;		q[3] = quaternion_buffer.z;

		w[0] = gyro_t_buffer.x;			w[1] = gyro_t_buffer.y;
		w[2] = gyro_t_buffer.z;
		__enable_irq();
		return;
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

	w_drift[0] = 0;		w_drift[1] = 0;		w_drift[2] = 0;

//	dt_q15 = ((1UL <<15) * 35)/80;
	float x = ((float)((float)Q15 * (float)Q4)/(float)SENSOR_FUSION_FREQUENCY_IMU);
	dt_q15 = CLAMP_INT32_TO_INT16((int32_t)x);

//	dt_q15 = (1UL <<15)/80;
	beta_f = 0.2;
	beta_yaw = 0.3;
	if(SYSTEM == IMU){
		beta_f = 0.05;
		beta_yaw = 0.5;
	}


	q_madgwick_out[0] = Q15_ONE;	q_madgwick_out[1] = 0;
	q_madgwick_out[2] = 0;			q_madgwick_out[3] = 0;
	drift_gain = 1;//CLAMP_INT32_TO_INT16((int32_t)(0.01f * (float)Q15));
	switch(SENSORFUSION_METHODE){
		case (SF_MADGWICK):
		complementary_filter_init();

		break;
		case (SF_COMPLEMENTARY):
		break;
		case(SF_EKF):
				init_EKF();
		break;
		case(SF_AXIS_ALIGN):
				axis_aligned_init();
		break;
		case(SF_COMPARE_ALL):
		complementary_filter_init();
			axis_aligned_init();
			vqf_init();
			mahony_filter_init();
		break;
		default:
		break;
	}


	if(COMMUNICATION_IMU_MAG){
		if(LIS3MDL_Init() != HAL_OK) ERROR_Blink_LED();
		if(MPU6000_Init() != HAL_OK) ERROR_Blink_LED();
	}
	if(COMMUNICATION_OPT_FLW){
//		MTF02_RX_Init(&huart8);
		if(UART8_RX_Start() != HAL_OK){
//			Error_Handler();
			HAL_Delay(1000);
			if(UART8_RX_Start() != HAL_OK){
				uint8_t test = 0;
				ERROR_Blink_LED();

			}

		}
	}


}

xyz_16t debug_vqf_out,debug_mahony_out, debug_axis_alignend_out, debug_ekf_out;
uint8_t debug_reset_sensor_fusion = 0;
void task_imu_sensor_fusion(void){
//	 ignore the first interrupts during the initalisation
	if(sensor_fusion_takeout < 15){
		sensor_fusion_takeout ++;
	}else{
//		stopp_time_measurement();
//								start_time_measurement();

		if(calibrate_encoder_){
			calibrate_encoder_ = 0;
			set_encoder_to_zero();
		}

		if(ADAPTIVE_MADGWICK == ON){
			int16_t rot_rate_degree = labs((int32_t)sf_values.gyro_t.x + (int32_t)sf_values.gyro_t.y + (int32_t)sf_values.gyro_t.z);
			rot_rate_degree = (2000 * rot_rate_degree) >> 15;
			if(rot_rate_degree <= 50){
				beta_f = 0.08f;
			}else{
				if(rot_rate_degree > 150){
					beta_f = 0.01f;
				}else{
					beta_f = 0.08f - ((((float)rot_rate_degree-50.0f) /  100.0f) * 0.07f);
				}
			}
//			beta_f = 0.05;
		}

		switch(SENSORFUSION_METHODE){
		case (SF_MADGWICK):

			MPU6000_Get_data_IT(&sf_values);
				// debug calc for beta
				beta_t = CLAMP_INT32_TO_INT16((int32_t)((float)INT16_MAX * beta_f));
				beta_yaw_t = CLAMP_INT32_TO_INT16((int32_t)((float)INT16_MAX * beta_yaw));
//		beta_t = CLAMP_INT32_TO_INT16((int32_t)((float)INT16_MAX * beta_f / 8.0f));
//		beta_yaw_t = CLAMP_INT32_TO_INT16((int32_t)((float)INT16_MAX * beta_yaw / 8.0f));
				switch(mag_ready_flag){
				case(true):

//						static uint16_t time_counter = 0;
//						if(time_counter == 1) start_time_measurement();
//						if(time_counter == 80) stopp_time_measurement();
//						time_counter++;
					read_data_mag(&sf_values);
					Madgwick_filter_acc_gyro_mag(beta_t, &sf_values);
					mag_ready_flag = false;
				break;
				default:
					Madgwick_filter_acc_gyro(beta_t, &sf_values);
				break;
				}

			break;
			case (SF_COMPLEMENTARY):
				MPU6000_Get_data_IT(&sf_values);
				switch(mag_ready_flag){
					case(true):
						read_data_mag(&sf_values);
						Complementary_filter_acc_gyro_mag(&sf_values,ACC_ON,MAG_ON);
						mag_ready_flag = false;
					break;
					default:

						Complementary_filter_acc_gyro_mag(&sf_values,ACC_ON,MAG_OFF);

					break;
					}

		break;
		case (SF_AXIS_ALIGN):
			MPU6000_Get_data_IT(&sf_values);
			int16_t q_ax_al[4];
			switch(mag_ready_flag){
				case(true):
					read_data_mag(&sf_values);
					axis_aligned_filter(&sf_values,q_ax_al,ACC_ON,MAG_ON);
					mag_ready_flag = false;
					sf_values.mag_updated = true;
				break;
				default:
					axis_aligned_filter(&sf_values,q_ax_al,ACC_ON,MAG_OFF);
					sf_values.mag_updated = false;
				break;
				}

	break;
		case (SF_EKF):

			MPU6000_Get_data_IT(&sf_values);
			switch(mag_ready_flag){

				case(true):
					read_data_mag(&sf_values);

				sf_values.mag_updated = true;
					mag_ready_flag = false;
				break;
				default:

//					Complementary_filter_acc_gyro_mag(&sf_values,ACC_ON,MAG_OFF);
					sf_values.mag_updated = false;
				break;

				}
			int16_t q_ekf[4], euler_ekf[3];
			execute_EKF_Fast_Q15(&sf_values,q_ekf);
			quat_to_euler_q15(q_ekf,euler_ekf);
			debug_ekf_out.x = euler_ekf[0];	debug_ekf_out.y = euler_ekf[1];	debug_ekf_out.z = euler_ekf[2];

	break;
		case (LOG_DATA_ONLY):
		MPU6000_Get_data_IT(&sf_values);
		switch(mag_ready_flag){
			case(true):
				read_data_mag(&sf_values);
			sf_values.mag_updated = true;
			mag_ready_flag = false;

			break;
			default:
				sf_values.mag_updated = false;


		}

		break;
			case (SF_COMPARE_ALL):
				int16_t q_axis_al[4], q_vqf[4], q_mahony[4];
					MPU6000_Get_data_IT(&sf_values);
						// debug calc for beta
						beta_t = CLAMP_INT32_TO_INT16((int32_t)((float)INT16_MAX * beta_f));
						beta_yaw_t = CLAMP_INT32_TO_INT16((int32_t)((float)INT16_MAX * beta_yaw));
		//		beta_t = CLAMP_INT32_TO_INT16((int32_t)((float)INT16_MAX * beta_f / 8.0f));
		//		beta_yaw_t = CLAMP_INT32_TO_INT16((int32_t)((float)INT16_MAX * beta_yaw / 8.0f));
						switch(mag_ready_flag){
						case(true):
		read_data_mag(&sf_values);

		//						static uint16_t time_counter = 0;
		//						if(time_counter == 1) start_time_measurement();
		//						if(time_counter == 80) stopp_time_measurement();
		//						time_counter++;
							if(0){
							read_data_mag(&sf_values);
//							Madgwick_filter_acc_gyro_mag(beta_t, &sf_values);

							Madgwick_filter_acc_gyro(beta_t, &sf_values);

//							axis_aligned_filter(&sf_values,q_axis_al,ACC_ON,MAG_ON);
							axis_aligned_filter(&sf_values,q_axis_al,ACC_ON,MAG_OFF);


//							Complementary_filter_acc_gyro_mag(&sf_values,ACC_ON,MAG_ON);
							Complementary_filter_acc_gyro_mag(&sf_values,ACC_ON,MAG_OFF);


							vqf_filter(&sf_values,q_vqf, MAG_OFF,debug_reset_sensor_fusion);


							mahony_filter(&sf_values,q_mahony,MAG_OFF, debug_reset_sensor_fusion);
							}else{

								Madgwick_filter_acc_gyro_mag(beta_t, &sf_values);
//								Madgwick_filter_acc_gyro(beta_t, &sf_values);

								Complementary_filter_acc_gyro_mag(&sf_values,ACC_ON,MAG_ON);

//								Complementary_filter_acc_gyro_mag(&sf_values,ACC_ON,MAG_OFF);
							}
							sf_values.mag_updated = true;
							mag_ready_flag = false;
						break;
						default:
							if(0){
							Madgwick_filter_acc_gyro(beta_t, &sf_values);


							Complementary_filter_acc_gyro_mag(&sf_values,ACC_ON,MAG_OFF);
							axis_aligned_filter(&sf_values,q_ax_al,ACC_ON,MAG_OFF);

							vqf_filter(&sf_values,q_vqf, MAG_OFF,debug_reset_sensor_fusion);


							mahony_filter(&sf_values,q_mahony,MAG_OFF, debug_reset_sensor_fusion);
							}else{
								Madgwick_filter_acc_gyro(beta_t, &sf_values);
								Complementary_filter_acc_gyro_mag(&sf_values,ACC_ON,MAG_OFF);
							}

							sf_values.mag_updated = false;
						break;
						}
						if(debug_reset_sensor_fusion > 0){
							debug_reset_sensor_fusion = 0;
						}
						// DEBUG results:
						int16_t euler[3];
						quat_to_euler_q15(q_vqf,euler);
						debug_vqf_out.x = euler[0];	debug_vqf_out.y = euler[1];	debug_vqf_out.z = euler[2];
						quat_to_euler_q15(q_mahony,euler);
						debug_mahony_out.x = euler[0];	debug_mahony_out.y = euler[1];	debug_mahony_out.z = euler[2];
						quat_to_euler_q15(q_axis_al,euler);
						debug_axis_alignend_out.x = euler[0];
						debug_axis_alignend_out.y = euler[1];
						debug_axis_alignend_out.z = euler[2];

					break;

			default:

			break;


		}


		set_log_data_flag(); 	// for data logging
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

int16_t debug_z_component;
int16_t acc_iir, acc_buthworth, acc_kf;
static void Madgwick_filter_acc_gyro_mag(int16_t beta, sensor_fusion *pHandle_sf){
	int16_t accel[3], accel_norm[3], gyro[3],mag_raw[3], mag_equalized[3], mag_norm[3],
			f_error[6], grad[4], grad_norm[4], omega[4], mag_quat[4], qDot[4], h[4],q_mul_w[4], q_con[4],
			bxz[2], J[6][4];
	//get values
	accel[0] = pHandle_sf->acc_t.x;		accel[1] = pHandle_sf->acc_t.y;
	accel[2] = pHandle_sf->acc_t.z;
	if(MADGWICK_W_SCALE == ON){
		gyro[0] = pHandle_sf->gyro_t.x;		gyro[1] = pHandle_sf->gyro_t.y;
		gyro[2] = pHandle_sf->gyro_t.z;
	}else{
		gyro[0] = q15_mul(pHandle_sf->gyro_t.x, GRAD2RAD_GYRO_MAX_Q15);
		gyro[1] = q15_mul(pHandle_sf->gyro_t.y, GRAD2RAD_GYRO_MAX_Q15);
		gyro[2] = q15_mul(pHandle_sf->gyro_t.z, GRAD2RAD_GYRO_MAX_Q15);
	}

	if(PREFILTER_SF){
		if(PREF_METHODE == IIR) 		madgwick_iir_filter_acc_Q15(accel, accel);
		if(PREF_METHODE == BUTTERWORTH)	madgwick_butterworth_lp20Hz_fs1k_Q30(accel,accel);
	}

//	gyro[0] -= pHandle_sf->gyro_drift_est.x;
//	gyro[1] -= pHandle_sf->gyro_drift_est.y;
//	gyro[2] -= pHandle_sf->gyro_drift_est.z;

	// compensate Drift
//	gyro[0] += q15_mul(q_drift[1],drift_gain);
//	gyro[1] += q15_mul(q_drift[2],drift_gain);
//	gyro[2] += q15_mul(q_drift[3],drift_gain);

//	gyro[0] -= (w_drift[0] >> 5);
//	gyro[1] -= (w_drift[1] >> 5);
//	gyro[2] -= (w_drift[2] >> 7);

	debug_gyro_drift.x = w_drift[0];	debug_gyro_drift.y = w_drift[1];
	debug_gyro_drift.z = w_drift[1];

	debug_gyro.x = gyro[0];				debug_gyro.y = gyro[1];
	debug_gyro.z = gyro[2];

	mag_raw[0] = pHandle_sf->mag_t.x;	mag_raw[1] = pHandle_sf->mag_t.y;
	mag_raw[2] = pHandle_sf->mag_t.z; // raw z-axis point up

	omega[0] = 0;						omega[1] = debug_omega.x = gyro[0];
	omega[2] = debug_omega.y = gyro[1];	omega[3] = debug_omega.z = gyro[2];

	debug_mag_raw.x = mag_raw[0];		debug_mag_raw.y = mag_raw[1];
	debug_mag_raw.z = mag_raw[2];
//	madgwick_mag_butterworth_lp20Hz_fs1k_Q30(mag_raw,mag_raw);
	hardiron_apply_q15(mag_raw);

	debug_mag_hard_iron.x = mag_raw[0];	debug_mag_hard_iron.y = mag_raw[1];
	debug_mag_hard_iron.z = mag_raw[2];
	softiron_apply_q15(mag_raw, mag_equalized);

	debug_mag_soft.x = mag_equalized[0];debug_mag_soft.y = mag_equalized[1];
	debug_mag_soft.z = mag_equalized[2];

	norm_3d_vector(accel, accel_norm);
	norm_3d_vector(mag_equalized, mag_norm);
	debug_mag_norm.x = mag_equalized[1];	debug_mag_norm.y = mag_equalized[0];
	debug_mag_norm.z = mag_equalized[2];
	magKF_Q15(mag_norm);

	debug_mag_norm_ekf.x = mag_norm[1];	debug_mag_norm_ekf.y = mag_norm[0];
	debug_mag_norm_ekf.z = mag_norm[2];
//	norm_3d_vector(mag_equalized, mag_norm);


//	mag_quat[0] = 0;				mag_quat[1] = mag_norm[1];
//	mag_quat[2] = mag_norm[0];		mag_quat[3] = mag_norm[2];
	mag_quat[0] = 0;				mag_quat[1] = mag_norm[0];
	mag_quat[2] = mag_norm[1];		mag_quat[3] = mag_norm[2];

	// h = q ⊗ [0; mag] ⊗ q*    // getestet und funkitoniert ✅
	q_t_conj_function_in_out_q15(q_madgwick_out, q_con); // getestet und funkitoniert ✅
	multiplicateQuaternionQ15(q_madgwick_out,mag_quat,q_mul_w); // getestet und funkitoniert ✅
	multiplicateQuaternionQ15(q_mul_w,q_con,h); // getestet und funkitoniert ✅

//	rotate_quat_sandwich_q15(q_madgwick_out,mag_quat,q_con,h);

//	Normalize4DvectorQ15(h, h);
	get_bx_bz_q15(h, bxz); // getestet und funkitoniert ✅

//	if(bxz[1] > -7000){
//		bxz[0] = bxz[1] = 0;
//	}
//	bxz[0] = - bxz[0]; // correct the yaw direction

//	iir_filter_bx_bz_q15(bxz);

	debug_q_mult.w = q_mul_w[0];	debug_q_mult.x = q_mul_w[1];
	debug_q_mult.y = q_mul_w[2];	debug_q_mult.z = q_mul_w[3];

	debug_h1 = h[0];				debug_h2 = h[1];
	debug_h3 = h[2];				debug_h4 = h[3];

	debug_bx = bxz[0];				debug_bz = bxz[1];

	error_function_bigQ15(accel_norm, mag_norm, q_madgwick_out, f_error, bxz); // getestet und funkitoniert ✅
	get_jacobi_bigQ15(J,q_madgwick_out, bxz); // getestet und funkitoniert ✅
	compute_gradient_bigQ15(grad, J, f_error); // getestet und funkitoniert ✅
	Normalize4DvectorQ15(grad, grad_norm);

//	gyro_drift_Q15(q_madgwick_out,grad_norm, w_drift);

	multiplicateQuaternionQ15(q_madgwick_out,omega,qDot); // getestet und funkitoniert ✅
//	divideQuaternionBy2(qDot); // getestet und funkitoniert ✅
	/*
	 * Matlab:
	 * qDot = qDot - beta * grad;
	 */

//	positve_quaternion_test_Q15(qDot);
	debug_z_component = q15_mul(grad_norm[3], beta_yaw_t);

	qDot[0] = debug_qDot.w = CLAMP_INT32_TO_INT16((int32_t)qDot[0] - Q4_SHIFT_ROUND((int32_t)q15_mul(grad_norm[0], beta_yaw_t)));
	qDot[1] = debug_qDot.x = CLAMP_INT32_TO_INT16((int32_t)qDot[1] - Q4_SHIFT_ROUND((int32_t)q15_mul(grad_norm[1], beta_yaw_t)));
	qDot[2] = debug_qDot.y = CLAMP_INT32_TO_INT16((int32_t)qDot[2] - Q4_SHIFT_ROUND((int32_t)q15_mul(grad_norm[2], beta_yaw_t)));
	qDot[3] = debug_qDot.z = CLAMP_INT32_TO_INT16((int32_t)qDot[3] - Q4_SHIFT_ROUND((int32_t)q15_mul(grad_norm[3], beta_yaw_t)));

//	NormalizeQuaternionQ15(qDot, qDot);
	q15_qDot_mu_dt_with_rest(q_madgwick_out,qDot, dt_q15); // here in dt_q15 the calc gyro_grad_2_rad
	NormalizeQuaternionQ15(q_madgwick_out, q_madgwick_out);
//	positve_quaternion_test_Q15(q_madgwick_out);

	quat_to_euler_q15(q_madgwick_out, euler_debug);
	euler_debug_madgwick_roll = euler_debug[0];// * 360.0f / (float)INT16_MAX;
	euler_debug_madgwick_pitch = euler_debug[1];// * 360.0f / (float)INT16_MAX;
	euler_debug_madgwick_yaw = euler_debug[2];// * 360.0f / (float)INT16_MAX;

//
//	if(q_madgwick_out[0] < 0){
//		q_madgwick_out[0] = -q_madgwick_out[0];
//		q_madgwick_out[1] = -q_madgwick_out[1];
//		q_madgwick_out[2] = -q_madgwick_out[2];
//		q_madgwick_out[3] = -q_madgwick_out[3];
//	}
	// Safe quaternion / gyro two times
	// 1. Time
	sf_writing = true;
	pHandle_sf->quaternion.w = q_madgwick_out[0];
	pHandle_sf->quaternion.x = q_madgwick_out[1];
	pHandle_sf->quaternion.y = q_madgwick_out[2];
	pHandle_sf->quaternion.z = q_madgwick_out[3];
//	pHandle_sf->gyro_t_rad.x = gyro[0];
//	pHandle_sf->gyro_t_rad.y = gyro[1];
//	pHandle_sf->gyro_t_rad.z = gyro[2];

	sf_writing = false;
	// 2. Time
	quaternion_buffer.w = q_madgwick_out[0];
	quaternion_buffer.x = q_madgwick_out[1];
	quaternion_buffer.y = q_madgwick_out[2];
	quaternion_buffer.z = q_madgwick_out[3];

	gyro_t_buffer.x =  gyro[0];
	gyro_t_buffer.y =  gyro[1];
	gyro_t_buffer.z =  gyro[2];
	// end
}
xyz_16t accel_filter;
static void Madgwick_filter_acc_gyro(int16_t beta, sensor_fusion *pHandle_sf){

	int16_t accel[3], accel_norm[3], gyro[3], f_error[3], grad[4], grad_norm[4],omega[4], qDot[4], J[3][4];



	accel[0] = pHandle_sf->acc_t.x;		accel[1] = pHandle_sf->acc_t.y;
	accel[2] = pHandle_sf->acc_t.z;
	if(MADGWICK_W_SCALE == ON){
		gyro[0] = pHandle_sf->gyro_t.x;		gyro[1] = pHandle_sf->gyro_t.y;
		gyro[2] = pHandle_sf->gyro_t.z;
	}else{
//		gyro[0] = q15_mul(pHandle_sf->gyro_t.x, (GRAD2RAD_GYRO_MAX_Q15)); // scaling the impact of gyro a little bit
//		gyro[1] = q15_mul(pHandle_sf->gyro_t.y, (GRAD2RAD_GYRO_MAX_Q15));
//		gyro[2] = q15_mul(pHandle_sf->gyro_t.z, GRAD2RAD_GYRO_MAX_Q15);
	}
	if(PREFILTER_SF){
//	iir_filter_gyro_Q15(gyro,gyro);
		if(PREF_METHODE == IIR) 		madgwick_iir_filter_acc_Q15(accel, accel);
		if(PREF_METHODE == BUTTERWORTH)	madgwick_butterworth_lp20Hz_fs1k_Q30(accel,accel);
	}

	int16_t acc_iir_t[3],acc_buth_t[3], acc_kf_[3];
	acc_kf_[0] = accel[0]; acc_kf_[1] = accel[1]; acc_kf_[2] = accel[2];

//	madgwick_iir_filter_acc_Q15(accel,acc_iir_t);

//
//	acc_KF_Q15(acc_kf_);
//
//	start_time_measurement();
//	madgwick_butterworth_lp20Hz_fs1k_Q30(accel,acc_buth_t);
//	stopp_time_measurement();
//
//	acc_iir = acc_iir_t[0];
//	acc_kf = acc_kf_[0];
//	acc_buthworth = acc_buth_t[0];
	// compensate Drift
//	gyro[0] += q15_mul(q_drift[1],drift_gain);
//	gyro[1] += q15_mul(q_drift[2],drift_gain);
//	gyro[2] += q15_mul(q_drift[3],drift_gain);

//	gyro[0] -= (w_drift[0]);
//	gyro[1] -= (w_drift[1]);
//	gyro[2] -= (w_drift[2]);

	debug_gyro_drift.x = w_drift[0];	debug_gyro_drift.y = w_drift[1];
	debug_gyro_drift.z = w_drift[2];


	omega[0] = 0;						omega[1] = debug_omega.x = gyro[0];
	omega[2] = debug_omega.y = gyro[1];	omega[3] = debug_omega.z = gyro[2];

	accel_filter.x = accel[0];
	accel_filter.y = accel[1];
	accel_filter.z = accel[2];

	norm_3d_vector(accel, accel_norm); // getestet und funkitoniert ✅
	error_function_small(accel_norm, q_madgwick_out, f_error); // getestet und funkitoniert ✅
	get_jacobi_small(J, q_madgwick_out);// getestet und funkitoniert ✅
	compute_gradient(grad, J, f_error);// getestet und funkitoniert ✅
	Normalize4DvectorQ15(grad, grad_norm);

	gyro_drift_Q15(q_madgwick_out,grad, w_drift);
	// Benötige Testfunktion ab hier:
	multiplicateQuaternionQ15(q_madgwick_out,omega,qDot); // getestet und funkitoniert ✅
//	divideQuaternionBy2(qDot); // getestet und funkitoniert ✅
	/*
	 * Matlab:
	 * qDot = qDot - beta * grad;
	 */
	qDot[0] = debug_qDot.w = CLAMP_INT32_TO_INT16((int32_t)qDot[0] - Q4_SHIFT_ROUND((int32_t)q15_mul(grad_norm[0], beta)));
	qDot[1] = debug_qDot.x = CLAMP_INT32_TO_INT16((int32_t)qDot[1] - Q4_SHIFT_ROUND((int32_t)q15_mul(grad_norm[1], beta)));
	qDot[2] = debug_qDot.y = CLAMP_INT32_TO_INT16((int32_t)qDot[2] - Q4_SHIFT_ROUND((int32_t)q15_mul(grad_norm[2], beta)));
	qDot[3] = debug_qDot.z = CLAMP_INT32_TO_INT16((int32_t)qDot[3] - Q4_SHIFT_ROUND((int32_t)q15_mul(grad_norm[3], beta)));
//	NormalizeQuaternionQ15(qDot, qDot);
	q15_qDot_mu_dt_with_rest(q_madgwick_out,qDot, dt_q15);

	debug_q_out.w = q_madgwick_out[0];		debug_q_out.x = q_madgwick_out[1];
	debug_q_out.y = q_madgwick_out[2];		debug_q_out.z = q_madgwick_out[3];

	NormalizeQuaternionQ15(q_madgwick_out, q_madgwick_out);
//	positve_quaternion_test_Q15(q_madgwick_out);

	// Safe quaternion / gyro two times
	// 1. Time
	sf_writing = true;
	pHandle_sf->quaternion.w = q_madgwick_out[0];
	pHandle_sf->quaternion.x = q_madgwick_out[1];
	pHandle_sf->quaternion.y = q_madgwick_out[2];
	pHandle_sf->quaternion.z = q_madgwick_out[3];

	sf_writing = false;
	// 2. Time
	quaternion_buffer.w = q_madgwick_out[0];
	quaternion_buffer.x = q_madgwick_out[1];
	quaternion_buffer.y = q_madgwick_out[2];
	quaternion_buffer.z = q_madgwick_out[3];

	gyro_t_buffer.x =  gyro[0];
	gyro_t_buffer.y =  gyro[1];
	gyro_t_buffer.z =  gyro[2];
	// end

	debug_q_out_norm.w = q_madgwick_out[0];
	debug_q_out_norm.x = q_madgwick_out[1];
	debug_q_out_norm.y = q_madgwick_out[2];
	debug_q_out_norm.z = q_madgwick_out[3];

	if(DEBUG_MODE == ON){
	quat_to_euler_q15(q_madgwick_out, euler_debug);
	euler_debug_madgwick_roll = euler_debug[0];// * 360.0f / (float)INT16_MAX;
	euler_debug_madgwick_pitch = euler_debug[1];// * 360.0f / (float)INT16_MAX;
	euler_debug_madgwick_yaw = euler_debug[2];// * 360.0f / (float)INT16_MAX;
	}
}
// ######## QUATERNION INLINE FUNCTIONS ############
//static inline int16_t q15_mul(int16_t a, int16_t b) {
//    return CLAMP_INT32_TO_INT16(Q15_SHIFT_ROUND((int32_t)a * b)); // mit Rundung
//}

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
int16_t debug_z_prod;
static void q15_qDot_mu_dt_with_rest(int16_t *q_in, const int16_t *qDot, const int16_t dt){
	static wxyz_16t rest = {0, 0, 0, 0};
    int32_t delta, sum; //prod
    int64_t prod;


    // w
    prod = (int64_t)qDot[0] * dt + rest.w;
    delta = prod >> 15;
    rest.w = CLAMP_INT32_TO_INT16(prod - (delta << 15));
    sum = (int32_t)q_in[0] + delta;
    q_in[0] = CLAMP_INT32_TO_INT16(sum);

    // x
    prod = (int64_t)qDot[1] * dt + rest.x;
    delta = prod >> 15;
    rest.x = CLAMP_INT32_TO_INT16(prod - (delta << 15));
    sum = (int32_t)q_in[1] + delta;
    q_in[1] = CLAMP_INT32_TO_INT16(sum);

    // y
    prod = (int64_t)qDot[2] * dt + rest.y;
    delta = prod >> 15;
    rest.y = CLAMP_INT32_TO_INT16(prod - (delta << 15));
    sum = (int32_t)q_in[2] + delta;
    q_in[2] = CLAMP_INT32_TO_INT16(sum);

    // z
    prod = (int64_t)qDot[3] * dt + rest.z;
    debug_z_prod = prod;
    delta = prod >> 15;
    rest.z = CLAMP_INT32_TO_INT16(prod - (delta << 15));
    sum = (int32_t)q_in[3] + delta;
    q_in[3] = CLAMP_INT32_TO_INT16(sum);
}

static void q15_qDot_mu_dt_q30(int16_t *q_out, const int16_t *qDot, const int16_t dt, const uint8_t reset){
	static int64_t q_int[4] = {0};
	if(reset > 0){
		q_int[0] = 0;	q_int[1] = 0;	q_int[2] = 0;	q_int[3] = 0;
	}
	for(uint8_t i = 0; i<4;i++){
		q_int[i] += (int64_t)qDot[i] * dt;
		q_int[i] = CLAMP(q_int[i], -Q60, Q60);
		q_out[i] = CLAMP_INT32_TO_INT16(Q15_SHIFT_ROUND(q_int[i]));
	}
}

static inline void divideQuaternionBy2(int16_t *q) {
    q[0] = Q1_SHIFT_ROUND(q[0]);	q[1] = Q1_SHIFT_ROUND(q[1]);
    q[2] = Q1_SHIFT_ROUND(q[2]);	q[3] = Q1_SHIFT_ROUND(q[3]);
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
    // 64-bit multiply to be safe
    uint64_t aa = (int64_t)h[1] * (int64_t)h[1];
    uint64_t bb = (int64_t)h[2] * (int64_t)h[2];
    uint64_t sum = aa + bb;             // max ~2^31 → fits in 32 bit

    // sqrt result is Q15 scale directly
    uint32_t root = sqrt_fast_uint((uint32_t)sum);

    if(root > 32767) root = 32767;      // saturate to Q15
    bxz[0] = (int16_t)root;
    bxz[1] = h[3];
}

static void gyro_drift_Q15(const int16_t *q_est, const int16_t *qDot, int32_t *w_drift_grad){
//	static int16_t delta_t = CLAMP_INT32_TO_INT16((int32_t)((GRAD2RAD_GYRO/(float)SENSOR_FUSION_FREQUENCY_IMU)*(float)Q15));
	static int16_t delta_t = CLAMP_INT32_TO_INT16((int32_t)(((2.0f * 0.01f) /(float)SENSOR_FUSION_FREQUENCY_IMU)*(float)Q15));
	int16_t q_est_con[4], q_rot[4];
	int32_t q_transform_Q15_to_rad[3];
	q_t_conj_function_in_out_q15(q_est,q_est_con);
	multiplicateQuaternionQ15(q_est_con,qDot,q_rot);
//	multQuaternionWith2(q_rot); // take the 2* in the delta_t
	multQuatwithConstQ15(q_rot, delta_t);
	q_transform_Q15_to_rad[0] = CLAMP_INT32_TO_INT16(((int32_t)q_rot[1] * 938) >> 15);
	q_transform_Q15_to_rad[1] = CLAMP_INT32_TO_INT16(((int32_t)q_rot[2] * 938) >> 15);
	q_transform_Q15_to_rad[2] = CLAMP_INT32_TO_INT16(((int32_t)q_rot[3] * 938) >> 15);

	w_drift_grad[0] = CLAMP_INT32_TO_INT16(w_drift_grad[0] + q_transform_Q15_to_rad[0]);
	w_drift_grad[1] = CLAMP_INT32_TO_INT16(w_drift_grad[1] + q_transform_Q15_to_rad[1]);
	w_drift_grad[2] = CLAMP_INT32_TO_INT16(w_drift_grad[2] + q_transform_Q15_to_rad[2]);
//	add2QuaternionQ15(q_rot, w_drift_grad, w_drift_grad);
//	NormalizeQuaternionQ15(q_drift, q_drift);
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

	int16_t a[3] = {26273, -8758, 17515};
	int16_t b[3] = {0,0,Q15};
	int16_t q[4];

	minimal_rotation(a,b,q);

	HAL_Delay(1);


}
static void copy3DvectorQ15(const int16_t *in, int16_t *out){
	out[0] = in[0];
	out[1] = in[1];
	out[2] = in[2];
}
/*
 * % === Parameter ===
fc = 12;         % Cutoff-Frequenz in Hz
fs = 1000;       % Abtastfrequenz in Hz (Sampling rate)
n  = 2;          % Ordnung des Filters

% === Berechnung der Koeffizienten ===
[b, a] = butter(n, fc/(fs/2), 'low');  % digitaler Butterworth-Tiefpass

% === Ausgabe der Float-Koeffizienten ===
disp('Float-Koeffizienten:')
disp('b = '), disp(b)
disp('a = '), disp(a)

% === Umwandlung in Q15-Fixed-Point ===
scale = 2^15;
b_q15 = round(b * scale);
a_q15 = round(a(2:end) * scale);  % a(1) ist immer 1 → wird weggelassen

% === Ausgabe der Q15-Koeffizienten ===
disp('Q15-Koeffizienten (für C-Code):')
fprintf('int16_t b[3] = { %d, %d, %d };\n', b_q15)
fprintf('int16_t a[2] = { %d, %d };\n', a_q15)
 */
static void butterworth_filter_acc_Q15(const int16_t *acc, int16_t *acc_filt){
	const int16_t b0 = 44,	b1 = 88,	b2 = 44, // Q15
			a1 = -31023 ,a2 = 29454; // a1(Q14), a2(Q15)
	static int16_t x_n1[3] = {0}, x_n2[3] = {0},y_n1[3] = {0}, y_n2[3] = {0};
	int32_t x;
	x = ((((int32_t)y_n1[0] * -a1) >> 1) + (((int32_t)y_n2[0] * -a2) >> 2)) >> 3; // Q25
	x += ((((int32_t)acc[0] * b0) + ((int32_t)x_n1[0] * b1) + ((int32_t)x_n2[0] * b2)) >> 5); // Q25
	x = Q10_SHIFT_ROUND(x);
	acc_filt[0] = CLAMP_INT32_TO_INT16(x);

	x = ((((int32_t)y_n1[1] * -a1) >> 1) + (((int32_t)y_n2[1] * -a2) >> 2)) >> 3; // Q25
	x += ((((int32_t)acc[1] * b0) + ((int32_t)x_n1[1] * b1) + ((int32_t)x_n2[1] * b2)) >> 5); // Q25
	x = Q10_SHIFT_ROUND(x);
	acc_filt[1] = CLAMP_INT32_TO_INT16(x);

	x = ((((int32_t)y_n1[2] * -a1) >> 1) + (((int32_t)y_n2[2] * -a2) >> 2)) >> 3; // Q25
	x += ((((int32_t)acc[2] * b0) + ((int32_t)x_n1[2] * b1) + ((int32_t)x_n2[2] * b2)) >> 5); // Q25
	x = Q10_SHIFT_ROUND(x);
	acc_filt[2] = CLAMP_INT32_TO_INT16(x);

	copy3DvectorQ15(x_n1, x_n2);
	copy3DvectorQ15(acc, x_n1);
	copy3DvectorQ15(y_n1, y_n2);
	copy3DvectorQ15(acc_filt, y_n1);
}

static void madgwick_iir_filter_acc_Q15(const int16_t *acc_raw, int16_t *acc_filter){
//	const int16_t fc = 20; //hz
//	static int32_t a = CLAMP_INT32_TO_INT16((int32_t)((PI_MULTIPLY_2 * (float)fc * (1.0f/(float)SENSOR_FUSION_FREQUENCY_IMU))/(PI_MULTIPLY_2 * (float)fc * (1.0f/(float)SENSOR_FUSION_FREQUENCY_IMU) + 1.0f)));
//	const int16_t a = 4294; // ~0.131 * Q15 // 24 Hz
	const int16_t a = 3658; // 20 Hz
	static int32_t acc_old[3] = {0};


	acc_old[0] = ((a * (int32_t)acc_raw[0]) >> 15) + ((((int32_t)Q15 - a) * acc_old[0]) >> 15);
	acc_old[1] = ((a * (int32_t)acc_raw[1]) >> 15) + ((((int32_t)Q15 - a) * acc_old[1]) >> 15);
	acc_old[2] = ((a * (int32_t)acc_raw[2]) >> 15) + ((((int32_t)Q15 - a) * acc_old[2]) >> 15);

	acc_filter[0] = acc_old[0] = CLAMP_INT32_TO_INT16(acc_old[0]);
	acc_filter[1] = acc_old[1] = CLAMP_INT32_TO_INT16(acc_old[1]);
	acc_filter[2] = acc_old[2] = CLAMP_INT32_TO_INT16(acc_old[2]);
}

static void madgwick_butterworth_lp20Hz_fs1k_Q30(const int16_t *acc_q15, int16_t *acc_filt_q15){
	static int32_t x1[3] = {0}, x2[3] = {0}; // Q30
	    static int32_t y1[3] = {0}, y2[3] = {0}; // Q30

	    for(int i=0;i<3;i++){
	        int32_t x0 = CLAMP(((acc_q15[i]) << 15),-Q30,Q30); // Q30

	        // y = b0*x0 + b1*x1 + b2*x2 - a1*y1 - a2*y2   (alles Q30)
	        int64_t y  = Q30_MUL(cf_bw.b0_Q30, x0);
	        y         += Q30_MUL(cf_bw.b1_Q30, x1[i]);
	        y         += Q30_MUL(cf_bw.b2_Q30, x2[i]);
	        y         -= Q30_MUL(cf_bw.a1_Q30, y1[i]); // a1 ist negativ -> korrektes Vorzeichen durch "-"
	        y         -= Q30_MUL(cf_bw.a2_Q30, y2[i]);

	        // Output Q15 (mit Rundung) + Clamp
	        acc_filt_q15[i] = CLAMP_INT32_TO_INT16(Q15_SHIFT_ROUND(y));

	        // Zustände schieben (alles Q30)
	        x2[i] = x1[i]; x1[i] = x0;
	        y2[i] = y1[i]; y1[i] = (int32_t)y; // y in Q30
	    }
}
static void madgwick_mag_butterworth_lp20Hz_fs1k_Q30(const int16_t *acc_q15, int16_t *acc_filt_q15){
	static int32_t x1[3] = {0}, x2[3] = {0}; // Q30
	    static int32_t y1[3] = {0}, y2[3] = {0}; // Q30

	    for(int i=0;i<3;i++){
	        int32_t x0 = CLAMP(((acc_q15[i]) << 15),-Q30,Q30); // Q30

	        // y = b0*x0 + b1*x1 + b2*x2 - a1*y1 - a2*y2   (alles Q30)
	        int64_t y  = Q30_MUL(cf_bw.b0_Q30, x0);
	        y         += Q30_MUL(cf_bw.b1_Q30, x1[i]);
	        y         += Q30_MUL(cf_bw.b2_Q30, x2[i]);
	        y         -= Q30_MUL(cf_bw.a1_Q30, y1[i]); // a1 ist negativ -> korrektes Vorzeichen durch "-"
	        y         -= Q30_MUL(cf_bw.a2_Q30, y2[i]);

	        // Output Q15 (mit Rundung) + Clamp
	        acc_filt_q15[i] = CLAMP_INT32_TO_INT16(Q15_SHIFT_ROUND(y));

	        // Zustände schieben (alles Q30)
	        x2[i] = x1[i]; x1[i] = x0;
	        y2[i] = y1[i]; y1[i] = (int32_t)y; // y in Q30
	    }
}
// ################### COMPLEMENTARY FILTER (cf) ###############################
static inline void copy_q(const int16_t *q_in, int16_t *q_copy){
	q_copy[0] = q_in[0];	q_copy[1] = q_in[1];	q_copy[2] = q_in[2];	q_copy[3] = q_in[3];
}
static inline void neg_q_Q15(int16_t *q){
	q[0] = -q[0];	q[1] = -q[1];	q[2] = -q[2];	q[3] = -q[3];
}

static void compl_filter_SLERP_EMA_quaternion_Q15(const int16_t *q_in, int16_t *q_out){

	const int16_t a_div2 = 1190; // fc = 12 Hz, Ts = 1/1000, a_div2 = a/2 ... \alpha = 1 - e^{-2\pi f_c T_s}
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

//	if((q_out[0] < 0) && sign_q){
//		// you're correct
//	}else{
//		neg_q_Q15(q_out);
//	}

}

static void compl_iir_filter_acc_Q15(const int16_t *acc_raw, int16_t *acc_filter){

//	static int32_t a = CLAMP_INT32_TO_INT16((int32_t)((PI_MULTIPLY_2 * (float)fc * (1.0f/(float)SENSOR_FUSION_FREQUENCY_IMU))/(PI_MULTIPLY_2 * (float)fc * (1.0f/(float)SENSOR_FUSION_FREQUENCY_IMU) + 1.0f)));
	const int16_t a = 2297; // ~0.0701 * Q15 // 12 Hz
	static int32_t acc_old[3] = {0};


	acc_old[0] = ((a * (int32_t)acc_raw[0]) >> 15) + ((((int32_t)Q15 - a) * acc_old[0]) >> 15);
	acc_old[1] = ((a * (int32_t)acc_raw[1]) >> 15) + ((((int32_t)Q15 - a) * acc_old[1]) >> 15);
	acc_old[2] = ((a * (int32_t)acc_raw[2]) >> 15) + ((((int32_t)Q15 - a) * acc_old[2]) >> 15);

	acc_filter[0] = acc_old[0] = CLAMP_INT32_TO_INT16(acc_old[0]);
	acc_filter[1] = acc_old[1] = CLAMP_INT32_TO_INT16(acc_old[1]);
	acc_filter[2] = acc_old[2] = CLAMP_INT32_TO_INT16(acc_old[2]);
}


static void iir_filter_gyro_Q15(const int16_t *gyro_raw, int16_t *gyro_filter){
//	const int16_t fc = 40; //hz
//	static int32_t a = CLAMP_INT32_TO_INT16((int32_t)((PI_MULTIPLY_2 * (float)fc * (1.0f/(float)SENSOR_FUSION_FREQUENCY_IMU))/(PI_MULTIPLY_2 * (float)fc * (1.0f/(float)SENSOR_FUSION_FREQUENCY_IMU) + 1.0f)));
//	const int16_t a = 6581; // ~0.2 * Q15 // 20 Hz
	const int16_t a = 3329; // fc = 18
	static int32_t acc_old[3] = {0};


	acc_old[0] = ((a * (int32_t)gyro_raw[0]) >> 15) + ((((int32_t)Q15 - a) * acc_old[0]) >> 15);
	acc_old[1] = ((a * (int32_t)gyro_raw[1]) >> 15) + ((((int32_t)Q15 - a) * acc_old[1]) >> 15);
	acc_old[2] = ((a * (int32_t)gyro_raw[2]) >> 15) + ((((int32_t)Q15 - a) * acc_old[2]) >> 15);

	gyro_filter[0] = acc_old[0] = CLAMP_INT32_TO_INT16(acc_old[0]);
	gyro_filter[1] = acc_old[1] = CLAMP_INT32_TO_INT16(acc_old[1]);
	gyro_filter[2] = acc_old[2] = CLAMP_INT32_TO_INT16(acc_old[2]);
}



static void integrate_gyro_dot_Q15(const int16_t *q_est, const int16_t *q_dot, const int16_t delta_t, int16_t *q_int){

	q_int[0] = CLAMP_INT32_TO_INT16((int32_t)q15_mul(q_dot[0],delta_t) + q_est[0]);
	q_int[1] = CLAMP_INT32_TO_INT16((int32_t)q15_mul(q_dot[1],delta_t) + q_est[1]);
	q_int[2] = CLAMP_INT32_TO_INT16((int32_t)q15_mul(q_dot[2],delta_t) + q_est[2]);
	q_int[3] = CLAMP_INT32_TO_INT16((int32_t)q15_mul(q_dot[3],delta_t) + q_est[3]);

	NormalizeQuaternionQ15(q_int, q_int);
}

static void cf_positive_quaternion_test_Q15(int16_t *q){
	static int16_t q_old[4] = {Q15, 0, 0,0};
	// unit quaternion: "naa 32 bit are enouth"
	int32_t x = q15_mul(q[0],q_old[0]) + q15_mul(q[1],q_old[1]) + q15_mul(q[2],q_old[2]) + q15_mul(q[3],q_old[3]);

	if(x < 0) q_t_flipp(q);

	q_old[0] = q[0];
	q_old[1] = q[1];
	q_old[2] = q[2];
	q_old[3] = q[3];
}

//static void horizontal_projection(const int16_t *l, int16_t *q){
//	int16_t q_1, q_4;
//	int32_t  tau_q30,lx_sqrt_tau_q30; // bitte umschreiben zu tau_q15,lx_sqrt_tau_q15
//	uint32_t x;
//	uint64_t x64;
//	if((q15_mul(l[0], l[0]) + q15_mul(l[1], l[1])) < 5){
//		q[0] = Q15,		q[1] = 0;	q[2] = 0;	q[3] = 0;
//		return;
//	}
//	x = ((int32_t)l[0] * (int32_t)l[0]); // Q30
//	x += ((int32_t)l[1] * (int32_t)l[1]); // Q30
//	tau_q30 = x;
//
////	ly_div_sqrt2 = q15_mul(l[0], SQRT_2_OVER_2_Q15);
//	x = CLAMP((tau_q30 << 1), -Q30, Q30); // tau * 2
////	sqrt_2tau = sqrt_fast_uint(x);
//	if(l[0] >= 0){
//		lx_sqrt_tau_q30 = sqrt_fast_uint(tau_q30) * l[0];
//		x = (lx_sqrt_tau_q30 + tau_q30) >> 1; // Q30
//		x64 = ((uint64_t)(x << 30) / tau_q30); // Q60/Q30 = Q30
//		x = (x64 > Q30)? Q30: (uint32_t)x64;
//		q_1 = CLAMP_INT32_TO_INT16((int32_t)sqrt_fast_uint(x));
//
//		// sqrt(ly^2/(2*(tau+lxsqrt(tau)))
//		x = ((int32_t)l[1] * (int32_t)l[1]); // Q30
//		x64 = ((uint64_t)x << 29); // Q60 : -> (x/2) << 30
//		x = (lx_sqrt_tau_q30 + tau_q30);
//		x64 = x64/x;
//		x = (x64 > Q30)? Q30: (uint32_t)x64;
//		q_4 = CLAMP_INT32_TO_INT16((int32_t)sqrt_fast_uint(x));
//	}else{
//		lx_sqrt_tau_q30 = sqrt_fast_uint(tau_q30) * (-l[0]);
//		x = (lx_sqrt_tau_q30 + tau_q30) >> 1; // Q30
//		x64 = ((uint64_t)(x << 30) / tau_q30); // Q60/Q30 = Q30
//		x = (x64 > Q30)? Q30: (uint32_t)x64;
//		q_4 = CLAMP_INT32_TO_INT16((int32_t)sqrt_fast_uint(x));
//
//		// sqrt(ly^2/(2*(tau-lxsqrt(tau)))
//		x = ((int32_t)l[1] * (int32_t)l[1]); // Q30
//		x64 = ((uint64_t)x << 29); // Q60 : -> (x/2) << 30
//		x = (lx_sqrt_tau_q30 + tau_q30);
//		x64 = x64/x;
//		x = (x64 > Q30)? Q30: (uint32_t)x64;
//		q_1 = CLAMP_INT32_TO_INT16((int32_t)sqrt_fast_uint(x));
//	}
//
//	q[0] = q_1;
//	q[1] = 0;
//	q[2] = 0;
//	q[3] = q_4;
//}
static inline int16_t q15_div(int32_t a, int32_t b)
{
    if (b == 0)
        return (a >= 0) ? Q15 : -Q15; // Division durch 0 absichern

    int32_t result = (a << 15) / b;   // Q15-Ergebnis

    if (result >  32767) result =  32767;
    if (result < -32767) result = -32767;

    return (int16_t)result;
}
static void horizontal_projection(const int16_t *l, int16_t *q)
{
	int16_t q_1, q_4;
	int16_t tau_q15, lx_sqrt_tau_q15;
	int32_t x;
	if ((q15_mul(l[0], l[0]) + q15_mul(l[1], l[1])) < 5)
	{
		q[0] = Q15; q[1] = 0; q[2] = 0; q[3] = 0;
		return;
	}

	// tau = l0^2 + l1^2  (Q15)
	x = (int32_t)l[0] * (int32_t)l[0];  // Q30
	x += (int32_t)l[1] * (int32_t)l[1]; // Q30
	tau_q15 = (int16_t)CLAMP((x >> 15), -Q15, Q15);


//	if (l[0] <= 0)

//	if (l[0] >= 0)
	if (l[0] >= 0)
	{
		// lx_sqrt_tau_q15 = l0 * sqrt(tau)
		lx_sqrt_tau_q15 = q15_mul(l[0], sqrt_fast_uint(tau_q15));

		// q1 = sqrt((tau + l0*sqrt(tau)) / (2*tau))
		x = (int32_t)(tau_q15 + lx_sqrt_tau_q15); // Q15
		x = q15_div(x, (tau_q15 << 1));           // (tau + lxsqrt)/2tau  -> Q15
		q_1 = sqrt_fast_uint(CLAMP(x, 0, Q15));

		// q4 = sqrt((l1^2) / (2*(tau + l0*sqrt(tau))))
		x = q15_mul(l[1], l[1]);                       // Q15
		x = q15_div(x, (int32_t)(tau_q15 + lx_sqrt_tau_q15) << 1);
		q_4 = sqrt_fast_uint(CLAMP(x, 0, Q15));
	}
	else
	{
		lx_sqrt_tau_q15 = q15_mul(-l[0], sqrt_fast_uint(tau_q15));

		// q4 = sqrt((tau - l0*sqrt(tau)) / (2*tau))
		x = (int32_t)(tau_q15 - lx_sqrt_tau_q15); // Q15
		x = q15_div(x, (tau_q15 << 1));           // (tau - lxsqrt)/2tau
		q_4 = sqrt_fast_uint(CLAMP(x, 0, Q15));

		// q1 = sqrt((l1^2) / (2*(tau - l0*sqrt(tau))))
		x = q15_mul(l[1], l[1]);                       // Q15
		x = q15_div(x, (int32_t)(tau_q15 - lx_sqrt_tau_q15) << 1);
		q_1 = sqrt_fast_uint(CLAMP(x, 0, Q15));
	}

	q[0] = q_1;
	q[1] = 0;
	q[2] = 0;
	q[3] = q_4;
}

static int16_t float_to_Q15(float x){
	return CLAMP_INT32_TO_INT16((int32_t)(x * (float)Q15));
}

static void compl_minimal_rotation(const int16_t *a, const int16_t *b, int16_t *q_out)
{
    int16_t v[3], c_q15;
    static int16_t q_out_last[4] = {Q15,0,0,0};
    crossproduct_3x3_Q15(a, b, v);
    dotporduct_3x3_Q15(a, b, &c_q15); // dot in Q15 ∈ [-Q15, Q15]

    // --- 180°-Sonderfall: a ≈ -b
    // Schwelle leicht lockern, um numerische Spikes abzufangen
    if (c_q15 <= (-Q15 + 64)) {
        // Achse ⟂ a wählen: nimm eine Basis, die NICHT fast parallel ist
        int16_t ref[3] = { Q15, 0, 0 }; // [1,0,0] in Q15
        if ((a[0] > 29490) || (a[0] < -29490)) { // |ax| > ~0.9
            ref[0] = 0; ref[1] = 32767; ref[2] = 0; // [0,1,0]
        }
        crossproduct_3x3_Q15(a, ref, v);

        norm_3d_vector(v, v); // Achse normieren
        q_out[0] = 0;         // cos(180°/2) = 0
        q_out[1] = v[0];      // sin(180°/2)=1 → Vektorteil = Achse
        q_out[2] = v[1];
        q_out[3] = v[2];

        int16_t dot;
        dotporduct_4x4_Q15(q_out,q_out_last, &dot);
        if(dot <0) q_t_flipp(q_out);
        return;
    }

    // --- Regulärer Fall: s = sqrt((1 + dot)/2) in Q15
    // (1 + c)/2 in Q15:
    int32_t half_q15 = ((int32_t)Q15 + (int32_t)c_q15) >> 1;    // Q15
    // Für die Wurzel in Q15 arbeite in Q30 → sqrt(Q30) = Q15
    uint32_t half_q30 = (uint32_t)half_q15 << 15;                 // Q30
    uint16_t s_q15 = (uint16_t)sqrt_fast_uint(half_q30);               // Q15

    // den = 2*s in Q15
    int32_t den = ((int32_t)s_q15) << 1;
    if (den < 8) { // extrem kleiner Winkel → Identität
        q_out[0] = Q15; q_out[1] = q_out[2] = q_out[3] = 0;
        return;
    }

    // q_vec = (a×b) / (2*s)
    // Division Q15/Q15 → um Q15 beizubehalten: (v<<15)/den
    q_out[0] = (int16_t)CLAMP_INT32_TO_INT16((int32_t)s_q15);
    q_out[1] = (int16_t)CLAMP_INT32_TO_INT16(((int32_t)v[0] << 15) / den);
    q_out[2] = (int16_t)CLAMP_INT32_TO_INT16(((int32_t)v[1] << 15) / den);
    q_out[3] = (int16_t)CLAMP_INT32_TO_INT16(((int32_t)v[2] << 15) / den);

    NormalizeQuaternionQ15(q_out, q_out);
}



static void butterworth_lp20Hz_fs1k_Q30(const int16_t *acc_q15, int16_t *acc_filt_q15){
	static int32_t x1[3] = {0}, x2[3] = {0}; // Q30
	    static int32_t y1[3] = {0}, y2[3] = {0}; // Q30

	    for(int i=0;i<3;i++){
	        int32_t x0 = CLAMP(((acc_q15[i]) << 15),-Q30,Q30); // Q30

	        // y = b0*x0 + b1*x1 + b2*x2 - a1*y1 - a2*y2   (alles Q30)
	        int64_t y  = Q30_MUL(cf_bw.b0_Q30, x0);
	        y         += Q30_MUL(cf_bw.b1_Q30, x1[i]);
	        y         += Q30_MUL(cf_bw.b2_Q30, x2[i]);
	        y         -= Q30_MUL(cf_bw.a1_Q30, y1[i]); // a1 ist negativ -> korrektes Vorzeichen durch "-"
	        y         -= Q30_MUL(cf_bw.a2_Q30, y2[i]);

	        // Output Q15 (mit Rundung) + Clamp
	        acc_filt_q15[i] = CLAMP_INT32_TO_INT16(Q15_SHIFT_ROUND(y));

	        // Zustände schieben (alles Q30)
	        x2[i] = x1[i]; x1[i] = x0;
	        y2[i] = y1[i]; y1[i] = (int32_t)y; // y in Q30
	    }
}

//static int32_t norm_of_3D_vector(const int16_t *v){
//	uint32_t x;
//	x = (((int32_t)v[0] * (int32_t)v[0])) + (((int32_t)v[1] * (int32_t)v[1])) + (((int32_t)v[2] * (int32_t)v[2])); // Q30
//	return (int32_t)sqrt_fast_uint(x);
//}
int16_t debug_cf_error;
int16_t limit_high = 600;
int16_t limit_low = 50;
static void adaptive_acc_cf_Q15(const int16_t *acc_raw, int16_t *beta){
	int32_t x;
	static int16_t beta_min,beta_max,beta_diff;
	static bool init = 0;
	if(!init){
		init = 1;
		beta_min = CLAMP_INT32_TO_INT16((int32_t)((float)Q15 * COMPL_FILTER_MIN_BETA));
		beta_max = CLAMP_INT32_TO_INT16((int32_t)((float)Q15 * COMPL_FILTER_MAX_BETA));
		beta_diff = CLAMP((beta_max - beta_min),0,Q15);
	}

	x = CLAMP_INT32_TO_INT16(norm_of_3D_vector(acc_raw) - (Q11)); // norm(a) -g // acc -> 16g/Q15 // g = (Q15 / 12)
	x = abs(x);
	debug_cf_error = x;
//	const int16_t limit_high = 50;
//	const int16_t limit_low = 5;
	int16_t diff = abs(limit_high - limit_low);

	if(x > limit_high){
		beta[0] = beta_min; //= 0;
	}else{
		if(x < limit_low){
			beta[0] = beta_max; //(Q15-1);
		}else{
//			beta[0] = CLAMP((((int32_t)Q15 - ((x - limit_low) * (int32_t)Q15)/diff)), 0, (Q15-1));
			beta[0] = CLAMP((((int32_t)beta_max - ((x - limit_low) * (int32_t)beta_diff)/diff)), 0, beta_max);
		}
	}

}
xyz_16t comp_gyro_deb, debug_acc_norm, mag_compl_input, debug_mag_data;
wxyz_16t comp_gyro, comp_acc_corr, comp_qest, comp_minRot, comp_mag_corr,comp_qest_filter;
int16_t debug_cf_beta, debug_cf_convention, only_pitch_component;
float comp_euler_pitch,comp_euler_roll, comp_euler_yaw;
float comp_beta_mag_f = 0.3;
static void Complementary_filter_acc_gyro_mag(sensor_fusion *pHandle_sf, const bool acc_on,const bool mag_on){
	int16_t acc[3], acc_norm[3], acc_filter[3], gyro[3],qDot[4],q_gyro[4],omega[4], mag_raw[3], mag_equalized[3], mag_norm[3],mag_q[3];
	int16_t q_est[4];
	const int16_t q_0[4] = {Q15, 0,0,0};
//	static int16_t delta_t = CLAMP_INT32_TO_INT16((int32_t)((float)(34.9f/(float)SENSOR_FUSION_FREQUENCY_IMU) * (float)Q15));
	static int16_t delta_t = CLAMP_INT32_TO_INT16((int32_t)((float)((1.0f * (float)Q4)/(float)SENSOR_FUSION_FREQUENCY_IMU) * (float)Q15)); // Q4 -> to get rad/s
	//get values
	acc[0] = pHandle_sf->acc_t.x;
	acc[1] = pHandle_sf->acc_t.y;
	acc[2] = pHandle_sf->acc_t.z;

	gyro[0] = q15_mul(pHandle_sf->gyro_t.x, GRAD2RAD_GYRO_MAX_Q15);
	gyro[1] = q15_mul(pHandle_sf->gyro_t.y, GRAD2RAD_GYRO_MAX_Q15);
	gyro[2] = q15_mul(pHandle_sf->gyro_t.z, GRAD2RAD_GYRO_MAX_Q15);

	omega[0] = 0;	omega[1] = gyro[0];	omega[2] = gyro[1];	omega[3] = gyro[2];

	if(PREFILTER_SF){
//	iir_filter_gyro_Q15(gyro,gyro);
	if(PREF_METHODE == IIR) compl_iir_filter_acc_Q15(acc, acc);
	if(PREF_METHODE == BUTTERWORTH) butterworth_lp20Hz_fs1k_Q30(acc,acc);
	}

	norm_3d_vector(acc, acc_norm);
	multiplicateQuaternionQ15(q_compl_out,omega,qDot);
//	divideQuaternionBy2(qDot); // 	// we include this part in delta_t
	integrate_gyro_dot_Q15(q_compl_out, qDot,delta_t,q_est);

//	comp_gyro.w = q_est[0];	comp_gyro.x = q_est[1];
//	comp_gyro.y = q_est[2];	comp_gyro.z = q_est[3];

	if(acc_on == true){
		int16_t v[3], min_rot_a[4], q_acc_corr[4], q_est_conj[4],adapt_beta;
		const int16_t g[3] = {0,0,-Q15};
		q_t_conj_function_in_out_q15(q_est,q_est_conj);
		rotate_vector_Q15(q_est_conj,acc_norm,v);
//		rotate_vector_Q15(acc_norm,q_est_conj,v);
		debug_acc_norm.x = acc_norm[0];	debug_acc_norm.y = acc_norm[1];
		debug_acc_norm.z = acc_norm[2];
		debug_cf_v_x = v[0];
		debug_cf_v_y = v[1];
		debug_cf_v_z = v[2];
//		int16_t test_v[3] = {Q15,0,0};
//		int16_t test_g[3] = {0,Q15,0};
//		minimal_rotation(v, g, min_rot_a); //
		minimal_rotation(g, v, min_rot_a); //
//		if(min_rot_a[0] < 0) q_t_flipp(min_rot_a); // need the same convention for nLERP
//		compl_minimal_rotation(test_v, test_g, min_rot_a); //
		comp_minRot.w = min_rot_a[0]; 	comp_minRot.x = min_rot_a[1];
		comp_minRot.y = min_rot_a[2];	comp_minRot.z = min_rot_a[3];
		adaptive_acc_cf_Q15(acc, &adapt_beta);
		debug_cf_beta = adapt_beta;
		debug_cf_convention = min_rot_a[0];

		nLERP_quaternion_Q15(q_0,min_rot_a, adapt_beta,q_acc_corr);
		// Debug

		comp_acc_corr.w = q_acc_corr[0];	comp_acc_corr.x = q_acc_corr[1];
		comp_acc_corr.y = q_acc_corr[2];	comp_acc_corr.z = q_acc_corr[3];


		multiplicateQuaternionQ15(q_est, q_acc_corr, q_est);
//		multiplicateQuaternionQ15(q_est, q_corr_conj, q_est);

	}

	if(mag_on == true){
		mag_raw[0] = pHandle_sf->mag_t.x;
		mag_raw[1] = pHandle_sf->mag_t.y;
		mag_raw[2] = pHandle_sf->mag_t.z; // raw z-axis point up
		hardiron_apply_q15(mag_raw);
		softiron_apply_q15(mag_raw, mag_equalized);
		norm_3d_vector(mag_equalized, mag_norm);

		debug_mag_data.x = mag_norm[0];
		debug_mag_data.y = mag_norm[1];
		debug_mag_data.z = mag_norm[2];

		int16_t q_est2_con[4], l[3],q_mag[4], q_mag_corr[4];

		q_t_conj_function_in_out_q15(q_est,q_est2_con);
		rotate_vector_Q15(q_est2_con,mag_norm,l);
		horizontal_projection(l,q_mag);

		if(q_mag[0] < 0) q_t_flipp(q_mag); //
		int16_t mag_beta = float_to_Q15(comp_beta_mag_f);
		nLERP_quaternion_Q15(q_mag,q_0, mag_beta,q_mag_corr);
		comp_mag_corr.w = q_mag_corr[0];	comp_mag_corr.x = q_mag_corr[1];
		comp_mag_corr.y = q_mag_corr[2];	comp_mag_corr.z = q_mag_corr[3];
//		q_t_conj_function(q_mag_corr);
//		q_mag_corr[0] = Q15;	q_mag_corr[1] = 0; q_mag_corr[2] = 0; q_mag_corr[3] = 0;
		multiplicateQuaternionQ15(q_est, q_mag_corr, q_est);
		if(q_est[0] < 0) q_t_flipp(q_est); //
	}
	if(OFF){
	int16_t q_est_filter[4];
	compl_filter_SLERP_EMA_quaternion_Q15(q_est,q_est_filter);
//	cf_positive_quaternion_test_Q15(q_est); // TEST
	comp_qest.w = q_est[0];		comp_qest.x = q_est[1];
	comp_qest.y = q_est[2];		comp_qest.z = q_est[3];
	comp_qest_filter.w = q_est_filter[0];	comp_qest_filter.x = q_est_filter[1];
	comp_qest_filter.y = q_est_filter[2];	comp_qest_filter.z = q_est_filter[3];
	}
//	q_compl_out[0] = q_gyro[0];	q_compl_out[1] = q_gyro[1];
//	q_compl_out[2] = q_gyro[2];	q_compl_out[3] = q_gyro[3];
	if(DEBUG_MODE == ON){
		int16_t q_est_[4],euler_debug[3];
		copy_q(q_est, q_est_);
//		q_t_conj_function_in_out_q15(q_est, q_est_);
		quat_to_euler_q15(q_est_, euler_debug);
		comp_euler_roll = (int16_t)(euler_debug[0]-Q15);// * 180.0f / (float)Q15;// * 360.0f / (float)INT16_MAX;
		comp_euler_pitch = -(int16_t)(euler_debug[1]-Q15);// * 180.0f / (float)Q15;// * 360.0f / (float)INT16_MAX;
		comp_euler_yaw = -(float)euler_debug[2];// * 180.0f / (float)Q15;// * 360.0f / (float)INT16_MAX;
	}

	q_t_conj_function_in_out_q15(q_est,q_est);
	q_compl_out[0] = q_est[0];	q_compl_out[1] = q_est[1];
	q_compl_out[2] = q_est[2];	q_compl_out[3] = q_est[3];


//	int16_t q_twist[4], q_est_co[4];
//	q_t_conj_function_in_out_q15(q_est,q_est_co);
//	twist_y_axis(q_est_co, q_twist);
//	if(q_twist[0] < 0) q_t_flipp(q_twist);
//	only_pitch_component = (q15_atan2(q_twist[2],q_twist[0]) << 1);
//	int32_t x = (int32_t)q_est_co
//	only_pitch_component = sin_Q15()
}


static void complementary_filter_init(void){
	if(PREF_METHODE == BUTTERWORTH){
		cf_bw.a1_Q30 = -1957103774; 	// -1.82269493 * 2^30
		cf_bw.a2_Q30 = 898916953; 		//  0.83718165 * 2^30
		cf_bw.b0_Q30 = 3888751;   		// 0.00362168 * 2^30
		cf_bw.b1_Q30 = 7777502;   		// 0.00724336 * 2^30
		cf_bw.b2_Q30 = 3888751;   		// 0.00362168 * 2^30
	}
}
void create_init_yaw_quaternion(int16_t *q_out){
	int16_t a[3], m[3], h[2];
	int32_t x;
	m[0] = sf_values.mag_t.x;	m[1] = sf_values.mag_t.y;	m[2] = sf_values.mag_t.z;
	a[0] = sf_values.acc_t.x;	a[1] = sf_values.acc_t.y;	a[2] = sf_values.acc_t.z;

	hardiron_apply_q15(m);
	softiron_apply_q15(m, m);

	norm_3d_vector(a, a);
	norm_3d_vector(m, m);


	x = ((int32_t)a[1] * (int32_t)a[1]) + ((int32_t)a[2] * (int32_t)a[2]);
	x = CLAMP_INT32_TO_INT16((int32_t)sqrt_fast_uint((uint32_t)x));
	int16_t psi 	= q15_atan2(-a[1],a[2]);
	int16_t theta	= q15_atan2(a[0],(int16_t)x);

	int16_t s_theta_s_psi = q15_mul(sin_Q15(theta), sin_Q15(psi));
	int16_t s_psi_c_theta = q15_mul(sin_Q15(psi), cos_Q15(theta));
	int16_t h_x		= CLAMP_INT32_TO_INT16((int32_t)q15_mul(m[0], cos_Q15(theta)) - (int32_t)q15_mul(m[2], sin_Q15(theta)));
	int16_t h_y		= CLAMP_INT32_TO_INT16((int32_t)q15_mul(m[0], s_theta_s_psi) + (int32_t)q15_mul(m[1],cos_Q15(psi)) + (int32_t)q15_mul(m[2],s_psi_c_theta));

	int16_t yaw_angle = q15_atan2(h_x, h_y);
//	int16_t yaw_angle = q15_atan2(h_y, h_x);
	q_out[0] = cos_Q15(-(yaw_angle >> 1));
	q_out[1] = 0;
	q_out[2] = 0;
	q_out[3] = sin_Q15(-(yaw_angle >> 1));

}


static void ERROR_Blink_LED(void){
	while(1){
		system_stop_function();
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET); // LED AN
		HAL_Delay(300);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET); // LED AN
		HAL_Delay(300);
	}
}



//// Q15 Konstanten
//#define Q15_MAX   32767
//#define Q15_ONE   32767
//#define CLAMP16(x)   ( (x) > 32767 ? 32767 : ((x) < -32768 ? -32768 : (x)) )
//#define CLAMP32(x,lo,hi) ( (x) < (lo) ? (lo) : ((x) > (hi) ? (hi) : (x)) )





// Optional: am Boden Zusatz-Gewichte (additiv zur Basis)
// kR_q15 = wieviel R zusätzlich am Boden (Q15/„gleiche Einheit“ wie R_q15)
// kTOL_q15 = zusätzliche Toleranz am Boden (Q15, gleiche Einheit wie NORM_TOL_q15)
#ifndef kR_q15
#define kR_q15   600   // z.B. = R0 -> am Boden doppelte Mess-Varianz
#endif
#ifndef kTOL_q15
#define kTOL_q15 1638  // z.B. = TOL0 -> am Boden Toleranz +5% (gesamt ~±10%)
#endif


// ============================ KF =============================

//static void magKF_Q15(int16_t *mag_norm)
//{
//    // ------------------------ Adaptiv-Teil (Höhenabhängig) ------------------------
//    // g(h) = (h0/(h+h0))^3   -> hier in Q10 berechnet und am Ende nach Q15 konvertiert
//    const int16_t h0_cm = 20;                 // Übergangshöhe: 20 cm
//    const int32_t h0_q10 = (int32_t)h0_cm * Q10;
//    int32_t h_cm        = (int32_t)get_hight_of_drone_cm_irq_save();
//    int32_t h_q10       = h_cm * Q10;
//
//    // r = h0/(h+h0) in Q10
//    int32_t den_q10 = h_q10 + h0_q10;         // Q10
//    if (den_q10 <= 0) den_q10 = 1;
//    int32_t r_q10 = ( (int64_t)h0_q10 << 10 ) / den_q10; // (Q10 <<10)/Q10 -> Q10
//
//    // g_q10 = r^3 in Q10: erst r^2 (Q10), dann * r (Q10): (r*r >>10) * r >>10
//    int32_t r2_q10 = ( (int64_t)r_q10 * r_q10 ) >> 10;  // Q10
//    int32_t g_q10  = ( (int64_t)r2_q10 * r_q10 ) >> 10; // Q10
//    if (g_q10 < 0) g_q10 = 0; if (g_q10 > Q10) g_q10 = Q10;
//
//    // nach Q15 heben (Q10 -> Q15: <<5)
//    int32_t g_q15 = g_q10 << 5; // ∈[0, Q15]
//
//    // ------------------------- Tuning (Basiswerte) -------------------------
//    const int16_t Q_base_q15   = 20;   // Prozessrauschen (klein = mehr Glättung)
//    const int16_t R0_q15       = 600;  // Basis-Messrauschen
//    const int16_t TOL0_q15     = 3277; // ±10% (falls du ±5% willst: 1638)
//
//    // Adaptive Werte
//    // R(h) = R0 + kR * g(h)
//    int32_t R_q15_h   = R0_q15 + ( ( (int32_t)kR_q15   * g_q15 ) >> 15 );
//    if (R_q15_h < 1) R_q15_h = 1;
//
//    // TOL(h) = TOL0 + kTOL * g(h)
//    int32_t NORM_TOL_q15_h = TOL0_q15 + ( ( (int32_t)kTOL_q15 * g_q15 ) >> 15 );
//    if (NORM_TOL_q15_h < 0) NORM_TOL_q15_h = 0;
//
//    // ------------------------- Zustände (static) -------------------------
//    static int32_t x_q15[3] = {0};   // Zustand (Q15) in 32-bit
//    static int32_t P_q31[3] = {0};   // Kovarianz (Q31), diagonal
//    static uint8_t init = 0;
//
//    if (!init) {
//        // Initialisierung beim ersten Aufruf
//        x_q15[0] = mag_norm[0];
//        x_q15[1] = mag_norm[1];
//        x_q15[2] = mag_norm[2];
//        // Start-Kovarianz ~1.0 in Q31: Q15 << 15
//        P_q31[0] = (int32_t)Q15 << 15;
//        P_q31[1] = (int32_t)Q15 << 15;
//        P_q31[2] = (int32_t)Q15 << 15;
//        init = 1;
//        // kein Update im Init-Schritt – weiter unten wird normiert & zurückgegeben
//    } else {
//        // ------------------------- Gating (Norm) -------------------------
//        int32_t mx = mag_norm[0], my = mag_norm[1], mz = mag_norm[2];
//        int64_t n2 = (int64_t)mx*mx + (int64_t)my*my + (int64_t)mz*mz; // Q30
//        uint32_t n = sqrt_fast_uint((uint32_t)(n2 > 0 ? n2 : 0));       // ~Q15
//
//        bool accept_update = false;
//        if (n > 0) {
//            int32_t diff = (int32_t)n - Q15; // Abweichung der Norm zu 1.0 (Q15)
//            if (diff < 0) diff = -diff;
//            accept_update = (diff <= NORM_TOL_q15_h);
//        }
//
//        // Adaptives Prozessrauschen (hier Basis; du kannst es z.B. mit Gyro erweitern)
//        int16_t Q_q15 = Q_base_q15;
//
//        // ------------------------- KF Update (pro Achse) -------------------------
//        for (uint8_t i = 0; i < 3; i++) {
//            // Prediction
//            int32_t x_pred = x_q15[i];                           // Random-Walk
//            int32_t P_pred = P_q31[i] + ((int32_t)Q_q15 << 15);  // Q->Q31
//
//            int32_t x_upd = x_pred;
//            int32_t P_upd = P_pred;
//
//            if (accept_update) {
//                // K = P_pred / (P_pred + R(h))
//                int32_t denom = P_pred + ((int32_t)R_q15_h << 15); // Q31
//                if (denom <= 0) denom = 1;
//                int32_t K_q15 = (int32_t)(((int64_t)P_pred << 15) / denom); // Q15
//                if (K_q15 < 0) K_q15 = 0; if (K_q15 > Q15) K_q15 = Q15;
//
//                int32_t y_q15  = mag_norm[i];               // Messung (Q15)
//                int32_t innov  = y_q15 - x_pred;            // Q15
//                x_upd = x_pred + (int32_t)(((int64_t)K_q15 * innov) >> 15);
//                // P = (1-K) P_pred
//                P_upd = (int32_t)(((int64_t)(Q15 - K_q15) * P_pred) >> 15);
//            }
//
//            // Speichern
//            x_q15[i] = x_upd;     // im Q15-Bereich bleiben – Normierung folgt
//            P_q31[i] = P_upd;
//        }
//    }
//
//    // ------------------------- Normierung (rein Integer) -------------------------
//    int32_t vx = (int16_t)x_q15[0];
//    int32_t vy = (int16_t)x_q15[1];
//    int32_t vz = (int16_t)x_q15[2];
//
//    int64_t n2x = (int64_t)vx*vx + (int64_t)vy*vy + (int64_t)vz*vz; // Q30
//    if (n2x > 0) {
//        uint32_t n = sqrt_fast_uint((uint32_t)n2x); // ~Q15
//        if (n == 0) n = 1;
//        // scale = Q15 / n (Q15), v_scaled = (v * scale)>>15
//        int32_t scale_q15 = (int32_t)(((int64_t)Q15 << 15) / n);
//        vx = (int32_t)(((int64_t)vx * scale_q15) >> 15);
//        vy = (int32_t)(((int64_t)vy * scale_q15) >> 15);
//        vz = (int32_t)(((int64_t)vz * scale_q15) >> 15);
//    }
//
//    mag_norm[0] = (int16_t)CLAMP_INT32_TO_INT16(vx);
//    mag_norm[1] = (int16_t)CLAMP_INT32_TO_INT16(vy);
//    mag_norm[2] = (int16_t)CLAMP_INT32_TO_INT16(vz);
//}

//static void magKF_Q15(int16_t *mag_norm)
//{
//    // ------------------------ Adaptiv-Teil (Höhenabhängig) ------------------------
//#if HIGH_CORRECTION
//    // g(h) = (h0/(h+h0))^3   -> hier in Q10 berechnet und am Ende nach Q15 konvertiert
//    const int16_t h0_cm = 20;                 // Übergangshöhe: 20 cm
//    const int32_t h0_q10 = (int32_t)h0_cm * Q10;
//    int32_t h_cm        = (int32_t)get_hight_of_drone_cm_irq_save();
//    int32_t h_q10       = h_cm * Q10;
//
//    // r = h0/(h+h0) in Q10
//    int32_t den_q10 = h_q10 + h0_q10;         // Q10
//    if (den_q10 <= 0) den_q10 = 1;
//    int32_t r_q10 = ( (int64_t)h0_q10 << 10 ) / den_q10; // (Q10 <<10)/Q10 -> Q10
//
//    // g_q10 = r^3 in Q10
//    int32_t r2_q10 = ( (int64_t)r_q10 * r_q10 ) >> 10;  // Q10
//    int32_t g_q10  = ( (int64_t)r2_q10 * r_q10 ) >> 10; // Q10
//    if (g_q10 < 0) g_q10 = 0; if (g_q10 > Q10) g_q10 = Q10;
//
//    // nach Q15 heben
//    int32_t g_q15 = g_q10 << 5; // ∈[0, Q15]
//#else
//    // kein Höhen-Einfluss → g(h)=1.0 in Q15
//    int32_t g_q15 = Q15;
//#endif
//
//    // ------------------------- Tuning (Basiswerte) -------------------------
//    const int16_t Q_base_q15   = 20;   // Prozessrauschen
//    const int16_t R0_q15       = 600;  // Basis-Messrauschen
//    const int16_t TOL0_q15     = 3277; // ±10%
//
//    // Adaptive Werte
//    int32_t R_q15_h       = R0_q15 + ( ((int32_t)kR_q15   * g_q15) >> 15 );
//    if (R_q15_h < 1) R_q15_h = 1;
//
//    int32_t NORM_TOL_q15_h = TOL0_q15 + ( ((int32_t)kTOL_q15 * g_q15) >> 15 );
//    if (NORM_TOL_q15_h < 0) NORM_TOL_q15_h = 0;
//
//        // ------------------------- Zustände (static) -------------------------
//        static int32_t x_q15[3] = {0};   // Zustand (Q15) in 32-bit
//        static int32_t P_q31[3] = {0};   // Kovarianz (Q31), diagonal
//        static uint8_t init = 0;
//
//        if (!init) {
//            // Initialisierung beim ersten Aufruf
//            x_q15[0] = mag_norm[0];
//            x_q15[1] = mag_norm[1];
//            x_q15[2] = mag_norm[2];
//            // Start-Kovarianz ~1.0 in Q31: Q15 << 15
//            P_q31[0] = (int32_t)Q15 << 15;
//            P_q31[1] = (int32_t)Q15 << 15;
//            P_q31[2] = (int32_t)Q15 << 15;
//            init = 1;
//            // kein Update im Init-Schritt – weiter unten wird normiert & zurückgegeben
//        } else {
//            // ------------------------- Gating (Norm) -------------------------
//            int32_t mx = mag_norm[0], my = mag_norm[1], mz = mag_norm[2];
//            int64_t n2 = (int64_t)mx*mx + (int64_t)my*my + (int64_t)mz*mz; // Q30
//            uint32_t n = sqrt_fast_uint((uint32_t)(n2 > 0 ? n2 : 0));       // ~Q15
//
//            bool accept_update = false;
//            if (n > 0) {
//                int32_t diff = (int32_t)n - Q15; // Abweichung der Norm zu 1.0 (Q15)
//                if (diff < 0) diff = -diff;
//                accept_update = (diff <= NORM_TOL_q15_h);
//            }
//
//            // Adaptives Prozessrauschen (hier Basis; du kannst es z.B. mit Gyro erweitern)
//            int16_t Q_q15 = Q_base_q15;
//
//            // ------------------------- KF Update (pro Achse) -------------------------
//            for (uint8_t i = 0; i < 3; i++) {
//                // Prediction
//                int32_t x_pred = x_q15[i];                           // Random-Walk
//                int32_t P_pred = P_q31[i] + ((int32_t)Q_q15 << 15);  // Q->Q31
//
//                int32_t x_upd = x_pred;
//                int32_t P_upd = P_pred;
//
//                if (accept_update) {
//                    // K = P_pred / (P_pred + R(h))
//                    int32_t denom = P_pred + ((int32_t)R_q15_h << 15); // Q31
//                    if (denom <= 0) denom = 1;
//                    int32_t K_q15 = (int32_t)(((int64_t)P_pred << 15) / denom); // Q15
//                    if (K_q15 < 0) K_q15 = 0; if (K_q15 > Q15) K_q15 = Q15;
//
//                    int32_t y_q15  = mag_norm[i];               // Messung (Q15)
//                    int32_t innov  = y_q15 - x_pred;            // Q15
//                    x_upd = x_pred + (int32_t)(((int64_t)K_q15 * innov) >> 15);
//                    // P = (1-K) P_pred
//                    P_upd = (int32_t)(((int64_t)(Q15 - K_q15) * P_pred) >> 15);
//                }
//
//                // Speichern
//                x_q15[i] = x_upd;     // im Q15-Bereich bleiben – Normierung folgt
//                P_q31[i] = P_upd;
//            }
//        }
//
//        // ------------------------- Normierung (rein Integer) -------------------------
//        int32_t vx = (int16_t)x_q15[0];
//        int32_t vy = (int16_t)x_q15[1];
//        int32_t vz = (int16_t)x_q15[2];
//
//        int64_t n2x = (int64_t)vx*vx + (int64_t)vy*vy + (int64_t)vz*vz; // Q30
//        if (n2x > 0) {
//            uint32_t n = sqrt_fast_uint((uint32_t)n2x); // ~Q15
//            if (n == 0) n = 1;
//            // scale = Q15 / n (Q15), v_scaled = (v * scale)>>15
//            int32_t scale_q15 = (int32_t)(((int64_t)Q15 << 15) / n);
//            vx = (int32_t)(((int64_t)vx * scale_q15) >> 15);
//            vy = (int32_t)(((int64_t)vy * scale_q15) >> 15);
//            vz = (int32_t)(((int64_t)vz * scale_q15) >> 15);
//        }
//
//        mag_norm[0] = (int16_t)CLAMP_INT32_TO_INT16(vx);
//        mag_norm[1] = (int16_t)CLAMP_INT32_TO_INT16(vy);
//        mag_norm[2] = (int16_t)CLAMP_INT32_TO_INT16(vz);
//}
void magKF_Q15(int16_t *mag_norm)
{
    // --- Tuning (Q15-Skalierung) ---
    // Prozess- und Messrausch-"Varianzen" in Q15 (werden intern nach Q31 gehoben).
    // Größeres Q -> schneller, weniger Glättung; Größeres R -> mehr Glättung.
    const int16_t Q_base_q15 = 10; //20;    // ~0.0006
    const int16_t R_q15      = 600;   // ~0.0183

    // Gating-Schwellen (in Q15):
    // Norm nahe 1: erlaubter Bereich [1 - tol, 1 + tol]
    const int16_t NORM_TOL_q15 = 1638; // ~0.05

    // --- Zustände (static) ---
    static int32_t x_q15[3] = {0};       // Zustand (Q15 in int32_t)
    static int32_t P_q31[3] = {0};       // Kovarianz diagonal (Q31)
    static uint8_t init = 0;

    // --- Initialisierung beim ersten Aufruf ---
    if (!init) {
        x_q15[0] = mag_norm[0];
        x_q15[1] = mag_norm[1];
        x_q15[2] = mag_norm[2];
        // Startkovarianz moderat
        P_q31[0] = (int32_t)Q15 << 15;  // ~1.0 in Q31
        P_q31[1] = (int32_t)Q15 << 15;
        P_q31[2] = (int32_t)Q15 << 15;
        init = 1;
        // Normieren und zurückgeben
        // (damit ist der erste Output direkt konsistent)
    } else {
        // --- Norm prüfen (Gating) ---
        int32_t mx = mag_norm[0], my = mag_norm[1], mz = mag_norm[2];
        int64_t n2 = (int64_t)mx*mx + (int64_t)my*my + (int64_t)mz*mz; // Q30
        uint32_t n = sqrt_fast_uint((uint32_t)(n2 > 0 ? n2 : 0));      // ~Q15

        // Gate: nur updaten, wenn | ||m|| - 1 | <= tol
        bool accept_update = false;
        if (n > 0) {
            int32_t diff = (int32_t)n - Q15; // Q15-Differenz zur 1.0
            if (diff < 0) diff = -diff;
            accept_update = (diff <= NORM_TOL_q15);
        }

        // --- Adaptives Q (optional leicht adaptiv nach Innovation) ---
        // Innovation auf Normebene: wenn Messnorm weit weg, Q höher (aber wir gaten ohnehin)
        int16_t Q_q15 = Q_base_q15;

        // --- KF pro Achse (diagonal) ---
        for (uint8_t i = 0; i < 3; i++) {
            // Prediction
            int32_t x_pred = x_q15[i];
            int32_t P_pred = P_q31[i] + ((int32_t)Q_q15 << 15); // Q->Q31

            // Optional: ohne Update übernehmen
            int32_t x_upd = x_pred;
            int32_t P_upd = P_pred;

            if (accept_update) {
                // Kalman-Gain: K = P_pred / (P_pred + R)
                int32_t denom = P_pred + ((int32_t)R_q15 << 15); // Q31
                if (denom <= 0) denom = 1;
                // K_q15 = (P_pred / denom) in Q15
                int32_t K_q15 = (int32_t)(((int64_t)P_pred << 15) / denom);
                if (K_q15 < 0)      K_q15 = 0;
                if (K_q15 > Q15) K_q15 = Q15;

                // Update
                int32_t y_q15 = mag_norm[i];
                int32_t innov = y_q15 - x_pred;              // Q15
                x_upd = x_pred + (int32_t)(((int64_t)K_q15 * innov) >> 15); // Q15
                // P = (1-K) P_pred
                P_upd = (int32_t)(((int64_t)(Q15 - K_q15) * P_pred) >> 15);
            }
            // Speichern
            x_q15[i] = CLAMP_INT32_TO_INT16(x_upd); // bleibt in Q15-Grenzen
            P_q31[i] = P_upd;
        }
    }

    // --- Normierung auf Einheitsvektor (Q15), rein integer ---
    int32_t vx = (int16_t)x_q15[0];
    int32_t vy = (int16_t)x_q15[1];
    int32_t vz = (int16_t)x_q15[2];

    int64_t n2x = (int64_t)vx*vx + (int64_t)vy*vy + (int64_t)vz*vz; // Q30
    if (n2x > 0) {
        uint32_t n = sqrt_fast_uint((uint32_t)n2x); // ~Q15
        if (n == 0) n = 1;
        // scale = Q15_ONE / n  (beides Q15 → scale in Q15)
        // v_scaled = (v * scale) >> 15
        int32_t scale_q15 = (int32_t)(((int64_t)Q15 << 15) / n); // Q15
        vx = (int32_t)(((int64_t)vx * scale_q15) >> 15);
        vy = (int32_t)(((int64_t)vy * scale_q15) >> 15);
        vz = (int32_t)(((int64_t)vz * scale_q15) >> 15);
    }

    mag_norm[0] = (int16_t)CLAMP_INT32_TO_INT16(vx);
    mag_norm[1] = (int16_t)CLAMP_INT32_TO_INT16(vy);
    mag_norm[2] = (int16_t)CLAMP_INT32_TO_INT16(vz);
}

static void acc_KF_Q15(int16_t *acc_q15)
{
    // --- Tuning (Q15-Skalierung) ---
    // Prozess- und Messrauschen (intuitiv: kleiner Q = stärker geglättet)
    const int16_t Q_base_q15 = 80;   // etwas höher als Magnetometer
    const int16_t R_q15      = 1200;  // ~ etwas mehr Glättung

    // 1g in Q15 bei ±16g Range:
    // 1g = (1/16)*Q15 = 0.0625 * 32768 ≈ 2048
    const int16_t ONE_G_q15 = 2048;

    // Toleranz auf die Norm (z. B. ±0.2g erlaubt)
    const int16_t NORM_TOL_q15 = 2000; //400; // ~0.4g

//    // --- Tuning für starke Glättung ---
//    const int16_t Q_base_q15 = 4;     // kleiner = träger, stärker geglättet
//    const int16_t R_q15      = 2000;  // größer = Messung weniger stark gewichtet
//
//    // 1g in Q15 bei ±16g Range (1g = 0.0625 * 32768)
//    const int16_t ONE_G_q15 = 2048;
//
//    // erlaubte Abweichung der Norm (z. B. ±0.15g)
//    const int16_t NORM_TOL_q15 = 300;

    // --- Zustände (static) ---
    static int32_t x_q15[3] = {0};
    static int32_t P_q31[3] = {0};
    static uint8_t init = 0;

    // --- Initialisierung ---
    if (!init) {
        x_q15[0] = acc_q15[0];
        x_q15[1] = acc_q15[1];
        x_q15[2] = acc_q15[2];
        P_q31[0] = (int32_t)Q15 << 15;
        P_q31[1] = (int32_t)Q15 << 15;
        P_q31[2] = (int32_t)Q15 << 15;
        init = 1;
    } else {
        // --- Norm prüfen (Gating) ---
        int32_t ax = acc_q15[0];
        int32_t ay = acc_q15[1];
        int32_t az = acc_q15[2];
        int64_t n2 = (int64_t)ax*ax + (int64_t)ay*ay + (int64_t)az*az;
        uint32_t n = sqrt_fast_uint((uint32_t)(n2 > 0 ? n2 : 0)); // Q15-Skala relativ

        bool accept_update = false;
        if (n > 0) {
            int32_t diff = (int32_t)n - ONE_G_q15;
            if (diff < 0) diff = -diff;
            accept_update = (diff <= NORM_TOL_q15);
        }

        // --- KF-Update pro Achse ---
        for (uint8_t i = 0; i < 3; i++) {
            int32_t x_pred = x_q15[i];
            int32_t P_pred = P_q31[i] + ((int32_t)Q_base_q15 << 15);

            int32_t x_upd = x_pred;
            int32_t P_upd = P_pred;

            if (accept_update) {
                int32_t denom = P_pred + ((int32_t)R_q15 << 15);
                if (denom <= 0) denom = 1;

                int32_t K_q15 = (int32_t)(((int64_t)P_pred << 15) / denom);
                if (K_q15 < 0)      K_q15 = 0;
                if (K_q15 > Q15) K_q15 = Q15;

                int32_t y_q15 = acc_q15[i];
                int32_t innov = y_q15 - x_pred;
                x_upd = x_pred + (int32_t)(((int64_t)K_q15 * innov) >> 15);
                P_upd = (int32_t)(((int64_t)(Q15 - K_q15) * P_pred) >> 15);
            }

            x_q15[i] = CLAMP_INT32_TO_INT16(x_upd);
            P_q31[i] = P_upd;
        }
    }

    // --- Normierung auf 1g (Q15) ---
    int32_t vx = (int16_t)x_q15[0];
    int32_t vy = (int16_t)x_q15[1];
    int32_t vz = (int16_t)x_q15[2];

    int64_t n2x = (int64_t)vx*vx + (int64_t)vy*vy + (int64_t)vz*vz;
    if (n2x > 0) {
        uint32_t n = sqrt_fast_uint((uint32_t)n2x);
        if (n == 0) n = 1;
        int32_t scale_q15 = (int32_t)(((int64_t)ONE_G_q15 << 15) / n);
        vx = (int32_t)(((int64_t)vx * scale_q15) >> 15);
        vy = (int32_t)(((int64_t)vy * scale_q15) >> 15);
        vz = (int32_t)(((int64_t)vz * scale_q15) >> 15);
    }

    acc_q15[0] = (int16_t)CLAMP_INT32_TO_INT16(vx);
    acc_q15[1] = (int16_t)CLAMP_INT32_TO_INT16(vy);
    acc_q15[2] = (int16_t)CLAMP_INT32_TO_INT16(vz);
}

//// ====== Kalman-Filter für Magnetometer (Q15), ohne Höhe, mit Norm-Gating ======
///**
// * @brief  Kleiner KF für Mag-Daten (Q15), robust gegen Störungen:
// *         - Update wird abhängig von der Normabweichung | ||m|| - 1 | gedämpft oder gesperrt.
// *         - Rein fixed-point (Q15/Q31), ohne float.
// * @param  mag_norm  in/out  int16_t[3], Q15, bereits hard/soft korrigiert & ~normiert
// */
//static void magKF_Q15(int16_t *mag_norm)
//{
//    // --- Tuning-Parameter ---
//    // Prozess- und Messrauschen (Q15-Skala, intern in Q31 weiterverarbeitet)
//    const int16_t Q_base_q15    = 20;    // kleiner -> mehr Glättung
//    const int16_t R0_q15        = 600;   // Basis-Messrauschen
//
//    // Soft-/Hard-Gating anhand der Normabweichung d = | ||m|| - 1 |
//    // Soft-Region: d in [0 .. TOL_SOFT] -> R wird linear bis +kR angehoben
//    // Hard-Region: d > TOL_HARD -> Update komplett sperren
//    const int16_t TOL_SOFT_q15 = 8192;   // ≈ ±25%
//    const int16_t TOL_HARD_q15 = 16384;   // ≈ ±50%
//    const int16_t kR_q15        = 600;   // Zusatz-R bei d >= TOL_SOFT (z.B. = R0 -> am Rand doppelt)
//
//    // --- Zustände (static) ---
//    static int32_t x_q15[3] = {0};     // Zustand (Q15) je Achse in int32
//    static int32_t P_q31[3] = {0};     // Kovarianz (Q31) diagonal
//    static uint8_t init = 0;
//
//    if (!init) {
//        // Initialisierung
//        x_q15[0] = mag_norm[0];
//        x_q15[1] = mag_norm[1];
//        x_q15[2] = mag_norm[2];
//        P_q31[0] = (int32_t)Q15 << 15; // ≈1.0 in Q31
//        P_q31[1] = (int32_t)Q15 << 15;
//        P_q31[2] = (int32_t)Q15 << 15;
//        init = 1;
//    } else {
//        // --- Norm & Abweichung d = | ||m|| - 1 |
//        int32_t mx = mag_norm[0], my = mag_norm[1], mz = mag_norm[2];
//        int64_t n2 = (int64_t)mx*mx + (int64_t)my*my + (int64_t)mz*mz; // Q30
//        uint32_t n = sqrt_fast_uint((uint32_t)(n2 > 0 ? n2 : 0));       // ~Q15
//        int32_t d = (n > Q15) ? (int32_t)(n - Q15) : (int32_t)(Q15 - n); // |n - 1| in Q15
//
//        // --- Hard-Gating: große Abweichung -> Update sperren
//        bool accept_update = (d <= TOL_HARD_q15);
//
//        // --- Adaptives R: für kleinere Abweichungen linear anheben
//        // g = clamp(d / TOL_SOFT, 0..1) in Q15
//        int32_t g_q15 = 0;
//        if (d > 0) {
//            int32_t num = (int32_t)d << 15;               // Q15 * Q15 / Q15 -> Q15
//            int32_t den = (int32_t)TOL_SOFT_q15;
//            if (den <= 0) den = 1;
//            g_q15 = num / den;                            // Q15
//            if (g_q15 > Q15) g_q15 = Q15;
//        }
//        // R(hier: nach Normabweichung) = R0 + kR * g
//        int32_t R_q15_eff = R0_q15 + ( ( (int32_t)kR_q15 * g_q15 ) >> 15 );
//        if (R_q15_eff < 1) R_q15_eff = 1;
//
//        // --- KF pro Achse (diagonal) ---
//        for (uint8_t i = 0; i < 3; i++) {
//            // Prediction (Random Walk)
//            int32_t x_pred = x_q15[i];
//            int32_t P_pred = P_q31[i] + ((int32_t)Q_base_q15 << 15); // Q->Q31
//
//            int32_t x_upd = x_pred;
//            int32_t P_upd = P_pred;
//
//            if (accept_update) {
//                // K = P_pred / (P_pred + R_eff)
//                int32_t denom = P_pred + ((int32_t)R_q15_eff << 15); // Q31
//                if (denom <= 0) denom = 1;
//                int32_t K_q15 = (int32_t)(((int64_t)P_pred << 15) / denom); // Q15
//                if (K_q15 < 0) K_q15 = 0; if (K_q15 > Q15) K_q15 = Q15;
//
//                int32_t y_q15  = mag_norm[i];        // Messung (Q15)
//                int32_t innov  = y_q15 - x_pred;     // Q15
//                x_upd = x_pred + (int32_t)(((int64_t)K_q15 * innov) >> 15);
//                // P = (1-K) * P_pred
//                P_upd = (int32_t)(((int64_t)(Q15 - K_q15) * P_pred) >> 15);
//            }
//            x_q15[i] = x_upd;   // im Q15-Bereich halten -> Normierung folgt
//            P_q31[i] = P_upd;
//        }
//    }
//
//    // --- Normierung auf Einheitsvektor (Q15), rein integer ---
//    int32_t vx = (int16_t)x_q15[0];
//    int32_t vy = (int16_t)x_q15[1];
//    int32_t vz = (int16_t)x_q15[2];
//
//    int64_t n2x = (int64_t)vx*vx + (int64_t)vy*vy + (int64_t)vz*vz; // Q30
//    if (n2x > 0) {
//        uint32_t n = sqrt_fast_uint((uint32_t)n2x); // ~Q15
//        if (n == 0) n = 1;
//        // scale = Q15 / n (Q15), v_scaled = (v * scale)>>15
//        int32_t scale_q15 = (int32_t)(((int64_t)Q15 << 15) / n);
//        vx = (int32_t)(((int64_t)vx * scale_q15) >> 15);
//        vy = (int32_t)(((int64_t)vy * scale_q15) >> 15);
//        vz = (int32_t)(((int64_t)vz * scale_q15) >> 15);
//    }
//
//    mag_norm[0] = CLAMP_INT32_TO_INT16(vx);
//    mag_norm[1] = CLAMP_INT32_TO_INT16(vy);
//    mag_norm[2] = CLAMP_INT32_TO_INT16(vz);
//}




