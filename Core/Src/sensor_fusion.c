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
// ##### SETINGS #####
float beta_f;



int16_t ax, ay, az, gx, gy, gz, mx, my, mz, dt_q15, beta_t;
int16_t q_madgwick_out[4];

// Flags
volatile bool mag_ready_flag;
// Output (debug)
int16_t euler_debug[3];
float euler_debug_pitch,euler_debug_roll,euler_debug_yaw;
xyz_16t debug_gyro, debug_omega;
wxyz_16t debug_qDot, debug_q_out, debug_q_out_norm, debug_h;


uint8_t sensor_fusion_takeout;

//#define CLAMP_INT32_TO_INT16(x) ((x) > INT16_MAX ? INT16_MAX : ((x) < INT16_MIN ? INT16_MIN : (int16_t)(x)))
static inline int16_t q15_mul(int16_t a, int16_t b);
static inline void divideQuaternionBy2(int16_t *q);
static inline void multQuatwithConstQ15(int16_t* q, const int16_t x);
static void error_function_small(int16_t *accel, int16_t *q, int16_t *f_error);
static void error_function_bigQ15(int16_t *accel, int16_t *mag,int16_t *q, int16_t *f_error, int16_t *bxz);
static void get_jacobi_small(int16_t J[3][4], int16_t *q);
static void get_jacobi_bigQ15(int16_t J[6][4], int16_t *q, int16_t *bxz);
static void compute_gradient(int16_t grad[4], int16_t J[3][4], int16_t f_error[3]);
static void compute_gradient_bigQ15(int16_t *grad, int16_t J[6][4], int16_t *f_error);
static void get_bx_bz_q15(const int16_t *h, int16_t *bxz);
static void Madgwick_filter_acc_gyro_mag(int16_t beta, sensor_fusion *pHandle_sf);
static void Madgwick_filter_acc_gyro(int16_t beta, sensor_fusion *pHandle_sf);

sensor_fusion* get_data_ptr(void) {
    return &sf_values;
}

void init_sensors(void)
{
	sensor_fusion_takeout = 0;
	mag_ready_flag = false;

//	dt_q15 = (1UL <<15)/SENSOR_FUSION_FREQUENCY_IMU;
	dt_q15 = (1UL <<15)/80;
	beta_f = 0.05;


	q_madgwick_out[0] = Q15_ONE;
	q_madgwick_out[1] = 0;
	q_madgwick_out[2] = 0;
	q_madgwick_out[3] = 0;

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
			beta_t = (int16_t)((float)INT16_MAX * beta_f);


			switch(mag_ready_flag){
			case(true):



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


	int16_t accel[3], accel_norm[3], gyro[3],mag[3], mag_norm[3],
			f_error[6], grad[4], grad_norm[4], omega[4], qDot[4], h[4], q_con[4],
			bxz[2];
	int16_t J[6][4];

	//get values

	accel[0] = pHandle_sf->acc_t.x;
	accel[1] = pHandle_sf->acc_t.y;
	accel[2] = pHandle_sf->acc_t.z;
	gyro[0] = pHandle_sf->gyro_t.x;
	gyro[1] = pHandle_sf->gyro_t.y;
	gyro[2] = pHandle_sf->gyro_t.z;

	mag[0] = pHandle_sf->mag_t.x;
	mag[1] = pHandle_sf->mag_t.y;
	mag[2] = pHandle_sf->mag_t.z;

	omega[0] = 0;
	omega[1] = debug_omega.x = q15_mul(gyro[0],GYRO_GRAD_TO_RAD_Q15);
	omega[2] = debug_omega.y = q15_mul(gyro[1],GYRO_GRAD_TO_RAD_Q15);
	omega[3] = debug_omega.z = q15_mul(gyro[2],GYRO_GRAD_TO_RAD_Q15);

	norm_3d_vector(accel, accel_norm);
	norm_3d_vector(mag, mag_norm);

	// h = q ⊗ [0; mag] ⊗ q*
	q_t_conj_function_in_out_q15(q_madgwick_out, q_con);
	multiplicateQuaternionQ15(q_madgwick_out,omega,h);

	debug_q_out.w = q_madgwick_out[0];
	debug_q_out.x = q_madgwick_out[1];
	debug_q_out.y = q_madgwick_out[2];
	debug_q_out.z = q_madgwick_out[3];

	debug_h.w = h[0];
	debug_h.x = h[1];
	debug_h.y = h[2];
	debug_h.z = h[3];

	multiplicateQuaternionQ15(h,q_con,h);

	// caclfulate bx and bz
	get_bx_bz_q15(h, bxz);

	error_function_bigQ15(accel, mag_norm, q_madgwick_out, f_error, bxz);
	get_jacobi_bigQ15(J,q_madgwick_out, bxz);
	compute_gradient_bigQ15(grad, J, f_error);
	Normalize4DvectorQ15(grad, grad_norm);
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

	q_madgwick_out[0] += q15_mul(qDot[0],dt_q15);
	q_madgwick_out[1] += q15_mul(qDot[1],dt_q15);
	q_madgwick_out[2] += q15_mul(qDot[2],dt_q15);
	q_madgwick_out[3] += q15_mul(qDot[3],dt_q15);


	NormalizeQuaternionQ15(q_madgwick_out, q_madgwick_out);
	debug_q_out_norm.w = q_madgwick_out[0];
	debug_q_out_norm.x = q_madgwick_out[1];
	debug_q_out_norm.y = q_madgwick_out[2];
	debug_q_out_norm.z = q_madgwick_out[3];


	quat_to_euler_q15(q_madgwick_out, euler_debug);
	euler_debug_roll = (float)euler_debug[0];// * 360.0f / (float)INT16_MAX;
	euler_debug_pitch = (float)euler_debug[1];// * 360.0f / (float)INT16_MAX;
	euler_debug_yaw = (float)euler_debug[2];// * 360.0f / (float)INT16_MAX;





}

static void Madgwick_filter_acc_gyro(int16_t beta, sensor_fusion *pHandle_sf){


	int16_t accel[3], accel_norm[3], gyro[3], f_error[3], grad[4], grad_norm[4],omega[4], qDot[4];
	int16_t J[3][4];

	accel[0] = pHandle_sf->acc_t.x;
	accel[1] = pHandle_sf->acc_t.y;
	accel[2] = pHandle_sf->acc_t.z;
	gyro[0] = pHandle_sf->gyro_t.x;
	gyro[1] = pHandle_sf->gyro_t.y;
	gyro[2] = pHandle_sf->gyro_t.z;


	omega[0] = 0;
	omega[1] = debug_gyro.x = q15_mul(gyro[0],GYRO_GRAD_TO_RAD_Q15);
	omega[2] = debug_gyro.y = q15_mul(gyro[1],GYRO_GRAD_TO_RAD_Q15);
	omega[3] = debug_gyro.z = q15_mul(gyro[2],GYRO_GRAD_TO_RAD_Q15);


	norm_3d_vector(accel, accel_norm); // getestet und funkitoniert ✅
	error_function_small(accel_norm, q_madgwick_out, f_error); // getestet und funkitoniert ✅
	get_jacobi_small(J, q_madgwick_out);// getestet und funkitoniert ✅
	compute_gradient(grad, J, f_error);// getestet und funkitoniert ✅
	Normalize4DvectorQ15(grad, grad_norm);
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

	q_madgwick_out[0] += q15_mul(qDot[0],dt_q15);
	q_madgwick_out[1] += q15_mul(qDot[1],dt_q15);
	q_madgwick_out[2] += q15_mul(qDot[2],dt_q15);
	q_madgwick_out[3] += q15_mul(qDot[3],dt_q15);

	debug_q_out.w = q_madgwick_out[0];
	debug_q_out.x = q_madgwick_out[1];
	debug_q_out.y = q_madgwick_out[2];
	debug_q_out.z = q_madgwick_out[3];

	NormalizeQuaternionQ15(q_madgwick_out, q_madgwick_out);
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
static inline void divideQuaternionBy2(int16_t *q) {
    q[0] >>= 1;
    q[1] >>= 1;
    q[2] >>= 1;
    q[3] >>= 1;
}
static inline void multQuatwithConstQ15(int16_t* q, const int16_t x){
	q[0] = q15_mul(q[0], x);
	q[1] = q15_mul(q[1], x);
	q[2] = q15_mul(q[2], x);
	q[3] = q15_mul(q[3], x);
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

static void error_function_bigQ15(int16_t *accel, int16_t *mag,int16_t *q, int16_t *f_error, int16_t *bxz){
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

	x_ut 		= ((0x7FFF << 15) - ((uint32_t)(q[2] * q[2]) << 1 ) - ((uint32_t)(q[3] * q[3]) << 1)); // 2*(0.5 - q3^2 - q4^2) all in q30
	x_t 		= CLAMP_INT32_TO_INT16(x_ut >> 15);
	x_t			= q15_mul(x_t, bxz[0]);
	x_t			+=(q15_mul((q_24 - q_13), bxz[1]) << 1);
	x_t			-= mag[0];
	f_error[3]	= CLAMP_INT32_TO_INT16(x_t);

	x_t 		= (q15_mul((q_23 - q_14), bxz[0]) << 1);
	x_t			+= (q15_mul((q_12 + q_34), bxz[1]) << 1);
	x_t			-= mag[1];
	f_error[4]	= CLAMP_INT32_TO_INT16(x_t);


	x_ut		= ((0x7FFF << 15) - ((uint32_t)(q[1] * q[1]) << 1 ) - ((uint32_t)(q[2] * q[2]) << 1));
	x_t			= CLAMP_INT32_TO_INT16(x_ut >> 15);
	x_t			= q15_mul(x_t, bxz[0]);
	x_t 		+= (q15_mul((q_13 + q_24), bxz[1]) << 1);
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
    J[3][0] = CLAMP_INT32_TO_INT16(-q15_mul(bz, q3) << 1);                                    // -2*bz*q3
    J[3][1] = CLAMP_INT32_TO_INT16( q15_mul(bz, q2) << 1);                                    //  2*bz*q2
    J[3][2] = CLAMP_INT32_TO_INT16((-((q15_mul(bx, q3) << 2)) - (q15_mul(bz, q0) << 1)));     // -4*bx*q3 - 2*bz*q0
    J[3][3] = CLAMP_INT32_TO_INT16((-((q15_mul(bx, q2) << 2)) + (q15_mul(bz, q1) << 1)));     // -4*bx*q2 + 2*bz*q1

    // Zeile 4
    J[4][0] = CLAMP_INT32_TO_INT16((-q15_mul(bx, q3) << 1) + (q15_mul(bz, q1) << 1));         // -2*bx*q3 + 2*bz*q1
    J[4][1] = CLAMP_INT32_TO_INT16(( q15_mul(bx, q2) << 1) + (q15_mul(bz, q0) << 1));         //  2*bx*q2 + 2*bz*q0
    J[4][2] = CLAMP_INT32_TO_INT16(( q15_mul(bx, q1) << 1) + (q15_mul(bz, q3) << 1));         //  2*bx*q1 + 2*bz*q3
    J[4][3] = CLAMP_INT32_TO_INT16((-q15_mul(bx, q0) << 1) + (q15_mul(bz, q2) << 1));         // -2*bx*q0 + 2*bz*q2

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
	uint32_t x = (uint32_t)(h[1] * h[1]) + (uint32_t)(h[2] * h[2]);
	bxz[0] = sqrt_fast_uint(x);
	bxz[1] = h[3];


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
	int16_t qDot_test[4];
    int16_t q_madgwick_out1[4] = { 1835, -16406, -26937, 8696 };
    int16_t omega_test[4] = {0, 860, -13831, 5076};
    multiplicateQuaternionQ15(q_madgwick_out1,omega_test,qDot_test);
    divideQuaternionBy2(qDot_test);

    // Beispielhafte Testdaten – 5 zufällige Quaternionen & Beschleunigungen (in Q15)
//    const int16_t test_quats[NUM_TESTS][4] = {
//        {32767, 0, 0, 0},
//        {23170, 23170, 0, 0},
//        {23170, 0, 23170, 0},
//        {16384, 16384, 16384, 16384},
//        {28377, 16384, 0, 16384}
//    };
//
//    const int16_t test_accel[NUM_TESTS][3] = {
//        {0, 0, 32767},
//        {5000, 2000, 32000},
//        {10000, 0, 30000},
//        {-3000, 6000, 32000},
//        {0, -10000, 28000}
//    };
//
//    for (int i = 0; i < NUM_TESTS; i++) {
//        // Kopiere Testdaten
//        for (int j = 0; j < 4; j++) test_results[i].q[j] = test_quats[i][j];
//        for (int j = 0; j < 3; j++) test_results[i].accel[j] = test_accel[i][j];
//
//        // Schrittweise Berechnung
//        norm_3d_vector(test_results[i].accel, test_results[i].accel_norm);
//        error_function_small(test_results[i].accel_norm, test_results[i].q, test_results[i].f_error);
//        get_jacobi_small(test_results[i].J, test_results[i].q);
//        compute_gradient(test_results[i].grad, test_results[i].J, test_results[i].f_error);
//    }

}





