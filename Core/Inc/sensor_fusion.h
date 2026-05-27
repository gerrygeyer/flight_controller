/*
 * sensor_fusion.h
 *
 *  Created on: May 16, 2025
 *      Author: gerrygeyer
 */

#ifndef INC_SENSOR_FUSION_H_
#define INC_SENSOR_FUSION_H_

#include <parameter.h>
#include <stdio.h>
#include <stdbool.h>
#include <lis3mdl.h>
#include <mpu600.h>
#include <sys_math.h>
//#include <tof400c_vl53l1x.h>
//#include "../../sensors/imu/mpu6050.h"

#define CF_MAG_BETA					0

#define SENSOR_FUSION_FREQUENCY_IMU 999 	// Hz
#define SENSOR_FUSION_FREQUENCY_MAG	80		// Hz
#define GYRO_GRAD_TO_RAD_Q15		9370 // 2^15/2000 * pi/180 * 2^15
#define GYRO_GRAD_TO_RAD_DELTA_T_Q15	(DEGREE_TO_RAD * 2000.0f / (float)SENSOR_FUSION_FREQUENCY_IMU) * (float)Q15

#define GRAD2RAD_GYRO 	34.906585f
//#define GRAD2RAD_GYRO_MAX_Q15 	939 // (1/2000'°/s')* (2*pi/360°) -> Gyro * 938.7341; max output = 34,9 rad/s
#define GRAD2RAD_GYRO_MAX_Q15 	30039 // Q5 / (1/2000'°/s')* (2*pi/360°) -> Gyro * 938.7341; max output = 34,9 rad/s
#define GRAD2RAD_GYR_MAX_SC_Q15	7510 //GRAD2RAD_GYRO_MAX_Q15 << 3
/*
 * normierter_wert = deg_per_sec / 2000
→ also: (rad_per_sec * 180 / π) / 2000
→ entspricht: rad_per_sec * (180 / (π * 2000)) ≈ 0.02865
0.02865 * 32768 ≈ 939
*/
//#define RAD_TO_GYRO_GRAD			939

void init_sensors(void);
void read_distance_sensor(void);
void read_bno085_values(void);
bool BNO085_TestConnection(uint8_t *rx_buf, uint16_t buf_len);
//void Madgwick_filter_acc_gyro(int16_t beta, sensor_fusion *pHandle_sf);

void task_imu_sensor_fusion(void);
void mag_ready(void);


void get_quaternion_Q15(int16_t *q, int16_t *w, int16_t *a);

/**
 * @brief Returns a direct pointer to the live sensor fusion data.
 * @warning This pointer may point to data currently being written!
 *          Use only if you can guarantee no concurrent access.
 * @return Pointer to the active sensor_fusion struct.
 */
sensor_fusion* read_sensorfusion_data(void);

/**
 * @brief   Computes an initial yaw offset quaternion from accelerometer and magnetometer data.
 *
 * @details This function estimates the initial yaw angle using tilt-compensated magnetometer
 *          measurements together with accelerometer data. The resulting quaternion represents
 *          a pure rotation around the world Z-axis that cancels the initial yaw offset.
 *          It can be applied to the attitude estimator so that the yaw angle is set to zero at startup.
 *
 * @param[out]  q_out   Pointer to a 4-element array (Q15 format) that will store the quaternion
 *                      [w, x, y, z] representing the yaw offset.
 *
 * @note    The input data is taken from global sensor structures (accelerometer and magnetometer)
 *          and must be calibrated (hard-iron and soft-iron compensation applied) before use.
 *
 * @warning This function should only be called once during initialization, when the system is stationary
 *          and valid accelerometer/magnetometer readings are available.
 *
 * @see     hardiron_apply_q15(), softiron_apply_q15(), norm_3d_vector()
 */
void create_init_yaw_quaternion(int16_t *q_out);
// ######## TEST ##########

void run_test_vectors(void);


#endif /* INC_SENSOR_FUSION_H_ */
