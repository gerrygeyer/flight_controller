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

#define SENSOR_FUSION_FREQUENCY_IMU 1000 // Hz
#define GYRO_GRAD_TO_RAD_Q15		9370 // 2^15/2000 * pi/180 * 2^15
#define GYRO_GRAD_TO_RAD_DELTA_T_Q15	(DEGREE_TO_RAD * 2000.0f / (float)SENSOR_FUSION_FREQUENCY_IMU) * (float)Q15

#define GRAD2RAD_GYRO 	34.906585f

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

sensor_fusion* get_data_ptr(void);



// ######## TEST ##########

void run_test_vectors(void);


#endif /* INC_SENSOR_FUSION_H_ */
