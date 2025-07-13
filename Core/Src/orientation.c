/*
 * orientation.c
 *
 *  Created on: Oct 25, 2024
 *      Author: Gerry Geyer
 */

#include <orientation.h>
#include <main.h>
#include <stdio.h>
#include <parameter.h>
#include <sys_math.h>
//#include <imu.h>
//#include "mpu6050.h"


extern I2C_HandleTypeDef hi2c1;

Orientation quanternion;

at_angl_f euler_f, euler_degree_f;
at_angl_f euler_acceleration;
at_angl_t euler_t;
at_angl_f euler_f_old, euler_f_d;


//MPU6050_Data sensorData;

//uint32_t time;

float debug_yaw;

void init_orientation(void){

//	BNO055_Init_WithInterrupt();
//	MPU6050_Init();

	euler_f.pitch = 0.0f;
	euler_f.roll = 0.0f;
	euler_f.yaw = 0.0f;

	euler_acceleration.pitch = 0.0f;
	euler_acceleration.roll = 0.0f;
	euler_acceleration.yaw = 0.0f;
}


void read_orientation(void){



	float qw, qx, qy, qz;
//	BNO055_ReadQuaternion(&qw, &qx, &qy, &qz);
//	HAL_Delay(100);
//	BNO055_InterruptPending();
//	uint8_t sys, mag,gyro,acc;
//	BNO055_CheckCalibration(&sys, &gyro, &acc, &mag);


//	uint8_t x = (uint8_t)BNO055_InterruptPending();
//
////	float qw, qx, qy, qz;
//	if (BNO055_ReadQuaternion(&qw, &qx, &qy, &qz) == HAL_OK) {
//	    // Toggle Test-LED oder Breakpoint setzen
//	}




}

void read_imu_values(void){

//	time = stopp_time_measurement();
//	start_time_measurement();
//	MPU6050_ReadData(&sensorData);


}

void get_orientation(at_angl_f *pHandle){
	pHandle->pitch 	= euler_f.pitch;
	pHandle->yaw 	= euler_f.yaw;
	pHandle->roll 	= euler_f.roll;
}

void get_angular_rate(at_angl_f *pHandle){
	pHandle->pitch		= euler_acceleration.pitch;
	pHandle->roll		= euler_acceleration.roll;
	pHandle->yaw		= euler_acceleration.yaw;
}





