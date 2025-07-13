/*
 * parameter.h
 *
 *  Created on: Oct 27, 2024
 *      Author: Gerry Geyer
 */

#ifndef INC_PARAMETER_H_
#define INC_PARAMETER_H_

#include <stdint.h>
#include <stdbool.h>

#define OK			0
#define NOT_OK		1

#define ON			1
#define OFF			0

#define HIGH		1
#define LOW			0

#define LQR 		0x01
#define PID			0x02

#define SET_IMU_OFFSET		OK
#define SYSTEM_FREQUENCY	100 // Hz
#define SYSTEM_TS			(float)(1.0f/(float)SYSTEM_FREQUENCY)

#define MAX_SPEED_MOTOR_RPM 7000

#define DRONE_PARAM_K		0.0000077f
#define DRONE_PARAM_L		0.16f
#define DRONE_PARAM_B		0.000000144f


#define DRONE_PARAM_K_E6	7.7f
#define DRONE_PARAM_KL_E6	1.232f
#define DRONE_PARAM_B_E6	0.144f
#define E6					1000000


#define MOTOR_STOPP 0x00
#define MOTOR_START 0xAA
#define MOTOR_INIT	0xDD

// LQR Control
#define W_TRIM_RPM	4000

// typedef structs
typedef struct{
	float pitch;
	float roll;
	float yaw;
}at_angl_f;

typedef struct{
	int16_t pitch;
	int16_t roll;
	int16_t yaw;
}at_angl_t;

typedef struct{

	uint8_t state_m_1;
	uint8_t state_m_2;
	uint8_t state_m_3;
	uint8_t state_m_4;

	int16_t m_1;
	int16_t m_2;
	int16_t m_3;
	int16_t m_4;
}motor_t;

typedef struct{
	float T;
	float pitch;
	float roll;
	float yaw;
}at_control_f;

typedef struct{
	float u1;
	float u2;
	float u3;
	float u4;
}control_output_f;


typedef struct{
	float error;
	float kp;
	float ki;
	float kd;
	float I_mem;
	float D_last;
	float control_frequency;
	float anti_windup_mem;
}PID_f;

typedef struct{
	float pitch;
	float roll;
	float yaw;

	float p;
	float q;
	float r;

	float int_e_pitch;
	float int_e_roll;
	float int_e_yaw;

}LQR_f;


typedef struct{
	float W;
	float X;
	float Y;
	float Z;
}Orientation;

typedef struct{
	int16_t x;
	int16_t y;
	int16_t z;
}xyz_16t;

typedef struct{
	int16_t w;
	int16_t x;
	int16_t y;
	int16_t z;
}wxyz_16t;


typedef struct{

	xyz_16t acc_t;
	xyz_16t mag_t;
	xyz_16t gyro_t;

	int16_t pitch;
	int16_t roll;
	int16_t yaw;

}sensor_fusion;

typedef struct{
	bool hard_fault;
	bool MemManage;
	bool BusFault;
	bool DivtoZero;
	bool NMI;
	bool SVC;
}error_flag;

#endif /* INC_PARAMETER_H_ */
