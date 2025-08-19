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

#define ATTITUDE_FREQUENCY	500 // Hz

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

#define MAX_SPEED_MOTOR_RPM 8192 // its Q13
#define MAX_SPEED_MOTOR_RAD 1024 // its Q10 (its a little bit more than Q9)

#define DRONE_PARAM_K		0.000021f
#define DRONE_PARAM_L		0.16f
#define DRONE_PARAM_B		0.0000001898f



#define DRONE_PARAM_K_E6	21.0f
#define DRONE_PARAM_KL_E6	3.36f // = 0,00000336
#define DRONE_PARAM_B_E6	0.1898f
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
	int32_t x;
	int32_t y;
	int32_t z;
}xyz_32t;

typedef struct{
	int16_t w;
	int16_t x;
	int16_t y;
	int16_t z;
}wxyz_16t;



/**
 * @brief sensor fusion data
 * @note  is created only once and passed through
 * 			via the “sensor_fusion* get_data_ptr(void)”
 * 			function
 * @see   get_data_ptr();
 */
typedef struct{

	xyz_16t acc_t; 	/**< acc_xyz in Q15 representation */
	xyz_16t mag_t;	/**< mag_xyz in Q15 representation */
	xyz_16t gyro_t;	/**< gyro_xyz in Q15 representation */

	int16_t pitch;	/**< pitch in Q15 representation (for better visualisaton/debug) */
	int16_t roll;	/**< pitch in Q15 representation (for better visualisaton/debug) */
	int16_t yaw;	/**< pitch in Q15 representation (for better visualisaton/debug) */

	wxyz_16t quaternion;	/** Sensor fusion output q */
	xyz_16t gyro_t_rad; /** raw gyro data in rad with Q15 = 34.9066 rad/s */

}sensor_fusion;

typedef struct{
	bool hard_fault;
	bool MemManage;
	bool BusFault;
	bool DivtoZero;
	bool NMI;
	bool SVC;
}error_flag;

typedef struct{
	 int32_t Jxx;
	 int32_t Jyy;
	 int32_t Jzz;
}system_parameter;

/**
 * @brief StructDescription
 * @note  Optionaler Hinweis zur Verwendung
 * @see   ReferenzOderModulname
 */
typedef struct
{
	int16_t Iq; /**< FieldDesc1 */
	int16_t speed_rpm; /**< FieldDesc2 */
	uint16_t battery_voltage; /**< FieldDesc3 */
	uint8_t system_state; /**< FieldDesc3 */
	uint8_t service;
} recive_motor;

/**
 * @brief Struct for diffrent signals of the 4 motors
 * @note  Optionaler Hinweis zur Verwendung
 * @see   ReferenzOderModulname
 */
typedef struct
{
	int16_t m1; /**< Motor 1 (RPM, Iq,...) */
	int16_t m2; /**< Motor 2 (RPM, Iq,...)  */
	int16_t m3; /**< Motor 3 (RPM, Iq,...)  */
	int16_t m4; /**< Motor 4 (RPM, Iq,...)  */
} motor_signals_16t;



#endif /* INC_PARAMETER_H_ */
