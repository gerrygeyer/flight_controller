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
#include <math.h>

#define IMU_FREQUENCY		1000 // Hz
#define ATTITUDE_FREQUENCY	500 // Hz
#define POSITION_FREQUENCY	100 // Hz
#define OPTICAL_FLOW_FREQ	50 // Hz


#define POS_FREQ_DIV		(5-1) // 500 / 100
// #### POSITION CONTROL SETTINGS ####
#define POS_ACC_LP_FC			20.0f
#define POS_SPEED_HIGHT_LP_FC	5.0f
#define POS_SPEED_XY_LP_FC		2.0f



#define OK			0
#define NOT_OK		1

#define ON			1
#define OFF			0

#define HIGH		1
#define LOW			0

#define LQR 		0x01
#define PID			0x02

#define DRONE		0
#define IMU			1

#define STEP_FUNCTION		0
#define RAMP_FUNCTION		1
#define UP_DOWN_FUNCTION	2

#define ATT_LQR_CONTROL		1
#define ATT_P2_CONTROL		2

#define SF_MADGWICK			0
#define SF_COMPLEMENTARY	1
#define SF_EKF				2
#define SF_AXIS_ALIGN		3
#define LOG_DATA_ONLY		100

#define ACC_ON				1
#define ACC_OFF				0
#define MAG_ON				1
#define MAG_OFF				0

#define LOG_DATA			1
#define SOLVE_COST_FCT		2


#define SET_IMU_OFFSET		OK
#define SYSTEM_FREQUENCY	100 // Hz
#define SYSTEM_TS			(float)(1.0f/(float)SYSTEM_FREQUENCY)

#define MAX_SPEED_MOTOR_RPM 8192 // its Q13
#define MAX_SPEED_MOTOR_RAD 1024 // its Q10 (its a little bit more than Q9)

#define MAX_HIGHT			200	// cm

#define DRONE_PARAM_K		0.000021f
#define DRONE_PARAM_L		0.16f
#define DRONE_PARAM_B		0.0000001898f



#define DRONE_PARAM_K_E6	21.0f
#define DRONE_PARAM_KL_E6	3.36f // = 0,00000336
#define DRONE_PARAM_B_E6	0.1898f
#define E6					1000000

#define DRONE_WEIGHT_KG		1.282f //

#define COST_FCT_ITERATIONS	20000
#define COST_FCT_TOLEARANCE	1e-18f

// determined experimentally
#define IMU_GYRO_SCAL_X		1.1961f
#define IMU_GYRO_SCAL_Y		1.1777f
#define IMU_GYRO_SCAL_Z		1.2169f

// Optical flow settings

#define OPT_FLOW_SENSOR_DIST	50 // distance from sensor to rotation center
#define OPT_FLOW_CORR_FACTOR	2.094f

// #### WAIT CONSTANTS ###

#define YAW_OFFSET_TIME		ATTITUDE_FREQUENCY // -> 1 sec



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

	bool mag_updated;

	xyz_16t gyro_drift_est;
	xyz_16t acc_drift_est;

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


/**
 * @brief Nonlinear attitude control P^2
 * @note  Optionaler Hinweis zur Verwendung
 * @see   ReferenzOderModulname
 */
typedef struct
{
	at_angl_f P1; /**< FieldDesc1 */
	at_angl_f P2; /**< FieldDesc2 */

} P2_attitude_control;

/**
 * @brief Optical Flow Sensor
 * @note  Included the important data
 * @see   ReferenzOderModulname
 */
typedef struct
{
	uint16_t deltaT; /**< Time between last measurement */
	uint32_t distance; /**< FieldDesc2 */
	uint8_t dist_strenght; /**< FieldDesc2 */
	uint8_t dist_pres; /**< FieldDesc2 */
	int32_t flow_vel_x; /**< velocity in x direction in m/s */
	int32_t flow_vel_y; /**< velocity in y direction in m/s */
	uint8_t flow_qual; /**< FieldDesc3 */
	uint8_t flow_stat; /**< FieldDesc3 */
} optical_flow_data;


typedef struct __attribute__((packed)) {
    uint32_t system_time_ms;
    uint32_t distance_mm;
    uint8_t  distance_strength;
    uint8_t  distance_precision;
    uint8_t  distance_status;
    uint8_t  reserved1;
    int16_t  flow_vel_x;
    int16_t  flow_vel_y;
    uint8_t  flow_quality;
    uint8_t  flow_status;
    uint16_t reserved2;
} mtf01_payload_t;

typedef struct {
	int16_t x1;
	int16_t x2;
	int16_t x3;
	int16_t x4;
	int16_t x5;
	int16_t x6;
}lqr_state;



#endif /* INC_PARAMETER_H_ */
