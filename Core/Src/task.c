/*
 * task.c
 *
 *  Created on: Oct 25, 2024
 *      Author: Gerry Geyer
 */


#include <task.h>
#include <parameter.h>
#include <orientation.h>
#include <communication.h>
#include <attitude_control.h>
#include <control_functions.h>
#include <main.h>
#include <settings.h>
#include <sensor_fusion.h>
#include <Distance/distance_sensor.h>
#include <log_data.h>
#include <position_control.h>
#include <Optical_flow/uart_ring.h>
//#include <Optical_flow/mtf02.h>
#include "Optical_flow/mtf02_ring_buffer.h"
#include <stdbool.h>
#include <encoder.h>

uint8_t system_state;
motor_t motor;
at_control_f attitude_control_signals;
mtf01_payload_t optical_flow;
bool init_tasks = false;

int16_t tauQ10;
int16_t hight_mm_corrected;
int16_t hight_zero_position;

// used for Test function
uint32_t control_signals_counter;
uint8_t control_flag, stop_flag;



wxyz_16t system_q, system_q_ref;
int16_t q_yaw_correction[4], q_axis_correction[4], system_r33;
int16_t a_imu[3], w_imu[3];
at_angl_t system_euler_angle;
motor_signals_16t motor_seed_esc;

uint8_t reset_position_control =1;

// debug
int16_t debug_tau_1, debug_tau_2, debug_tau_3;
int16_t debug_speed_counter = 0;
xyz_16t debug_w;
int32_t debug_motor_speed_m1, debug_motor_speed_m2, debug_motor_speed_m3, debug_motor_speed_m4;
float euler_debug_ax_roll, euler_debug_ax_pitch,euler_debug_ax_yaw;

at_angl_f debug_ref_euler, debug_sys_euler;

static void highspeed_task(void);
static void middle_speed_task(void);
static void LED_heartbeat(bool flag);
static void sysetem_ready_check(bool *flag, const int16_t *q);
static void get_control_quaternion_from_euler_command(at_control_f euler, int16_t *q_ref);
static void read_DMA_buffer(void);
static int16_t get_r33_from_quaternion(const int16_t *q);
static void init_SAFETY_FUNCTION(void);
static void SAFETY_function_tasks(void);

void task_init(void){

	tauQ10 = 0;
	hight_zero_position = 0;
	reset_position_control = 1;

	// ONLY 'SYSTEM_STOP' ARE ALLOWED IF BATTERY ARE CONNECTED

//	system_state = SYSTEM_INIT;
//	if(DEBUG_MODE == ON){
//		system_state = SYSTEM_START;
//	}else{
		system_state = SYSTEM_INIT;//SYSTEM_STOP;
//	}

	q_yaw_correction[0] = Q15;
	q_yaw_correction[1] = 0;
	q_yaw_correction[2] = 0;
	q_yaw_correction[3] = 0;

	q_axis_correction[0] = Q15;
	q_axis_correction[1] = 0;
	q_axis_correction[2] = 0;
	q_axis_correction[3] = 0;




	run_test_vectors();

//	init_orientation();
	init_attitude_control();
	init_control_functions();

	init_motor();
	init_position_control();
	init_SAFETY_FUNCTION();
	attitude_control_signals.T = 0;
	attitude_control_signals.pitch = 0;
	attitude_control_signals.roll = 0;
	attitude_control_signals.yaw = 0;

	control_signals_counter = 0;
	control_flag = OFF;
	stop_flag = OFF;

	HAL_Delay(80);
	init_sensors();
	init_tasks = true;

//
//	int16_t test =  32767;
//	int16_t angle_test = q15_acos(test);
//
//	int x = 0;



//	calculate_offset_values_imu();

}
/**
 * @brief		Service functions
 *
 * @details 	Triggers control functions. Protects the system: in the event of a brief
 * 				malfunction or if an interrupt is not triggered. After a predefined time
 * 				without a reaction, periferie is retriggered,
 */
void service_functions(void){
	MPU6000_Service();
	LIS3MDL_Service();
	service_recive_motor_information();
}

void time_management(void){

	start_time_measurement();
	read_DMA_buffer();		//
	if(init_tasks){

	service_functions();	// Service functions
	middle_speed_task();	// position control
	highspeed_task();		// attitude control
	SAFETY_function_tasks();// safty functions
	stopp_time_measurement();
	}
//	Tick_500Hz_Handler(&optical_flow);

}
wxyz_16t q_filter,q_debug_yaw;
int16_t angle_from_encoder_Q15;debug_yaw_estimation;
/**
 * @brief		Highspeed Task
 *
 * @details 	All high-frequency tasks are carried out here, such as attitude control
 * 				and communication with the motors.
 */
static void highspeed_task(void){
	static int16_t q_yaw[4] = {Q15, 0, 0, 0};
	static uint16_t init_yaw_counter = 0;
	static bool init_yaw_sucsess = 0;
	int16_t q[4], w[3],a[3],q_att_ref[4];
	int16_t tau[3], u[4];
	float u_f[4][1], w_f[4][1];
	static int32_t ramp_speed = 0;

	static bool system_ready = 0;
	// debug
	int16_t euler[3];


	get_quaternion_Q15(q, w,a);

	if(TEST_STATION_ENCODER == ON){
		angle_from_encoder_Q15 = (int16_t)((int16_t)read_encoder_value());

	}



	debug_w.x = w[0];
	debug_w.y = w[1];
	debug_w.z = w[2];

	a_imu[0] = a[0];	a_imu[1] = a[1];	a_imu[2] = a[2];
	w_imu[0] = w[0];	w_imu[1] = w[1];	w_imu[2] = w[2];

	if(SET_YAW_TO_ZERO == ON){
		if(!init_yaw_sucsess){
			if(init_yaw_counter++ > YAW_OFFSET_TIME){
				create_init_yaw_quaternion(q_yaw);
				init_yaw_sucsess = 1;
				q_debug_yaw.w = q_yaw[0];	q_debug_yaw.x = q_yaw[1]; q_debug_yaw.y = q_yaw[2]; q_debug_yaw.z = q_yaw[3];
			}
		}
		multiplicateQuaternionQ15(q_yaw, q, q);
//		multiplicateQuaternionQ15(q, q_yaw, q);
	}
	if(POSTFILTER_ATT){
		filter_SLERP_EMA_quaternion_Q15(q, q);
	}


	system_r33 = get_r33_from_quaternion(q);
//	if(q[0]<0){
//		q[0] = -q[0];
//		q[1] = -q[1];
//		q[2] = -q[2];
//		q[3] = -q[3];
//	}
	q_filter.w = q[0];
	q_filter.x = q[1];
	q_filter.y = q[2];
	q_filter.z = q[3];
//	correct_q_axis(q,w);
	int16_t euler_yaw[3];

	quat_to_euler_q15(q_yaw, euler_yaw);
	debug_yaw_estimation = euler_yaw[2];


	quat_to_euler_q15(q, euler);
	system_euler_angle.roll = euler[0];// * 180.0f / (float)Q15;
	system_euler_angle.pitch = euler[1];// * 180.0f / (float)Q15;
	system_euler_angle.yaw = euler[2];// * 180.0f / (float)Q15;
//	correct_q_axis

	motor_seed_esc = get_motorspeed_from_ESC();


	sysetem_ready_check(&system_ready, q);
	LED_heartbeat(system_ready);

	switch(system_state){

	/*
	 * SYSTEM_INIT:	Set the inital measurement of the optical flow as the offset
	 * 				for the drone. Max Time are 2 sec. after 2 Second the
	 * 				optical flow cant give a good measurement the init hight are 0;
	 */
	case SYSTEM_INIT:
		static uint16_t wait_for_optical_flow_counter = 0;
		reset_position_control = 1;
		if(COMMUNICATION_OPT_FLW == ON){
			if(optical_flow.distance_mm > 0){
				hight_zero_position = optical_flow.distance_mm;
				system_state = SYSTEM_STOP;
				wait_for_optical_flow_counter = 0;
			}else{
				if(wait_for_optical_flow_counter++ > (2*IMU_FREQUENCY)){ // wait 2 sec
					hight_zero_position = 0;
					system_state = SYSTEM_STOP;
					wait_for_optical_flow_counter = 0;
				}
			}

		}

		create_init_yaw_quaternion(q_yaw);


		stop_flag = OFF;
		break;
/*
 * SYSTEM_STOP:	Send the Stop command to Motor 1-4. For more safty we make it only once, than stop sending
 * 				commands, so the internally Safty switch of the motor are activate and we are shure that the
 * 				motor dont move anymore.
 */
	case SYSTEM_STOP:
		reset_position_control = 1;
		motor.m_1 = 0;
		motor.m_2 = 0;
		motor.m_3 = 0;
		motor.m_4 = 0;

		motor.state_m_1 = MOTOR_STOPP;
		motor.state_m_2 = MOTOR_STOPP;
		motor.state_m_3 = MOTOR_STOPP;
		motor.state_m_4 = MOTOR_STOPP;
		set_postion_control_stats_to_zero();
		if(stop_flag == OFF){

			stop_flag = ON;


			// send the speed command to M1 - M4
			if(run_motors(&motor,1) != OK){
				// error
			}
		}

		clear_control_functions();
		ramp_speed = 0;

	break;

	case TEST_MOTOR:

		motor.m_1 = 1000;
		motor.m_2 = 1000;
		motor.m_3 = 1000;
		motor.m_4 = 1000;

		motor.state_m_1 = MOTOR_START;
		motor.state_m_2 = MOTOR_START;
		motor.state_m_3 = MOTOR_START;
		motor.state_m_4 = MOTOR_START;

		if(run_motors(&motor,0) != OK){
			// error
		}

	break;
	/*
	 * SYSTEM_RAMP:	increase the motor Speed in one ramp function.
	 * 				The values 3000 and 8 proved to be good in tests.
	 */
	case SYSTEM_RAMP:
		stop_flag = OFF;
		reset_position_control = 0;

		motor.m_1 = ramp_speed;
		motor.m_2 = ramp_speed;
		motor.m_3 = ramp_speed;
		motor.m_4 = ramp_speed;

		motor.state_m_1 = MOTOR_START;
		motor.state_m_2 = MOTOR_START;
		motor.state_m_3 = MOTOR_START;
		motor.state_m_4 = MOTOR_START;

		// send the speed command to M1 - M4
		if(run_motors(&motor,0) != OK){
			// error
		}

		clear_control_functions();

		if(ramp_speed < 3000){
			ramp_speed += 8;
		}else{
			system_state = SYSTEM_START;
			if(SET_YAW_TO_ZERO == ON) twist_z_axis(q, q_yaw);
		}

//		create_init_yaw_quaternion(q_yaw);
		init_yaw_sucsess = 1;
		q_debug_yaw.w = q_yaw[0];	q_debug_yaw.x = q_yaw[1]; q_debug_yaw.y = q_yaw[2]; q_debug_yaw.z = q_yaw[3];

	break;
	case SYSTEM_START:
		stop_flag = OFF;
		reset_position_control = 0;


		switch (control_flag){
		case ON:
			attitude_control_signals = create_attitude_control_signals();
			break;
		default:
			control_signals_counter = 0;
			break;
		}
		control_signals_counter += 1;

		get_control_quaternion_from_euler_command(attitude_control_signals,q_att_ref);

		if(POSITION_CONTROL == OFF){
			system_q_ref.w = q_att_ref[0];	system_q_ref.x = q_att_ref[1];
			system_q_ref.y = q_att_ref[2];	system_q_ref.z = q_att_ref[3];
		}else{
			q_att_ref[0] = system_q_ref.w;	q_att_ref[1] = system_q_ref.x;
			q_att_ref[2] = system_q_ref.y;	q_att_ref[3] = system_q_ref.z;
		}


		// only debug/control:
		int16_t euler_ref_debug[3], euler_sys_debug[3];
		quat_to_euler_q15(q_att_ref,euler_ref_debug);
		debug_ref_euler.pitch = (float)euler_ref_debug[0]; //* 360.0f / (float)Q15;
		debug_ref_euler.roll = (float)euler_ref_debug[1]; //* 360.0f / (float)Q15;
		debug_ref_euler.yaw = (float)euler_ref_debug[2]; // * 360.0f / (float)Q15;

		quat_to_euler_q15(q,euler_sys_debug);
		debug_sys_euler.pitch = (float)euler_sys_debug[0]; // * 360.0f / (float)Q15;
		debug_sys_euler.roll = (float)euler_sys_debug[1]; // * 360.0f / (float)Q15;
		debug_sys_euler.yaw = (float)euler_sys_debug[2]; // * 360.0f / (float)Q15;


		switch (ATTITUDE_CONTROL){
		case ATT_LQR_CONTROL:
			attitude_control_quaternion_lqr_q15(q,q_att_ref,w,tau); // kontrolliert
			break;
		case ATT_P2_CONTROL:
			attitude_control_quaternion_nonlinear_q15(q,q_att_ref,w,tau);
			break;
		default:
			system_state = SYSTEM_STOP;
		}



		debug_tau_1 = tau[0]; // debug
		debug_tau_2 = tau[1];	// debug
		debug_tau_3 = tau[2]; // devug

		u[0] = tauQ10;
		u[1] = tau[0];
		u[2] = tau[1];
		u[3] = tau[2];

		transform_u2_motorSpeed(u,&motor);

		debug_motor_speed_m1 = motor.m_1;
		debug_motor_speed_m2 = motor.m_2;
		debug_motor_speed_m3 = motor.m_3;
		debug_motor_speed_m4 = motor.m_4;

		motor.state_m_1 = MOTOR_START;
		motor.state_m_2 = MOTOR_START;
		motor.state_m_3 = MOTOR_START;
		motor.state_m_4 = MOTOR_START;

		// send the speed command to M1 - M4
		if(run_motors(&motor,0) != OK){
			// error
		}
		ramp_speed = 0;
	break;
	default: // equal to SYSTEM_STOP (redundant)
		stop_flag = OFF;

		motor.m_1 = 0;
		motor.m_2 = 0;
		motor.m_3 = 0;
		motor.m_4 = 0;

		motor.state_m_1 = MOTOR_STOPP;
		motor.state_m_2 = MOTOR_STOPP;
		motor.state_m_3 = MOTOR_STOPP;
		motor.state_m_4 = MOTOR_STOPP;
		set_postion_control_stats_to_zero();
		// send the speed command to M1 - M4
		if(run_motors(&motor,1) != OK){
			// error
		}
		clear_control_functions();
		ramp_speed = 0;
	break;
	}

	system_q.w = q[0];
	system_q.x = q[1];
	system_q.y = q[2];
	system_q.z = q[3];

}



// Function to stop the System
void system_stop_function(void){
	system_state = SYSTEM_STOP;
}





at_control_f create_attitude_control_signals(void){
	at_control_f Output;
	uint32_t x;

	switch(TEST_FUNCTION){

	case STEP_FUNCTION:
		x = control_signals_counter;
		x /= TIME_PERIOD;

		const float angle_degree = 15.0f;
		const float angle_degree_combined = 10.0f;

		switch (x){



		case TIME_1:
			Output.pitch = 0.0;
			Output.roll = 0.0;
			Output.yaw = 0.0;
			break;

		case TIME_2:
			Output.pitch = angle_degree;
			Output.roll = 0.0;
			Output.yaw = 0.0;
			break;

		case TIME_3:
			Output.pitch = 0.0;
			Output.roll = 0.0;
			Output.yaw = 0.0;
			break;
		case TIME_4:
			Output.pitch = 0.0;
			Output.roll = angle_degree;
			Output.yaw = 0.0;
			break;
		case TIME_5:
			Output.pitch = 0.0;
			Output.roll = 0.0;
			Output.yaw = 0.0;
			break;
		case TIME_6:
			Output.pitch = 0.0;
			Output.roll = 0.0;
			Output.yaw = angle_degree + 10.0f;
			break;
		case TIME_7:
			Output.pitch = 0.0;
			Output.roll = 0.0;
			Output.yaw = 0.0;
			break;
		case TIME_8:
			Output.pitch = angle_degree_combined; //angle_degree/2.0f;//15.0f;
			Output.roll = angle_degree_combined; // angle_degree/2.0f;//15.0f;
			Output.yaw = angle_degree_combined; //angle_degree/2.0f;//15.0f;
			break;
		case TIME_9:
			Output.pitch = 0.0;
			Output.roll = 0.0;
			Output.yaw = 0.0;
			break;
		default:
			control_signals_counter = 0;
			control_flag = OFF;

			Output.pitch = 0.0;
			Output.roll = 0.0;
			Output.yaw = 0.0;
		break;
		}
	break;

	case RAMP_FUNCTION:
		x = control_signals_counter;
		x /= TIME_PERIOD_SHORT;
		switch (x){

			case TIME_1:
			case TIME_2:
				Output.pitch = (0.02f*(control_signals_counter/5));
				Output.roll = 0.0;
				Output.yaw = 0.0;
				break;

			case TIME_3:
				Output.pitch = 20.0f-(0.02f*((control_signals_counter/5)-1000));
				Output.roll = 0.0;
				Output.yaw = 0.0;
				break;

			case TIME_4:
				Output.pitch = 10.0f+(0.02f*((control_signals_counter/5)-1500));
				Output.roll = 0.0;
				Output.yaw = 0.0;
				break;
			case TIME_5:
			case TIME_6:
				Output.pitch = 20.0f-(0.02*((control_signals_counter/5)-2000));
				Output.roll = 0.0f;
				Output.yaw = 0.0;
				break;
			case TIME_7:
				Output.pitch = 0.0;
				Output.roll = 0.0;
				Output.yaw = 0.0;
				break;
			case TIME_8:
			case TIME_9:
				Output.pitch = 0.0f;
				Output.roll = (0.02f*((control_signals_counter/5)-3500));
				Output.yaw = 0.0f;
				break;
			case TIME_10:
				Output.pitch = 0.0;
				Output.roll = 20.0f-(0.02f*((control_signals_counter/5)-4500));
				Output.yaw = 0.0;
				break;
			case TIME_11:
				Output.pitch = 0.0;
				Output.roll = 10.0f+(0.02f*((control_signals_counter/5)-5000));
				Output.yaw = 0.0;
				break;

			case TIME_12:
			case TIME_13:
				Output.pitch = 0.0f;
				Output.roll = 20.0f-(0.02*(control_signals_counter/5)-5500);
				Output.yaw = 0.0;
				break;
			case TIME_14:
				Output.pitch = 0.0;
				Output.roll = 0.0;
				Output.yaw = 0.0;
				break;
			default:
				control_signals_counter = 0;
				control_flag = OFF;

				Output.pitch = 0.0;
				Output.roll = 0.0;
				Output.yaw = 0.0;
			break;

		}

	break;
		case UP_DOWN_FUNCTION:
			x = control_signals_counter;
			x /= TIME_PERIOD_SHORT;

			switch (x){

			case TIME_1:
				Output.pitch = 0.0;
				Output.roll = 0.0;
				Output.yaw = 0.0;
				break;

			case TIME_2:
				Output.pitch = 20.0f;
				Output.roll = 0.0;
				Output.yaw = 0.0;
				break;

			case TIME_3:
				Output.pitch = 0.0;
				Output.roll = 0.0;
				Output.yaw = 0.0;
				break;
			case TIME_4:
				Output.pitch = -20.0;
				Output.roll = 0.0f;
				Output.yaw = 0.0;
				break;
			case TIME_5:
				Output.pitch = 0.0;
				Output.roll = 0.0;
				Output.yaw = 0.0;
				break;
			case TIME_6:
				Output.pitch = 0.0;
				Output.roll = 20.0;
				Output.yaw = 0.0f;
				break;
			case TIME_7:
				Output.pitch = 0.0;
				Output.roll = 0.0;
				Output.yaw = 0.0;
				break;
			case TIME_8:
				Output.pitch = 0.0f;
				Output.roll = -20.0f;
				Output.yaw = 0.0f;
				break;
			case TIME_9:
				Output.pitch = 0.0;
				Output.roll = 0.0;
				Output.yaw = 0.0;
				break;
			case TIME_10:
				Output.pitch = 0.0;
				Output.roll = 0.0;
				Output.yaw = 20.0;
				break;
			case TIME_11:
				Output.pitch = 0.0;
				Output.roll = 0.0;
				Output.yaw = 0.0;
				break;
			case TIME_12:
				Output.pitch = 0.0;
				Output.roll = 0.0;
				Output.yaw = -20.0;
				break;
			case TIME_13:
				Output.pitch = 0.0;
				Output.roll = 0.0;
				Output.yaw = 0.0;
				break;
			default:
				control_signals_counter = 0;
				control_flag = OFF;

				Output.pitch = 0.0;
				Output.roll = 0.0;
				Output.yaw = 0.0;
			break;
			}
		break;
	break;


	default:
		control_signals_counter = 0;
		control_flag = OFF;

		Output.pitch = 0.0;
		Output.roll = 0.0;
		Output.yaw = 0.0;
	break;



	}
	return Output;



}

uint16_t get_hight_of_drone_cm_irq_save(void){
	uint16_t Output;
	__disable_irq();
	Output = (uint16_t)CLAMP(((int32_t)optical_flow.distance_mm/10 - hight_zero_position),0,Q15);
	__enable_irq();
	return (Output);
}

static void get_control_quaternion_from_euler_command(at_control_f euler, int16_t *q_ref){
	int32_t roll, pitch, yaw;
	roll = CLAMP_INT32_TO_INT16((int32_t)(euler.roll * (float)Q15 / 360.0f));
	pitch = CLAMP_INT32_TO_INT16((int32_t)(euler.pitch * (float)Q15 / 360.0f));
	yaw = CLAMP_INT32_TO_INT16((int32_t)(euler.yaw * (float)Q15 / 360.0f));

	euler_to_quat_Q15(pitch, roll, yaw,q_ref);
	NormalizeQuaternionQ15(q_ref, q_ref);

}


static void LED_heartbeat(bool flag){
	static uint16_t counter = 0;

	if(flag == 1){
		if(counter > 1024){
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET); // LED AN
		}else{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);   // LED AUS
		}
	}else{
		if((counter >> 1) > 512){
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET); // LED AN
		}else{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);   // LED AUS
		}
	}

	counter++;
	if(counter > 2048) counter = 0;


}

static void sysetem_ready_check(bool *flag, const int16_t *q){
	if(!flag){
		int32_t x = (uint16_t)q[0] + (uint16_t)q[1] + (uint16_t)q[2] + (uint16_t)q[3];
		if(x > Q14){
			*flag = 1;
		}
	}

}
int16_t optical_flow_raw_x, optical_flow_raw_y;
static void read_DMA_buffer(void){
	if(MTF01_GetFrame(&optical_flow)){
//		uint8_t test =0;

		if(OPTICAL_FLOW_ROTATE){
			optical_flow.flow_vel_x = -optical_flow.flow_vel_x;
//			optical_flow.flow_vel_y = optical_flow.flow_vel_y;
		}
		int16_t vel_dummy[2];
		optical_flow_raw_x = optical_flow.flow_vel_x; optical_flow_raw_y =optical_flow.flow_vel_y;
		vel_dummy[0] = optical_flow.flow_vel_x;	vel_dummy[1] = optical_flow.flow_vel_y;
		position_median_filter_speed_xy_f(vel_dummy);
		optical_flow.flow_vel_x = vel_dummy[0];	optical_flow.flow_vel_y = vel_dummy[1];
	}
}

static int16_t get_r33_from_quaternion(const int16_t *q){
	uint32_t xu;
	int32_t x;

	xu = ((int32_t)q[1] * q[1]) + ((int32_t)q[2] * q[2]); //Q30 ~Q31
	x = (int32_t)Q15 - ((xu + (1 << 13)) >> 14); // 2 * Q15
	return CLAMP_INT32_TO_INT16(x);
}

int16_t max_inclination, max_inclination_ground;
static void init_SAFETY_FUNCTION(void){
	int16_t angle_q15 = CLAMP_INT32_TO_INT16(SAFE_MAX_INCLINATION * (float)Q15 / 180.0f);
	max_inclination = cos_Q15(angle_q15);
	angle_q15 = CLAMP_INT32_TO_INT16(SAFE_MAX_INCLI_GROUND * (float)Q15 / 180.0f);
	max_inclination_ground = cos_Q15(angle_q15);
}

// ############# SAFTY FUNCTIONS
/*
 *  When the drone is close to the ground, the pitch and roll angles
 *  must not vary too much, otherwise there is a risk of the drone
 *  rolling over.
 *
 *  We say if we constantly < 30 cm over the ground, than the drone must be safe
 */
static void SAFETY_START_functon(void){
	static int8_t memory_count = 0;
	static int16_t r33_memory[3] = {0};

	if(hight_mm_corrected < 300){
		memory_count = (memory_count < 100)? (memory_count+1): memory_count;
	}else{
		memory_count = (memory_count > 0)? (memory_count-1):0;
	}

	if(memory_count > 50){
		r33_memory[2] = r33_memory[1]; r33_memory[1] = r33_memory[0];
		r33_memory[0] = system_r33;



		//const int16_t max_inclination = 25102; // ~40° r33 = cos(\theta)
		if((r33_memory[0] < max_inclination_ground) && (r33_memory[1] < max_inclination_ground) && (r33_memory[2] < max_inclination_ground)){
			system_stop_function();
		}

		const int16_t max_acc_ground = 4915; // = 300°/s -> Q15_2000°/s * (300/2000)
		if((abs(w_imu[0]) > max_acc_ground) || (abs(w_imu[1]) > max_acc_ground) ){
			system_stop_function();
		}

	}


}

// During flight, an inclination of more than 50° is not permitted.
static void SAFTETY_FLY_function(void){
	static int16_t r33_memory[3] = {0};
	r33_memory[2] = r33_memory[1]; r33_memory[1] = r33_memory[0];
	r33_memory[0] = system_r33;

	const int16_t max_inclination = 21063; // ~85° r33 = cos(\theta)
	if((r33_memory[0] < max_inclination) && (r33_memory[1] < max_inclination) && (r33_memory[2] < max_inclination)){
		system_stop_function();
	}
}

static void SAFETY_function_tasks(void){
	if(SAFTY_GROUND_LIMITATION == OFF) return;
	SAFETY_START_functon();
	SAFTETY_FLY_function();
}


int16_t gyro_real;
uint8_t test_position_flag = 0;
uint8_t start_position_control = 0;
static void middle_speed_task(void){
	static uint8_t middle_speed_counter = 0;
	static uint16_t pos_timeout_counter = 0;
	static int16_t x_ref[6] = {0,0,80,0,0,0};
	/* pos ref= [x_pos, y_pos, z_pos, x_speed, y_speed, z_speed]
	 * unit for pos are in cm, unit for speed, are cm/s
	 */

	if(middle_speed_counter++ < POS_FREQ_DIV) return;
	middle_speed_counter = 0;
	if(pos_timeout_counter++ < 300) return; // wait 3 sec; and let the sensor fusion correct the position

	// ######### START MIDDLE SPEED ##########

	if(TEST_STATION_ENCODER == ON) gyro_real = read_encoder_rotational_speed(100);
	if(POSITION_CONTROL == OFF) return;

	if(test_position_flag >0){
		test_position_flag = 0;
		x_ref[0] = 30;	x_ref[1] = 30;	x_ref[2] = 130;
		x_ref[3] = 0;	x_ref[4] = 0;	x_ref[5] = 0;
	}
//	if(start_position_control > 0){

		position_control(x_ref,&system_q, &system_q_ref,a_imu,w_imu, &optical_flow, &tauQ10, &hight_mm_corrected,reset_position_control);

//	}else{
//		position_control(x_ref,&system_q, &system_q_ref,a_imu,w_imu, &optical_flow, &tauQ10, &hight_mm_corrected,1);
//		system_q_ref.w = Q15; system_q_ref.x = 0; system_q_ref.y = 0; system_q_ref.z = 0;
//	}
//	position_control(x_ref,&system_q, &system_q_ref,a_imu,w_imu, &optical_flow, &tauQ10, &hight_mm_corrected,reset_position_control);

//	integrate_a_to_v

}

