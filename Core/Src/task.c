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

uint8_t system_state;
motor_t motor;
at_control_f attitude_control_signals;
// used for Test function
uint32_t control_signals_counter;
uint8_t control_flag, stop_flag;



wxyz_16t system_q;
int16_t q_yaw_correction[4], q_axis_correction[4];

motor_signals_16t motor_seed_esc;

// debug
int16_t debug_tau_1, debug_tau_2, debug_tau_3;
int16_t debug_speed_counter = 0;
xyz_16t debug_w;
float euler_debug_ax_roll, euler_debug_ax_pitch,euler_debug_ax_yaw;

static void highspeed_task(void);

void task_init(void){

//	system_state = SYSTEM_INIT;
//	system_state = SYSTEM_START;
	system_state = SYSTEM_STOP;

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
	init_sensors();
	init_motor();
	attitude_control_signals.T = 0;
	attitude_control_signals.pitch = 0;
	attitude_control_signals.roll = 0;
	attitude_control_signals.yaw = 0;

	control_signals_counter = 0;
	control_flag = OFF;
	stop_flag = OFF;



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
	set_log_data_flag();
	service_functions();
	highspeed_task();
}

/**
 * @brief		Highspeed Task
 *
 * @details 	All high-frequency tasks are carried out here, such as attitude control
 * 				and communication with the motors.
 */
static void highspeed_task(void){
	int16_t q[4], w[3],q_att_ref[4];
	int16_t tau[3], u[4];
	float u_f[4][1], w_f[4][1];
	static int32_t ramp_speed = 0;
	// debug
	int16_t euler[3];


	get_quaternion_Q15(q, w);

	debug_w.x = w[0];
	debug_w.y = w[1];
	debug_w.z = w[2];
//	correct_q_axis(q,w);

	quat_to_euler_q15(q, euler);
	euler_debug_ax_roll = (float)euler[0];//* 360.0f / (float)INT16_MAX;
	euler_debug_ax_pitch = (float)euler[1];//* 360.0f / (float)INT16_MAX;
	euler_debug_ax_yaw = (float)euler[2];//* 360.0f / (float)INT16_MAX;
//	correct_q_axis

	motor_seed_esc = get_motorspeed_from_ESC();



	switch(system_state){

	case SYSTEM_INIT:
		static uint16_t init_state_counter = 0;
		static uint8_t offset_imu = 0;

		if(offset_imu == 0){
			offset_imu = calculate_offset_values_imu();
		}
		ramp_speed = 0;
		stop_flag = OFF;
		break;

	case SYSTEM_STOP:

		if(stop_flag == OFF){
			stop_flag = ON;

			motor.m_1 = 0;
			motor.m_2 = 0;
			motor.m_3 = 0;
			motor.m_4 = 0;

			motor.state_m_1 = MOTOR_STOPP;
			motor.state_m_2 = MOTOR_STOPP;
			motor.state_m_3 = MOTOR_STOPP;
			motor.state_m_4 = MOTOR_STOPP;

			// send the speed command to M1 - M4
			if(run_motors(&motor) != OK){
				// error
			}
		}

		clear_control_functions();
		ramp_speed = 0;

	break;
	case SYSTEM_RAMP:
		stop_flag = OFF;



		motor.m_1 = ramp_speed;
		motor.m_2 = -ramp_speed;
		motor.m_3 = ramp_speed;
		motor.m_4 = ramp_speed;

		motor.state_m_1 = MOTOR_START;
		motor.state_m_2 = MOTOR_START;
		motor.state_m_3 = MOTOR_START;
		motor.state_m_4 = MOTOR_START;

		// send the speed command to M1 - M4
		if(run_motors(&motor) != OK){
			// error
		}

		clear_control_functions();

		if(ramp_speed < 3000){
			ramp_speed += 6;
		}else{
			system_state = SYSTEM_START;
		}

	break;
	case SYSTEM_START:
		stop_flag = OFF;
		read_distance_sensor();
		read_orientation();


		switch (control_flag){
		case ON:
			attitude_control_signals = create_attitude_control_signals();
			break;
		default:
			control_signals_counter = 0;
			break;
		}
		control_signals_counter += 1;

//		q_att_ref[0] = (int16_t)( 0.9818f * Q15);
//		q_att_ref[1] = (int16_t)( 0.0641 * Q15);
//		q_att_ref[2] = (int16_t)( -0.1436 * Q15);
//		q_att_ref[3] = (int16_t)( 0.1060 * Q15);

		q_att_ref[0] = (int16_t)( 1.0f * Q15);
		q_att_ref[1] = (int16_t)( 0.0f * Q15);
		q_att_ref[2] = (int16_t)( 0.0f * Q15);
		q_att_ref[3] = (int16_t)( 0.0f * Q15);
//		q_att_ref[0] = (int16_t)(13731);
//		q_att_ref[1] = (int16_t)(-15570);
//		q_att_ref[2] = (int16_t)(12978);
//		q_att_ref[3] = (int16_t)(21780);

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
//


		debug_tau_1 = tau[0]; // debug
		debug_tau_2 = tau[1];	// debug
		debug_tau_3 = tau[2]; // devug

		u[0] = 0;
		u[1] = tau[0];
		u[2] = tau[1];
		u[3] = tau[2];




		transform_u2_motorSpeed(u,&motor);


//		debug_speed_counter++;
//		if(debug_speed_counter > 10000) debug_speed_counter = -10000;
//		motor.m_1 = debug_speed_counter;
//		motor.m_2 = debug_speed_counter;
//		motor.m_3 = debug_speed_counter;
//		motor.m_4 = debug_speed_counter;



		motor.state_m_1 = MOTOR_START;
		motor.state_m_2 = MOTOR_START;
		motor.state_m_3 = MOTOR_START;
		motor.state_m_4 = MOTOR_START;

		// send the speed command to M1 - M4
		if(run_motors(&motor) != OK){
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

		// send the speed command to M1 - M4
		if(run_motors(&motor) != OK){
			// error
		}
		clear_control_functions();
		ramp_speed = 0;
	break;
	}

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
			Output.pitch = 0.0;
			Output.roll = 20.0f;
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
			Output.yaw = 20.0f;
			break;
		case TIME_7:
			Output.pitch = 0.0;
			Output.roll = 0.0;
			Output.yaw = 0.0;
			break;
		case TIME_8:
			Output.pitch = 15.0f;
			Output.roll = 15.0f;
			Output.yaw = 15.0f;
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
				Output.pitch = (0.02f*control_signals_counter);
				Output.roll = 0.0;
				Output.yaw = 0.0;
				break;

			case TIME_3:
				Output.pitch = 20.0f-(0.02f*(control_signals_counter-1000));
				Output.roll = 0.0;
				Output.yaw = 0.0;
				break;

			case TIME_4:
				Output.pitch = 10.0f+(0.02f*(control_signals_counter-1500));
				Output.roll = 0.0;
				Output.yaw = 0.0;
				break;
			case TIME_5:
			case TIME_6:
				Output.pitch = 20.0f-(0.02*(control_signals_counter-2000));
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
				Output.roll = (0.02f*(control_signals_counter-3500));
				Output.yaw = 0.0f;
				break;
			case TIME_10:
				Output.pitch = 0.0;
				Output.roll = 20.0f-(0.02f*(control_signals_counter-4500));
				Output.yaw = 0.0;
				break;
			case TIME_11:
				Output.pitch = 0.0;
				Output.roll = 10.0f+(0.02f*(control_signals_counter-5000));
				Output.yaw = 0.0;
				break;

			case TIME_12:
			case TIME_13:
				Output.pitch = 0.0f;
				Output.roll = 20.0f-(0.02*(control_signals_counter-5500));
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


