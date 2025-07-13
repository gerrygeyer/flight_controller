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

uint8_t system_state;
motor_t motor;
at_control_f attitude_control_signals;
// used for Test function
uint32_t control_signals_counter;
uint8_t control_flag;
uint8_t hearthbeat_system;

void task_init(void){

//	system_state = SYSTEM_START;
	system_state = SYSTEM_STOP;

	run_test_vectors();

//	init_orientation();
	init_attitude_control();
	init_control_functions();
	init_sensors();
	//init_motor();
	attitude_control_signals.T = 0;
	attitude_control_signals.pitch = 0;
	attitude_control_signals.roll = 0;
	attitude_control_signals.yaw = 0;

	control_signals_counter = 0;
	control_flag = OFF;

	hearthbeat_system = 0;
}

void time_management(void){


	set_log_data_flag();

	MPU6000_Service();
	LIS3MDL_Service();


	hearthbeat_system ++;

	switch(system_state){
	case SYSTEM_STOP:

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

	break;
	case SYSTEM_START:
		read_distance_sensor();
//		read_distance_lidar1();
//		read_distance_information();
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 1);
		// ask the imu for orientation
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

		run_attitude_control(&motor, &attitude_control_signals);

		motor.state_m_1 = MOTOR_START;
		motor.state_m_2 = MOTOR_START;
		motor.state_m_3 = MOTOR_START;
		motor.state_m_4 = MOTOR_START;

		// send the speed command to M1 - M4
		if(run_motors(&motor) != OK){
			// error
		}
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 0); // Toggle Pin for Time measurement
	break;
	default: // equal to SYSTEM_STOP (redundant)
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


