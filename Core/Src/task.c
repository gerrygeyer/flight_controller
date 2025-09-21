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

// used for Test function
uint32_t control_signals_counter;
uint8_t control_flag, stop_flag;



wxyz_16t system_q, system_q_ref;
int16_t q_yaw_correction[4], q_axis_correction[4], system_r33;
int16_t a_imu[3], w_imu[3];
at_angl_t system_euler_angle;
motor_signals_16t motor_seed_esc;

// debug
int16_t debug_tau_1, debug_tau_2, debug_tau_3;
int16_t debug_speed_counter = 0;
xyz_16t debug_w;
float euler_debug_ax_roll, euler_debug_ax_pitch,euler_debug_ax_yaw;

at_angl_f debug_ref_euler, debug_sys_euler;

static void highspeed_task(void);
static void middle_speed_task(void);
static void LED_heartbeat(bool flag);
static void sysetem_ready_check(bool *flag, const int16_t *q);
static void get_control_quaternion_from_euler_command(at_control_f euler, int16_t *q_ref);
static void read_DMA_buffer(void);
static int16_t get_r33_from_quaternion(const int16_t *q);
static void SAFETY_function_tasks(void);

void task_init(void){

	tauQ10 = 0;

	// ONLY 'SYSTEM_STOP' ARE ALLOWED IF BATTERY ARE CONNECTED

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

	init_motor();
	init_position_control();
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


	read_DMA_buffer();		//
	if(init_tasks){

	service_functions();	// Service functions
	middle_speed_task();	// position control
	highspeed_task();		// attitude control
	SAFETY_function_tasks();// safty functions
	}
//	Tick_500Hz_Handler(&optical_flow);

}
wxyz_16t q_filter,q_debug_yaw;
int16_t angle_from_encoder_Q15;
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
	angle_from_encoder_Q15 = (int16_t)((int16_t)read_encoder_value());

	get_quaternion_Q15(q, w,a);

	debug_w.x = w[0];
	debug_w.y = w[1];
	debug_w.z = w[2];

	a_imu[0] = a[0];	a_imu[1] = a[1];	a_imu[2] = a[2];
	w_imu[0] = w[0];	w_imu[1] = w[1];	w_imu[2] = w[2];

	if(!init_yaw_sucsess){
		if(init_yaw_counter++ > YAW_OFFSET_TIME){
			create_init_yaw_quaternion(q_yaw);
			init_yaw_sucsess = 1;
			q_debug_yaw.w = q_yaw[0];	q_debug_yaw.x = q_yaw[1]; q_debug_yaw.y = q_yaw[2]; q_debug_yaw.z = q_yaw[3];
		}
	}
	multiplicateQuaternionQ15(q_yaw, q, q);
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

	quat_to_euler_q15(q, euler);
	system_euler_angle.roll = euler[0];// * 180.0f / (float)Q15;
	system_euler_angle.pitch = euler[1];// * 180.0f / (float)Q15;
	system_euler_angle.yaw = euler[2];// * 180.0f / (float)Q15;
//	correct_q_axis

	motor_seed_esc = get_motorspeed_from_ESC();


	sysetem_ready_check(&system_ready, q);
	LED_heartbeat(system_ready);

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
			set_postion_control_stats_to_zero();
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
			ramp_speed += 8;
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

		get_control_quaternion_from_euler_command(attitude_control_signals,q_att_ref);

		system_q_ref.w = q_att_ref[0];	system_q_ref.x = q_att_ref[1];
		system_q_ref.y = q_att_ref[2];	system_q_ref.z = q_att_ref[3];

		// only debug/control:
		int16_t euler_ref_debug[3], euler_sys_debug[3];
		quat_to_euler_q15(q_att_ref,euler_ref_debug);
		debug_ref_euler.pitch = (float)euler_ref_debug[0] * 360.0f / (float)Q15;
		debug_ref_euler.roll = (float)euler_ref_debug[1] * 360.0f / (float)Q15;
		debug_ref_euler.yaw = (float)euler_ref_debug[2] * 360.0f / (float)Q15;

		quat_to_euler_q15(q,euler_sys_debug);
		debug_sys_euler.pitch = (float)euler_sys_debug[0] * 360.0f / (float)Q15;
		debug_sys_euler.roll = (float)euler_sys_debug[1] * 360.0f / (float)Q15;
		debug_sys_euler.yaw = (float)euler_sys_debug[2] * 360.0f / (float)Q15;

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
		set_postion_control_stats_to_zero();
		// send the speed command to M1 - M4
		if(run_motors(&motor) != OK){
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

int16_t gyro_real;
static void middle_speed_task(void){
	static uint8_t middle_speed_counter = 0;
	static uint16_t pos_timeout_counter = 0;
	int16_t x_ref[6] = {0};

	if(middle_speed_counter++ < POS_FREQ_DIV) return;
	middle_speed_counter = 0;
	if(pos_timeout_counter++ < 300) return; // wait 3 sec; and let the sensor fusion correct the position

	// ######### START MIDDLE SPEED ##########

	gyro_real = read_encoder_rotational_speed(100);

	if(POSITION_CONTROL == OFF) return;
	position_control(x_ref,&system_q, &system_q_ref,a_imu,w_imu, &optical_flow, &tauQ10, &hight_mm_corrected);


//	integrate_a_to_v

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
			Output.yaw = angle_degree;
			break;
		case TIME_7:
			Output.pitch = 0.0;
			Output.roll = 0.0;
			Output.yaw = 0.0;
			break;
		case TIME_8:
			Output.pitch = angle_degree/2.0f;//15.0f;
			Output.roll = angle_degree/2.0f;//15.0f;
			Output.yaw = angle_degree/2.0f;//15.0f;
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
			flag = 1;
		}
	}

}

static void read_DMA_buffer(void){
	if(MTF01_GetFrame(&optical_flow)){
//		uint8_t test =0;

		if(OPTICAL_FLOW_ROTATE){
			optical_flow.flow_vel_x = -optical_flow.flow_vel_x;
			optical_flow.flow_vel_y = -optical_flow.flow_vel_y;
		}
	}
}

static int16_t get_r33_from_quaternion(const int16_t *q){
	uint32_t xu;
	int32_t x;

	xu = ((int32_t)q[1] * q[1]) + ((int32_t)q[2] * q[2]); //Q30 ~Q31
	x = (int32_t)Q15 - ((xu + (1 << 13)) >> 14); // 2 * Q15
	return CLAMP_INT32_TO_INT16(x);
}

static void init_SAFETY_FUNCTION(void){
	// wow, so much init
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



		const int16_t max_inclination = 25102; // ~40° r33 = cos(\theta)
		if((r33_memory[0] < max_inclination) && (r33_memory[1] < max_inclination) && (r33_memory[2] < max_inclination)){
			system_stop_function();
		}

		const int16_t max_acc_ground = 4915; // = 300°/s -> Q15_2000°/s * (300/2000)
		if((abs(w_imu[0]) > max_acc_ground) || (abs(w_imu[1]) > max_acc_ground) ){
			system_stop_function();
		}

	}


}

// During flight, an inclination of more than 85° is not permitted.
static void SAFTETY_FLY_function(void){
	static int16_t r33_memory[3] = {0};
	r33_memory[2] = r33_memory[1]; r33_memory[1] = r33_memory[0];
	r33_memory[0] = system_r33;

	const int16_t max_inclination = 2856; // ~85° r33 = cos(\theta)
	if((r33_memory[0] < max_inclination) && (r33_memory[1] < max_inclination) && (r33_memory[2] < max_inclination)){
		system_stop_function();
	}
}

static void SAFETY_function_tasks(void){
	if(SAFTY_GROUND_LIMITATION == OFF) return;
	SAFETY_START_functon();
	SAFTETY_FLY_function();
}

