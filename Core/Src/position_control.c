/*
 * position_control.c
 *
 *  Created on: Sep 4, 2025
 *      Author: gerrygeyer
 */


#include <position_control.h>
#include <solve_cost_function.h>
#include <parameter.h>


float Ki_pos_xy,Ki_pos_z, Ki_acc_xy,Ki_acc_z, dt;
int16_t weight_kg_q10;
int16_t x_pos_acc[6] =  {0};
int16_t opt_flow_w_corr;

static inline void get_array_from_struct_Q15(const wxyz_16t *q_struct, int16_t *q_array);
static inline void get_struct_from_array_Q15(const int16_t *q_array, wxyz_16t *q_struct);
static inline void copy_q(const int16_t *q_in, int16_t *q_copy);
static inline void neg_q_Q15(int16_t *q);
static inline void multQuatwithConstQ15(int16_t* q, const int16_t x);
static void get_state_error(const int16_t *x_pos_acc, const int16_t *x_pos_acc_ref, int16_t *x_error);
static void integrate_error(const int16_t *error, const int16_t *deltaT_K_i, int16_t *integral);
static void get_delta_K(float Ki_pos_xy, float Ki_pos_z, float Ki_acc_xy, float Ki_acc_z, float dt, int16_t *delta_K);
static void lqr_q15(const int16_t *x_error,int16_t *u_out, const int16_t K_q10[3][6]);
static void remove_gravity_q10(int16_t *u_q5);
//static void iir_filter_acc_Q15(const int16_t *acc_raw, int16_t *acc_filter);
static void integrate_a_to_v(int16_t *a, int16_t *v_sq6_q15);
static void integrate_v_to_m(int16_t *v_sq6_q15, int16_t *m_sq6_q15);
static void position_filter_SLERP_EMA_quaternion_Q15(const int16_t *q_in, int16_t *q_out);
static void get_hight_and_speed_from_optical_flow(const mtf01_payload_t *optic_flow, const int16_t *q,const int16_t *a, int16_t *hight_mm, int16_t *speed_NED);
static void position_iir_filter_acc_Q15(const int16_t *acc_raw, int16_t *acc_filter, const float fc, const float Ts);
static void position_iir_filter_speed_hight_Q15(const int16_t *pos_raw, int16_t *pos_filter, const float fc, const float Ts);
static void position_iir_filter_speed_xy_Q15(const int16_t *pos_raw, int16_t *pos_filter, const float fc, const float Ts);
static void position_iir_filter_w12_phase_shift_Q15(const int16_t *pos_raw, int16_t *pos_filter, float fc, float Ts);
static void remove_constant_offset(const int16_t *u, int16_t *y);
static void position_iir_filter_hight_Q15(const int16_t *pos_raw, int16_t *pos_filter, const float fc, const float Ts);
static void position_iir_filter_f(int32_t *v_x, int32_t * v_y, float fc, float frequency);

void init_position_control(void){

	Ki_pos_xy 	= 1.0f;
	Ki_pos_z 	= 1.0f;
	Ki_acc_xy 	= 1.0f;
	Ki_acc_z 	= 1.0f;
	dt			= 1.0f / (float)POSITION_FREQUENCY;

	weight_kg_q10 = CLAMP_INT32_TO_INT16((int32_t)(DRONE_WEIGHT_KG * (float)Q10));
	opt_flow_w_corr = CLAMP_INT32_TO_INT16((int32_t)((float)Q15 * OPT_FLOW_CORR_FACTOR / 34.9f));

	if(LOOP_EXERCISE == SOLVE_COST_FCT){
		float Q_p[6] = {0.1,0.1,0.1, 2, 2, 2};
		float R_p[3] = {1, 1, 1};
		float Ts = 1/((float)POSITION_FREQUENCY);
		run_cost_fct(Q_p[0],Q_p[1],Q_p[2],Q_p[3],Q_p[4],Q_p[5],R_p[0],R_p[1],R_p[2],Ts);

//		float rho_max, rho_axis[3], lam1[3], lam2[3];
//		bool stable = lqr_stability_check_2x2(Ad, K, &rho_max, rho_axis, lam1, lam2);
//
//
//		float margin = 1.0f - rho_max;   // >0 -> stabil, je größer desto robuster
//		// break here
//		uint8_t test = 0;

	}
}
xyz_16t debug_position, debug_pos_vel, debug_pos_speed, debug_pos_vel_filter;
int16_t debug_position_hight = 0;
int16_t debug_tauQ10;
wxyz_16t q_pos_ref_output;
void position_control(const int16_t *x_pos_acc_ref,const wxyz_16t *system_q, wxyz_16t *system_q_ref,const int16_t *a, const int16_t *w, const mtf01_payload_t *optic_flow, int16_t *tauQ10, int16_t *hight_mm_corrected){

	static int16_t speed_NED_cm_s[3] = {0}, position_cm[3] = {0},hight_mm = 0, hight_mm_old = 0;
	static uint8_t optical_flow_50hz = 0;
	static uint8_t optical_flow_10hz = 0;
	uint32_t time_old = 0;
	int16_t q_ref[4], q_system[4],a_body[3], a_NED[3];

	const int16_t delta_t_pos = 655; // (1/50) * Q15
	const int16_t delta_t_cm_s = 1029; // scaling_a / Frequency -> (16*g [m/s^2])/100[s] -> ((16*g/100) [cm/s^2] / 100[s] ) * Q15

	get_array_from_struct_Q15(system_q, q_system);
//	position_filter_SLERP_EMA_quaternion_Q15(q_system,q_system);
	position_iir_filter_acc_Q15(a,a_body, POS_ACC_LP_FC , (1.0f/100.0f));

	if(optical_flow_50hz++ > 0){
		optical_flow_50hz = 0;


	// take the optical flow if the sensor is avalible and gives good results; Plan B: thake the acc for position (bad)
		if(((optic_flow->flow_quality > 30) && (optic_flow->distance_mm > 10)) && COMMUNICATION_OPT_FLW){
			uint32_t time_between_measurements = optic_flow->system_time_ms - time_old;
			if(time_between_measurements > 0){
				get_hight_and_speed_from_optical_flow(optic_flow, q_system,w,&hight_mm, speed_NED_cm_s);
				time_old = optic_flow->system_time_ms;
				position_cm[2] = hight_mm/10;
				debug_position_hight = position_cm[2];

			}else{
				// no new value, take the old one
	//			time_old = optic_flow->system_time_ms;

			}

	//		int32_t delta_time_opt_flow = ((int32_t)optic_flow->system_time_ms - time_old);

			if(optical_flow_10hz++ > 3){
					optical_flow_10hz = 0;

					stopp_time_measurement();
					start_time_measurement();

				const int32_t frequency = 10; // 50 hz -> optical flow ferquency (but only 10 Hz distance measurement); 10mm = 1 cm
				const int32_t cm_to_mm_div = 10;
				speed_NED_cm_s[2] = CLAMP_INT32_TO_INT16((((int32_t)hight_mm - (int32_t)hight_mm_old) * frequency) / cm_to_mm_div);
				position_iir_filter_speed_hight_Q15(&speed_NED_cm_s[2],&speed_NED_cm_s[2],POS_SPEED_HIGHT_LP_FC,(1.0f/50.0f));
				hight_mm_old = (int32_t)hight_mm;
			}
	//			time_old = (int32_t)optic_flow->system_time_ms;

		}else{

			rotate_vector_Q15(q_system, a, a_NED);
			a_NED[2] = CLAMP_INT32_TO_INT16((int32_t)a_NED[2] - (1<<11)); // remove gravity -> g = Q15/16 = Q11

			speed_NED_cm_s[0] = CLAMP_INT32_TO_INT16((int32_t)speed_NED_cm_s[0] + q15_mul(a_NED[0],delta_t_cm_s));
			speed_NED_cm_s[1] = CLAMP_INT32_TO_INT16((int32_t)speed_NED_cm_s[1] + q15_mul(a_NED[1],delta_t_cm_s));
			speed_NED_cm_s[2] = CLAMP_INT32_TO_INT16((int32_t)speed_NED_cm_s[2] + q15_mul(a_NED[2],delta_t_cm_s));

	//		debug_pos_speed.x = speed_NED_old[0];	debug_pos_speed.y = speed_NED_old[1];	debug_pos_speed.z = speed_NED_old[2];

			position_cm[2] = CLAMP_INT32_TO_INT16(position_cm[2] + q15_mul(speed_NED_cm_s[2],delta_t_pos));

		}
	position_iir_filter_speed_xy_Q15(speed_NED_cm_s,speed_NED_cm_s,POS_SPEED_XY_LP_FC,(1/50.0f));
	debug_pos_vel.x = speed_NED_cm_s[0];	debug_pos_vel.y = speed_NED_cm_s[1]; debug_pos_vel.z = speed_NED_cm_s[2];
//	int16_t speed_filter[3];
//	remove_constant_offset(speed_NED_cm_s,speed_NED_cm_s);
//	debug_pos_vel_filter.x = speed_filter[0]; debug_pos_vel_filter.y = speed_filter[1]; debug_pos_vel_filter.z = speed_filter[2];
	position_cm[0] = CLAMP_INT32_TO_INT16(position_cm[0] + q15_mul(speed_NED_cm_s[0],delta_t_pos));
	position_cm[1] = CLAMP_INT32_TO_INT16(position_cm[1] + q15_mul(speed_NED_cm_s[1],delta_t_pos));
//	debug_pos_vel.x = a_NED[0];	debug_pos_vel.y = a_NED[1]; debug_pos_vel.z = a_NED[2];
	debug_position.x = position_cm[0];	debug_position.y = position_cm[1];	debug_position.z = position_cm[2];

	x_pos_acc[0] = position_cm[0];		x_pos_acc[1] = position_cm[1];		x_pos_acc[2] = position_cm[2];
	x_pos_acc[3] = speed_NED_cm_s[0];	x_pos_acc[4] = speed_NED_cm_s[1];	x_pos_acc[5] = speed_NED_cm_s[2];

	x_pos_acc[2] = CLAMP(x_pos_acc[2], 0, MAX_HIGHT);
}
	pc_lqr(x_pos_acc,x_pos_acc_ref,q_ref,tauQ10);
	debug_tauQ10 = tauQ10[0];
	hight_mm_corrected[0] = hight_mm;
	q_pos_ref_output.w = q_ref[0]; q_pos_ref_output.x = q_ref[1]; q_pos_ref_output.y = q_ref[2]; q_pos_ref_output.z = q_ref[3];

}

void set_postion_control_stats_to_zero(void){
	for(uint8_t i = 0; i<6; i++){
		x_pos_acc[i] = 0;
	}
}
static lqr_state debug_function_get_struct_from_array(const int16_t *array){
	lqr_state Output;

	Output.x1 = array[0]; Output.x2 = array[1]; Output.x3 = array[2];
	Output.x4 = array[3]; Output.x5 = array[4]; Output.x6 = array[5];

	return Output;
}

xyz_16t debug_position_uq10;
lqr_state lqr_x_err_debug, lqr_x_ref, lqr_x_soll;
void pc_lqr(const int16_t *x_pos_acc, const int16_t *x_pos_acc_ref, int16_t *q_ref, int16_t *tauQ10){

	int16_t x_pos_acc_err[6], delta_K[6], x[6], u_q10[3], v_norm_q10;
	static int16_t K_q10[3][6] = {
	    {   324,      0,      0,   1661,      0,      0},
	    {     0,    324,      0,      0,   1661,      0},
	    {     0,      0,    324,      0,      0,   1661}
	};
	if(LOOP_EXERCISE == SOLVE_COST_FCT){
		uint8_t k_ready = getK_matrix(K_q10);
		if (k_ready == 1){
			uint8_t test = 0;
		}
	}

	get_delta_K(Ki_pos_xy,Ki_pos_z,Ki_acc_xy,Ki_acc_z,dt,delta_K); // <- here we tatke the conversion from cm to m

	lqr_x_ref = debug_function_get_struct_from_array(x_pos_acc_ref);
	lqr_x_soll = debug_function_get_struct_from_array(x_pos_acc);
	get_state_error(x_pos_acc,x_pos_acc_ref,x_pos_acc_err);
	lqr_x_err_debug.x1 = x_pos_acc_err[0]; lqr_x_err_debug.x2 = x_pos_acc_err[1]; lqr_x_err_debug.x3 = x_pos_acc_err[2];
	lqr_x_err_debug.x4 = x_pos_acc_err[3]; lqr_x_err_debug.x5 = x_pos_acc_err[4]; lqr_x_err_debug.x6 = x_pos_acc_err[5];
	integrate_error(x_pos_acc_err,delta_K,x);
	lqr_q15(x,u_q10, K_q10);
	// come from cm to m
	remove_gravity_q10(u_q10);
	debug_position_uq10.x = u_q10[0]; debug_position_uq10.y = u_q10[1]; debug_position_uq10.z = u_q10[2];
	v_norm_q10 = norm_of_3D_vector(u_q10);
	q_ref[0] = CLAMP_INT32_TO_INT16(v_norm_q10 + u_q10[2]);
	q_ref[1] = -u_q10[1];
	q_ref[2] = u_q10[0];
	q_ref[3] = 0;

	NormalizeQuaternionQ15(q_ref, q_ref);
	// Fth = norm(u) * m
	tauQ10[0] = CLAMP_INT32_TO_INT16(Q10_SHIFT_ROUND((int32_t)weight_kg_q10 * (int32_t)v_norm_q10));
//	v_norm_q10 = norm_of_3D_vector(u_q10);
}

static inline void get_array_from_struct_Q15(const wxyz_16t *q_struct, int16_t *q_array){
	q_array[0] = q_struct->w;	q_array[1] = q_struct->x;	q_array[2] = q_struct->y;	q_array[3] = q_struct->z;
}
static inline void get_struct_from_array_Q15(const int16_t *q_array, wxyz_16t *q_struct){
	q_struct->w = q_array[0];	q_struct->x = q_array[1];	q_struct->y = q_array[2];	q_struct->z = q_array[3];
}

static inline void copy_q(const int16_t *q_in, int16_t *q_copy){
	q_copy[0] = q_in[0];	q_copy[1] = q_in[1];	q_copy[2] = q_in[2];	q_copy[3] = q_in[3];
}

static inline void neg_q_Q15(int16_t *q){
	q[0] = -q[0];	q[1] = -q[1];	q[2] = -q[2];	q[3] = -q[3];
}
static inline void multQuatwithConstQ15(int16_t* q, const int16_t x){
	q[0] = q15_mul(q[0], x);
	q[1] = q15_mul(q[1], x);
	q[2] = q15_mul(q[2], x);
	q[3] = q15_mul(q[3], x);
}


static void get_state_error(const int16_t *x_pos_acc, const int16_t *x_pos_acc_ref, int16_t *x_error){
	x_error[0] = CLAMP_INT32_TO_INT16((int32_t)x_pos_acc[0] - (int32_t)x_pos_acc_ref[0]);
	x_error[1] = CLAMP_INT32_TO_INT16((int32_t)x_pos_acc[1] - (int32_t)x_pos_acc_ref[1]);
	x_error[2] = CLAMP_INT32_TO_INT16((int32_t)x_pos_acc[2] - (int32_t)x_pos_acc_ref[2]);
	x_error[3] = CLAMP_INT32_TO_INT16((int32_t)x_pos_acc[3] - (int32_t)x_pos_acc_ref[3]);
	x_error[4] = CLAMP_INT32_TO_INT16((int32_t)x_pos_acc[4] - (int32_t)x_pos_acc_ref[4]);
	x_error[5] = CLAMP_INT32_TO_INT16((int32_t)x_pos_acc[5] - (int32_t)x_pos_acc_ref[5]);
}

static void get_delta_K(float Ki_pos_xy, float Ki_pos_z, float Ki_acc_xy, float Ki_acc_z, float dt, int16_t *delta_K){

	const float cm_to_m = 0.01; // 1/ 100
	delta_K[0] = CLAMP_INT32_TO_INT16((int32_t)((float)Q15 * Ki_pos_xy * dt * cm_to_m));
	delta_K[1] = CLAMP_INT32_TO_INT16((int32_t)((float)Q15 * Ki_pos_xy * dt * cm_to_m));
	delta_K[2] = CLAMP_INT32_TO_INT16((int32_t)((float)Q15 * Ki_pos_z * dt * cm_to_m));
	delta_K[3] = CLAMP_INT32_TO_INT16((int32_t)((float)Q15 * Ki_acc_xy * dt * cm_to_m));
	delta_K[4] = CLAMP_INT32_TO_INT16((int32_t)((float)Q15 * Ki_acc_xy * dt * cm_to_m));
	delta_K[5] = CLAMP_INT32_TO_INT16((int32_t)((float)Q15 * Ki_acc_z * dt * cm_to_m));

}

static void integrate_error(const int16_t *error, const int16_t *deltaT_K_i, int16_t *integral){

	static int32_t error_int[6];
	int32_t x[6];

	x[0] = ((int32_t)error[0] * (int32_t)deltaT_K_i[0]) >> 5;  // Q25
	x[1] = ((int32_t)error[1] * (int32_t)deltaT_K_i[1]) >> 5;
	x[2] = ((int32_t)error[2] * (int32_t)deltaT_K_i[2]) >> 5;
	x[3] = ((int32_t)error[3] * (int32_t)deltaT_K_i[3]) >> 5;
	x[4] = ((int32_t)error[4] * (int32_t)deltaT_K_i[4]) >> 5;
	x[5] = ((int32_t)error[5] * (int32_t)deltaT_K_i[5]) >> 5;


	error_int[0] = CLAMP((error_int[0] + x[0]),-Q25,Q25);
	error_int[1] = CLAMP((error_int[1] + x[1]),-Q25,Q25);
	error_int[2] = CLAMP((error_int[2] + x[2]),0,Q25); // we are on ground, so...
	error_int[3] = CLAMP((error_int[3] + x[3]),-Q25,Q25);
	error_int[4] = CLAMP((error_int[4] + x[4]),-Q25,Q25);
	error_int[5] = CLAMP((error_int[5] + x[5]),-Q25,Q25);

	integral[0] = CLAMP_INT32_TO_INT16(Q10_SHIFT_ROUND(error_int[0])); // Q10_SHIFT_ROUND: shift with rounding
	integral[1] = CLAMP_INT32_TO_INT16(Q10_SHIFT_ROUND(error_int[1]));
	integral[2] = CLAMP_INT32_TO_INT16(Q10_SHIFT_ROUND(error_int[2]));
	integral[3] = CLAMP_INT32_TO_INT16(Q10_SHIFT_ROUND(error_int[3]));
	integral[4] = CLAMP_INT32_TO_INT16(Q10_SHIFT_ROUND(error_int[4]));
	integral[5] = CLAMP_INT32_TO_INT16(Q10_SHIFT_ROUND(error_int[5]));

}



static void lqr_q15(const int16_t *x_error,int16_t *u_out, const int16_t K_q10[3][6]){
	int32_t x, sum;



	for (int i = 0; i < 3; i++) {
		sum = 0;

		x = (int32_t)K_q10[i][0] * x_error[0]; // Q10 * Q15 = Q25
		sum += (x >> 2); // Q23

		x = (int32_t)K_q10[i][1] * x_error[1];
		sum += (x >> 2); // Q23

		x = (int32_t)K_q10[i][2] * x_error[2];
		sum += (x >> 2); // Q23

		x = (int32_t)K_q10[i][3] * x_error[3];
		sum += (x >> 2); // Q23

		x = (int32_t)K_q10[i][4] * x_error[4];
		sum += (x >> 2); // Q23

		x = (int32_t)K_q10[i][5] * x_error[5];
		sum += (x >> 2); // Q23

		sum = Q13_SHIFT_ROUND(sum); // back to Q15 //-> u_max = Q5
		u_out[i] = -CLAMP_INT32_TO_INT16(sum);
	}
}

static void remove_gravity_q10(int16_t *u_q5){
//	const int32_t g = 10045; //9.81 * 2^10
	u_q5[2] = CLAMP_INT32_TO_INT16((int32_t)u_q5[2] - 10045);
}

void get_speed_and_pos_from_acc(sensor_fusion *pHandle_sf, int16_t *speed_ms_q10, int32_t *distance_m_q10){
	int16_t acc_body[3], acc_world[3],q_pos[4];

	acc_body[0] = pHandle_sf->acc_t.x;	acc_body[1] = pHandle_sf->acc_t.y;	acc_body[2] = pHandle_sf->acc_t.z;
	q_pos[0] = pHandle_sf->quaternion.w;	q_pos[1] = pHandle_sf->quaternion.x;
	q_pos[2] = pHandle_sf->quaternion.y;	q_pos[3] = pHandle_sf->quaternion.z;

	position_iir_filter_acc_Q15(acc_body, acc_body, 10.0f, (1.0f/100.0f)); // fc = 10 hz Ts = 1/100 s
	rotate_vector_Q15(q_pos, acc_body, acc_world);

}




static void position_iir_filter_acc_Q15(const int16_t *acc_raw, int16_t *acc_filter, const float fc, const float Ts){

//	static int32_t a = CLAMP_INT32_TO_INT16((int32_t)((PI_MULTIPLY_2 * (float)fc * (1.0f/(float)SENSOR_FUSION_FREQUENCY_IMU))/(PI_MULTIPLY_2 * (float)fc * (1.0f/(float)SENSOR_FUSION_FREQUENCY_IMU) + 1.0f)));
	static int16_t a;
	static bool init = 0;
	if(!init){
		init = 1;
		a = CLAMP_INT32_TO_INT16((int32_t)((PI_MULTIPLY_2 * Ts * fc)/((PI_MULTIPLY_2 * Ts * fc + 1.0f)) * (float)Q15)); // ~0.3859 * Q15 // fc = 10 Hz Ts = 1/100
	}
	static int32_t acc_old[3] = {0};


	acc_old[0] = ((a * (int32_t)acc_raw[0]) >> 15) + ((((int32_t)Q15 - a) * acc_old[0]) >> 15);
	acc_old[1] = ((a * (int32_t)acc_raw[1]) >> 15) + ((((int32_t)Q15 - a) * acc_old[1]) >> 15);
	acc_old[2] = ((a * (int32_t)acc_raw[2]) >> 15) + ((((int32_t)Q15 - a) * acc_old[2]) >> 15);

	acc_filter[0] = acc_old[0] = CLAMP_INT32_TO_INT16(acc_old[0]);
	acc_filter[1] = acc_old[1] = CLAMP_INT32_TO_INT16(acc_old[1]);
	acc_filter[2] = acc_old[2] = CLAMP_INT32_TO_INT16(acc_old[2]);
}

static void integrate_a_to_v(int16_t *a, int16_t *v_sq6_q15){
	static int32_t v_old_q25[3];
	const int16_t delta_t =  804; // (16g/Q6) * 0.01 * (2^15)
	v_old_q25[0] = CLAMP(((v_old_q25[0] + ((int32_t)a[0] * delta_t)) >> 5),-Q30,Q30);
	v_old_q25[1] = CLAMP(((v_old_q25[1] + ((int32_t)a[1] * delta_t)) >> 5),-Q30,Q30);
	v_old_q25[2] = CLAMP(((v_old_q25[2] + ((int32_t)a[2] * delta_t)) >> 5),-Q30,Q30);
	v_sq6_q15[0] = Q10_SHIFT_ROUND(v_old_q25[0]);
	v_sq6_q15[1] = Q10_SHIFT_ROUND(v_old_q25[1]);
	v_sq6_q15[2] = Q10_SHIFT_ROUND(v_old_q25[2]);
}

static void integrate_v_to_m(int16_t *v_sq6_q15, int16_t *m_sq6_q15){
	static int32_t m_old_q25[3];
	const int16_t delta_t = 328; // 0.01 * (2^15)
	m_old_q25[0] = CLAMP(((m_old_q25[0] + ((int32_t)v_sq6_q15[0] * delta_t)) >> 5),-Q30,Q30);
	m_old_q25[1] = CLAMP(((m_old_q25[1] + ((int32_t)v_sq6_q15[1] * delta_t)) >> 5),-Q30,Q30);
	m_old_q25[2] = CLAMP(((m_old_q25[2] + ((int32_t)v_sq6_q15[2] * delta_t)) >> 5),-Q30,Q30);
	m_sq6_q15[0] = Q10_SHIFT_ROUND(m_old_q25[0]);
	m_sq6_q15[1] = Q10_SHIFT_ROUND(m_old_q25[1]);
	m_sq6_q15[2] = Q10_SHIFT_ROUND(m_old_q25[2]);
}

static void position_iir_filter_speed_x_Q15(const int32_t *pos_raw, int32_t *pos_filter, const float fc, const float Ts){

//	static int32_t a = CLAMP_INT32_TO_INT16((int32_t)((PI_MULTIPLY_2 * (float)fc * (1.0f/(float)SENSOR_FUSION_FREQUENCY_IMU))/(PI_MULTIPLY_2 * (float)fc * (1.0f/(float)SENSOR_FUSION_FREQUENCY_IMU) + 1.0f)));
	static int16_t a;
	static bool init = 0;
	if(!init){
		init = 1;
		a = CLAMP_INT32_TO_INT16((int32_t)((PI_MULTIPLY_2 * Ts * fc)/((PI_MULTIPLY_2 * Ts * fc + 1.0f)) * (float)Q15)); // ~0.3859 * Q15 // fc = 10 Hz Ts = 1/100
	}
	static int32_t pos_old = 0;


	pos_old = ((a * (int32_t)pos_raw[0]) >> 15) + ((((int32_t)Q15 - a) * pos_old) >> 15);


	pos_filter[0] = pos_old = CLAMP_INT32_TO_INT16(pos_old);

}
static void position_iir_filter_hight_Q15(const int16_t *pos_raw, int16_t *pos_filter, const float fc, const float Ts){

//	static int32_t a = CLAMP_INT32_TO_INT16((int32_t)((PI_MULTIPLY_2 * (float)fc * (1.0f/(float)SENSOR_FUSION_FREQUENCY_IMU))/(PI_MULTIPLY_2 * (float)fc * (1.0f/(float)SENSOR_FUSION_FREQUENCY_IMU) + 1.0f)));
	static int16_t a;
	static bool init = 0;
	if(!init){
		init = 1;
		a = CLAMP_INT32_TO_INT16((int32_t)((PI_MULTIPLY_2 * Ts * fc)/((PI_MULTIPLY_2 * Ts * fc + 1.0f)) * (float)Q15)); // ~0.3859 * Q15 // fc = 10 Hz Ts = 1/100
	}
	static int32_t pos_old = 0;


	pos_old = ((a * (int32_t)pos_raw[0]) >> 15) + ((((int32_t)Q15 - a) * pos_old) >> 15);


	pos_filter[0] = pos_old = CLAMP_INT32_TO_INT16(pos_old);

}
static void position_iir_filter_speed_hight_Q15(const int16_t *pos_raw, int16_t *pos_filter, const float fc, const float Ts){

//	static int32_t a = CLAMP_INT32_TO_INT16((int32_t)((PI_MULTIPLY_2 * (float)fc * (1.0f/(float)SENSOR_FUSION_FREQUENCY_IMU))/(PI_MULTIPLY_2 * (float)fc * (1.0f/(float)SENSOR_FUSION_FREQUENCY_IMU) + 1.0f)));
	static int16_t a;
	static bool init = 0;
	if(!init){
		init = 1;
		a = CLAMP_INT32_TO_INT16((int32_t)((PI_MULTIPLY_2 * Ts * fc)/((PI_MULTIPLY_2 * Ts * fc + 1.0f)) * (float)Q15)); // ~0.3859 * Q15 // fc = 10 Hz Ts = 1/100
	}
	static int32_t pos_old = 0;


	pos_old = ((a * (int32_t)pos_raw[0]) >> 15) + ((((int32_t)Q15 - a) * pos_old) >> 15);


	pos_filter[0] = pos_old = CLAMP_INT32_TO_INT16(pos_old);

}
static void position_iir_filter_speed_y_Q15(const int32_t *pos_raw, int32_t *pos_filter, const float fc, const float Ts){

//	static int32_t a = CLAMP_INT32_TO_INT16((int32_t)((PI_MULTIPLY_2 * (float)fc * (1.0f/(float)SENSOR_FUSION_FREQUENCY_IMU))/(PI_MULTIPLY_2 * (float)fc * (1.0f/(float)SENSOR_FUSION_FREQUENCY_IMU) + 1.0f)));
	static int16_t a;
	static bool init = 0;
	if(!init){
		init = 1;
		a = CLAMP_INT32_TO_INT16((int32_t)((PI_MULTIPLY_2 * Ts * fc)/((PI_MULTIPLY_2 * Ts * fc + 1.0f)) * (float)Q15)); // ~0.3859 * Q15 // fc = 10 Hz Ts = 1/100
	}
	static int32_t pos_old = 0;

	pos_old = ((a * (int32_t)pos_raw[0]) >> 15) + ((((int32_t)Q15 - a) * pos_old) >> 15);
	pos_filter[0] = pos_old = CLAMP_INT32_TO_INT16(pos_old);

}

static void position_iir_filter_speed_xy_Q15(const int16_t *pos_raw, int16_t *pos_filter, const float fc, const float Ts){

//	static int32_t a = CLAMP_INT32_TO_INT16((int32_t)((PI_MULTIPLY_2 * (float)fc * (1.0f/(float)SENSOR_FUSION_FREQUENCY_IMU))/(PI_MULTIPLY_2 * (float)fc * (1.0f/(float)SENSOR_FUSION_FREQUENCY_IMU) + 1.0f)));
	static int16_t a;
	static bool init = 0;
	if(!init){
		init = 1;
		a = CLAMP_INT32_TO_INT16((int32_t)((PI_MULTIPLY_2 * Ts * fc)/((PI_MULTIPLY_2 * Ts * fc + 1.0f)) * (float)Q15)); // ~0.3859 * Q15 // fc = 10 Hz Ts = 1/100
	}
	static int32_t pos_old[2] = {0};

	pos_old[0] = ((a * (int32_t)pos_raw[0]) >> 15) + ((((int32_t)Q15 - a) * pos_old[0]) >> 15);
	pos_filter[0] = pos_old[0] = CLAMP_INT32_TO_INT16(pos_old[0]);

	pos_old[1] = ((a * (int32_t)pos_raw[1]) >> 15) + ((((int32_t)Q15 - a) * pos_old[1]) >> 15);
	pos_filter[1] = pos_old[1] = CLAMP_INT32_TO_INT16(pos_old[1]);


}

static void position_iir_filter_w12_phase_shift_Q15(const int16_t *pos_raw, int16_t *pos_filter, float fc, float Ts){

//	static int32_t a = CLAMP_INT32_TO_INT16((int32_t)((PI_MULTIPLY_2 * (float)fc * (1.0f/(float)SENSOR_FUSION_FREQUENCY_IMU))/(PI_MULTIPLY_2 * (float)fc * (1.0f/(float)SENSOR_FUSION_FREQUENCY_IMU) + 1.0f)));
	static int16_t a;
	static bool init = 0;
	if(!init){
		init = 1;
		a = CLAMP_INT32_TO_INT16((int32_t)((PI_MULTIPLY_2 * Ts * fc)/((PI_MULTIPLY_2 * Ts * fc + 1.0f)) * (float)Q15)); // ~0.3859 * Q15 // fc = 10 Hz Ts = 1/100
	}
//	a = CLAMP_INT32_TO_INT16((int32_t)((PI_MULTIPLY_2 * Ts * fc)/((PI_MULTIPLY_2 * Ts * fc + 1.0f)) * (float)Q15));
	static int32_t pos_old[2] = {0};

	pos_old[0] = ((a * (int32_t)pos_raw[0]) >> 15) + ((((int32_t)Q15 - a) * pos_old[0]) >> 15);
	pos_filter[0] = pos_old[0] = CLAMP_INT32_TO_INT16(pos_old[0]);

	pos_old[1] = ((a * (int32_t)pos_raw[1]) >> 15) + ((((int32_t)Q15 - a) * pos_old[1]) >> 15);
	pos_filter[1] = pos_old[1] = CLAMP_INT32_TO_INT16(pos_old[1]);


}

static void remove_constant_offset(const int16_t *u, int16_t *y){
	const float a = 0.0124f; // Ts = (1/50) fc = 0.1f;
	static float y_prev[3];

	for(uint8_t i = 0; i<3; i++){
		y_prev[i] = a * (float)u[i] + ((1.0f - a)*y_prev[i]);
		y_prev[i] = CLAMP(y_prev[i], -5.0f, 5.0f);

		y[i] = (u[i] - y_prev[i]);
	}

}



static void position_filter_SLERP_EMA_quaternion_Q15(const int16_t *q_in, int16_t *q_out){

	const int16_t a_div2 = 16353; // fc = 100 Hz, Ts = 1/100, a_div2 = a/2 ... \alpha = 1 - e^{-2\pi f_c T_s}
	static int16_t q_last[4] = {0};
	int16_t dot, q[4], q_last_con[4], q_err[4], e[3], delta_q[4];
	bool sign_q;
	copy_q(q_in, q);
	if(q_in[0] < 0) sign_q = 1;

	dotporduct_4x4_Q15(q_last,q, &dot);
	if(dot < 0) neg_q_Q15(q);
	q_t_conj_function_in_out_q15(q_last,q_last_con);
	multiplicateQuaternionQ15(q_last_con, q, q_err);

	ln_q15_unit_quaternions_multiplicate_2(q_err,e);
//	quat_div_2_Q15(e);
	multQuatwithConstQ15(e,a_div2);

	exponential_mapping_error_Q15(e,delta_q);
	multiplicateQuaternionQ15(q_last, delta_q, q_out);

	NormalizeQuaternionQ15(q_out, q_out);

	copy_q(q_out, q_last);

	if((q_out[0] < 0) && sign_q){
		// you're correct
	}else{
		neg_q_Q15(q_out);
	}
}

static int16_t get_r33_from_quaternion(const int16_t *q){
	uint32_t xu;
	int32_t x;

	xu = ((int32_t)q[1] * q[1]) + ((int32_t)q[2] * q[2]); //Q30 ~Q31
	x = (int32_t)Q15 - ((xu + (1 << 13)) >> 14); // 2 * Q15
	return CLAMP_INT32_TO_INT16(x);
}
int16_t debug_pos_vel_x, debug_pos_vel_y,debug_pos_vel1_x,debug_pos_vel1_y, debug_pos_wx, debug_pos_wy, debug_pos_r11,debug_pos_r12,debug_pos_r22,debug_pos_r21, debug_pos_rot_velx, debug_pos_rot_vely;
int16_t debug_pos_vel_x2, debug_pos_vel_y2, w1_corr, w2_corr;

float debug_speed_lp = 0.1f;
static void rotate_speed_to_world_frame(const mtf01_payload_t *optic_flow, const int16_t *q_in, const int16_t *w, const int16_t *corr_height, int16_t *v_NED){
	// we are in NED frame
	int32_t r11, r12, r21, r22, speed_B_x, speed_B_y, speed_E, speed_N;
	int16_t q[4],w_filter[2];
	q[0] = q_in[0];	q[1] = q_in[1]; q[2] = q_in[2]; q[3] = q_in[3];
//	q_t_conj_function(q);
	r11 = (int32_t)Q15 - ((((int32_t)q[2] * q[2]) + ((int32_t)q[3] * q[3]) + (1<<13)) >> 14);
	r12 = Q14_SHIFT_ROUND((int32_t)q[1] * q[2] - (int32_t)q[0] * q[3]);
	r21 = Q14_SHIFT_ROUND((int32_t)q[1] * q[2] + (int32_t)q[0] * q[3]);
	r22 = (int32_t)Q15 - ((((int32_t)q[1] * q[1]) + ((int32_t)q[3] * q[3]) + (1<<13)) >> 14);

	r11 = debug_pos_r11 = CLAMP_INT32_TO_INT16(r11);	r12 = debug_pos_r12 = CLAMP_INT32_TO_INT16(r12);
	r21 = debug_pos_r21 = CLAMP_INT32_TO_INT16(r21);	r22 = debug_pos_r22 = CLAMP_INT32_TO_INT16(r22);

	debug_pos_wx = w[0]; debug_pos_wy = w[1];

	int8_t direction = -1;
//	debug_pos_vel1_x = ((int32_t)corr_height[0] * optic_flow->flow_vel_x)/1000;
//	debug_pos_vel1_y = ((int32_t)corr_height[0] * optic_flow->flow_vel_y)/1000;


	position_iir_filter_w12_phase_shift_Q15(w,w_filter,1.5f,(1.0f/50.0f)); // need filter to compensate phase shift
//

	int16_t w1_corr = q15_mul(w_filter[1], (opt_flow_w_corr * direction));
	int16_t w2_corr = q15_mul(w_filter[0], (opt_flow_w_corr * direction));

	speed_B_x = debug_pos_vel_x = (((int32_t)corr_height[0] * optic_flow->flow_vel_x)/1000) - w1_corr; // cm/s
	speed_B_y = debug_pos_vel_y = (((int32_t)corr_height[0] * optic_flow->flow_vel_y)/1000) + (-w2_corr); // cm/s


	speed_B_x = CLAMP_INT32_TO_INT16(speed_B_x);
	speed_B_y = CLAMP_INT32_TO_INT16(speed_B_y);

//	position_iir_filter_speed_x_Q15(&speed_B_x,&speed_B_x,2.0f, (1.0f/50.0f));
//	position_iir_filter_speed_y_Q15(&speed_B_y,&speed_B_y,2.0f, (1.0f/50.0f));
	position_iir_filter_f(&speed_B_x,&speed_B_y,debug_speed_lp,50.0f);


	speed_N  = Q15_SHIFT_ROUND(r11 * speed_B_x + r12 * speed_B_y);
	speed_E  = Q15_SHIFT_ROUND(r21 * speed_B_x + r22 * speed_B_y);

	v_NED[0] = CLAMP_INT32_TO_INT16(speed_N);
	v_NED[1] = CLAMP_INT32_TO_INT16(speed_E);

	remove_constant_offset(v_NED,v_NED);

	debug_pos_vel_x2 = v_NED[0];
	debug_pos_vel_y2 = v_NED[1];

}


static void get_hight_and_speed_from_optical_flow(const mtf01_payload_t *optic_flow, const int16_t *q,const int16_t *w, int16_t *hight_mm, int16_t *speed_NED){
	static int32_t hight_mm_old = 0, time_old = 0;
	int32_t x, r33;

	r33 = (int32_t)get_r33_from_quaternion(q);
	x = (r33 * (int32_t)optic_flow->distance_mm) >> 15;
	int32_t optic_flow_h_correctetion = (OPT_FLOW_SENSOR_DIST * r33) >> 15;
	x = x + optic_flow_h_correctetion;
	hight_mm[0] = CLAMP_INT32_TO_INT16(x);
	position_iir_filter_hight_Q15(hight_mm, hight_mm, 5.0f, (1.0f/50.0f));
	rotate_speed_to_world_frame(optic_flow,q,w,hight_mm,speed_NED);


}

static void position_iir_filter_f(int32_t *v_x, int32_t * v_y, float fc, float frequency){
	static float a;
	static float v_last[2];
//	static bool init = false;
//	if(!init){
//		init = true;
		a = ((PI_MULTIPLY_2 * fc *(1.0f/frequency))/(PI_MULTIPLY_2 * fc *(1.0f/frequency) + 1.0f));
//	}
	v_last[0] = a * (float)v_x[0] + (1.0f - a) * v_last[0];
	v_last[1] = a * (float)v_y[0] + (1.0f - a) * v_last[1];

	v_last[0] = CLAMP(v_last[0], -Q15,Q15);
	v_last[1] = CLAMP(v_last[1], -Q15,Q15);

	v_x[0] = (int16_t)v_last[0];
	v_y[0] = (int16_t)v_last[1];

}

