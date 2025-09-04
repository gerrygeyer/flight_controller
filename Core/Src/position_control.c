/*
 * position_control.c
 *
 *  Created on: Sep 4, 2025
 *      Author: gerrygeyer
 */


#include <position_control.h>


float Ki_pos_xy,Ki_pos_z, Ki_acc_xy,Ki_acc_z, dt;

static void get_state_error(const int16_t *x_pos_acc, const int16_t *x_pos_acc_ref, int16_t *x_error);
static void integrate_error(const int16_t *error, const int16_t *deltaT_K_i, int16_t *integral);
static void get_delta_K(float Ki_pos_xy, float Ki_pos_z, float Ki_acc_xy, float Ki_acc_z, float dt, int16_t *delata_K);
static void lqr_q15(const int16_t *x_error,int16_t *u_out);
static void remove_gravity_q5(int16_t *u_q5);

void init_position_control(void){

	Ki_pos_xy 	= 1.0f;
	Ki_pos_z 	= 1.0f;
	Ki_acc_xy 	= 1.0f;
	Ki_acc_z 	= 1.0f;
	dt			= 1.0f / (float)POSITION_FREQUENCY;

}



void pc_lqr(const int16_t *x_pos_acc, const int16_t *x_pos_acc_ref, int16_t *q_ref, int16_t *tauQ5){

	int16_t x_pos_acc_err[6], delta_K[6], x[6], u_q10[3], v_norm_q10[3];

	get_delta_K(Ki_pos_xy,Ki_pos_z,Ki_acc_xy,Ki_acc_z,dt,delta_K);
	get_state_error(x_pos_acc,x_pos_acc_ref,x_pos_acc_err);
	integrate_error(x_pos_acc_err,delta_K,x);
	lqr_q15(x,u_q10);
	remove_gravity_q5(u_q10);
	v_norm_q10 = norm_of_3D_vector(u_q10);


}

static void get_state_error(const int16_t *x_pos_acc, const int16_t *x_pos_acc_ref, int16_t *x_error){
	x_error[0] = CLAMP_INT32_TO_INT16((int32_t)x_pos_acc[0] - (int32_t)x_pos_acc_ref[0]);
	x_error[1] = CLAMP_INT32_TO_INT16((int32_t)x_pos_acc[1] - (int32_t)x_pos_acc_ref[1]);
	x_error[2] = CLAMP_INT32_TO_INT16((int32_t)x_pos_acc[2] - (int32_t)x_pos_acc_ref[2]);
	x_error[3] = CLAMP_INT32_TO_INT16((int32_t)x_pos_acc[3] - (int32_t)x_pos_acc_ref[3]);
	x_error[4] = CLAMP_INT32_TO_INT16((int32_t)x_pos_acc[4] - (int32_t)x_pos_acc_ref[4]);
	x_error[5] = CLAMP_INT32_TO_INT16((int32_t)x_pos_acc[5] - (int32_t)x_pos_acc_ref[5]);
}

static void get_delta_K(float Ki_pos_xy, float Ki_pos_z, float Ki_acc_xy, float Ki_acc_z, float dt, int16_t *delata_K){

	delata_K[0] = CLAMP_INT32_TO_INT16((int32_t)((float)Q15 * Ki_pos_xy * dt));
	delata_K[1] = CLAMP_INT32_TO_INT16((int32_t)((float)Q15 * Ki_pos_xy * dt));
	delata_K[2] = CLAMP_INT32_TO_INT16((int32_t)((float)Q15 * Ki_pos_z * dt));
	delata_K[3] = CLAMP_INT32_TO_INT16((int32_t)((float)Q15 * Ki_acc_xy * dt));
	delata_K[4] = CLAMP_INT32_TO_INT16((int32_t)((float)Q15 * Ki_acc_xy * dt));
	delata_K[5] = CLAMP_INT32_TO_INT16((int32_t)((float)Q15 * Ki_acc_z * dt));

}

static void integrate_error(const int16_t *error, const int16_t *deltaT_K_i, int16_t *integral){

	static int32_t error_int[6];
	int32_t x[6];

	x[0] = (int32_t)error[0] * (int32_t)deltaT_K_i[0];
	x[1] = (int32_t)error[1] * (int32_t)deltaT_K_i[1];
	x[2] = (int32_t)error[2] * (int32_t)deltaT_K_i[2];
	x[3] = (int32_t)error[3] * (int32_t)deltaT_K_i[3];
	x[4] = (int32_t)error[4] * (int32_t)deltaT_K_i[4];
	x[5] = (int32_t)error[5] * (int32_t)deltaT_K_i[5];

	integral[0] = CLAMP_INT32_TO_INT16(x[0] - error_int[0]);
	integral[1] = CLAMP_INT32_TO_INT16(x[1] - error_int[1]);
	integral[2] = CLAMP_INT32_TO_INT16(x[2] - error_int[2]);
	integral[3] = CLAMP_INT32_TO_INT16(x[3] - error_int[3]);
	integral[4] = CLAMP_INT32_TO_INT16(x[4] - error_int[4]);
	integral[5] = CLAMP_INT32_TO_INT16(x[5] - error_int[5]);

	error_int[0] = x[0];
	error_int[1] = x[1];
	error_int[2] = x[2];
	error_int[3] = x[3];
	error_int[4] = x[4];
	error_int[5] = x[5];

}



static void lqr_q15(const int16_t *x_error,int16_t *u_out){
	int32_t x, sum;

	const int16_t K_q10[3][6] = {
	    {   324,      0,      0,   1661,      0,      0},
	    {     0,    324,      0,      0,   1661,      0},
	    {     0,      0,    324,      0,      0,   1661}
	};

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

static void remove_gravity_q5(int16_t *u_q5){
//	const int32_t g = 10045; //9.81 * 2^10
	u_q5[2] = CLAMP_INT32_TO_INT16((int32_t)u_q5[2] - 10045);
}



