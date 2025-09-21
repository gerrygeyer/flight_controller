/*
 * EKF.c
 *
 *  Created on: Aug 24, 2025
 *      Author: gerrygeyer
 */

#include <EKF.h>

uint16_t ekf_gyro_ts, ekf_acc_ts,ekf_ts;

static void hard_and_softiron_compensation(const int16_t *mag_t, int16_t *mag_norm);
static inline int16_t q15_add(const int16_t x, const int16_t y);
static inline int16_t q15_sub(const int16_t x, const int16_t y);
static inline void divideQuaternionBy2(int16_t *q);
static inline void multQuaternionWith2(int16_t *q);
static inline void multQuatwithConstQ15(int16_t* q, const int16_t x);
static inline void add2QuaternionQ15(const int16_t *q1, const int16_t *q2, int16_t *q_out);
static void create_A_matrix(const int16_t *omega, const int16_t *acc_b, const int16_t *qk, int16_t *A);
static void create_G_matrix(const int16_t *omega, const int16_t *acc_b, const int16_t *qk, int16_t *G);
static void create_Qc_matrix(float sigma_g, float sigma_a,float sigma_bg, float sigma_ba,int16_t Qc[12][12]);
static void accumulate_Phi_blocks_q15(const int16_t *A, int16_t *Phi, int16_t dt_gyro_eff, int16_t dt_acc_eff, int16_t dt_q15);
static inline int16_t float_to_q15(float x);
static inline int16_t q15_from_float(float x);
static inline int16_t dt_gyro_eff_q15(float dt_s);
static inline int16_t dt_acc_eff_q15(float dt_s, float vel_q15_per_mps);
void compute_Qd_q30(const int16_t G[15][12], const int16_t Qc[12][12], int16_t dt_q15, int32_t Qd[15][15]);
void update_P_q30(const int16_t Phi[15][15], const int32_t P_in[15][15], const int32_t Qd[15][15], int32_t P_out[15][15]);

void init_EKF(void){

	ekf_gyro_ts = (int16_t)((float)Q15 * 34.9f / (float)EKF_FRQ);
	ekf_acc_ts = (int16_t)((float)Q15 * 156.96f / (float)EKF_FRQ); // max 16g; g = 9.81 -> 16 * 9.81 = 156.96
	ekf_ts = (int16_t)((float)Q15 * 1.0f / (float)EKF_FRQ);
}

void execute_EKF_Fast_Q15(sensor_fusion *pHandle_sf, int16_t *p_out){

	static int16_t q_k[4] = {Q15, 0, 0, 0};
	static int16_t vk[3] = {0,0,0};
	static int16_t pk[3] = {0,0,0};
	static int16_t dt_q15,dtw_q15,dta_q15;
	static int16_t Qc[12][12];
	static int32_t P[15][15];

	static int16_t bg_q15[3] = {0,0,0}; // bias gyro estimation
	static int16_t ba_q15[3] = {0,0,0}; // bias accel estimation

	static bool ekf_init = 0;

	int16_t accel[3],accel_norm[3], gyro[3], q_gyro_dot[4], acc_w[3];
	int16_t A[16][16], Phi[15][15], G[15][12];

	int32_t Qd[15][15];

	float sigma_g = 0.03;
	float sigma_a = 0.25;
	float sigma_bg = 0.0008;
	float sigma_ba = 0.02;

	const int16_t Ig_vec[3] = {0,0,Q15};

	if(ekf_init == 0){
		create_Qc_matrix(sigma_g, sigma_a,sigma_bg,sigma_ba, Qc);

		float dt_s = 0.001; //sec -> 1000 Hz

		dt_q15      = q15_from_float(dt_s);
		dtw_q15     = dt_gyro_eff_q15(dt_s);               // dt * 34.9
		dta_q15     = dt_acc_eff_q15(dt_s, Q15);       // dt * 16g / v_scale

		init_covariance_P(P);
	}


	accel[0] = pHandle_sf->acc_t.x - ba_q15[0];  // Accel ±16g -> Q15 = 15g
	accel[1] = pHandle_sf->acc_t.y - ba_q15[1];
	accel[2] = pHandle_sf->acc_t.z - ba_q15[2];

//	accel[0] -= pHandle_sf->acc_drift_est.x;
//	accel[1] -= pHandle_sf->acc_drift_est.y;
//	accel[2] -= pHandle_sf->acc_drift_est.z;

	gyro[0] = pHandle_sf->gyro_t.x - bg_q15[0]; // gyro -> Q15 -> 2000°/S -> 34.9 rad/s
	gyro[1] = pHandle_sf->gyro_t.y - bg_q15[1];
	gyro[2] = pHandle_sf->gyro_t.z - bg_q15[2];

//	gyro[0] -= pHandle_sf->gyro_drift_est.x;
//	gyro[1] -= pHandle_sf->gyro_drift_est.y;
//	gyro[2] -= pHandle_sf->gyro_drift_est.z;


//	q_gyro[0] = 0;
//	q_gyro[1] = gyro[0];
//	q_gyro[2] = gyro[1];
//	q_gyro[3] = gyro[2];
//
//	q_accel[0] = 0;
//	q_accel[1] = accel[0];
//	q_accel[2] = accel[1];
//	q_accel[3] = accel[2];

	// PREDICTION

	multiplicateQuaternionQ15(p_out,q_k,q_gyro_dot);
	divideQuaternionBy2(q_gyro_dot);
	q_k[0] = q15_add(q_k[0],q15_mul(q_gyro_dot[0], ekf_gyro_ts));
	q_k[1] = q15_add(q_k[1],q15_mul(q_gyro_dot[1], ekf_gyro_ts));
	q_k[2] = q15_add(q_k[2],q15_mul(q_gyro_dot[2], ekf_gyro_ts));
	q_k[3] = q15_add(q_k[3],q15_mul(q_gyro_dot[3], ekf_gyro_ts));
	NormalizeQuaternionQ15(q_k,q_k);

	// Rotate acc to world & subtract gravity for velocity dynamics

	rotate_vector_Q15(q_k,accel,acc_w);

	acc_w[2] = (acc_w[2] - (Q15 >> 4)); // acc - g // g = Q15 / 16

	vk[0] = q15_add(vk[0],q15_mul(acc_w[0], ekf_acc_ts));
	vk[1] = q15_add(vk[1], q15_mul(acc_w[1], ekf_acc_ts));
	vk[2] = q15_add(vk[2],q15_mul(acc_w[2], ekf_acc_ts));

	pk[0] = q15_add(pk[0],q15_mul(vk[0], ekf_ts));
	pk[1] = q15_add(pk[1],q15_mul(vk[1], ekf_ts));
	pk[2] = q15_add(pk[2],q15_mul(vk[2], ekf_ts));


	// ---- Linearized error dynamics: x_err = [dth; dv; dp; dbg; dba]
	create_A_matrix(gyro,accel, q_k, &A[0][0]);
	create_G_matrix(gyro,accel, q_k, &G[0][0]);

	accumulate_Phi_blocks_q15(&A[0][0], &Phi[0][0], dtw_q15, dta_q15, dt_q15);

	compute_Qd_q30(G, Qc, dt_q15, Qd);
	update_P_q30(Phi, P, Qd, P);

	// ########## UPDATE #################
	int16_t acc_norm = norm_of_3D_vector(accel);
	if((acc_norm > 16384) && (acc_norm < 24.576)){ // > 0.8 * g && < 1.2 * g
		int16_t zpred_a[3], r_acc[3];
		rotate_vector_Q15(q_k, Ig_vec, zpred_a);  // expected gravity direction in body
		norm_3d_vector(accel, accel_norm);
		r_acc[0] = CLAMP_INT32_TO_INT16(accel_norm[0] - zpred_a[0]);
		r_acc[1] = CLAMP_INT32_TO_INT16(accel_norm[1] - zpred_a[1]);
		r_acc[2] = CLAMP_INT32_TO_INT16(accel_norm[2] - zpred_a[2]);

//		ekf_update15_acc_qfixed(P, q_q15, v_q15, p_q15, bg_q15, ba_q15,
//		                        zpred_a_q15, Racc_q30, r_acc_q15);

	}

	int16_t mag_raw[3],mag_norm[3];

	mag_raw[0] = pHandle_sf->mag_t.x;	mag_raw[1] = pHandle_sf->mag_t.y;
	mag_raw[2] = pHandle_sf->mag_t.z; // raw z-axis point up

	hard_and_softiron_compensation(mag_raw, mag_norm);



}

// start static functions
// int16_t mag_t[3];
static void hard_and_softiron_compensation(const int16_t *mag_t, int16_t *mag_norm){
	int16_t mag[3];

	for(uint8_t i = 0; i<3; i++){
		mag[i] = mag_t[i];
	}

	hardiron_apply_q15(mag);
	softiron_apply_q15(mag, mag);
	norm_3d_vector(mag, mag_norm);
}



static inline int16_t q15_add(const int16_t x, const int16_t y){
	return CLAMP_INT32_TO_INT16((int32_t)x + (int32_t)y);
}
static inline int16_t q15_sub(const int16_t x, const int16_t y){
	return CLAMP_INT32_TO_INT16((int32_t)x - (int32_t)y);
}

static inline void divideQuaternionBy2(int16_t *q) {
    q[0] >>= 1;
    q[1] >>= 1;
    q[2] >>= 1;
    q[3] >>= 1;
}

static inline void multQuaternionWith2(int16_t *q){
	q[0] = CLAMP_INT32_TO_INT16(((int32_t)q[0] << 1));
	q[1] = CLAMP_INT32_TO_INT16(((int32_t)q[1] << 1));
	q[2] = CLAMP_INT32_TO_INT16(((int32_t)q[2] << 1));
	q[3] = CLAMP_INT32_TO_INT16(((int32_t)q[3] << 1));
}

static inline void multQuatwithConstQ15(int16_t* q, const int16_t x){
	q[0] = q15_mul(q[0], x);
	q[1] = q15_mul(q[1], x);
	q[2] = q15_mul(q[2], x);
	q[3] = q15_mul(q[3], x);
}

static inline void add2QuaternionQ15(const int16_t *q1, const int16_t *q2, int16_t *q_out){
	int32_t q_x[4];
	q_x[0] = (int32_t)q1[0] + (int32_t)q2[0];
	q_x[1] = (int32_t)q1[1] + (int32_t)q2[1];
	q_x[2] = (int32_t)q1[2] + (int32_t)q2[2];
	q_x[3] = (int32_t)q1[3] + (int32_t)q2[3];

	q_out[0] = CLAMP_INT32_TO_INT16(q_x[0]);
	q_out[1] = CLAMP_INT32_TO_INT16(q_x[1]);
	q_out[2] = CLAMP_INT32_TO_INT16(q_x[2]);
	q_out[3] = CLAMP_INT32_TO_INT16(q_x[3]);
}



/* 3x3 = quat_R(q): world<-body, q = [w,x,y,z] (Q15) */
static void quat_R_q15(const int16_t *q, int16_t R[3][3]) {
    int16_t w=q[0], x=q[1], y=q[2], z=q[3];

    /* Precompute products (Q15*Q15 -> Q15) */
    int16_t xx = q15_mul(x,x), yy = q15_mul(y,y), zz = q15_mul(z,z);
    int16_t xy = q15_mul(x,y), xz = q15_mul(x,z), yz = q15_mul(y,z);
    int16_t wx = q15_mul(w,x), wy = q15_mul(w,y), wz = q15_mul(w,z);

    /* 1 - 2*(y^2+z^2) etc.  -> careful with constants */

    int16_t yy_zz = q15_add(yy, zz);
    int16_t xx_zz = q15_add(xx, zz);
    int16_t xx_yy = q15_add(xx, yy);

    int16_t two_xy = CLAMP_INT32_TO_INT16(xy << 1);
    int16_t two_xz = CLAMP_INT32_TO_INT16(xz << 1);
    int16_t two_yz = CLAMP_INT32_TO_INT16(yz << 1);
    int16_t two_wx = CLAMP_INT32_TO_INT16(wx << 1);
    int16_t two_wy = CLAMP_INT32_TO_INT16(wy << 1);
    int16_t two_wz = CLAMP_INT32_TO_INT16(wz << 1);

    int16_t two_yy_zz = CLAMP_INT32_TO_INT16(yy_zz << 1);
	int16_t two_xx_zz = CLAMP_INT32_TO_INT16(xx_zz << 1);
	int16_t two_xx_yy = CLAMP_INT32_TO_INT16(xx_yy << 1);
    /* R = [
         1-2(yy+zz)   2(xy - wz)   2(xz + wy)
         2(xy + wz)   1-2(xx+zz)   2(yz - wx)
         2(xz - wy)   2(yz + wx)   1-2(xx+yy) ] */


    int16_t one_minus_2yyzz = q15_sub(Q15, two_yy_zz);
    int16_t one_minus_2xxzz = q15_sub(Q15, two_xx_zz);
    int16_t one_minus_2xxyy = q15_sub(Q15, two_xx_yy);

    R[0][0] = one_minus_2yyzz;
    R[0][1] = q15_sub(two_xy, two_wz);
    R[0][2] = q15_add(two_xz, two_wy);

    R[1][0] = q15_add(two_xy, two_wz);
    R[1][1] = one_minus_2xxzz;
    R[1][2] = q15_sub(two_yz, two_wx);

    R[2][0] = q15_sub(two_xz, two_wy);
    R[2][1] = q15_add(two_yz, two_wx);
    R[2][2] = one_minus_2xxyy;
}

/* B = skew(v): 3x3, v in Q15 */
static void skew_q15(const int16_t *v, int16_t B[3][3]) {
    int16_t vx=v[0], vy=v[1], vz=v[2];
    B[0][0]=0;      B[0][1]= q15_sub(0, vz);  B[0][2]= vy;
    B[1][0]= vz;    B[1][1]= 0;               B[1][2]= q15_sub(0, vx);
    B[2][0]= q15_sub(0, vy);  B[2][1]= vx;    B[2][2]= 0;
}

/* C = A * B, all 3x3, Q15 */
static void mat3_mul_q15(const int16_t A[3][3], const int16_t B[3][3], int16_t C[3][3]) {
    for (int i=0;i<3;i++) {
        for (int j=0;j<3;j++) {
            int32_t acc = 0;
            for (int k=0;k<3;k++) {
                acc += ((int32_t)A[i][k] * (int32_t)B[k][j]) >> 2; // Q28
            }
            acc = Q13_SHIFT_ROUND(acc);
            C[i][j] = CLAMP_INT32_TO_INT16(acc);
        }
    }
}

/* Set 3x3 block in row r, col c (0-based) inside 16x16 flat array A */
static void set_block3(int16_t *A, int r, int c, const int16_t B[3][3]) {
    for (int i=0;i<3;i++) {
        int16_t *row = &A[(r+i)*16];
        row[c+0] = B[i][0];
        row[c+1] = B[i][1];
        row[c+2] = B[i][2];
    }
}
/* Set ±I3 into A at (r,c) */
static void set_I3(int16_t *A, int r, int c, int sign) {
    int16_t val = (sign > 0) ? Q15 : (int16_t)-Q15;
    for (int i=0;i<3;i++) {
        int16_t *row = &A[(r+i)*16];
        row[c+0] = 0; row[c+1] = 0; row[c+2] = 0;
        row[c+i] = val;
    }
}

/* Negate 3x3: C = -B */
static void neg3(const int16_t B[3][3], int16_t C[3][3]) {
    for (int i=0;i<3;i++)
        for (int j=0;j<3;j++)
            C[i][j] = (int16_t)(-B[i][j]);
}

static void zero_A16(int16_t *A) {
    for (int i=0;i<16*16;i++) A[i] = 0;
}

/* omega[3], acc_b[3], qk[4], A[16][16] — all Q15 */
static void create_A_matrix(const int16_t *omega, const int16_t *acc_b, const int16_t *qk, int16_t *A)
{
    /* Clear */
    zero_A16(A);

    /* --- A(1:3,1:3) = -skew(omega) --- */
    int16_t S_om[3][3];
    skew_q15(omega, S_om);
    int16_t neg_S_om[3][3];
    neg3(S_om, neg_S_om);
    set_block3(A, /*row*/0, /*col*/0, neg_S_om);

    /* --- A(1:3,10:12) = -I3 --- */
    set_I3(A, /*row*/0, /*col*/9, /*sign*/-1);

    /* --- Compute Rwb from qk --- */
    int16_t Rwb[3][3];
    quat_R_q15(qk, Rwb);

    /* --- A(4:6,1:3) = - Rwb * skew(acc_b) --- */
    int16_t S_acc[3][3];
    skew_q15(acc_b, S_acc);
    int16_t RS[3][3];
    mat3_mul_q15(Rwb, S_acc, RS);
    int16_t neg_RS[3][3];
    neg3(RS, neg_RS);
    set_block3(A, /*row*/3, /*col*/0, neg_RS);

    /* --- A(4:6,13:15) = - Rwb --- */
    int16_t neg_Rwb[3][3];
    neg3(Rwb, neg_Rwb);
    set_block3(A, /*row*/3, /*col*/12, neg_Rwb);

    /* --- A(7:9,4:6) = +I3 --- */
    set_I3(A, /*row*/6, /*col*/3, /*sign*/+1);

    /* The rest stays zero (bias random walks handled via G/Q, not A) */
}

static void zero_G15x12(int16_t *G) {
    for (int i = 0; i < 15*12; i++) G[i] = 0;
}

static void set_block3_15x12(int16_t *G, int r, int c, const int16_t M[3][3]) {
    for (int i = 0; i < 3; i++) {
        int16_t *row = &G[(r+i)*12];
        row[c+0] = M[i][0];
        row[c+1] = M[i][1];
        row[c+2] = M[i][2];
    }
}

/* ±I3 an Position (r,c) setzen (Q15) */
static void set_I3_15x12(int16_t *G, int r, int c, int sign) {
    int16_t val = (sign > 0) ? (int16_t)32767 : (int16_t)-32767;  // Q15 ±1
    for (int i = 0; i < 3; i++) {
        int16_t *row = &G[(r+i)*12];
        row[c+0] = 0; row[c+1] = 0; row[c+2] = 0;
        row[c+i] = val;
    }
}

/* === PUBLIC API ===
   omega[3], acc_b[3], qk[4] alle Q15; G ist flach (15x12) */
static void create_G_matrix(const int16_t *omega,
                            const int16_t *acc_b,
                            const int16_t *qk,
                            int16_t *G)
{
    (void)omega; (void)acc_b; // aktuell ungenutzt (Platz für spätere Modelle)

    zero_G15x12(G);

    /* Rwb: world <- body aus qk = [w,x,y,z] (Q15, normiert) */
    int16_t Rwb[3][3];
    quat_R_q15(qk, Rwb);

    /* G(1:3, 0:2) = -I3  -> dtheta von -n_g getrieben */
    set_I3_15x12(G, /*row*/0,  /*col*/0,  /*sign*/-1);

    /* G(4:6, 3:5) =  Rwb -> dv von +Rwb*n_a getrieben */
    set_block3_15x12(G, /*row*/3,  /*col*/3, Rwb);

    /* G(10:12, 6:8) = I3 -> dbg von +n_bg getrieben */
    set_I3_15x12(G, /*row*/9,  /*col*/6,  /*sign*/+1);

    /* G(13:15, 9:11) = I3 -> dba von +n_ba getrieben */
    set_I3_15x12(G, /*row*/12, /*col*/9,  /*sign*/+1);
}



/* Hilfsfunktion: float -> Q15 (saturiert) */
static inline int16_t float_to_q15(float x) {
    int32_t t = (int32_t)(x * 32767.0f + (x >= 0 ? 0.5f : -0.5f));
    if (t >  32767) t =  32767;
    if (t < -32767) t = -32767;
    return (int16_t)t;
}

/* Erzeugt Qc[12][12] in Q15 aus float-Noiseparametern
 * */
/**
 * @brief		Create Qc[12][12] from float-Noiseparameter
 *
 * @details 	[Optional: Ausführlichere Beschreibung, ggf. Verhalten, Nebenwirkungen, Einschränkungen]
 *
 * @param  		param [Beschreibung des Eingabeparameters 1]
 *
 * @note     	[Optional: Zusatzhinweis, z. B. nicht thread-safe]
 * @warning		all sigmal values should between 0 and 1
 * @see			andere_funktion() [Optionaler Verweis]
 */
static void create_Qc_matrix(float sigma_g,   // gyro noise [rad/s/√Hz]
                      float sigma_a,   // accel noise [m/s²/√Hz]
                      float sigma_bg,  // gyro bias RW [rad/s/√s]
                      float sigma_ba,  // accel bias RW [m/s²/√s]
                      int16_t Qc[12][12])
{
    // alles auf 0 setzen
    for (int i = 0; i < 12; i++)
        for (int j = 0; j < 12; j++)
            Qc[i][j] = 0;

    // Normierungen auf die Q15-Sensor-Skalen
    float s_g_n  = sigma_g  / FS_GYRO_RAD_S;
    float s_a_n  = sigma_a  / FS_ACC_MPS2;
    float s_bg_n = sigma_bg / FS_GYRO_RAD_S;
    float s_ba_n = sigma_ba / FS_ACC_MPS2;

    // Quadrieren
    float var_g  = s_g_n  * s_g_n;
    float var_a  = s_a_n  * s_a_n;
    float var_bg = s_bg_n * s_bg_n;
    float var_ba = s_ba_n * s_ba_n;

    // In Q15 umwandeln
    int16_t q_g  = CLAMP_INT32_TO_INT16((int32_t)(var_g * (float)Q15));
    int16_t q_a  = CLAMP_INT32_TO_INT16((int32_t)(var_a * (float)Q15));
    int16_t q_bg = CLAMP_INT32_TO_INT16((int32_t)(var_bg * (float)Q15));
    int16_t q_ba = CLAMP_INT32_TO_INT16((int32_t)(var_ba * (float)Q15));

    // Auf die Diagonalblöcke setzen
    for (int i = 0; i < 3; i++) {
        Qc[i][i]       = q_g;   // gyro noise
        Qc[3+i][3+i]   = q_a;   // accel noise
        Qc[6+i][6+i]   = q_bg;  // gyro bias RW
        Qc[9+i][9+i]   = q_ba;  // accel bias RW
    }
}



// dt in Sekunden -> Q15
static inline int16_t q15_from_float(float x) {
    int32_t t = (int32_t)(x * 32767.0f + (x>=0 ? 0.5f : -0.5f));
    if (t >  32767) t =  32767;
    if (t < -32767) t = -32767;
    return (int16_t)t;
}

// effektive Δt-Faktoren (in Q15, < 1 bei typischen Raten)
static inline int16_t dt_gyro_eff_q15(float dt_s) { // dt * FS_gyro
    return q15_from_float(dt_s * FS_GYRO_RAD_S);
}
static inline int16_t dt_acc_eff_q15(float dt_s, float vel_q15_per_mps) { // dt * FS_acc / vel-scale
    // vel_q15_per_mps: deine Geschwindigkeits-Skalierung (Q15-Zählwerte pro 1 m/s)
    // falls v ebenfalls in SI->Q15 1 m/s ↔ 32767 kodiert ist, setze vel_q15_per_mps = 32767.0f
    return q15_from_float(dt_s * FS_ACC_MPS2 / vel_q15_per_mps);
}

// A wurde mit omega_q15, acc_q15 (roh) gefüllt
// Wir bilden Φ = I + A_ω * dt_ω + A_a * dt_a + A_const * dt

static void accumulate_Phi_blocks_q15(const int16_t *A, int16_t *Phi,
                               int16_t dt_gyro_eff, int16_t dt_acc_eff, int16_t dt_q15)
{
    // Phi = I
    for (int r=0;r<15;r++){
        for (int c=0;c<15;c++) Phi[r*15+c] = 0;
        Phi[r*15+r] = Q15; // Q15_ONE
    }

    // --- Gyro-getriebene Teile ---
    // (1) A(1:3,1:3) = -skew(omega_q15)  -> * dt_gyro_eff
    for (int i=0;i<3;i++){
        for (int j=0;j<3;j++){
            int idxA = (0+i)*16 + (0+j);
            int idxP = (0+i)*15 + (0+j);
            int32_t acc = (int32_t)A[idxA] * dt_gyro_eff;   // Q15*Q15 -> Q30
            acc = Q15_SHIFT_ROUND(acc); // -> Q15
            int32_t s = (int32_t)Phi[idxP] + acc;
            s = CLAMP_INT32_TO_INT16(s);
            Phi[idxP] = (int16_t)s;
        }
    }
    // (2) A(1:3,10:12) = -I3 -> * dt_gyro_eff
    for (int i=0;i<3;i++){
        int idxP = (0+i)*15 + (9+i);
        int32_t acc = (int32_t)(-32767) * dt_gyro_eff; // -I * dtω
        acc = Q15_SHIFT_ROUND(acc);
        int32_t s = (int32_t)Phi[idxP] + acc;
        s = CLAMP_INT32_TO_INT16(s);
        Phi[idxP] = (int16_t)s;
    }

    // --- Acc-getriebene Teile ---
    // (3) A(4:6,1:3) = -R*skew(acc_q15) -> * dt_acc_eff
    for (int i=0;i<3;i++){
        for (int j=0;j<3;j++){
            int idxA = (3+i)*16 + (0+j);
            int idxP = (3+i)*15 + (0+j);
            int32_t acc = (int32_t)A[idxA] * dt_acc_eff;    // Q30
            acc = Q15_SHIFT_ROUND(acc);
            int32_t s = (int32_t)Phi[idxP] + acc;
            s = CLAMP_INT32_TO_INT16(s);
            Phi[idxP] = (int16_t)s;
        }
    }
    // (4) A(4:6,13:15) = -R -> * dt_acc_eff
    for (int i=0;i<3;i++){
        for (int j=0;j<3;j++){
            int idxA = (3+i)*16 + (12+j);
            int idxP = (3+i)*15 + (12+j);
            int32_t acc = (int32_t)A[idxA] * dt_acc_eff;
            acc = Q15_SHIFT_ROUND(acc);
            int32_t s = (int32_t)Phi[idxP] + acc;
            s = CLAMP_INT32_TO_INT16(s);
            Phi[idxP] = (int16_t)s;
        }
    }

    // --- „konstanter“ Block ---
    // (5) A(7:9,4:6) = +I3 -> * dt  (hier genügt echtes dt in Sekunden im Q15)
    for (int i=0;i<3;i++){
        int idxP = (6+i)*15 + (3+i);
        int32_t acc = (int32_t)Q15 * dt_q15;  // +I * dt
        acc = Q15_SHIFT_ROUND(acc);
        int32_t s = (int32_t)Phi[idxP] + acc;
        s = CLAMP_INT32_TO_INT16(s);
        Phi[idxP] = (int16_t)s;
    }
}

// G: Q15, Qc: Q15, dt: Q15  -> Qd: Q30
void compute_Qd_q30(const int16_t G[15][12],
                    const int16_t Qc[12][12],
                    int16_t dt_q15,
                    int32_t Qd[15][15])
{
    int32_t T[15][12];
    // T = G*Qc (Q15*Q15 -> Q30)
    for(int r=0;r<15;r++){
        for(int c=0;c<12;c++){
            int64_t acc=0;
            for(int k=0;k<12;k++) acc += (int64_t)G[r][k]*(int64_t)Qc[k][c];
            if (acc >  0x3FFFFFFFLL) acc =  0x3FFFFFFFLL;
            if (acc < -0x3FFFFFFFLL) acc = -0x3FFFFFFFLL;
            T[r][c]=(int32_t)acc;
        }
    }
    // Qd = T*G^T * dt  (Q30*Q15 -> Q45 >>15 -> Q30; dann *dt Q15 -> Q45 >>15 -> Q30)
    for(int r=0;r<15;r++){
        for(int c=0;c<15;c++){
            int64_t acc=0;
            for(int k=0;k<12;k++){
                int64_t t = (int64_t)T[r][k]*(int64_t)G[c][k]; // Q45
                t += (t>=0 ? (1LL<<14):-(1LL<<14)); t >>= 15;   // Q30
                acc += t;
            }
            acc = acc * dt_q15;                                 // Q45
            acc += (acc>=0 ? (1LL<<14):-(1LL<<14)); acc >>= 15; // Q30
            if (acc >  0x3FFFFFFFLL) acc =  0x3FFFFFFFLL;
            if (acc < -0x3FFFFFFFLL) acc = -0x3FFFFFFFLL;
            Qd[r][c]=(int32_t)acc;
        }
    }
}

// Phi: 15x15 Q15, P_in: 15x15 Q15, Qd: 15x15 Q15  (alles Q15)
// P_out: 15x15 Q15
// shift: zusätzlicher Downscale während der beiden Matmul-Schritte (empf.: 1..2)
// Phi: 15x15 Q15, P_in: 15x15 Q30, Qd: 15x15 Q30  -> P_out: Q30
void update_P_q30(const int16_t Phi[15][15],
                  const int32_t P_in[15][15],
                  const int32_t Qd[15][15],
                  int32_t P_out[15][15])
{
//    const int N=15;
    int32_t T[15][15];

    // T = Phi * P_in   (Q15*Q30 -> Q45 >>15 -> Q30)
    for (int r=0;r<N;r++){
        for (int c=0;c<N;c++){
            int64_t acc=0;
            for (int k=0;k<N;k++){
            	acc += (int64_t)Phi[r][k] * (int64_t)P_in[k][c]; // Q45
            }
            acc += (acc>=0 ? (1LL<<14):-(1LL<<14));
            acc >>= 15; // Q30
            if (acc >  0x3FFFFFFFLL) acc =  0x3FFFFFFFLL;
            if (acc < -0x3FFFFFFFLL) acc = -0x3FFFFFFFLL;
            T[r][c]=(int32_t)acc;
        }
    }

    // P_out = T * Phi^T + Qd   (Q30*Q15 -> Q45 >>15 -> Q30)
    for (int r=0;r<N;r++){
        for (int c=0;c<N;c++){
            int64_t acc=0;
            for (int k=0;k<N;k++){
                acc += (int64_t)T[r][k]*(int64_t)Phi[c][k]; // Q45
            }
            acc += (acc>=0 ? (1LL<<14):-(1LL<<14));
            acc >>= 15; // Q30
            acc += Qd[r][c];
            if (acc >  0x3FFFFFFFLL) acc =  0x3FFFFFFFLL;
            if (acc < -0x3FFFFFFFLL) acc = -0x3FFFFFFFLL;
            P_out[r][c]=(int32_t)acc;
        }
    }

    // optional: symmetrisieren
    for (int i=0;i<N;i++){
        for (int j=i+1;j<N;j++){
            int32_t s = (P_out[i][j] + P_out[j][i]) >> 1;
            P_out[i][j]=s; P_out[j][i]=s;
        }
    }
}


/* ---------- Q15/Q30 Helpers ---------- */
//static inline int16_t sat_q15(int32_t x){
//    if (x >  32767) return  32767;
//    if (x < -32767) return -32767;
//    return (int16_t)x;
//}
static inline int32_t sat_q30(int64_t x){
    if (x >  0x3FFFFFFFLL) return  0x3FFFFFFF;
    if (x < -0x3FFFFFFFLL) return -0x3FFFFFFF;
    return (int32_t)x;
}
static inline int32_t q15_to_q30(int16_t a){ return ((int32_t)a) << 15; }        // a * 2^15
static inline int16_t q30_to_q15(int32_t a){                                     // round
    int64_t t = (int64_t)a;
    t += (t >= 0 ? (1LL<<14) : -(1LL<<14));
    t >>= 15;
    return CLAMP_INT32_TO_INT16((int32_t)t);
}

/* Q15*Q15 -> Q30 (ohne Shift, Summe in 64-bit) */
static inline int64_t mul_q15q15_to_q30_acc(int16_t a, int16_t b){
    return (int64_t)a * (int64_t)b;  // Q30
}
/* Q30*Q15 -> Q45 (Summe in 64-bit) */
static inline int64_t mul_q30q15_to_q45_acc(int32_t a, int16_t b){
    return (int64_t)a * (int64_t)b;  // Q45
}
/* Q30*Q30 -> Q60 (Summe in 64-bit) */
static inline int64_t mul_q30q30_to_q60_acc(int32_t a, int32_t b){
    return (int64_t)a * (int64_t)b;  // Q60
}

/* ---------- Mat/Vec Utilities ---------- */
static inline void skew3_q15(const int16_t v[3], int16_t S[3][3]){
    // S = skew(v) in Q15
    S[0][0]=0;         S[0][1]= (int16_t)(-v[2]); S[0][2]= v[1];
    S[1][0]= v[2];     S[1][1]= 0;                S[1][2]= (int16_t)(-v[0]);
    S[2][0]= (int16_t)(-v[1]); S[2][1]= v[0];     S[2][2]= 0;
}

/* Normierung Quaternion Q15 (fixed mit float für die Wurzel – sehr leichtgewichtig) */
static void normalize_quat_q15(int16_t q[4]){
    // Summe der Quadrate in Q30
    int64_t s = 0;
    for(int i=0;i<4;i++){
        int32_t qi_q30 = ((int32_t)q[i])*((int32_t)q[i]); // Q30? -> eigentlich Q0, aber wir normalisieren per float
        s += qi_q30;
    }
    // in float normieren (nur 4 Werte) – sicher & schnell
    float qf[4] = { q[0]/32767.0f, q[1]/32767.0f, q[2]/32767.0f, q[3]/32767.0f };
    float n = sqrtf(qf[0]*qf[0]+qf[1]*qf[1]+qf[2]*qf[2]+qf[3]*qf[3]);
    if (n < 1e-12f) return;
    for (int i=0;i<4;i++){
        float x = qf[i]/n;
        int32_t t = (int32_t)(x * 32767.0f + (x>=0?0.5f:-0.5f));
        q[i] = sat_q15(t);
    }
}

/* Quaternion Multiplikation in Q15: qout = q1 ⊗ q2 (beide Q15), Ergebnis Q15 */
//static void quat_mul_q15(const int16_t q1[4], const int16_t q2[4], int16_t qout[4]){
//    // Benutze Q30 Akkus, dann >>15
//    int64_t w = (int64_t)q1[0]*q2[0] - (int64_t)q1[1]*q2[1] - (int64_t)q1[2]*q2[2] - (int64_t)q1[3]*q2[3];
//    int64_t x = (int64_t)q1[0]*q2[1] + (int64_t)q1[1]*q2[0] + (int64_t)q1[2]*q2[3] - (int64_t)q1[3]*q2[2];
//    int64_t y = (int64_t)q1[0]*q2[2] - (int64_t)q1[1]*q2[3] + (int64_t)q1[2]*q2[0] + (int64_t)q1[3]*q2[1];
//    int64_t z = (int64_t)q1[0]*q2[3] + (int64_t)q1[1]*q2[2] - (int64_t)q1[2]*q2[1] + (int64_t)q1[3]*q2[0];
//    // Q30 -> Q15 mit Rundung
//    w += (w>=0 ? (1LL<<14) : -(1LL<<14)); w >>= 15;
//    x += (x>=0 ? (1LL<<14) : -(1LL<<14)); x >>= 15;
//    y += (y>=0 ? (1LL<<14) : -(1LL<<14)); y >>= 15;
//    z += (z>=0 ? (1LL<<14) : -(1LL<<14)); z >>= 15;
//    qout[0] = sat_q15((int32_t)w);
//    qout[1] = sat_q15((int32_t)x);
//    qout[2] = sat_q15((int32_t)y);
//    qout[3] = sat_q15((int32_t)z);
//}

/* Small-angle Quaternion in Q15: dq = [1, 0.5*dth] */
static void small_angle_quat_q15(const int16_t dth[3], int16_t dq[4]){
    // 0.5 in Q15 ~ 16384
    dq[0] = Q15_ONE;
    dq[1] = sat_q15( ((int32_t)dth[0] * 16384 + (dth[0]>=0? (1<<14):-(1<<14))) >> 15 );
    dq[2] = sat_q15( ((int32_t)dth[1] * 16384 + (dth[1]>=0? (1<<14):-(1<<14))) >> 15 );
    dq[3] = sat_q15( ((int32_t)dth[2] * 16384 + (dth[2]>=0? (1<<14):-(1<<14))) >> 15 );
    normalize_quat_q15(dq);
}

/* ---------- Hauptupdate ---------- */
void ekf_update15_acc_qfixed(
    int32_t P[N][N],          // Q30 (in/out)
    int16_t q[4],             // Q15 (in/out)
    int16_t v[3],             // Q15 (in/out)
    int16_t p[3],             // Q15 (in/out)
    int16_t bg[3],            // Q15 (in/out)
    int16_t ba[3],            // Q15 (in/out)
    const int16_t zpred_a[3], // Q15 (in)
    const int32_t R[3][3],    // Q30 (in)
    const int16_t r[3]        // Q15 (in)
){
    /* 1) H = [ -skew(zpred_a)  0  0  0  0 ] in Q15 */
    int16_t Sz[3][3]; skew3_q15(zpred_a, Sz);
    int16_t H[MZ][N]; memset(H, 0, sizeof(H));
    for (int i=0;i<3;i++)
        for (int j=0;j<3;j++)
            H[i][j] = (int16_t)(-Sz[i][j]);

    /* 2) S = H P H^T + R  -> Q30 */
    // HP = H*P  (3x15 * 15x15) -> 3x15, Q30
    int32_t HP[3][N];
    for (int i=0;i<3;i++){
        for (int k=0;k<N;k++){
            int64_t acc = 0;
            for (int j=0;j<N;j++){
                acc += mul_q15q15_to_q30_acc(H[i][j], q30_to_q15(P[j][k])); // (H Q15) * (P Q30 -> Q15)
            }
            // acc ist Summe Q30; keine weitere Skalierung
            HP[i][k] = sat_q30(acc);
        }
    }
    // S = HP * H^T + R  -> (3x15 * 15x3) + R
    int32_t S[MZ][MZ];
    for (int i=0;i<3;i++){
        for (int j=0;j<3;j++){
            int64_t acc = 0;
            for (int k=0;k<N;k++){
                // HP[i][k] (Q30) * H[j][k] (Q15) -> Q45 -> >>15 => Q30
                int64_t t = mul_q30q15_to_q45_acc(HP[i][k], H[j][k]);
                t += (t>=0 ? (1LL<<14) : -(1LL<<14));
                t >>= 15;
                acc += t;  // Q30
            }
            acc += R[i][j];
            S[i][j] = sat_q30(acc);
        }
    }

    /* 3) K = P H^T S^{-1}
          - PHt = P * H^T  (15x15 * 15x3) -> 15x3, Q30
          - S^{-1} in float (nur 3x3), danach K in Q30
    */
    int32_t PHt[N][3];
    for (int i=0;i<N;i++){
        for (int j=0;j<3;j++){
            int64_t acc = 0;
            for (int k=0;k<N;k++){
                // P[i][k] Q30 * H[j][k] Q15 -> Q45 -> >>15 => Q30
                int64_t t = mul_q30q15_to_q45_acc(P[i][k], H[j][k]);
                t += (t>=0 ? (1LL<<14) : -(1LL<<14));
                t >>= 15;
                acc += t;
            }
            PHt[i][j] = sat_q30(acc);
        }
    }

    // S^{-1}: konvertiere S (Q30) -> float, invertiere, dann zurück als Q30
    float Sf[3][3], Sinvf[3][3];
    for (int i=0;i<3;i++)
        for (int j=0;j<3;j++)
            Sf[i][j] = (float)S[i][j] / (float)Q30;

    // invertiere 3x3 (Cholesky + Triangular-Lösungen in float)
    // L = chol(Sf)
    float Lf[3][3];
    // einfache Cholesky:
    {
        float a11 = Sf[0][0]; if (a11<=0) return;
        float l11 = sqrtf(a11);
        float l21 = Sf[1][0]/l11;
        float l31 = Sf[2][0]/l11;
        float a22 = Sf[1][1]-l21*l21; if (a22<=0) return;
        float l22 = sqrtf(a22);
        float l32 = (Sf[2][1]-l31*l21)/l22;
        float a33 = Sf[2][2]-l31*l31-l32*l32; if (a33<=0) return;
        float l33 = sqrtf(a33);
        Lf[0][0]=l11; Lf[0][1]=0;   Lf[0][2]=0;
        Lf[1][0]=l21; Lf[1][1]=l22; Lf[1][2]=0;
        Lf[2][0]=l31; Lf[2][1]=l32; Lf[2][2]=l33;
    }
    // Sinv via L^{-T} L^{-1}
    for (int e=0;e<3;e++){
        float evec[3]={0,0,0}; evec[e]=1.0f;
        // y: L y = e
        float y0 = evec[0]/Lf[0][0];
        float y1 = (evec[1]-Lf[1][0]*y0)/Lf[1][1];
        float y2 = (evec[2]-Lf[2][0]*y0-Lf[2][1]*y1)/Lf[2][2];
        // x: L^T x = y
        float x2 = y2/Lf[2][2];
        float x1 = (y1-Lf[2][1]*x2)/Lf[1][1];
        float x0 = (y0-Lf[1][0]*x1-Lf[2][0]*x2)/Lf[0][0];
        Sinvf[0][e]=x0; Sinvf[1][e]=x1; Sinvf[2][e]=x2;
    }
    // K = PHt * Sinv   (15x3 = 15x3 * 3x3), Ergebnis Q30
    int32_t K[N][3];
    for (int i=0;i<N;i++){
        for (int j=0;j<3;j++){
            double acc = 0.0; // dicker Akkumulator
            for (int k=0;k<3;k++){
                // PHt Q30 -> float, Sinv float
                acc += ((double)PHt[i][k]/(double)Q30) * (double)Sinvf[k][j];
            }
            // zurück nach Q30
            int64_t t = (int64_t)llround(acc * (double)Q30);
            K[i][j] = sat_q30(t);
        }
    }

    /* 4) dx = K * r   (15x1), K Q30 * r Q15 -> Q45 -> >>15 => Q30, dann nach Q15 */
    int16_t dx_q15[N];
    for (int i=0;i<N;i++){
        int64_t acc = 0;
        for (int k=0;k<3;k++){
            int64_t t = mul_q30q15_to_q45_acc(K[i][k], r[k]); // Q45
            acc += t;
        }
        // Q45 -> Q30
        acc += (acc>=0 ? (1LL<<14):-(1LL<<14));
        acc >>= 15;
        dx_q15[i] = q30_to_q15((int32_t)acc);  // Q30 -> Q15
    }

    /* 5) State Injection (Q15) */
    // dtheta:
    int16_t dq[4], qnew[4];
    small_angle_quat_q15(&dx_q15[0], dq);
    multiplicateQuaternionQ15(dq, q, qnew);
    // renorm
    normalize_quat_q15(qnew);
    for (int i=0;i<4;i++) q[i] = qnew[i];

    // dv, dp, dbg, dba:
    for (int i=0;i<3;i++){
        v[i]  = CLAMP_INT32_TO_INT16((int32_t)v[i]  + dx_q15[3+i]);
        p[i]  = CLAMP_INT32_TO_INT16((int32_t)p[i]  + dx_q15[6+i]);
        bg[i] = CLAMP_INT32_TO_INT16((int32_t)bg[i] + dx_q15[9+i]);
        ba[i] = CLAMP_INT32_TO_INT16((int32_t)ba[i] + dx_q15[12+i]);
    }

    /* 6) Joseph-Form: P = (I-KH)P(I-KH)^T + K R K^T  (alles Q30) */
    // KH = K * H   (15x3 * 3x15) -> 15x15, K Q30 * H Q15 -> Q45 >>15 -> Q30
    int32_t KH[N][N];
    for (int i=0;i<N;i++){
        for (int j=0;j<N;j++){
            int64_t acc = 0;
            for (int k=0;k<3;k++){
                int64_t t = mul_q30q15_to_q45_acc(K[i][k], H[k][j]);
                t += (t>=0 ? (1LL<<14) : -(1LL<<14));
                t >>= 15; // Q30
                acc += t;
            }
            KH[i][j] = sat_q30(acc);
        }
    }
    // I-KH in Q30
    int32_t ImKH[N][N];
    for (int i=0;i<N;i++){
        for (int j=0;j<N;j++){
            int64_t val = (i==j ? Q30 : 0) - (int64_t)KH[i][j];
            ImKH[i][j] = sat_q30(val);
        }
    }
    // T = (I-KH) * P  -> Q30*Q30 => Q60 >>30 => Q30
    int32_t T[N][N];
    for (int i=0;i<N;i++){
        for (int j=0;j<N;j++){
            int64_t acc = 0;
            for (int k=0;k<N;k++){
                int64_t t = mul_q30q30_to_q60_acc(ImKH[i][k], P[k][j]); // Q60
                // >>30 mit Rundung:
                t += (t>=0 ? (1LL<<29):-(1LL<<29));
                t >>= 30;
                acc += t; // Q30
            }
            T[i][j] = sat_q30(acc);
        }
    }
    // Pn = T * (I-KH)^T
    int32_t Pn[N][N];
    for (int i=0;i<N;i++){
        for (int j=0;j<N;j++){
            int64_t acc = 0;
            for (int k=0;k<N;k++){
                int64_t t = mul_q30q30_to_q60_acc(T[i][k], ImKH[j][k]); // (I-KH)^T
                t += (t>=0 ? (1LL<<29):-(1LL<<29));
                t >>= 30; // Q30
                acc += t;
            }
            Pn[i][j] = sat_q30(acc);
        }
    }
    // KRKt = K R K^T
    // U = K R   (15x3)
    int32_t U[N][3];
    for (int i=0;i<N;i++){
        for (int j=0;j<3;j++){
            int64_t acc = 0;
            for (int k=0;k<3;k++){
                int64_t t = mul_q30q30_to_q60_acc(K[i][k], R[k][j]); // Q60
                t += (t>=0 ? (1LL<<29):-(1LL<<29));
                t >>= 30; // Q30
                acc += t;
            }
            U[i][j] = sat_q30(acc);
        }
    }
    int32_t KRKt[N][N];
    for (int i=0;i<N;i++){
        for (int j=0;j<N;j++){
            int64_t acc = 0;
            for (int k=0;k<3;k++){
                int64_t t = mul_q30q30_to_q60_acc(U[i][k], K[j][k]); // Q60
                t += (t>=0 ? (1LL<<29):-(1LL<<29));
                t >>= 30; // Q30
                acc += t;
            }
            KRKt[i][j] = sat_q30(acc);
        }
    }
    // P = Pn + KRKt   (Q30)
    for (int i=0;i<N;i++)
        for (int j=0;j<N;j++){
            int64_t s = (int64_t)Pn[i][j] + (int64_t)KRKt[i][j];
            P[i][j] = sat_q30(s);
        }

    // Optional: symmetrisieren
    for (int i=0;i<N;i++){
        for (int j=i+1;j<N;j++){
            int32_t s = (P[i][j] + P[j][i]) >> 1;
            P[i][j]=s; P[j][i]=s;
        }
    }
}

void init_covariance_P(int32_t P[N][N]){
    // alles auf 0 setzen
    memset(P, 0, sizeof(int32_t)*N*N);

    // Varianzen in float
    float var_theta = powf(20.0f * (float)M_PI/180.0f, 2.0f); // rad^2
    float var_v     = powf(0.50f, 2.0f);                     // (m/s)^2
    float var_p     = powf(1.00f, 2.0f);                     // m^2
    float var_bg    = powf(0.01f, 2.0f);                     // (rad/s)^2
    float var_ba    = powf(0.10f, 2.0f);                     // (m/s^2)^2

    // in Q30
    int32_t q_theta = float_to_q30(var_theta);
    int32_t q_v     = float_to_q30(var_v);
    int32_t q_p     = float_to_q30(var_p);
    int32_t q_bg    = float_to_q30(var_bg);
    int32_t q_ba    = float_to_q30(var_ba);

    // Blöcke setzen
    for(int i=0;i<3;i++) P[i][i]       = q_theta; // delta theta
    for(int i=0;i<3;i++) P[3+i][3+i]   = q_v;     // velocity
    for(int i=0;i<3;i++) P[6+i][6+i]   = q_p;     // position
    for(int i=0;i<3;i++) P[9+i][9+i]   = q_bg;    // gyro bias
    for(int i=0;i<3;i++) P[12+i][12+i] = q_ba;    // accel bias
}

// end static functions
