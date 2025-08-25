/*
 * EKF.c
 *
 *  Created on: Aug 24, 2025
 *      Author: gerrygeyer
 */

#include <EKF.h>

uint16_t ekf_gyro_ts, ekf_acc_ts,ekf_ts;

static inline int16_t q15_mul(int16_t a, int16_t b);
static inline int16_t q15_add(const int16_t x, const int16_t y);
static inline int16_t q15_sub(const int16_t x, const int16_t y);
static inline void divideQuaternionBy2(int16_t *q);
static inline void multQuaternionWith2(int16_t *q);
static inline void multQuatwithConstQ15(int16_t* q, const int16_t x);
static inline void add2QuaternionQ15(const int16_t *q1, const int16_t *q2, int16_t *q_out);
static void rotate_vector_Q15(const int16_t *q, const int16_t *v, int16_t *v_out);
static void create_A_matrix(const int16_t *omega, const int16_t *acc_b, const int16_t *qk, int16_t *A);
static void create_G_matrix(const int16_t *omega, const int16_t *acc_b, const int16_t *qk, int16_t *G);

void init_EKF(void){

	ekf_gyro_ts = (int16_t)((float)Q15 * 34.9f / (float)EKF_FRQ);
	ekf_acc_ts = (int16_t)((float)Q15 * 156.96f / (float)EKF_FRQ); // max 16g; g = 9.81 -> 16 * 9.81 = 156.96
	ekf_ts = (int16_t)((float)Q15 * 1.0f / (float)EKF_FRQ);
}

void execute_EKF_Fast_Q15(sensor_fusion *pHandle_sf, int16_t *p_out){

	static int16_t q_k[4] = {Q15, 0, 0, 0};
	static int16_t vk[3] = {0,0,0};
	static int16_t pk[3] = {0,0,0};
	static int16_t Qc[12][12];

	static bool ekf_init = 0;

	int16_t accel[3], gyro[3], q_gyro[4], q_gyro_dot[4], q_accel[4], acc_w[3];
	int16_t A[16][16], G[15][12];

	float sigma_g = 0.03;
	float sigma_a = 0.25;
	float sigma_bg = 0.0008;
	float sigma_ba = 0.02;

	if(ekf_init == 0){
		create_Qc_matrix(sigma_g, sigma_a,sigma_bg,sigma_ba, Qc);
	}


	accel[0] = pHandle_sf->acc_t.x;  // Accel ±16g -> Q15 = 15g
	accel[1] = pHandle_sf->acc_t.y;
	accel[2] = pHandle_sf->acc_t.z;

	accel[0] -= pHandle_sf->acc_drift_est.x;
	accel[1] -= pHandle_sf->acc_drift_est.y;
	accel[2] -= pHandle_sf->acc_drift_est.z;

	gyro[0] = pHandle_sf->gyro_t.x; // gyro -> Q15 -> 2000°/S -> 34.9 rad/s
	gyro[1] = pHandle_sf->gyro_t.y;
	gyro[2] = pHandle_sf->gyro_t.z;

	gyro[0] -= pHandle_sf->gyro_drift_est.x;
	gyro[1] -= pHandle_sf->gyro_drift_est.y;
	gyro[2] -= pHandle_sf->gyro_drift_est.z;


	q_gyro[0] = 0;
	q_gyro[1] = gyro[0];
	q_gyro[2] = gyro[1];
	q_gyro[3] = gyro[2];

	q_accel[0] = 0;
	q_accel[1] = accel[0];
	q_accel[2] = accel[1];
	q_accel[3] = accel[2];

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



}

// start static functions

static inline int16_t q15_mul(int16_t a, int16_t b) {
    return (int16_t)(((int32_t)a * b + (1 << 14)) >> 15); // mit Rundung
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

static void rotate_vector_Q15(const int16_t *q, const int16_t *v, int16_t *v_out){
	int16_t q_con[4], q_v[4], q_v_out[4];

	q_v[0] = 0;
	q_v[1] = v[0];
	q_v[2] = v[1];
	q_v[3] = v[2];

	q_t_conj_function_in_out_q15(q,q_con);
	rotate_quat_sandwich_q15(q, q_v, q_con,q_v_out);

	v_out[0] = q_v_out[1];
	v_out[1] = q_v_out[2];
	v_out[2] = q_v_out[3];
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
            acc = (acc + (1 << 12)) >> 13;
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
void create_Qc_matrix(float sigma_g,   // gyro noise [rad/s/√Hz]
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

// end static functions
