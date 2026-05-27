/*
 * EKF.h
 *
 *  Created on: Aug 24, 2025
 *      Author: gerrygeyer
 */

#ifndef INC_EKF_H_
#define INC_EKF_H_


#include <control_functions.h>
#include <orientation.h>
#include <parameter.h>
#include <sys_math.h>
#include <settings.h>
#include <task.h>
#include <LIS3MDL.h>

#include <string.h>

#define EKF_FRQ	1000

#define FS_GYRO_RAD_S   (34.906585f)        // 2000 dps full-scale
#define FS_ACC_MPS2     (16.0f * 9.80665f)  // 16 g full-scale


#define N  15
#define MZ 3

void init_EKF(void);
void execute_EKF_Fast_Q15(sensor_fusion *pHandle_sf, int16_t *p_out);

void init_covariance_P(int32_t P[N][N]);
// Q-Fixformate
// - Q15: int16_t,  1.0 ->  32767
// - Q30: int32_t,  1.0 -> 1073741824 (1<<30)

void ekf_update15_acc_qfixed(
    int32_t P[15][15],          // in/out, Q30
    int16_t q[4],               // in/out, Q15  (Quaternion [w x y z])
    int16_t v[3],               // in/out, Q15  (deine v-Skalierung)
    int16_t p[3],               // in/out, Q15  (deine p-Skalierung)
    int16_t bg[3],              // in/out, Q15  (Gyro-Bias-Skalierung)
    int16_t ba[3],              // in/out, Q15  (Acc-Bias-Skalierung)
    const int16_t zpred_a[3],   // in,  Q15  (erwartete g-Richtung in Body)
    const int32_t R[3][3],      // in,  Q30  (Messrausch-Kovarianz)
    const int16_t r[3]          // in,  Q15  (Residual = a_unit - zpred_a)
);
void compute_Qd_q30(const int16_t G[15][12],
                    const int32_t Qc[12][12],
                    int16_t dt_q15,
                    int32_t Qd[15][15]);



#endif /* INC_EKF_H_ */
