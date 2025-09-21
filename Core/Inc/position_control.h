/*
 * position_control.h
 *
 *  Created on: Sep 4, 2025
 *      Author: gerrygeyer
 */

#ifndef INC_POSITION_CONTROL_H_
#define INC_POSITION_CONTROL_H_

#include <settings.h>
#include <parameter.h>
#include <sys_math.h>

void init_position_control(void);
void position_control(const int16_t *x_pos_acc_ref,const wxyz_16t *system_q, wxyz_16t *system_q_ref,const int16_t *a, const int16_t *w, const mtf01_payload_t *optic_flow, int16_t *tauQ10, int16_t *hight_mm_corrected);
void pc_lqr(const int16_t *x_pos_acc, const int16_t *x_pos_acc_ref, int16_t *q_ref, int16_t *tauQ10);
void set_postion_control_stats_to_zero(void);
#endif /* INC_POSITION_CONTROL_H_ */
