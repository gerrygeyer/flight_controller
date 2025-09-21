/*
 * solve_cost_function.h
 *
 *  Created on: Sep 7, 2025
 *      Author: gerrygeyer
 */

#ifndef INC_SOLVE_COST_FUNCTION_H_
#define INC_SOLVE_COST_FUNCTION_H_

#pragma once
#include <stdbool.h>
#include <main.h>

#define NSTATE 6
#define NCTRL  3

#define K_COLS  NSTATE
#define K_ROWS  NCTRL

void run_cost_function(void);

void run_cost_fct(float q1i,float q2i,float q3i,float q4i,float q5i,float q6i,
                  float r1i,float r2i,float r3i, float Ts_input);

uint8_t getK_matrix(int16_t Kout[K_ROWS][K_COLS]);

void build_Ad_Bd(double Ts, float Ad[36], float Bd[18]);
void build_Qd_Rd(double Ts, const float Qdiag[6], const float Rdiag[3],
                 float Qd[36], float Rd[9]);

bool lqr_dare_iter(const float Ad[36], const float Bd[18],
                   const float Qd[36], const float Rd[9],
                   float K[18], float Pout[36],
                   float tol, int max_it);
void build_Qd_Rd_scaled(double Ts,
                        const float Qdiag[6], const float Rdiag[3],
                        float Qd[36], float Rd[9],
                        bool scale_by_Ts);
// Wrapper für gesamtes 6D-System
bool lqr_dare_iter2x2(const float Ad[36], const float Bd[18],
                      const float Qd[36], const float Rd[9],
                      float K[18], float Pout[36],
                      float tol, int max_it);

double lqr_residuum_axis(double Ts,
                                double qp, double qv, double rr,
                                double p11, double p12, double p22);
bool lqr_stability_check_2x2(const float Ad[36], const float K[18],
                                    float *rho_max,
                                    float rho_axis[3],
                                    float lam1[3], float lam2[3]);
#endif /* INC_SOLVE_COST_FUNCTION_H_ */
