#ifndef CONTROL_H
#define CONTROL_H

#include "config.h"
#include "types.h"
#include "globals.h"

double calc_cost(double x_KF[NUM], double x_d[6][6], double u[6][6]);
void optimal_control_x(double x_T[6], double u[6][6]);
void optimal_control_f(double f_t[6][6], double u[6][6], double x_KF[NUM]);
void optimal_control_u(double u[6][6], double x_KF[NUM], double *angle_escort);

#endif // CONTROL_H