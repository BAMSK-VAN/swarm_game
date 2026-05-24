#ifndef FILTER_H
#define FILTER_H

#include "config.h"
#include "types.h"
#include "globals.h"

void sigma_point(double mean[NUM], double sqrt_cov[NUM][NUM], double k, double X[NUM][2*NUM+1]);
void kalman_estimate_EKF(double x[NUM], double z[5], double P[NUM][NUM], double M[NUM][NUM], double W[NUM][NUM], double V[5][5], double dt);
void kalman_estimate_UKF(double x[NUM], double z[5], double P[NUM][NUM], double M[NUM][NUM], double W[NUM][NUM], double V[5][5], double dt, double k);

#endif // FILTER_H