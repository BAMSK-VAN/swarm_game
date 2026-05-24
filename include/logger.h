#ifndef LOGGER_H
#define LOGGER_H

#include "config.h"
#include "types.h"
#include "globals.h"

//csvファイルでログを記録
void simulate_unknow(double t, double x[NUM], const char *filename, int first_call);
void simulate_vec(double t, const char *filename, int first_call, double all_cvx, double all_cvy, double all_va, double all_vb, double all_vangle);
void simulate_p(double t, double p[5], const char *filename, int first_call);
void estimate_swarm(double t, const char *filename, int first_call, double all_cx, double all_cy, double all_a, double all_b, double all_angle, double z_cx, double z_cy, double z_a, double z_b, double z_angle, double x[NUM]);
void drone_route(double t, const char *filename, int first_call, double angle_escort);
void check_camera(double t, double u[NUM_CAMERADRONES][NUM_BIRDS], double a[NUM_CAMERADRONES][NUM_BIRDS], const char *filename, int first_call);
void check_camera_q(double t, double q[NUM_CAMERADRONES][GROUP], const char *filename, int first_call);
void check_camera_Q(double t, double Z[GROUP][GROUP], const char *filename, int first_call);

#endif // LOGGER_H