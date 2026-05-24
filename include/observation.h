#ifndef OBSERVATION_H
#define OBSERVATION_H

#include "config.h"
#include "types.h"
#include "globals.h"

//カメラの観測
int in_fov(CameraDrone cd, Bird b);
void cameradrones_density(double u[NUM_CAMERADRONES][NUM_BIRDS]);
void cameradrones_density_check(double a[NUM_CAMERADRONES][NUM_BIRDS], double q[NUM_CAMERADRONES][GROUP], double Z[GROUP][GROUP]);
int all_cameradrones_found();
int all_camera_found(Bird b);
int camera_found();

//ドローンの観測動作
void update1_cameradrones();
void update2_cameradrones(double u[NUM_CAMERADRONES][NUM_BIRDS]);
void update3_cameradrones(double z_cx, double z_cy);

//主成分分析
void pca(double cov[2][2]);
void pca_v(double cov_v[2][2]);
void z_pca(double *z_cx, double *z_cy, double z_cov[2][2]);
void pca_t(double t, double *cx_t, double *cy_t, double cov_t[2][2]);
void all_pca(double *all_cx, double *all_cy, double all_cov[2][2]);
//void pca_test(double u[NUM_CAMERADRONES][NUM_BIRDS], double *test_cx, double *test_cy, double test_cov[2][2])
void compute_rotation_and_axes(double cov[2][2]);
void compute_rotation_and_axes_v(double cov_v[2][2]);
void z_compute_rotation_and_axes(double z_cov[2][2], double *z_angle_rad, double *z_a, double *z_b);
void compute_rotation_and_axes_t(double cov_t[2][2], double *angle_rad_t, double *a_t, double *b_t);
void all_compute_rotation_and_axes(double all_cov[2][2], double *all_angle_rad, double *all_a, double *all_b);

#endif // OBSERVATION_H