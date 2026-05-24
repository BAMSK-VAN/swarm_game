#include <stdio.h>
#include "../include/logger.h"

// simulate_unknow
//未知パラメータのシミュレーション
void simulate_unknow(double t, double x[NUM], const char *filename, int first_call) {
    FILE *fp = fopen(filename, first_call ? "w" : "a");
    if (!fp) return;

    if (first_call) {
        fprintf(fp, "time,ux,uy,ua,ub,uθ\n");
    }
        fprintf(fp, "%f,%f,%f,%f,%f,%f\n", t, x[5], x[6], x[7], x[8], x[9]);
    
    fclose(fp);
}

// simulate_vec
//未知パラメータのシミュレーション
void simulate_vec(double t, const char *filename, int first_call, double all_cvx, double all_cvy, double all_va, double all_vb, double all_vangle) {
    FILE *fp = fopen(filename, first_call ? "w" : "a");
    if (!fp) return;

    if (first_call) {
        fprintf(fp, "time,vx,vy,va,vb,vangle\n");
    }
        fprintf(fp, "%f,%f,%f,%f,%f,%f\n", t, all_cvx, all_cvy, all_va, all_vb, all_vangle);
    
    fclose(fp);
}

// simulate_p
void simulate_p(double t, double p[5], const char *filename, int first_call) {
    FILE *fp = fopen(filename, first_call ? "w" : "a");
    if (!fp) return;

    if (first_call) {
        fprintf(fp, "time,p1,p2,p3,p4,p5\n");
    }
        fprintf(fp, "%f,%f,%f,%f,%f,%f\n", t, p[0], p[1], p[2], p[3], p[4]);
    
    fclose(fp);
}

// estimate_swarm
//未知パラメータのシミュレーション
void estimate_swarm(double t, const char *filename, int first_call, double all_cx, double all_cy, double all_a, double all_b, double all_angle, double z_cx, double z_cy, double z_a, double z_b, double z_angle, double x[NUM]) {
    FILE *fp = fopen(filename, first_call ? "w" : "a");
    if (!fp) return;

    if (first_call) {
        fprintf(fp, "time,swarm_cx, swarm_cy, swarm_a, swarm_b, swarm_angle, estimate_cx, estimate_cy, estimate_a, estimate_b, estimate_angle, y_cx, y_cy, y_a, y_b, y_angle\n");
    }
        fprintf(fp, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n", t, all_cx, all_cy, all_a, all_b, all_angle, x[0], x[1], x[2], x[3], x[4], z_cx, z_cy, z_a, z_b, z_angle);
    
    fclose(fp);
}

// drone_route
//ドローン軌道と誘導角度の表示
void drone_route(double t, const char *filename, int first_call, double angle_escort) {
    FILE *fp = fopen(filename, first_call ? "w" : "a");
    if (!fp) return;

    if (first_call) {
        fprintf(fp, "time,drone1_x, drone1_y, drone2_x, drone2_y, drone3_x, drone3_y, drone3_vx, drone3_vy, angle_escort\n");
    }
        fprintf(fp, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n", t, cameradrones[0].x, cameradrones[0].y, cameradrones[1].x, cameradrones[1].y, drones[0].x, drones[0].y, drones[0].vx, drones[0].vy, angle_escort);
    
    fclose(fp);
}

// check_camera
//カメラちゃんとできてる？チェック
void check_camera(double t, double u[NUM_CAMERADRONES][NUM_BIRDS], double a[NUM_CAMERADRONES][NUM_BIRDS], const char *filename, int first_call) {
    FILE *fp = fopen(filename, first_call ? "w" : "a");
    if (!fp) return;

    if (first_call) {
        fprintf(fp, "time,noise_u,u\n");
    }
        fprintf(fp, "%f,%f,%f\n", t, u[0][0], a[0][0]);
    
    fclose(fp);
}

// check_camera_q
void check_camera_q(double t, double q[NUM_CAMERADRONES][GROUP], const char *filename, int first_call) {
    FILE *fp = fopen(filename, first_call ? "w" : "a");
    if (!fp) return;

    if (first_call) {
        fprintf(fp, "time,noise_q1,noise_q2,noise_q3,noise_q4,noise_q5,noise_q6,noise_q7,noise_q8,noise_q9,noise_q10,q1,q2,q3,q4,q5,q6,q7,q8,q9,q10\n");
    }
        fprintf(fp, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n", t, cameradrones[0].q[0], cameradrones[0].q[1], cameradrones[0].q[2], cameradrones[0].q[3], cameradrones[0].q[4], cameradrones[0].q[5], cameradrones[0].q[6], cameradrones[0].q[7], cameradrones[0].q[8], cameradrones[0].q[9], q[0][0], q[0][1], q[0][2], q[0][3], q[0][4], q[0][5], q[0][6], q[0][7], q[0][8], q[0][9]);
    
    fclose(fp);
}

// check_camera_Q
void check_camera_Q(double t, double Z[GROUP][GROUP], const char *filename, int first_call) {
    FILE *fp = fopen(filename, first_call ? "w" : "a");
    if (!fp) return;

    if (first_call) {
        fprintf(fp, "time,noise_Q1,noise_Q2,noise_Q3,noise_Q4,noise_Q5,noise_Q6,noise_Q7,noise_Q8,noise_Q9,noise_Q10,Q1,Q2,Q3,Q4,Q5,Q6,Q7,Q8,Q9,Q10\n");
    }
        fprintf(fp, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n", t, Q[5][0], Q[5][1], Q[5][2], Q[5][3], Q[5][4], Q[5][5], Q[5][6], Q[5][7], Q[5][8], Q[5][9], Z[5][0], Z[5][1], Z[5][2], Z[5][3], Z[5][4], Z[5][5], Z[5][6], Z[5][7], Z[5][8], Z[5][9]);
    
    fclose(fp);
}