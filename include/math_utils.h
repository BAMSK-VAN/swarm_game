#ifndef MATH_UTILS_H
#define MATH_UTILS_H

#include "config.h"

// 乱数・確率分布
float randFloat(float min, float max);//一様分布
double rand_normal(double mu, double sigma);//正規分布
double rand_normal_truncated(double mu, double sigma, double lower, double upper);//部分正規分布
double rand_normal_pdf(double x, double mu, double sigma);//正規分布の確率分布
double norm_cdf(double x);//正規分布の近似//使ってない
double norm_inv(double p);//使ってない
double rand_trunc_normal(double mu, double sigma, double a, double b);//部分正規分布

// ベクトル・行列演算
void rotateVector(float a, float b, float theta, float *x, float *y);//行列の回転
void eigen2x2(double a, double b, double c, double *eig1, double *eig2 , double eigv1[2][1] , double eigv2[2][1]);//2×2の固有値計算
void multiply_matrix_5(double A[5][5], double B[5][5], double C[5][5]);//行列の計算
void multiply_matrix_10(double A[10][10], double B[10][10], double C[10][10]);
void multiply_matrix_num(double A[NUM][NUM], double B[NUM][NUM], double C[NUM][NUM]);
void multiply_matrix(double A[6][6], double B[6][6], double C[6][6]);
void transpose_matrix_5(double A[5][5], double At[5][5]);
void transpose_matrix_num(double A[NUM][NUM], double At[NUM][NUM]);
void invert_matrix_5(double A[5][5], double inverse[5][5]);//逆行列
void invert_matrix_num(double A[NUM][NUM], double inverse[NUM][NUM]);
void cholesky(double A[NUM][NUM], double L[NUM][NUM]);//コレスキー分解

// 積分演算
double integrate(double (*func)(double), double a, double b, int n);
void integrate_mat(double (*x[6])(double), double a, double b, int n, double f_int[6]);

#endif // MATH_UTILS_H