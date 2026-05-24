#include <math.h>
#include <stdlib.h>
#include "../include/math_utils.h"
#include "../include/config.h"

// randFloat
float randFloat(float min, float max) {
    return min + (rand() / (float)RAND_MAX) * (max - min);
}

// rand_normal
double rand_normal(double mu, double sigma) {
    double u1 = (rand() + 1.0) / (RAND_MAX + 2.0); // 0以上1未満（0,1は除く）
    double u2 = (rand() + 1.0) / (RAND_MAX + 2.0);
    double z0 = sqrt(-2.0 * log(u1)) * sin(2.0 * PI * u2); // Box-Muller
    return mu + sigma * z0;
}

// rand_normal_truncated
double rand_normal_truncated(double mu, double sigma, double lower, double upper) {
    double u1 = (rand() + 1.0) / (RAND_MAX + 2.0);
    double u2 = (rand() + 1.0) / (RAND_MAX + 2.0);
    double z0 = sqrt(-2.0 * log(u1)) * cos(2.0 * M_PI * u2);
    double x = mu + sigma * z0;

    // tanhで押し込む
    double range = (upper - lower) / 2.0;
    return mu + range * tanh(x / sigma);
}

// rand_normal_pdf,山の高さを1に扱いやすい
double rand_normal_pdf(double x, double mu, double sigma){
    return exp(-(x-mu)*(x-mu)/(2*sigma*sigma));
}

// norm_cdf
double norm_cdf(double x) {
    return 0.5 * erfc(-x / sqrt(2.0));
}

// norm_inv
double norm_inv(double p) {
    // 係数（定数）
    static const double a1 = -3.969683028665376e+01;
    static const double a2 =  2.209460984245205e+02;
    static const double a3 = -2.759285104469687e+02;
    static const double a4 =  1.383577518672690e+02;
    static const double a5 = -3.066479806614716e+01;
    static const double a6 =  2.506628277459239e+00;

    static const double b1 = -5.447609879822406e+01;
    static const double b2 =  1.615858368580409e+02;
    static const double b3 = -1.556989798598866e+02;
    static const double b4 =  6.680131188771972e+01;
    static const double b5 = -1.328068155288572e+01;

    static const double c1 = -7.784894002430293e-03;
    static const double c2 = -3.223964580411365e-01;
    static const double c3 = -2.400758277161838e+00;
    static const double c4 = -2.549732539343734e+00;
    static const double c5 =  4.374664141464968e+00;
    static const double c6 =  2.938163982698783e+00;

    static const double d1 =  7.784695709041462e-03;
    static const double d2 =  3.224671290700398e-01;
    static const double d3 =  2.445134137142996e+00;
    static const double d4 =  3.754408661907416e+00;

    // 境界値
    const double plow  = 0.02425;
    const double phigh = 1 - plow;

    double q, r;

    if (p < plow) {
        q = sqrt(-2 * log(p));
        return (((((c1*q+c2)*q+c3)*q+c4)*q+c5)*q+c6) /
               ((((d1*q+d2)*q+d3)*q+d4)*q+1);
    } else if (p > phigh) {
        q = sqrt(-2 * log(1-p));
        return -(((((c1*q+c2)*q+c3)*q+c4)*q+c5)*q+c6) /
                ((((d1*q+d2)*q+d3)*q+d4)*q+1);
    } else {
        q = p - 0.5;
        r = q*q;
        return (((((a1*r+a2)*r+a3)*r+a4)*r+a5)*r+a6)*q /
               (((((b1*r+b2)*r+b3)*r+b4)*r+b5)*r+1);
    }
}

// rand_trunc_normal
double rand_trunc_normal(double mu, double sigma, double a, double b) {
    double alpha = (a - mu) / sigma;
    double beta  = (b - mu) / sigma;

    double u = (rand() + 1.0) / (RAND_MAX + 2.0);

    double p_alpha = norm_cdf(alpha);
    double p_beta  = norm_cdf(beta);

    double p = p_alpha + u * (p_beta - p_alpha);

    return mu + sigma * norm_inv(p);
}

// rotateVector
void rotateVector(float a, float b, float theta, float *x, float *y) {
    float rad = theta * PI / 180.0;  // 角度をラジアンに変換
    *x = a * cos(rad) - b * sin(rad);
    *y = a * sin(rad) + b * cos(rad);
}

// eigen2x2
void eigen2x2(double a, double b, double c, double *eig1, double *eig2 , double eigv1[2][1] , double eigv2[2][1]) {
    double tr = a + c;
    double det = a * c - b * b;
    double d = sqrt(tr * tr / 4.0 - det);

    *eig1 = tr / 2.0 + d;//固有値
    *eig2 = tr / 2.0 - d;

    if (*eig2 < 0) *eig2 = 0;

    eigv1[0][0] = b;              //固有ベクトル
    eigv1[1][0] = ((c-a)/2.0 + d);
    eigv2[0][0] = b;
    eigv2[1][0] = ((c-a)/2.0 - d);
}

// multiply_matrix_5
void multiply_matrix_5(double A[5][5], double B[5][5], double C[5][5]) {
    // C の初期化
    for (int i = 0; i < 5; i++)
        for (int j = 0; j < 5; j++)
            C[i][j] = 0.0;

    // 積の計算
    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 5; j++) {
            for (int t = 0; t < 5; t++) {
                C[i][j] += A[i][t] * B[t][j];
            }
        }
    }
}

// multiply_matrix_10
void multiply_matrix_10(double A[10][10], double B[10][10], double C[10][10]) {
    // C の初期化
    for (int i = 0; i < 10; i++)
        for (int j = 0; j < 10; j++)
            C[i][j] = 0.0;

    // 積の計算
    for (int i = 0; i < 10; i++) {
        for (int j = 0; j < 10; j++) {
            for (int t = 0; t < 10; t++) {
                C[i][j] += A[i][t] * B[t][j];
            }
        }
    }
}

// multiply_matrix_num
void multiply_matrix_num(double A[NUM][NUM], double B[NUM][NUM], double C[NUM][NUM]) {
    // C の初期化
    for (int i = 0; i < NUM; i++){
        for (int j = 0; j < NUM; j++)
            C[i][j] = 0.0;
    }

    // 積の計算
    for (int i = 0; i < NUM; i++) {
        for (int j = 0; j < NUM; j++) {
            for (int t = 0; t < NUM; t++) {
                C[i][j] += A[i][t] * B[t][j];
            }
        }
    }
}

// transpose_matrix_5
void transpose_matrix_5(double A[5][5], double At[5][5]) {
    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 5; j++) {
            At[j][i] = A[i][j];  // 転置：行と列を入れ替える
        }
    }
}

// transpose_matrix_num
void transpose_matrix_num(double A[NUM][NUM], double At[NUM][NUM]) {
    for (int i = 0; i < NUM; i++) {
        for (int j = 0; j < NUM; j++) {
            At[j][i] = A[i][j];  // 転置：行と列を入れ替える
        }
    }
}

// invert_matrix_5
// ガウス・ジョルダン法で逆行列を求める
void invert_matrix_5(double A[5][5], double inverse[5][5]) {
    double aug[5][2 * 5];

    // 拡大行列 [A | I] を作成
    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 5; j++) {
            aug[i][j] = A[i][j];
            aug[i][j + 5] = (i == j) ? 1.0 : 0.0;
        }
    }

    // 前進消去[I | A]の作成
    for (int i = 0; i < 5; i++) {
        // ピボットが0なら失敗
        if (fabs(aug[i][i]) < 1e-8) {
            break;
        }

        // 対角成分を1にする
        double diag = aug[i][i];
        for (int j = 0; j < 2 * 5; j++)
            aug[i][j] /= diag;

        // 他の行をゼロにする
        for (int k = 0; k < 5; k++) {
            if (k == i) continue;
            double factor = aug[k][i];
            for (int j = 0; j < 2 * 5; j++)
                aug[k][j] -= factor * aug[i][j];
        }
    }

    // 右半分が逆行列
    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 5; j++) {
            inverse[i][j] = aug[i][j + 5];
        }
    }
}

// invert_matrix_num
// ガウス・ジョルダン法で逆行列を求める
void invert_matrix_num(double A[NUM][NUM], double inverse[NUM][NUM]) {
    double aug[NUM][2 * NUM];

    // 拡大行列 [A | I] を作成
    for (int i = 0; i < NUM; i++) {
        for (int j = 0; j < NUM; j++) {
            aug[i][j] = A[i][j];
            aug[i][j + NUM] = (i == j) ? 1.0 : 0.0;
        }
    }

    // 前進消去[I | A]の作成
    for (int i = 0; i < NUM; i++) {
        // ピボットが0なら失敗
        if (fabs(aug[i][i]) < 1e-8) {
            break;
        }

        // 対角成分を1にする
        double diag = aug[i][i];
        for (int j = 0; j < 2 * NUM; j++)
            aug[i][j] /= diag;

        // 他の行をゼロにする
        for (int k = 0; k < NUM; k++) {
            if (k == i) continue;
            double factor = aug[k][i];
            for (int j = 0; j < 2 * NUM; j++)
                aug[k][j] -= factor * aug[i][j];
        }
    }

    // 右半分が逆行列
    for (int i = 0; i < NUM; i++) {
        for (int j = 0; j < NUM; j++) {
            inverse[i][j] = aug[i][j + NUM];
        }
    }
}

// multiply_matrix
// C = A × B → C は (6×6) 行列計算
void multiply_matrix(double A[6][6], double B[6][6], double C[6][6]) {
    // C の初期化
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++)
            C[i][j] = 0.0;

    // 積の計算
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            for (int t = 0; t < 6; t++) {
                C[i][j] += A[i][t] * B[t][j];
            }
        }
    }
}

// integrate
// 台形則による数値積分
double integrate(double (*func)(double), double a, double b, int n) {
    double h = (b - a) / n;  // 刻み幅
    double sum = 0.5 * (func(a) + func(b));  // 両端の寄与

    for (int i = 1; i < n; i++) {
        sum += func(a + i * h);
    }

    return sum * h;
}

// integrate_mat
// 台形則による数値積分(行列) aからbまで
void integrate_mat(double (*x[6])(double), double a, double b, int n, double f_int[6]) {
    double h = (b - a) / n;  // 刻み幅
    double sum[6], mat[6];

    for (int i = 0; i < 6; i++){
        sum[i] = 0.0;
        sum[i] = 0.5 * (x[i](a) + x[i](b));  // 両端の寄与
    }

    for (int j = 1; j < 6; j++) {
        for (int i = 1; i < n; i++)
        sum[j] += x[j](a + i * h);

        f_int[j] = sum[j] * h;
    }
}

// cholesky
//コレスキー分解
void cholesky(double A[NUM][NUM], double L[NUM][NUM]){
    for (int i = 0; i < NUM; i++) {
        for (int j = 0; j < NUM; j++) {
            L[i][j] = 0.0;
        }
    }

    for (int i = 0; i < NUM; i++) {
        for (int j = 0; j <= i; j++) {
            double sum = 0.0;

            if (i == j) {
                // 対角成分の計算: l_ii = sqrt( a_ii - sum(l_ik^2) )
                for (int k = 0; k < i; k++) {
                    sum += L[i][k] * L[i][k];
                }
                double val = A[i][i] - sum;
                L[i][i] = sqrt(val);
            }
            
            else {
                // 非対角成分の計算: l_ij = (1 / l_jj) * ( a_ij - sum(l_ik * l_jk) )
                for (int k = 0; k < j; k++) {
                    sum += L[i][k] * L[j][k];
                }
                L[i][j] = (A[i][j] - sum) / L[j][j];
            }
        }
    }
}