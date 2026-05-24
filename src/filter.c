#include <math.h>
#include <stdio.h>
#include "../include/filter.h"
#include "../include/math_utils.h"

// kalman_estimate_EKF
//拡張カルマンフィルター
// カルマンフィルタ予測ステップ（状態遷移行列Aを用いた更新）
// カルマンフィルタ予測ステップ（状態予測のみ。共分散行列の更新は省略）x(k,k-1)の計算
//   KalmanState *k  : 状態ベクトルと共分散行列を含むカルマン状態
//   double dt       : 時間ステップ（Δt）
void kalman_estimate_EKF(double x[NUM], double z[5], double P[NUM][NUM], double M[NUM][NUM], double W[NUM][NUM], double V[5][5], double dt) {
    // 状態遷移行列 A（群れの楕円情報：5次元：位置(x,y) + 長軸a + 短軸b + 角度θ）
    // x(k,k-1) = x(k-1,k-1) + Δt * (x(k-1,k-1))' によって次時刻の状態を予測x'=A^*x + B^*u + G^*w
    double A[NUM][NUM] = {
        {1.0, 0, 0, 0, 0, dt, 0, 0, 0, 0},   // x
        {0, 1.0, 0, 0, 0, 0, dt, 0, 0, 0},   // y
        {0, 0, 1.0, 0, 0, 0, 0, dt, 0, 0},   // a
        {0, 0, 0, 1.0, 0, 0, 0, 0, dt, 0},   // b
        {0, 0, 0, 0, 1.0, 0, 0, 0, 0, dt},    // θ
        {0, 0, 0, 0, 0, 1.0, 0, 0, 0, 0},  //unknow_cx-u
        {0, 0, 0, 0, 0, 0, 1.0, 0, 0, 0},  //unknow_cy-u
        {0, 0, 0, 0, 0, 0, 0, 1.0, 0, 0},  //unknow_a-u
        {0, 0, 0, 0, 0, 0, 0, 0, 1.0, 0},  //unknow_b-u
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 1.0}   //unknow_angle-u
    };

    /*double A[NUM][NUM] = {
        {1 + unknow.x[0] * dt, 0, 0, 0, 0, dt * kalman.k_cx, 0, 0, 0, 0,    dt, 0, 0, 0, 0, 0, 0, 0, 0, 0},   // x
        {0, 1 + unknow.x[1] * dt, 0, 0, 0, 0, dt * kalman.k_cy, 0, 0, 0,    0, dt, 0, 0, 0, 0, 0, 0, 0, 0},   // y
        {0, 0, 1 + unknow.x[2] * dt, 0, 0, 0, 0, dt * kalman.k_a, 0, 0,     0, 0, dt, 0, 0, 0, 0, 0, 0, 0},   // a
        {0, 0, 0, 1 + unknow.x[3] * dt, 0, 0, 0, 0, dt * kalman.k_b, 0,     0, 0, 0, dt, 0, 0, 0, 0, 0, 0},   // b
        {0, 0, 0, 0, 1 + unknow.x[4] * dt, 0, 0, 0, 0, dt * kalman.k_angle, 0, 0, 0, 0, dt, 0, 0, 0, 0, 0},    // θ
        {0, 0, 0, 0, 0,                    1, 0, 0, 0, 0,                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0},  //unknow
        {0, 0, 0, 0, 0,                    0, 1, 0, 0, 0,                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0},  //unknow
        {0, 0, 0, 0, 0,                    0, 0, 1, 0, 0,                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0},  //unknow
        {0, 0, 0, 0, 0,                    0, 0, 0, 1, 0,                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0},  //unknow
        {0, 0, 0, 0, 0,                    0, 0, 0, 0, 1,                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0},  //unknow
        {0, 0, 0, 0, 0,                    0, 0, 0, 0, 0,                    1, 0, 0, 0, 0, 0, 0, 0, 0, 0},  //unknow_cx-u
        {0, 0, 0, 0, 0,                    0, 0, 0, 0, 0,                    0, 1, 0, 0, 0, 0, 0, 0, 0, 0},  //unknow_cy-u
        {0, 0, 0, 0, 0,                    0, 0, 0, 0, 0,                    0, 0, 1, 0, 0, 0, 0, 0, 0, 0},  //unknow_a-u
        {0, 0, 0, 0, 0,                    0, 0, 0, 0, 0,                    0, 0, 0, 1, 0, 0, 0, 0, 0, 0},  //unknow_b-u
        {0, 0, 0, 0, 0,                    0, 0, 0, 0, 0,                    0, 0, 0, 0, 1, 0, 0, 0, 0, 0},  //unknow_angle-u
        {0, 0, 0, 0, 0,                    0, 0, 0, 0, 0,                    0, 0, 0, 0, 0, 1, 0, 0, 0, 0},
        {0, 0, 0, 0, 0,                    0, 0, 0, 0, 0,                    0, 0, 0, 0, 0, 0, 1, 0, 0, 0},
        {0, 0, 0, 0, 0,                    0, 0, 0, 0, 0,                    0, 0, 0, 0, 0, 0, 0, 1, 0, 0},
        {0, 0, 0, 0, 0,                    0, 0, 0, 0, 0,                    0, 0, 0, 0, 0, 0, 0, 0, 1, 0},
        {0, 0, 0, 0, 0,                    0, 0, 0, 0, 0,                    0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    };*/

    double B[NUM][5] = {//-----------------------↑ 行列に未知パラメータとして組み込んでるからB行列はいらない
        {dt, 0, 0, 0, 0},   // ux
        {0, dt, 0, 0, 0},   // uy
        {0, 0, dt, 0, 0},   // ua
        {0, 0, 0, dt, 0},   // ub
        {0, 0, 0, 0, dt},   // uθ
        {0, 0, 0, 0, 0},   // unknow
        {0, 0, 0, 0, 0},   // unknow
        {0, 0, 0, 0, 0},   // unknow
        {0, 0, 0, 0, 0},   // unknow
        {0, 0, 0, 0, 0},   // unknow
    };

    double G[NUM][NUM] = { //システムノイズの重み
        {1.0, 0, 0, 0, 0, 0, 0, 0, 0, 0},   // x
        {0, 1.0, 0, 0, 0, 0, 0, 0, 0, 0},   // y     
        {0, 0, 1.0, 0, 0, 0, 0, 0, 0, 0},   // a
        {0, 0, 0, 1.0, 0, 0, 0, 0, 0, 0},   // b
        {0, 0, 0, 0, 1.0, 0, 0, 0, 0, 0},   // θ
        {0, 0, 0, 0, 0, 1.0, 0, 0, 0, 0},   // unknow
        {0, 0, 0, 0, 0, 0, 1.0, 0, 0, 0},   // unknow
        {0, 0, 0, 0, 0, 0, 0, 1.0, 0, 0},   // unknow
        {0, 0, 0, 0, 0, 0, 0, 0, 1.0, 0},   // unknow
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 1.0},   // unknow
    };

    /*double G[NUM][NUM] = { //システムノイズの重み
        {sqrt(dt), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},   // x
        {0, sqrt(dt), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},   // y 
        {0, 0, sqrt(dt), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},   // a
        {0, 0, 0, sqrt(dt), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},   // b
        {0, 0, 0, 0, sqrt(dt), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},   // θ
        {0, 0, 0, 0, 0, sqrt(dt), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},   // unknow
        {0, 0, 0, 0, 0, 0, sqrt(dt), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},   // unknow
        {0, 0, 0, 0, 0, 0, 0, sqrt(dt), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},   // unknow
        {0, 0, 0, 0, 0, 0, 0, 0, sqrt(dt), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},   // unknow
        {0, 0, 0, 0, 0, 0, 0, 0, 0, sqrt(dt), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},   // unknow
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, sqrt(dt), 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, sqrt(dt), 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, sqrt(dt), 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, sqrt(dt), 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, sqrt(dt), 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, sqrt(dt), 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, sqrt(dt), 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, sqrt(dt), 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, sqrt(dt), 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, sqrt(dt)},
    };*/
    
    double H[5][NUM] = {
        {1.0, 0, 0, 0, 0, 0, 0, 0, 0, 0},   // x
        {0, 1.0, 0, 0, 0, 0, 0, 0, 0, 0},   // y
        {0, 0, 1.0, 0, 0, 0, 0, 0, 0, 0},   // a
        {0, 0, 0, 1.0, 0, 0, 0, 0, 0, 0},   // b
        {0, 0, 0, 0, 1.0, 0, 0, 0, 0, 0}    // θ
    };

    /*double H[5][NUM] = {
        {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0},   // x
        {0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0},   // y
        {0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0},   // a
        {0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0},   // b
        {0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1}    // θ
    };*/
    
    double a[5][5] = {
            {1.0, 0, 0, 0, 0},   // x
            {0, 1.0, 0, 0, 0},   // y
            {0, 0, 1.0, 0, 0},   // a
            {0, 0, 0, 1.0, 0},   // b
            {0, 0, 0, 0, 1.0},    // θ
        };

        double b[5][5] = {
            {dt, 0, 0, 0, 0},   // x
            {0, dt, 0, 0, 0},   // y
            {0, 0, dt, 0, 0},   // a
            {0, 0, 0, dt, 0},   // b
            {0, 0, 0, 0, dt},    // θ
        };

    // 新しい状態ベクトル！
    double x_new[NUM] = {0};

    //前の状態k-1を用いたkの状態推定
    for (int i = 0; i < 5; i++) {
        double sum = 0.0;
        for (int j = 0; j < 5; j++)
            sum += a[i][j] * x[j] + b[i][j] * x[j+5];
        x_new[i] = sum;
    }

    for (int i = 5; i < NUM; i++){
        x_new[i] = x[i];
    }

    //Mの共分散行列と逆行列 M = APA + GWG
    double AP [NUM][NUM];
    double A_trans [NUM][NUM];
    double APA [NUM][NUM];
    multiply_matrix_num(A, P, AP);
    transpose_matrix_num(A, A_trans);
    multiply_matrix_num(AP, A_trans, APA); //APA

    double GW [NUM][NUM];
    double G_trans [NUM][NUM];
    double GWG [NUM][NUM];
    multiply_matrix_num(G, W, GW);
    transpose_matrix_num(G, G_trans);
    multiply_matrix_num(GW, G_trans, GWG); //GWG


    for (int i = 0; i < NUM; i++) {
        for (int j = 0; j < NUM; j++)
            M[i][j] = APA[i][j] + GWG[i][j];
    }

    double M_inverse[NUM][NUM];
    invert_matrix_num(M, M_inverse);

    double V_inverse[5][5];
    invert_matrix_5(V, V_inverse);

    double HV[NUM][5];
    double HVH[NUM][NUM];
    double H_trans[NUM][5];
    
    //H行列の転置
    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < NUM; j++) {
            H_trans[j][i] = H[i][j];  // 転置：行と列を入れ替える
        }
    }

    //H^T * V^-1 * Hの計算
    // HV の初期化
    for (int i = 0; i < NUM; i++){
        for (int j = 0; j < 5; j++)
            HV[i][j] = 0.0;
    }

    // 積の計算
    for (int i = 0; i < NUM; i++) {
        for (int j = 0; j < 5; j++) {
            for (int t = 0; t < 5; t++) {
                HV[i][j] += H_trans[i][t] * V_inverse[t][j];
            }
        }
    }

    // HVH の初期化
    for (int i = 0; i < NUM; i++){
        for (int j = 0; j < NUM; j++)
            HVH[i][j] = 0.0;
    }

    // 積の計算
    for (int i = 0; i < NUM; i++) {
        for (int j = 0; j < NUM; j++) {
            for (int t = 0; t < 5; t++) {
                HVH[i][j] += HV[i][t] * H[t][j]; //S = HVH
            }
        }
    }

    double R[NUM][NUM];

    for (int i = 0; i < NUM; i++) {
        for (int j = 0; j < NUM; j++){
            R[i][j] = M_inverse[i][j] + HVH[i][j];
        }
    }

    invert_matrix_num(R, P);//P = (M^-1 + H^T * V^-1 * H)^-1

    double D[NUM][5];
    double K[NUM][5];

    // D の初期化
    for (int i = 0; i < NUM; i++){
        for (int j = 0; j < 5; j++)
            D[i][j] = 0.0;
    }

    // 積の計算
    for (int i = 0; i < NUM; i++) {
        for (int j = 0; j < 5; j++) {
            for (int t = 0; t < NUM; t++) {
                D[i][j] += P[i][t] * H_trans[t][j];
            }
        }
    }

    // K の初期化
    for (int i = 0; i < NUM; i++){
        for (int j = 0; j < 5; j++)
            K[i][j] = 0.0;
    }

    // 積の計算
    for (int i = 0; i < NUM; i++) {
        for (int j = 0; j < 5; j++) {
            for (int t = 0; t < 5; t++) {
                K[i][j] += D[i][t] * V_inverse[t][j];//カルマン行列K = PHV
            }
        }
    }

    double f[5];
    double sum[5];
    double k_sum[NUM];

// カルマンフィルタ更新ステップ（観測zと予測との差分を補正）
//(z(k)-Hx(k,k-1))
    for (int i = 0; i < 5; i++) {
        f[i] = 0.0;
        sum[i] = 0.0;
        for (int j = 0; j < NUM; j++)
            sum[i] += H[i][j] * x_new[j];
            f[i] = z[i] - sum[i];
    }

//（x(k,k) = x(k,k-1) + K(k)(z(k)-Hx(k,k-1))
    for (int i = 0; i < NUM; i++) {
        k_sum[i] = 0;
        for (int j = 0; j < 5; j++)
            k_sum[i] += K[i][j] * f[j];
            x[i] = x_new[i] + k_sum[i];
    }

    EKF.k_cx = x[0];
    EKF.k_cy = x[1];
    EKF.k_a = x[2];
    EKF.k_b = x[3];
    EKF.k_angle = x[4];

    /*for (int i = 0; i < 5; i++){
        bias.b[i] = x[i+NUM-5];
    }*/

    //printf("%f\n",K[0][0]);
}


/////////////////////////////////////////////////////////////////////////////////////////
//UKF

// sigma_point
//σ点の計算
void sigma_point(double mean[NUM], double sqrt_cov[NUM][NUM], double k, double X[NUM][2*NUM+1]){
    for (int i=0; i<NUM; i++){
        X[i][0] = mean[i];
    }

    for (int i=0; i<NUM; i++){
        for (int j=0; j<NUM; j++){
            X[j][i+1] = mean[j] + sqrt(NUM + k) * sqrt_cov[j][i];
        }

        for (int j=0; j<NUM; j++){
            X[j][i+NUM+1] = mean[j] - sqrt(NUM + k) * sqrt_cov[j][i];
        }
    }
}

// kalman_estimate_UKF
//無香料カルマンフィルタ
//   KalmanState *k  : 状態ベクトルと共分散行列を含むカルマン状態
//   double dt       : 時間ステップ（Δt）
void kalman_estimate_UKF(double x[NUM], double z[5], double P[NUM][NUM], double M[NUM][NUM], double W[NUM][NUM], double V[5][5], double dt, double k) {
    // x(k,k-1) = x(k-1,k-1) + Δt * (x(k-1,k-1))' によって次時刻の状態を予測x'=A^*x + B^*u + G^*w
    
    //σ点を収納する行列　各列ごとにi=0,1,2,...のσ点を収納 X[NUM][2*NUM+1]={x0,x1,x2,...}
    double X[NUM][2*NUM + 1] = {0.0};
    //重み
    double w[2*NUM + 1] = {0.0};
    //各要素に対応する重みの計算
    w[0] = k / (NUM + k);

    for (int i=1; i<2*NUM + 1; i++){
        w[i] = 1 / (2 * (NUM + k));
    }

    double L[NUM][NUM];
    cholesky(P,L);//コレスキー分解

    //σ点の計算
    for (int i=0; i<NUM; i++){
        X[i][0] = x[i];
    }

    for (int i=0; i<NUM; i++){
        for (int j=0; j<NUM; j++){
            X[j][i+1] = x[j] + sqrt(NUM + k) * L[j][i];
        }

        for (int j=0; j<NUM; j++){
            X[j][i+NUM+1] = x[j] - sqrt(NUM + k) * L[j][i];
        }
    }

    // 新しい状態ベクトルx(k+1|k)期待値
    double x_new[NUM] = {0.0};

    //状態更新後のσ点
    double X_new[NUM][2*NUM+1] = {0.0};

    double a[5][5] = {
            {1.0, 0, 0, 0, 0},   // x
            {0, 1.0, 0, 0, 0},   // y
            {0, 0, 1.0, 0, 0},   // a
            {0, 0, 0, 1.0, 0},   // b
            {0, 0, 0, 0, 1.0},    // θ
        };

        double b[5][5] = {
            {dt, 0, 0, 0, 0},   // x
            {0, dt, 0, 0, 0},   // y
            {0, 0, dt, 0, 0},   // a
            {0, 0, 0, dt, 0},   // b
            {0, 0, 0, 0, dt},    // θ
        };

    for (int i=0; i<2*NUM+1; i++){

        for (int o = 0; o < 5; o++) {
            double sum = 0.0;
            for (int l = 0; l < 5; l++)
                sum += a[o][l] * X[l][i] + b[o][l] * X[l+5][i];

            X_new[o][i] = sum;
        }

        for (int o = 5; o < NUM; o++){
            X_new[o][i] = X[o][i];
        }
    }

    //期待値演算x(k+1|k)
    for (int i=0; i<NUM; i++){
        double sum = 0.0;
            for (int j=0; j<2*NUM+1; j++){
                sum += w[j] * X_new[i][j];
            }
        x_new[i] = sum;
    }

    double P_k1_k[NUM][NUM] = {0};
    
    //共分散行列の計算
    for (int i = 0; i < NUM; i++) {
        for (int j = 0; j < NUM; j++) {
            double sum = 0.0;
            // P[i][j]の値を、全シグマ点(k)について合計する
            for (int o = 0; o < 2*NUM+1; o++) {
                sum += w[o] * (X_new[i][o] - x_new[i]) * (X_new[j][o] - x_new[j]);
            }
        
            P_k1_k[i][j] = sum + W[i][j];//プロセスノイズも考慮
        }
    }

    //
    //観測方程式のσ点更新計算
    //

    cholesky(P_k1_k,L);//コレスキー分解

    //σ点の再計算(計算コスト削減するなら省略可ただし精度落ちる)
    for (int i=0; i<NUM; i++){
        X_new[i][0] = x_new[i];
    }

    for (int i=0; i<NUM; i++){
        for (int j=0; j<NUM; j++){
            X_new[j][i+1] = x_new[j] + sqrt(NUM + k) * L[j][i];
        }

        for (int j=0; j<NUM; j++){
            X_new[j][i+NUM+1] = x_new[j] - sqrt(NUM + k) * L[j][i];
        }
    }

    // 観測の期待値y(k+1|k)
    double y[5] = {0.0};

    //観測方程式適用後のσ点
    double Y[5][2*NUM+1] = {0.0};
    
    for (int i=0; i<5; i++){
        for (int j=0; j<2*NUM+1; j++){
            Y[i][j] = X_new[i][j];
        }
    }

    //期待値演算y(k+1|k)
    for (int i=0; i<5; i++){
        double sum = 0.0;
            for (int j=0; j<2*NUM+1; j++){
                sum += w[j] * Y[i][j];
            }
        y[i] = sum;
    }

    double P_y[5][5] = {0.0};
    
    //共分散行列の計算
    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 5; j++) {
            double sum = 0.0;
            // P[i][j]の値を、全シグマ点(k)について合計する
            for (int o = 0; o < 2*NUM+1; o++) {
                sum += w[o] * (Y[i][o] - y[i]) * (Y[j][o] - y[j]);
            }
        
            P_y[i][j] = sum + V[i][j];//観測ノイズも考慮
        }
    }

    //共分散行列Pxyの計算
    double P_xy[NUM][5] = {0.0};

    for (int i = 0; i < NUM; i++) {
        for (int j = 0; j < 5; j++) {
            double sum = 0.0;
            for (int o = 0; o < 2*NUM+1; o++) {
                sum += w[o] * (X_new[i][o] - x_new[i]) * (Y[j][o] - y[j]);
            }
        
            P_xy[i][j] = sum;//観測ノイズも考慮
        }
    }

    double P_y_inverse[5][5] = {0.0};
    invert_matrix_5(P_y,P_y_inverse);

    double K[NUM][5] = {0.0};

    // カルマンゲインK の計算
    for (int i = 0; i < NUM; i++) {
        for (int j = 0; j < 5; j++) {
            double sum = 0.0;
            for (int t = 0; t < 5; t++) {
                sum += P_xy[i][t] * P_y_inverse[t][j];//カルマン行列K
            }
            K[i][j] = sum;
        }
    }

  
    double f[5] = {0.0};
    double k_sum[NUM] = {0.0};

// カルマンフィルタ更新ステップ（観測zと予測との差分を補正）
//(z(k)-Hx(k,k-1))
    for (int i = 0; i < 5; i++) {
        f[i] = z[i] - y[i];
    }

//（x(k,k) = x(k,k-1) + K(k)(z(k)-y(k,k-1))
    for (int i = 0; i < NUM; i++) {
        for (int j = 0; j < 5; j++)
            k_sum[i] += K[i][j] * f[j];
            x[i] = x_new[i] + k_sum[i];
    }

    UKF.k_cx = x[0];
    UKF.k_cy = x[1];
    UKF.k_a = x[2];
    UKF.k_b = x[3];
    UKF.k_angle = x[4];

    //更新後の共分散行列P(k+1|k+1)=P(k+1|k)-K*P_xy^T
    //P_xy行列の転置
    double P_xy_trans[5][NUM] = {0.0};
    for (int i = 0; i < NUM; i++) {
        for (int j = 0; j < 5; j++) {
            P_xy_trans[j][i] = P_xy[i][j];  // 転置：行と列を入れ替える
        }
    }
    //共分散行列計算
    for (int i=0; i<NUM; i++){
        for (int j=0; j<NUM; j++){
            double sum = 0.0;
            for (int t=0; t<5; t++){
                sum += K[i][t] * P_xy_trans[t][j];
            }
            P[i][j] = P_k1_k[i][j] - sum;
        }
    }

    /*for (int i = 0; i < 5; i++){
        bias.b[i] = x[i+NUM-5];
    }*/

    //printf("%f\n",K[0][0]);
}