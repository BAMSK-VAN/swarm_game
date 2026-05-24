#include <math.h>
#include <stdio.h>
#include "../include/control.h"

//コスト計算
double calc_cost(double x_KF[NUM], double x_d[6][6], double u[6][6]) {    
    double x_T[6] = {0}, cov_t[2][2], a[2] = {0}, b[2] = {0}, c[2] = {0}, d[2] = {0}, q[NUM_CAMERADRONES+NUM_DRONES][4] = {0};

    double x[6] = { 
                   cameradrones[0].x, 
                   cameradrones[0].y, 
                   cameradrones[1].x, 
                   cameradrones[1].y,
                   drones[0].x, 
                   drones[0].y, 
                  };

    //x1　長軸　正
    a[0] = x_KF[0] + x_KF[2] * cos(x_KF[4]);
    a[1] = x_KF[1] + x_KF[2] * sin(x_KF[4]);

    //x2　長軸　負
    b[0] = x_KF[0] - x_KF[2] * cos(x_KF[4]);
    b[1] = x_KF[1] - x_KF[2] * sin(x_KF[4]);

    //x3　短軸　正
    c[0] = x_KF[0] - x_KF[3] * sin(x_KF[4]);
    c[1] = x_KF[1] + x_KF[3] * cos(x_KF[4]);

    //x4　短軸　負
    d[0] = x_KF[0] + x_KF[3] * sin(x_KF[4]);
    d[1] = x_KF[1] - x_KF[3] * cos(x_KF[4]);

    for (int i = 0; i < NUM_CAMERADRONES + NUM_DRONES; i++){
        double l[4]= {0};

        l[0] = sqrt((x[2*i] - a[0]) * (x[2*i] - a[0]) + (x[2*i+1] - a[1]) * (x[2*i+1] - a[1]));
        l[1] = sqrt((x[2*i] - b[0]) * (x[2*i] - b[0]) + (x[2*i+1] - b[1]) * (x[2*i+1] - b[1]));
        l[2] = sqrt((x[2*i] - c[0]) * (x[2*i] - c[0]) + (x[2*i+1] - c[1]) * (x[2*i+1] - c[1]));
        l[3] = sqrt((x[2*i] - d[0]) * (x[2*i] - d[0]) + (x[2*i+1] - d[1]) * (x[2*i+1] - d[1]));
  
        if (l[0] <= l[1] && l[0] <= l[2] && l[0] <= l[3]){
            q[i][0] = q_d; q[i][1] = 0; q[i][2] = 0; q[i][3]  = 0;
        }

        else if (l[1] <= l[0] && l[1] <= l[2] && l[1] <= l[3]){
            q[i][0] = 0; q[i][1] = q_d; q[i][2] = 0; q[i][3]  = 0;
        }

        else if (l[2] <= l[0] && l[2] <= l[1] && l[2] <= l[3]){
            q[i][0] = 0; q[i][1] = 0; q[i][2] = q_d; q[i][3]  = 0;
        }

        else if (l[3] <= l[0] && l[3] <= l[1] && l[3] <= l[2]){
            q[i][0] = 0; q[i][1] = 0; q[i][2] = 0; q[i][3]  = q_d;
        }
    }

    /*printf("q[0] = %f\n",q[0]);
    printf("q[1] = %f\n",q[1]);
    printf("q[2] = %f\n",q[2]);
    printf("q[3] = %f\n",q[3]);*/

    //終端コストF=0で考える
    
    /*for (int i = 0; i < 6; i++){
        f_T[i] = 2 * x_T[i];//重み係数F=1
    }*/
    
    int n = 5;
    double T = 0.5;
    double h = T / n;  // 刻み幅

    //鳥の群れの関数
    //jはMPCのサンプリング点、小さい方から現在時刻から終端時刻までのサンプル点を記憶 0,0.1,0.2,0.3,0.4,0.5秒
    double x_s[NUM][6] = {0};//swarm

    for (int i = 0; i < 5; i++){
        for (int j = 0; j < 6; j++) {
            x_s[i][j] = x_KF[i] + x_KF[i+5] * (j * h);
        }
    }
    

    //随伴ベクトルΨ(t)の導出
    double d1x[6] = {0};
    double d1y[6] = {0};
    double d2x[6] = {0};
    double d2y[6] = {0};

    for (int i = 0; i < 6; i++){
        d1x[i] = (x_s[0][i] - x_d[0][i]);
        d1y[i] = (x_s[1][i] - x_d[1][i]);
        d2x[i] = (x_s[0][i] - x_d[2][i]);
        d2y[i] = (x_s[1][i] - x_d[3][i]);
    }

    double p[2][6] = {0};
    double range[6] = {0};
    double e[2][6] = {0};//単位ベクトル
    double g[2][6] = {0};

    for (int i = 0; i < 6; i++){
        p[0][i] = PROTECTED_AREA_x - x_s[0][i];
        p[1][i] = PROTECTED_AREA_y - x_s[1][i];

        range[i] = sqrt(p[0][i] * p[0][i] + p[1][i] * p[1][i]);
        e[0][i] = p[0][i] / range[i];//単位ベクトル
        e[1][i] = p[1][i] / range[i];

        g[0][i] = x_s[0][i] + x_s[2][i] * e[0][i];
        g[1][i] = x_s[1][i] + x_s[2][i] * e[1][i];
    }

    //群れ全体をカメラに収めるための距離を求める
    double d_dist[3][6] = {0};

    for (int i = 0; i < NUM_CAMERADRONES; i++){
    for (int j = 0; j <6; j++){
    double n[2] = {0};
    double m[2] = {0};
    double range_x = x_s[0][j] - x_d[2*i][j];
    double range_y = x_s[1][j] - x_d[2*i+1][j];
    double dist = sqrt(range_x * range_x + range_y * range_y);
    double range[2] = {range_x / dist, range_y / dist};//ドローンから群れへの単位ベクトル
    n[0] = cos(PI / 3) * range[0] - sin(PI / 3) * range[1];
    n[1] = sin(PI / 3) * range[0] + cos(PI / 3) * range[1];
    m[0] = cos(PI / 3) * range[0] + sin(PI / 3) * range[1];
    m[1] = -sin(PI / 3) * range[0] + cos(PI / 3) * range[1];
    double sigma[4] = {0};
    sigma[0] = (x_s[2][j] * x_s[2][j] + x_s[3][j] * x_s[3][j] + (x_s[2][j] * x_s[2][j] - x_s[3][j] * x_s[3][j]) * cos(2 * x_s[4][j])) / 2;
    sigma[3] = (x_s[2][j] * x_s[2][j] + x_s[3][j] * x_s[3][j] - (x_s[2][j] * x_s[2][j] - x_s[3][j] * x_s[3][j]) * cos(2 * x_s[4][j])) / 2;
    sigma[1] = sigma[2] = ((x_s[2][j] * x_s[2][j] - x_s[3][j] * x_s[3][j]) * sin(2 * x_s[4][j])) / 2;

    double R1 = sqrt(n[0] * n[0] * sigma[0] + 2 * n[0] * n[1] * sigma[1] + n[1] * n[1] * sigma[3]);
    double R2 = sqrt(m[0] * m[0] * sigma[0] + 2 * m[0] * m[1] * sigma[1] + m[1] * m[1] * sigma[3]);

    if (R1 >= R2) d_dist[i][j] = R1 / sin(0.5 * cameradrones[i].fov_angle_deg);
    else d_dist[i][j] = R2 / sin(0.5 * cameradrones[i].fov_angle_deg);
    }
    }

    for (int i = 0; i < 6; i++){
        d_dist[2][i] = (d_dist[0][i] + d_dist[1][i]) / 2;
    }

    double l[6] = {0};

    for (int i = 0; i < 6; i++){
        l[i] = sqrt(d_dist[0][i] * d_dist[0][i] + d_dist[1][i] * d_dist[1][i]);//カメラ精度を保証するためのドローン間の距離
    }

    //群れを誘導する方向を決定する
    double swarm_vx_n = x_KF[5] / sqrt(x_KF[5] * x_KF[5] + x_KF[6] * x_KF[6]);//群れの進行方向
    double swarm_vy_n = x_KF[6] / sqrt(x_KF[5] * x_KF[5] + x_KF[6] * x_KF[6]);
    double protect_to_swarm_x[6] = {0};//群れから保護エリアへの単位ベクトル
    double protect_to_swarm_y[6] = {0};
    double v_escort[2][6] = {0};//誘導する方向
    double escort_angle = PI / 4;//45度
    double drone_x[6] = {0};//誘導するための配置ベクトル方向
    double drone_y[6] = {0};
    double drone_vx[3][6] = {0};
    double drone_vy[3][6] = {0};
    double drone_point_angle = PI / 4;//45度

    for (int i = 0; i < 6; i++){
        protect_to_swarm_x[i] = (PROTECTED_AREA_x - x_s[0][i]) / sqrt((PROTECTED_AREA_x - x_s[0][i]) * (PROTECTED_AREA_x - x_s[0][i]) + (PROTECTED_AREA_x - x_s[1][i]) * (PROTECTED_AREA_x - x_s[1][i]));
        protect_to_swarm_y[i] = (PROTECTED_AREA_x - x_s[1][i]) / sqrt((PROTECTED_AREA_x - x_s[0][i]) * (PROTECTED_AREA_x - x_s[0][i]) + (PROTECTED_AREA_x - x_s[1][i]) * (PROTECTED_AREA_x - x_s[1][i]));

        drone_vx[0][i] = (x_d[0][i] + u[0][i]) / sqrt((x_d[0][i] + u[0][i]) * (x_d[0][i] + u[0][i]) + (x_d[1][i] + u[1][i]) * (x_d[1][i] + u[1][i]));
        drone_vy[0][i] = (x_d[1][i] + u[1][i]) / sqrt((x_d[0][i] + u[0][i]) * (x_d[0][i] + u[0][i]) + (x_d[1][i] + u[1][i]) * (x_d[1][i] + u[1][i]));
        drone_vx[1][i] = (x_d[2][i] + u[2][i]) / sqrt((x_d[2][i] + u[2][i]) * (x_d[2][i] + u[2][i]) + (x_d[3][i] + u[3][i]) * (x_d[3][i] + u[3][i]));
        drone_vy[1][i] = (x_d[3][i] + u[3][i]) / sqrt((x_d[2][i] + u[2][i]) * (x_d[2][i] + u[2][i]) + (x_d[3][i] + u[3][i]) * (x_d[3][i] + u[3][i]));
        drone_vx[2][i] = (x_d[4][i] + u[4][i]) / sqrt((x_d[4][i] + u[4][i]) * (x_d[4][i] + u[4][i]) + (x_d[5][i] + u[5][i]) * (x_d[5][i] + u[5][i]));
        drone_vy[2][i] = (x_d[5][i] + u[5][i]) / sqrt((x_d[4][i] + u[4][i]) * (x_d[4][i] + u[4][i]) + (x_d[5][i] + u[5][i]) * (x_d[5][i] + u[5][i]));

        if ((protect_to_swarm_x[i] * swarm_vy_n - protect_to_swarm_y[i] * swarm_vx_n) / (protect_to_swarm_x[i] * protect_to_swarm_x[i] + protect_to_swarm_y[i] * protect_to_swarm_y[i]) >= 0){//群れの進行方向が保護エリアの左右どっちに行くか,sinが正やったら正に回転
            v_escort[0][i] = cos(escort_angle) * swarm_vx_n - sin(escort_angle) * swarm_vy_n;
            v_escort[1][i] = sin(escort_angle) * swarm_vx_n + cos(escort_angle) * swarm_vy_n;

            drone_x[i] = cos(-drone_point_angle) * protect_to_swarm_x[i] - sin(-drone_point_angle) * protect_to_swarm_y[i];
            drone_y[i] = sin(-drone_point_angle) * protect_to_swarm_x[i] + cos(-drone_point_angle) * protect_to_swarm_y[i];
        }

        else {
            v_escort[0][i] = cos(-escort_angle) * swarm_vx_n - sin(-escort_angle) * swarm_vy_n;
            v_escort[1][i] = sin(-escort_angle) * swarm_vx_n + cos(-escort_angle) * swarm_vy_n;

            drone_x[i] = cos(drone_point_angle) * protect_to_swarm_x[i] - sin(drone_point_angle) * protect_to_swarm_y[i];
            drone_y[i] = sin(drone_point_angle) * protect_to_swarm_x[i] + cos(drone_point_angle) * protect_to_swarm_y[i];
        }
    }


    double L_t[6] = {0};//コスト汎関数のL(x,u)の各離散時間での値を保存　列の番号がサンプリング点を示す
    //L(x,u)のdL/dxの計算
    /*for (int i = 0; i < 6; i++){
        L_t[0][i] = -2 * (q_1 * ((d2x[i] * (d1x[i] * d2x[i] + d1y[i] * d2y[i]) * (d1x[i] * d1x[i] + d1y[i] * d1y[i])  -  d1x[i] * (d1x[i] * d2x[i] + d1y[i] * d2y[i]) * (d1x[i] * d2x[i] + d1y[i] * d2y[i])) / ((d1x[i] * d1x[i] + d1y[i] * d1y[i]) * (d1x[i] * d1x[i] + d1y[i] * d1y[i]) *(d2x[i] * d2x[i] + d2y[i] * d2y[i])))
                        + q_2 * (d1x[i] - d1x[i] * sqrt((d2x[i] * d2x[i] + d2y[i] * d2y[i]) / (d1x[i] * d1x[i] + d1y[i] * d1y[i])))
                        + q_3 * (x_d[0][i]-x_d[2][i]) 
                        + q_d_1 * (q[0][0] * (d1x[i] + x_s[2][i] * cos(x_s[4][i])) + q[0][1] * (d1x[i] - x_s[2][i] * cos(x_s[4][i])) + q[0][2] * (d1x[i] - x_s[3][i] * sin(x_s[4][i])) + q[0][3] * (d1x[i] + x_s[3][i] * sin(x_s[4][i])))
                        + s_d * (x_d[0][i] - PROTECTED_AREA_x));
    }*/

    for (int i = 0; i < 6; i++){
        L_t[i] = (q_1 * (((d1x[i] * d1x[i] * d2x[i] * d2x[i]) + 2 * (d1x[i] * d2x[i] * d1y[i] * d2y[i]) + (d1y[i] * d1y[i] * d2y[i] * d2y[i])) / ((d1x[i] * d1x[i] + d1y[i] * d1y[i]) * (d2x[i] * d2x[i] + d2y[i] * d2y[i])))
                + q_2 * ((sqrt(d1x[i] * d1x[i] + d1y[i] * d1y[i]) - sqrt(d2x[i] * d2x[i] + d2y[i] * d2y[i])) * (sqrt(d1x[i] * d1x[i] + d1y[i] * d1y[i]) - sqrt(d2x[i] * d2x[i] + d2y[i] * d2y[i])))
                + q_3 * (sqrt((x_d[0][i]-x_d[2][i]) * (x_d[0][i]-x_d[2][i]) + (x_d[1][i]-x_d[3][i]) * (x_d[1][i]-x_d[3][i])) - l[i]) * (sqrt((x_d[0][i]-x_d[2][i]) * (x_d[0][i]-x_d[2][i]) + (x_d[1][i]-x_d[3][i]) * (x_d[1][i]-x_d[3][i])) - l[i])
                + q_d * ((x_d[0][i] - g[0][i]) * (x_d[0][i] - g[0][i]) + (x_d[1][i] - g[1][i]) * (x_d[1][i] - g[1][i]) + (x_d[2][i] - g[0][i]) * (x_d[2][i] - g[0][i]) + (x_d[3][i] - g[1][i]) * (x_d[3][i] - g[1][i]) + (x_d[4][i] - g[0][i]) * (x_d[4][i] - g[0][i]) + (x_d[5][i] - g[1][i]) * (x_d[5][i] - g[1][i]))
                + s_d * ((x_d[0][i] - PROTECTED_AREA_x) * (x_d[0][i] - PROTECTED_AREA_x) + (x_d[1][i] - PROTECTED_AREA_x) * (x_d[1][i] - PROTECTED_AREA_x) + (x_d[2][i] - PROTECTED_AREA_x) * (x_d[2][i] - PROTECTED_AREA_x) + (x_d[3][i] - PROTECTED_AREA_x) * (x_d[3][i] - PROTECTED_AREA_x) + (x_d[4][i] - PROTECTED_AREA_x) * (x_d[4][i] - PROTECTED_AREA_x) + (x_d[5][i] - PROTECTED_AREA_x) * (x_d[5][i] - PROTECTED_AREA_x)));
    }

    //バリア関数のdBar/dxドローンの状態ベクトルの偏微分の計算
    double Bar[6] = {0};

    //計算式短縮用
    double d1vx[6] = {0};//ドローン1
    double d1vy[6] = {0};
    double d2vx[6] = {0};//ドローン2
    double d2vy[6] = {0};
    double d3vx[6] = {0};//ドローン3
    double d3vy[6] = {0};

    for (int i = 0; i < 6; i++){
        d1vx[i] = (exp(h) - 1) * (x_d[0][i] + u[0][i]) / h;
        d1vy[i] = (exp(h) - 1) * (x_d[1][i] + u[1][i]) / h;
        d2vx[i] = (exp(h) - 1) * (x_d[2][i] + u[2][i]) / h;
        d2vy[i] = (exp(h) - 1) * (x_d[3][i] + u[3][i]) / h;
        d3vx[i] = (exp(h) - 1) * (x_d[4][i] + u[4][i]) / h;
        d3vy[i] = (exp(h) - 1) * (x_d[5][i] + u[5][i]) / h;
    }

    for (int i = 0; i < 6; i++){
        Bar[i] = 3 * barrier_v * log(DRONE_MAX_SPEED) - barrier_v * log(DRONE_MAX_SPEED - sqrt(d1vx[i] * d1vx[i] + d1vy[i] * d1vy[i])) - barrier_v * log(DRONE_MAX_SPEED - sqrt(d2vx[i] * d2vx[i] + d2vy[i] * d2vy[i])) - barrier_v * log(DRONE_MAX_SPEED - sqrt(d3vx[i] * d3vx[i] + d3vy[i] * d3vy[i]));
    }

    double objective = 0.0;//実行コスト
    double bar = 0.0;//バリアコスト

    for(int i = 0; i < 6; i++){
        objective += L_t[i];
        bar += Bar[i];
    }
    return objective + bar;
}

//最適制御(ハミルトニアン)
//x(t)の微分方程式を解いて終端時刻での値を求める
//u[i][j]・・・jはMPCのサンプリング点、小さい方から現在時刻から終端時刻までのサンプル点を記憶
void optimal_control_x(double x_T[6], double u[6][6]){
    double x[6] = { 
                   cameradrones[0].x, 
                   cameradrones[0].y, 
                   cameradrones[1].x, 
                   cameradrones[1].y,
                   drones[0].x, 
                   drones[0].y, 
                  };
    /*
    double A[6][6] = {
        {1.0, 0, 0, 0, 0, 0},   // d1x
        {0, 1.0, 0, 0, 0, 0},   // d1y
        {0, 0, 1.0, 0, 0, 0},   // cd1x
        {0, 0, 0, 1.0, 0, 0},   // cd1y
        {0, 0, 0, 0, 1.0, 0},   // cd2x
        {0, 0, 0, 0, 0, 1.0}    // cd2y
    };

    double B[6][6] = {
        {1.0, 0, 0, 0, 0, 0},   // d1x
        {0, 1.0, 0, 0, 0, 0},   // d1y
        {0, 0, 1.0, 0, 0, 0},   // cd1x
        {0, 0, 0, 1.0, 0, 0},   // cd1y
        {0, 0, 0, 0, 1.0, 0},   // cd2x
        {0, 0, 0, 0, 0, 1.0}    // cd2y
    };

    double F[6][6] = { //終端コストの重み行列
        {1.0, 0, 0, 0, 0, 0},   // d1x
        {0, 1.0, 0, 0, 0, 0},   // d1y
        {0, 0, 1.0, 0, 0, 0},   // cd1x
        {0, 0, 0, 1.0, 0, 0},   // cd1y
        {0, 0, 0, 0, 1.0, 0},   // cd2x
        {0, 0, 0, 0, 0, 1.0}    // cd2y
    };

    //e^At 指数行列
    double e_A[6][6] = {
        {exp(A[0][0] * t), 0, 0, 0, 0, 0},
        {0, exp(A[1][1] * t), 0, 0, 0, 0},
        {0, 0, exp(A[2][2] * t), 0, 0, 0},
        {0, 0, 0, exp(A[3][3] * t), 0, 0},
        {0, 0, 0, 0, exp(A[4][4] * t), 0},
        {0, 0, 0, 0, 0, exp(A[5][5] * t)}  
    };
    */

    //x(τ)=e^Aτ *x(t) + (積分)nΣi=0 (e^A(τ-ti)*B*u(ti)(t(i+1)-ti))

    int n = 5;
    double T = 0.5;
    double h = T / n;  // 刻み幅

    //終端時刻でのドローンの位置予測
    for (int i = 0; i < 6; i++){
    double sum = 0.5 * (exp(T) * u[i][0] + u[i][5]);  // 両端の寄与 t秒の時とt + 0.5のとき

    for (int j = 1; j < n; j++) {
        sum += exp(T - (j * h)) * u[i][j];
    }

    x_T[i] = exp(T)* x[i] + sum * h;//ドローンxの終端時刻での値
    }

    
    /*for (int i = 0; i < 6; i++){
    double sum = 0.5 * (u[i][0] + u[i][5]);  // 両端の寄与 t秒の時とt + 0.5のとき

    for (int j = 1; j < n; j++) {
        sum += u[i][j];
    }

    x_T[i] = x[i] + sum * h;//ドローンxの終端時刻での値
    }*/
}

//最適制御(ハミルトニアン)
//随伴ベクトルΨ(t)の微分方程式を解く
//積分したい関数
void optimal_control_f(double f_t[6][6], double u[6][6], double x_KF[NUM]){ //Ψ(t)を求めるときの積分
    double x_T[6] = {0}, cov_t[2][2], a[2] = {0}, b[2] = {0}, c[2] = {0}, d[2] = {0}, q[NUM_CAMERADRONES+NUM_DRONES][4] = {0};

    double x[6] = { 
                   cameradrones[0].x, 
                   cameradrones[0].y, 
                   cameradrones[1].x, 
                   cameradrones[1].y,
                   drones[0].x, 
                   drones[0].y, 
                  };

    /*//x1　長軸　正
    a[0] = x_KF[0] + x_KF[2] * cos(x_KF[4]);
    a[1] = x_KF[1] + x_KF[2] * sin(x_KF[4]);

    //x2　長軸　負
    b[0] = x_KF[0] - x_KF[2] * cos(x_KF[4]);
    b[1] = x_KF[1] - x_KF[2] * sin(x_KF[4]);

    //x3　短軸　正
    c[0] = x_KF[0] - x_KF[3] * sin(x_KF[4]);
    c[1] = x_KF[1] + x_KF[3] * cos(x_KF[4]);

    //x4　短軸　負
    d[0] = x_KF[0] + x_KF[3] * sin(x_KF[4]);
    d[1] = x_KF[1] - x_KF[3] * cos(x_KF[4]);

    for (int i = 0; i < NUM_CAMERADRONES + NUM_DRONES; i++){
        double l[4]= {0};

        l[0] = sqrt((x[2*i] - a[0]) * (x[2*i] - a[0]) + (x[2*i+1] - a[1]) * (x[2*i+1] - a[1]));
        l[1] = sqrt((x[2*i] - b[0]) * (x[2*i] - b[0]) + (x[2*i+1] - b[1]) * (x[2*i+1] - b[1]));
        l[2] = sqrt((x[2*i] - c[0]) * (x[2*i] - c[0]) + (x[2*i+1] - c[1]) * (x[2*i+1] - c[1]));
        l[3] = sqrt((x[2*i] - d[0]) * (x[2*i] - d[0]) + (x[2*i+1] - d[1]) * (x[2*i+1] - d[1]));
  
        if (l[0] <= l[1] && l[0] <= l[2] && l[0] <= l[3]){
            q[i][0] = q_d; q[i][1] = 0; q[i][2] = 0; q[i][3]  = 0;
        }

        else if (l[1] <= l[0] && l[1] <= l[2] && l[1] <= l[3]){
            q[i][0] = 0; q[i][1] = q_d; q[i][2] = 0; q[i][3]  = 0;
        }

        else if (l[2] <= l[0] && l[2] <= l[1] && l[2] <= l[3]){
            q[i][0] = 0; q[i][1] = 0; q[i][2] = q_d; q[i][3]  = 0;
        }

        else if (l[3] <= l[0] && l[3] <= l[1] && l[3] <= l[2]){
            q[i][0] = 0; q[i][1] = 0; q[i][2] = 0; q[i][3]  = q_d;
        }
    }*/

    /*printf("q[0] = %f\n",q[0]);
    printf("q[1] = %f\n",q[1]);
    printf("q[2] = %f\n",q[2]);
    printf("q[3] = %f\n",q[3]);*/

    //随伴ベクトルの終端時刻での値 0で考えてもいいかも
    double f_T[6] = {0};
    optimal_control_x(x_T, u); //ドローンの終端時刻での状態ベクトルの予測値

    //終端コストF=0で考える
    
    /*for (int i = 0; i < 6; i++){
        f_T[i] = 2 * x_T[i];//重み係数F=1
    }*/
    
    int n = 5;
    double T = 0.5;
    double h = T / n;  // 刻み幅

    //鳥の群れの関数
    //jはMPCのサンプリング点、小さい方から現在時刻から終端時刻までのサンプル点を記憶 0,0.1,0.2,0.3,0.4,0.5秒
    double x_s[NUM][6] = {0};//swarm

    for (int i = 0; i < 5; i++){
        for (int j = 0; j < 6; j++) {
            x_s[i][j] = x_KF[i] + x_KF[i+5] * (j * h);
        }
    }

    //ドローンの関数
    double x_d[6][6] = {0};//drone

    for (int k = 0; k < 6; k++) {
    for (int i = 0; i < 6; i++) {

        // i=0 (時刻0) のときは積分ゼロなので初期値を入れるだけ
        if (i == 0) {
            x_d[k][i] = x[k];
            continue; // 次のループ (i=1) へ
        }

        // i > 0 のときだけ積分計算
        double sum = 0.5 * (exp(i * h) * u[k][0] + u[k][i]);

        for (int j = 1; j < i; j++) {
            sum += exp((i * h) - (j * h)) * u[k][j];
        }

        x_d[k][i] = exp(i * h) * x[k] + sum * h;
    }
    }

    
    /*for (int k = 0; k < 6; k++){
    for (int i = 0; i < 6; i++){
    double sum = 0.5 * (u[k][0] + u[k][i]);  // 両端の寄与 t秒の時とt + 0.5のとき

    for (int j = 1; j < i; j++) {
        sum += u[k][j];
    }

    x_d[k][i] = x[k] + sum * h;
    }
    }*/
    

    //随伴ベクトルΨ(t)の導出
    double d1x[6] = {0};
    double d1y[6] = {0};
    double d2x[6] = {0};
    double d2y[6] = {0};
    double d3x[6] = {0};
    double d3y[6] = {0};

    for (int i = 0; i < 6; i++){
        d1x[i] = (x_s[0][i] - x_d[0][i]);
        d1y[i] = (x_s[1][i] - x_d[1][i]);
        d2x[i] = (x_s[0][i] - x_d[2][i]);
        d2y[i] = (x_s[1][i] - x_d[3][i]);
        d3x[i] = (x_s[0][i] - x_d[4][i]);
        d3y[i] = (x_s[1][i] - x_d[5][i]);
    }

    double p[2][6] = {0};
    double range[6] = {0};
    double e[2][6] = {0};//単位ベクトル
    double g[2][6] = {0};

    for (int i = 0; i < 6; i++){
        p[0][i] = PROTECTED_AREA_x - x_s[0][i];
        p[1][i] = PROTECTED_AREA_y - x_s[1][i];

        range[i] = sqrt(p[0][i] * p[0][i] + p[1][i] * p[1][i]);
        e[0][i] = p[0][i] / range[i];//単位ベクトル
        e[1][i] = p[1][i] / range[i];

        g[0][i] = x_s[0][i] + x_s[2][i] * e[0][i];
        g[1][i] = x_s[1][i] + x_s[2][i] * e[1][i];
    }

    //群れ全体をカメラに収めるための距離を求める
    double d_dist[3][6] = {0};
    double sigma[4][6] = {0};

    for (int j = 0; j < 6; j++){
        sigma[0][j] = (x_s[2][j] * x_s[2][j] + x_s[3][j] * x_s[3][j] + (x_s[2][j] * x_s[2][j] - x_s[3][j] * x_s[3][j]) * cos(2 * x_s[4][j])) / 2;
        sigma[3][j] = (x_s[2][j] * x_s[2][j] + x_s[3][j] * x_s[3][j] - (x_s[2][j] * x_s[2][j] - x_s[3][j] * x_s[3][j]) * cos(2 * x_s[4][j])) / 2;
        sigma[1][j] = sigma[2][j] = ((x_s[2][j] * x_s[2][j] - x_s[3][j] * x_s[3][j]) * sin(2 * x_s[4][j])) / 2;
    }

    double A_d1[6] = {0};
    double A_d2[6] = {0};
    double B_d1[6] = {0};
    double B_d2[6] = {0};
    //xで偏微分
    double A_d1_dx[6] = {0};
    double A_d2_dx[6] = {0};
    double B_d1_dx[6] = {0};
    double B_d2_dx[6] = {0};
    //yで偏微分
    double A_d1_dy[6] = {0};
    double A_d2_dy[6] = {0};
    double B_d1_dy[6] = {0};
    double B_d2_dy[6] = {0};

    for (int i = 0; i < 6; i++){
    double n_d1[2] = {0};
    double n_d2[2] = {0};
    double m_d1[2] = {0};
    double m_d2[2] = {0};
    double dist_d1 = sqrt(d1x[i] * d1x[i] + d1y[i] * d1y[i]);
    double dist_d2 = sqrt(d2x[i] * d2x[i] + d2y[i] * d2y[i]);
    double range_d1[2] = {d1x[i] / dist_d1, d1y[i] / dist_d1};//ドローンから群れへの単位ベクトル
    double range_d2[2] = {d2x[i] / dist_d2, d2y[i] / dist_d2};
    double theta_d1 = atan2(range_d1[1], range_d1[0]);
    double theta_d2 = atan2(range_d2[1], range_d2[0]);

    double theta_n_d1 = (-0.5 * cameradrones[0].fov_angle_deg * PI / 180 + (cameradrones[0].angle * PI / 180 - theta_d1));
    double theta_n_d2 = (-0.5 * cameradrones[1].fov_angle_deg * PI / 180 + (cameradrones[1].angle * PI / 180 - theta_d2));

    double theta_m_d1 = (0.5 * cameradrones[0].fov_angle_deg * PI / 180 + (cameradrones[0].angle * PI / 180 - theta_d1));
    double theta_m_d2 = (0.5 * cameradrones[1].fov_angle_deg * PI / 180 + (cameradrones[1].angle * PI / 180 - theta_d2));

    n_d1[0] = cos(0.5 * PI + theta_n_d1) * range_d1[0] - sin(0.5 * PI + theta_n_d1) * range_d1[1];
    n_d1[1] = sin(0.5 * PI + theta_n_d1) * range_d1[0] + cos(0.5 * PI + theta_n_d1) * range_d1[1];
    m_d1[0] = cos(0.5 * PI + theta_m_d1) * range_d1[0] - sin(0.5 * PI + theta_m_d1) * range_d1[1];
    m_d1[1] = sin(0.5 * PI + theta_m_d1) * range_d1[0] + cos(0.5 * PI + theta_m_d1) * range_d1[1];

    n_d2[0] = cos(0.5 * PI + theta_n_d2) * range_d2[0] - sin(0.5 * PI + theta_n_d2) * range_d2[1];
    n_d2[1] = sin(0.5 * PI + theta_n_d2) * range_d2[0] + cos(0.5 * PI + theta_n_d2) * range_d2[1];
    m_d2[0] = cos(0.5 * PI + theta_m_d2) * range_d2[0] - sin(0.5 * PI + theta_m_d2) * range_d2[1];
    m_d2[1] = sin(0.5 * PI + theta_m_d2) * range_d2[0] + cos(0.5 * PI + theta_m_d2) * range_d2[1];

    double R1_d1 = sqrt(n_d1[0] * n_d1[0] * sigma[0][i] + 2 * n_d1[0] * n_d1[1] * sigma[1][i] + n_d1[1] * n_d1[1] * sigma[3][i]);
    double R2_d1 = sqrt(m_d1[0] * m_d1[0] * sigma[0][i] + 2 * m_d1[0] * m_d1[1] * sigma[1][i] + m_d1[1] * m_d1[1] * sigma[3][i]);

    double angle1_d1 = atan2(sin(cameradrones[0].fov_angle_deg * PI / 180), (R1_d1 + R2_d1) / R1_d1 - 1 + cos(cameradrones[0].fov_angle_deg * PI / 180));
    double angle2_d1 = atan2(sin(cameradrones[0].fov_angle_deg * PI / 180), (R1_d1 + R2_d1) / R2_d1 - 1 + cos(cameradrones[0].fov_angle_deg * PI / 180));

    double d1_d1 = fabs(R1_d1 / sin(fabs(angle1_d1)));
    double d2_d1 = fabs(R2_d1 / sin(fabs(angle2_d1)));

    //printf("d=d1=d2のはず d1=%f, d2=%f\n", d1_d1,d2_d1);

    double R1_d2 = sqrt(n_d2[0] * n_d2[0] * sigma[0][i] + 2 * n_d2[0] * n_d2[1] * sigma[1][i] + n_d2[1] * n_d2[1] * sigma[3][i]);
    double R2_d2 = sqrt(m_d2[0] * m_d2[0] * sigma[0][i] + 2 * m_d2[0] * m_d2[1] * sigma[1][i] + m_d2[1] * m_d2[1] * sigma[3][i]);

    double angle1_d2 = atan2(sin(cameradrones[1].fov_angle_deg * PI / 180), (R1_d1 + R2_d1) / R1_d1 - 1 + cos(cameradrones[1].fov_angle_deg * PI / 180));
    double angle2_d2 = atan2(sin(cameradrones[1].fov_angle_deg * PI / 180), (R1_d1 + R2_d1) / R2_d1 - 1 + cos(cameradrones[1].fov_angle_deg * PI / 180));

    double d1_d2 = fabs(R1_d2 / sin(fabs(angle1_d2)));
    double d2_d2 = fabs(R2_d2 / sin(fabs(angle2_d2)));

    double angle_d1 = 0;
    double angle_d2 = 0;

    if (d1_d1 >= d2_d1){
        d_dist[0][i] = d1_d1;
        angle_d1 = 0.5 * PI + theta_n_d1;
    }
    else{
        d_dist[0][i] = d2_d1;
        angle_d1 = -0.5 * PI + theta_m_d1;
    }

    if (d1_d2 >= d2_d2){
        d_dist[1][i] = d1_d2;
        angle_d2 = 0.5 * PI + theta_n_d2;
    }
    else{
        d_dist[1][i] = d2_d2;
        angle_d2 = -0.5 * PI + theta_m_d2;
    }

    A_d1[i] = range_d1[0] * cos(angle_d1) - range_d1[1] * sin(angle_d1);//視野角の接線の法線単位ベクトル
    B_d1[i] = range_d1[0] * sin(angle_d1) + range_d1[1] * cos(angle_d1);//視野角の接線の法線単位ベクトル
    A_d2[i] = range_d2[0] * cos(angle_d2) - range_d2[1] * sin(angle_d2);//視野角の接線の法線単位ベクトル
    B_d2[i] = range_d2[0] * sin(angle_d2) + range_d2[1] * cos(angle_d2);//視野角の接線の法線単位ベクトル

    A_d1_dx[i] = (-2 * d1x[i] * d1x[i] + d1y[i] * d1y[i]) * cos(angle_d1) / (dist_d1 * (d1x[i] * d1x[i] + d1y[i] * d1y[i])) - d1x[i] * d1y[i] * sin(angle_d1) / (dist_d1 * (d1x[i] * d1x[i] + d1y[i] * d1y[i]));
    B_d1_dx[i] = (-2 * d1x[i] * d1x[i] + d1y[i] * d1y[i]) * sin(angle_d1) / (dist_d1 * (d1x[i] * d1x[i] + d1y[i] * d1y[i])) + d1x[i] * d1y[i] * cos(angle_d1) / (dist_d1 * (d1x[i] * d1x[i] + d1y[i] * d1y[i]));
    A_d2_dx[i] = (-2 * d2x[i] * d2x[i] + d2y[i] * d2y[i]) * cos(angle_d2) / (dist_d2 * (d2x[i] * d2x[i] + d2y[i] * d2y[i])) - d2x[i] * d2y[i] * sin(angle_d2) / (dist_d2 * (d2x[i] * d2x[i] + d2y[i] * d2y[i]));
    B_d2_dx[i] = (-2 * d2x[i] * d2x[i] + d2y[i] * d2y[i]) * sin(angle_d2) / (dist_d2 * (d2x[i] * d2x[i] + d2y[i] * d2y[i])) + d2x[i] * d2y[i] * cos(angle_d2) / (dist_d2 * (d2x[i] * d2x[i] + d2y[i] * d2y[i]));

    A_d1_dy[i] = d1x[i] * d1y[i] * cos(angle_d1) / (dist_d1 * (d1x[i] * d1x[i] + d1y[i] * d1y[i])) + (2 * d1x[i] * d1x[i] - d1y[i] * d1y[i]) * sin(angle_d1) / (dist_d1 * (d1x[i] * d1x[i] + d1y[i] * d1y[i]));
    B_d1_dy[i] = d1x[i] * d1y[i] * sin(angle_d1) / (dist_d1 * (d1x[i] * d1x[i] + d1y[i] * d1y[i])) - (2 * d1x[i] * d1x[i] - d1y[i] * d1y[i]) * cos(angle_d1) / (dist_d1 * (d1x[i] * d1x[i] + d1y[i] * d1y[i]));
    A_d2_dy[i] = d2x[i] * d2y[i] * cos(angle_d2) / (dist_d2 * (d2x[i] * d2x[i] + d2y[i] * d2y[i])) + (2 * d2x[i] * d2x[i] - d2y[i] * d2y[i]) * sin(angle_d2) / (dist_d2 * (d2x[i] * d2x[i] + d2y[i] * d2y[i]));
    B_d2_dy[i] = d2x[i] * d2y[i] * sin(angle_d2) / (dist_d2 * (d2x[i] * d2x[i] + d2y[i] * d2y[i])) - (2 * d2x[i] * d2x[i] - d2y[i] * d2y[i]) * cos(angle_d2) / (dist_d2 * (d2x[i] * d2x[i] + d2y[i] * d2y[i]));

    }

    /*for (int i = 0; i < 6; i++){
        if (d_dist[0][i] <= d_dist[1][i]) d_dist[2][i] = d_dist[0][i];
        if (d_dist[0][i] > d_dist[1][i]) d_dist[2][i] = d_dist[1][i];
    }*/

    for (int i = 0; i < 6; i++){
        d_dist[2][i] = (d_dist[0][i] + d_dist[1][i]) / 2;
    }

    printf("距離　ドローン1:%f, ドローン2:%f, ドローン3:%f\n", d_dist[0][0], d_dist[1][0], d_dist[2][0]);
    printf("実際の距離　ドローン1:%f, ドローン2:%f, ドローン3:%f\n", sqrt(d1x[0] * d1x[0] + d1y[0] * d1y[0]), sqrt(d2x[0] * d2x[0] + d2y[0] * d2y[0]), sqrt(d3x[0] * d3x[0] + d3y[0] * d3y[0]));

    double l_max[6] = {0};
    double l_min[6] = {0};
    double d_max = 300;

    for (int i = 0; i < 6; i++){
        l_max[i] = sqrt(d1x[i] * d1x[i] + d1y[i] * d1y[i]);//カメラ精度を保証するためのドローン間の距離
        l_min[i] = sqrt(d1x[i] * d1x[i] + d1y[i] * d1y[i] + d2x[i] * d2x[i] + d2y[i] * d2y[i] - 2 * sqrt(d1x[i] * d1x[i] + d1y[i] * d1y[i]) * sqrt(d2x[i] * d2x[i] + d2y[i] * d2y[i]) * cos(PI / 4));
    }

    printf("カメラドローン間の距離 %f<x<%f\n", l_min[0], l_max[0]);

    //群れを誘導する方向を決定する
    double swarm_vx_n = x_KF[5] / sqrt(x_KF[5] * x_KF[5] + x_KF[6] * x_KF[6]);//群れの進行方向
    double swarm_vy_n = x_KF[6] / sqrt(x_KF[5] * x_KF[5] + x_KF[6] * x_KF[6]);
    double protect_to_swarm_x[6] = {0};//群れから保護エリアへの単位ベクトル
    double protect_to_swarm_y[6] = {0};
    double v_escort[2][6] = {0};//誘導する方向
    double drone_x[6] = {0};//誘導するための配置ベクトル方向
    double drone_y[6] = {0};
    //計算式短縮用
    double d1vx[6] = {0};//ドローン1速度
    double d1vy[6] = {0};
    double d2vx[6] = {0};//ドローン2速度
    double d2vy[6] = {0};
    double d3vx[6] = {0};//ドローン3速度
    double d3vy[6] = {0};

    double q_d_1[6] = {0};//重み
    double q_d_2[6] = {0};
    double q_d_3[6] = {0};
    double q_v[6] = {0};

    double q_s[6] = {0};

    double drone_point_angle = PI/18;//10度

    for (int i = 0; i < 6; i++){
        protect_to_swarm_x[i] = (PROTECTED_AREA_x - x_s[0][i]) / sqrt((PROTECTED_AREA_x - x_s[0][i]) * (PROTECTED_AREA_x - x_s[0][i]) + (PROTECTED_AREA_y - x_s[1][i]) * (PROTECTED_AREA_y - x_s[1][i]));
        protect_to_swarm_y[i] = (PROTECTED_AREA_y - x_s[1][i]) / sqrt((PROTECTED_AREA_x - x_s[0][i]) * (PROTECTED_AREA_x - x_s[0][i]) + (PROTECTED_AREA_y - x_s[1][i]) * (PROTECTED_AREA_y - x_s[1][i]));

        d1vx[i] = (exp(h) - 1) * (x_d[0][i] + u[0][i]) / h;
        d1vy[i] = (exp(h) - 1) * (x_d[1][i] + u[1][i]) / h;
        d2vx[i] = (exp(h) - 1) * (x_d[2][i] + u[2][i]) / h;
        d2vy[i] = (exp(h) - 1) * (x_d[3][i] + u[3][i]) / h;
        d3vx[i] = (exp(h) - 1) * (x_d[4][i] + u[4][i]) / h;
        d3vy[i] = (exp(h) - 1) * (x_d[5][i] + u[5][i]) / h;

        /*d1vx[i] = u[0][i];
        d1vy[i] = u[1][i];
        d2vx[i] = u[2][i];
        d2vy[i] = u[3][i];
        d3vx[i] = u[4][i];
        d3vy[i] = u[5][i];*/

        if ((x_s[0][i] * x_s[0][i] / (PROTECTED_AREA_a * PROTECTED_AREA_a) + x_s[1][i] * x_s[1][i] / (PROTECTED_AREA_b * PROTECTED_AREA_b)) > 1){
            q_s[i] = 1 / (x_s[0][i] * x_s[0][i] / (PROTECTED_AREA_a * PROTECTED_AREA_a) + x_s[1][i] * x_s[1][i] / (PROTECTED_AREA_b * PROTECTED_AREA_b));
        }

        else {
            q_s[i] = 1;
        }

        if ((protect_to_swarm_x[i] * swarm_vx_n + protect_to_swarm_y[i] * swarm_vy_n) > 0 || sqrt((x_s[0][i] - PROTECTED_AREA_x) * (x_s[0][i] - PROTECTED_AREA_x) + (x_s[1][i] - PROTECTED_AREA_y) * (x_s[1][i] - PROTECTED_AREA_y)) <= DANGER_R){ //内積が９０度以内のときだけコストに入れる
            q_d_1[i] = Q_d_1 * q_s[i];
            q_d_2[i] = Q_d_2 * q_s[i];
            q_d_3[i] = Q_d_3 * q_s[i];
            q_v[i] = Q_v * q_s[i];

            double phi = atan2((protect_to_swarm_x[i] * swarm_vy_n - protect_to_swarm_y[i] * swarm_vx_n), (protect_to_swarm_x[i] * swarm_vx_n + protect_to_swarm_y[i] * swarm_vy_n));
            double escort_angle = PI - fabs(phi);//90度
            printf("誘導角度%f\n", escort_angle);

        if (phi >= 0){//群れの進行方向が保護エリアの左右どっちに行くか,sinが正やったら正に回転
            v_escort[0][i] = cos(escort_angle * q_s[i]) * swarm_vx_n - sin(escort_angle * q_s[i]) * swarm_vy_n;
            v_escort[1][i] = sin(escort_angle * q_s[i]) * swarm_vx_n + cos(escort_angle * q_s[i]) * swarm_vy_n;

            drone_x[i] = cos(-drone_point_angle / q_s[i]) * protect_to_swarm_x[i] - sin(-drone_point_angle / q_s[i]) * protect_to_swarm_y[i];
            drone_y[i] = sin(-drone_point_angle / q_s[i]) * protect_to_swarm_x[i] + cos(-drone_point_angle / q_s[i]) * protect_to_swarm_y[i];
        }

        else {
            v_escort[0][i] = cos(-escort_angle * q_s[i]) * swarm_vx_n - sin(-escort_angle * q_s[i]) * swarm_vy_n;
            v_escort[1][i] = sin(-escort_angle * q_s[i]) * swarm_vx_n + cos(-escort_angle * q_s[i]) * swarm_vy_n;

            drone_x[i] = cos(drone_point_angle) * protect_to_swarm_x[i] - sin(drone_point_angle) * protect_to_swarm_y[i];// / q_s[i]
            drone_y[i] = sin(drone_point_angle) * protect_to_swarm_x[i] + cos(drone_point_angle) * protect_to_swarm_y[i];
        }
        }

        else {
            q_d_1[i] = 0;
            q_d_2[i] = 0;
            q_d_3[i] = 0;
            q_v[i] = 0;
        }
    }

    printf("vescort=%f", swarm_vx_n);

    double dist_drone_x[6] = {0};
    double dist_drone_y[6] = {0};

    for(int i = 0; i < 6; i++){
        dist_drone_x[i] = x_d[0][i] - x_d[2][i];
        dist_drone_y[i] = x_d[1][i] - x_d[3][i];
    }

    printf("x=%f\n", sqrt(dist_drone_x[0] * dist_drone_x[0] + dist_drone_y[0] * dist_drone_y[0]));

    double delta_v = 0.1;//緩和バリア関数の設定値
    double delta_d = 0.1;
    double delta_l = 0.1;


    double L_t[6][6] = {0};//コスト汎関数のL(x,u)の各離散時間での値を保存　列の番号がサンプリング点を示す
    //L(x,u)のdL/dxの計算
    /*for (int i = 0; i < 6; i++){
        L_t[0][i] = -2 * (q_1 * ((d2x[i] * (d1x[i] * d2x[i] + d1y[i] * d2y[i]) * (d1x[i] * d1x[i] + d1y[i] * d1y[i])  -  d1x[i] * (d1x[i] * d2x[i] + d1y[i] * d2y[i]) * (d1x[i] * d2x[i] + d1y[i] * d2y[i])) / ((d1x[i] * d1x[i] + d1y[i] * d1y[i]) * (d1x[i] * d1x[i] + d1y[i] * d1y[i]) *(d2x[i] * d2x[i] + d2y[i] * d2y[i])))
                        + q_2 * (d1x[i] - d1x[i] * sqrt((d2x[i] * d2x[i] + d2y[i] * d2y[i]) / (d1x[i] * d1x[i] + d1y[i] * d1y[i])))
                        + q_3 * ((l[i] - sqrt(dist_drone_x[i] * dist_drone_x[i] + dist_drone_y[i] * dist_drone_y[i])) * (dist_drone_x[i] / sqrt(dist_drone_x[i] * dist_drone_x[i] + dist_drone_y[i] * dist_drone_y[i]) - (A_d1[i] * A_d1_dx[i] * sigma[0][i] + (A_d1_dx[i] * B_d1[i] + A_d1[i] * B_d1_dx[i]) * sigma[1][i] + B_d1[i] * B_d1_dx[i] * sigma[3][i]) / (l[i] * sin(0.5 * cameradrones[0].fov_angle_deg * PI / 180) * sin(0.5 * cameradrones[0].fov_angle_deg * PI / 180))))
                        + q_d_1 * ((x_s[0][i] + d_dist[0][i] * drone_x[i] - x_d[0][i]) * (1 - drone_x[i] * (A_d1[i] * A_d1_dx[i] * sigma[0][i] + (A_d1_dx[i] * B_d1[i] + A_d1[i] * B_d1_dx[i]) * sigma[1][i] + B_d1[i] * B_d1_dx[i] * sigma[3][i]) / (sqrt(A_d1[i] * A_d1[i] * sigma[0][i] + 2 * A_d1[i] * B_d1[i] * sigma[1][i] + B_d1[i] * B_d1[i] * sigma[3][i]) * sin(0.5 * cameradrones[0].fov_angle_deg * PI / 180))))
                        - q_d_1 * ((x_s[1][i] + d_dist[0][i] * drone_y[i] - x_d[1][i]) * (drone_y[i] * (A_d1[i] * A_d1_dx[i] * sigma[0][i] + (A_d1_dx[i] * B_d1[i] + A_d1[i] * B_d1_dx[i]) * sigma[1][i] + B_d1[i] * B_d1_dx[i] * sigma[3][i]) / (sqrt(A_d1[i] * A_d1[i] * sigma[0][i] + 2 * A_d1[i] * B_d1[i] * sigma[1][i] + B_d1[i] * B_d1[i] * sigma[3][i]) * sin(0.5 * cameradrones[0].fov_angle_deg * PI / 180))))
                        - q_d_3 * ((x_s[0][i] + d_dist[2][i] * drone_x[i] - x_d[4][i]) * 0.5 * (drone_x[i] * (A_d1[i] * A_d1_dx[i] * sigma[0][i] + (A_d1_dx[i] * B_d1[i] + A_d1[i] * B_d1_dx[i]) * sigma[1][i] + B_d1[i] * B_d1_dx[i] * sigma[3][i]) / (sqrt(A_d1[i] * A_d1[i] * sigma[0][i] + 2 * A_d1[i] * B_d1[i] * sigma[1][i] + B_d1[i] * B_d1[i] * sigma[3][i]) * sin(0.5 * cameradrones[0].fov_angle_deg * PI / 180))))
                        - q_d_3 * ((x_s[1][i] + d_dist[2][i] * drone_y[i] - x_d[5][i]) * 0.5 * (drone_y[i] * (A_d1[i] * A_d1_dx[i] * sigma[0][i] + (A_d1_dx[i] * B_d1[i] + A_d1[i] * B_d1_dx[i]) * sigma[1][i] + B_d1[i] * B_d1_dx[i] * sigma[3][i]) / (sqrt(A_d1[i] * A_d1[i] * sigma[0][i] + 2 * A_d1[i] * B_d1[i] * sigma[1][i] + B_d1[i] * B_d1[i] * sigma[3][i]) * sin(0.5 * cameradrones[0].fov_angle_deg * PI / 180))))
                        + q_v * ((v_escort[0][i] * (d1vx[i] * d1vx[i] + d1vy[i] * d1vy[i]) - (d1vx[i] * v_escort[0][i] + d1vy[i] * v_escort[1][i]) * d1vx[i]) / ((d1vx[i] * d1vx[i] + d1vy[i] * d1vy[i]) * sqrt(d1vx[i] * d1vx[i] + d1vy[i] * d1vy[i])))
                        );

        printf("実行コスト　各項 %f\n",(d2x[i] * (d1x[i] * d2x[i] + d1y[i] * d2y[i]) * (d1x[i] * d1x[i] + d1y[i] * d1y[i])  -  d1x[i] * (d1x[i] * d2x[i] + d1y[i] * d2y[i]) * (d1x[i] * d2x[i] + d1y[i] * d2y[i])) / ((d1x[i] * d1x[i] + d1y[i] * d1y[i]) * (d1x[i] * d1x[i] + d1y[i] * d1y[i]) *(d2x[i] * d2x[i] + d2y[i] * d2y[i])));
        printf("実行コスト　各項 %f\n",d1x[i] - d1x[i] * sqrt((d2x[i] * d2x[i] + d2y[i] * d2y[i]) / (d1x[i] * d1x[i] + d1y[i] * d1y[i])));
        printf("実行コスト　各項 %f\n",g[0][i] - x_d[0][i]);
    }*/

    for (int i = 0; i < 6; i++){
        L_t[0][i] = -2 * (q_1 * ((d2x[i] * (d1x[i] * d2x[i] + d1y[i] * d2y[i]) * (d1x[i] * d1x[i] + d1y[i] * d1y[i])  -  d1x[i] * (d1x[i] * d2x[i] + d1y[i] * d2y[i]) * (d1x[i] * d2x[i] + d1y[i] * d2y[i])) / ((d1x[i] * d1x[i] + d1y[i] * d1y[i]) * (d1x[i] * d1x[i] + d1y[i] * d1y[i]) *(d2x[i] * d2x[i] + d2y[i] * d2y[i])))
                        + q_2 * (d1x[i] - d1x[i] * sqrt((d2x[i] * d2x[i] + d2y[i] * d2y[i]) / (d1x[i] * d1x[i] + d1y[i] * d1y[i])))
                        + q_d_1[i] * (x_s[0][i] + d_dist[0][i] * drone_x[i] - x_d[0][i])
                        + q_v[i] * ((v_escort[0][i] * (d1vx[i] * d1vx[i] + d1vy[i] * d1vy[i]) - (d1vx[i] * v_escort[0][i] + d1vy[i] * v_escort[1][i]) * d1vx[i]) / ((d1vx[i] * d1vx[i] + d1vy[i] * d1vy[i]) * sqrt(d1vx[i] * d1vx[i] + d1vy[i] * d1vy[i])))
                        );

        //printf("実行コスト　各項 %f\n",q_1 * ((d2x[i] * (d1x[i] * d2x[i] + d1y[i] * d2y[i]) * (d1x[i] * d1x[i] + d1y[i] * d1y[i])  -  d1x[i] * (d1x[i] * d2x[i] + d1y[i] * d2y[i]) * (d1x[i] * d2x[i] + d1y[i] * d2y[i])) / ((d1x[i] * d1x[i] + d1y[i] * d1y[i]) * (d1x[i] * d1x[i] + d1y[i] * d1y[i]) *(d2x[i] * d2x[i] + d2y[i] * d2y[i]))));
        //printf("実行コスト　各項 %f\n",q_2 * (d1x[i] - d1x[i] * sqrt((d2x[i] * d2x[i] + d2y[i] * d2y[i]) / (d1x[i] * d1x[i] + d1y[i] * d1y[i]))));
        //printf("実行コスト　各項 %f\n",q_d_1[i] * (x_s[0][i] + d_dist[0][i] * drone_x[i] - x_d[0][i]));
        //printf("実行コスト　各項 %f\n", ((v_escort[0][i] * (d1vx[i] * d1vx[i] + d1vy[i] * d1vy[i]) - (d1vx[i] * v_escort[0][i] + d1vy[i] * v_escort[1][i]) * d1vx[i]) / ((d1vx[i] * d1vx[i] + d1vy[i] * d1vy[i]) * sqrt(d1vx[i] * d1vx[i] + d1vy[i] * d1vy[i]))));
    }

    for (int i = 0; i < 6; i++){
        L_t[1][i] = -2 * (q_1 * ((d2y[i] * (d1x[i] * d2x[i] + d1y[i] * d2y[i]) * (d1x[i] * d1x[i] + d1y[i] * d1y[i])  -  d1y[i] * (d1x[i] * d2x[i] + d1y[i] * d2y[i]) * (d1x[i] * d2x[i] + d1y[i] * d2y[i])) / ((d1x[i] * d1x[i] + d1y[i] * d1y[i]) * (d1x[i] * d1x[i] + d1y[i] * d1y[i]) *(d2x[i] * d2x[i] + d2y[i] * d2y[i])))
                        + q_2 * (d1y[i] - d1y[i] * sqrt((d2x[i] * d2x[i] + d2y[i] * d2y[i]) / (d1x[i] * d1x[i] + d1y[i] * d1y[i])))
                        + q_d_1[i] * (x_s[1][i] + d_dist[0][i] * drone_y[i] - x_d[1][i])
                        + q_v[i] * ((v_escort[1][i] * (d1vx[i] * d1vx[i] + d1vy[i] * d1vy[i]) - (d1vx[i] * v_escort[0][i] + d1vy[i] * v_escort[1][i]) * d1vy[i]) / ((d1vx[i] * d1vx[i] + d1vy[i] * d1vy[i]) * sqrt(d1vx[i] * d1vx[i] + d1vy[i] * d1vy[i])))
                        );
    }

    for (int i = 0; i < 6; i++){
        L_t[2][i] = -2 * (q_1 * ((d1x[i] * (d1x[i] * d2x[i] + d1y[i] * d2y[i]) * (d2x[i] * d2x[i] + d2y[i] * d2y[i])  -  d2x[i] * (d1x[i] * d2x[i] + d1y[i] * d2y[i]) * (d1x[i] * d2x[i] + d1y[i] * d2y[i])) / ((d1x[i] * d1x[i] + d1y[i] * d1y[i]) * (d2x[i] * d2x[i] + d2y[i] * d2y[i]) *(d2x[i] * d2x[i] + d2y[i] * d2y[i])))
                        + q_2 * (d2x[i] - d2x[i] * sqrt((d1x[i] * d1x[i] + d1y[i] * d1y[i]) / (d2x[i] * d2x[i] + d2y[i] * d2y[i])))
                        + q_d_2[i] * (x_s[0][i] + d_dist[1][i] * drone_x[i] - x_d[2][i])
                        + q_v[i] * ((v_escort[0][i] * (d2vx[i] * d2vx[i] + d2vy[i] * d2vy[i]) - (d2vx[i] * v_escort[0][i] + d2vy[i] * v_escort[1][i]) * d2vx[i]) / ((d2vx[i] * d2vx[i] + d2vy[i] * d2vy[i]) * sqrt(d2vx[i] * d2vx[i] + d2vy[i] * d2vy[i])))
                        );
    }

    for (int i = 0; i < 6; i++){
        L_t[3][i] = -2 * (q_1 * ((d1y[i] * (d1x[i] * d2x[i] + d1y[i] * d2y[i]) * (d2x[i] * d2x[i] + d2y[i] * d2y[i])  -  d2y[i] * (d1x[i] * d2x[i] + d1y[i] * d2y[i]) * (d1x[i] * d2x[i] + d1y[i] * d2y[i])) / ((d1x[i] * d1x[i] + d1y[i] * d1y[i]) * (d2x[i] * d2x[i] + d2y[i] * d2y[i]) *(d2x[i] * d2x[i] + d2y[i] * d2y[i])))
                        + q_2 * (d2y[i] - d2y[i] * sqrt((d1x[i] * d1x[i] + d1y[i] * d1y[i]) / (d2x[i] * d2x[i] + d2y[i] * d2y[i])))
                        + q_d_2[i] * (x_s[1][i] + d_dist[1][i] * drone_y[i] - x_d[3][i])
                        + q_v[i] * ((v_escort[1][i] * (d2vx[i] * d2vx[i] + d2vy[i] * d2vy[i]) - (d2vx[i] * v_escort[0][i] + d2vy[i] * v_escort[1][i]) * d2vy[i]) / ((d2vx[i] * d2vx[i] + d2vy[i] * d2vy[i]) * sqrt(d2vx[i] * d2vx[i] + d2vy[i] * d2vy[i])))
                        );
    }

    for (int i = 0; i < 6; i++){
        L_t[4][i] = -2 * (q_d_3[i] * (x_s[0][i] + d_dist[2][i] * drone_x[i] - x_d[4][i])
                        + q_v[i] * ((v_escort[0][i] * (d3vx[i] * d3vx[i] + d3vy[i] * d3vy[i]) - (d3vx[i] * v_escort[0][i] + d3vy[i] * v_escort[1][i]) * d3vx[i]) / ((d3vx[i] * d3vx[i] + d3vy[i] * d3vy[i]) * sqrt(d3vx[i] * d3vx[i] + d3vy[i] * d3vy[i])))
                        );
    }

    for (int i = 0; i < 6; i++){
        L_t[5][i] = -2 * (q_d_3[i] * (x_s[1][i] + d_dist[2][i] * drone_y[i] - x_d[5][i])
                        + q_v[i] * ((v_escort[1][i] * (d3vx[i] * d3vx[i] + d3vy[i] * d3vy[i]) - (d3vx[i] * v_escort[0][i] + d3vy[i] * v_escort[1][i]) * d3vy[i]) / ((d3vx[i] * d3vx[i] + d3vy[i] * d3vy[i]) * sqrt(d3vx[i] * d3vx[i] + d3vy[i] * d3vy[i])))
                        );

        printf("コストL_t[5][%d]%f\n", i, L_t[5][i]);
    }

    //+ q_3 * ((l_max[i] - sqrt(dist_drone_x[i] * dist_drone_x[i] + dist_drone_y[i] * dist_drone_y[i])) * (dist_drone_x[i] / sqrt(dist_drone_x[i] * dist_drone_x[i] + dist_drone_y[i] * dist_drone_y[i])))
    //+ q_3 * ((l[i] - sqrt(dist_drone_x[i] * dist_drone_x[i] + dist_drone_y[i] * dist_drone_y[i])) * (dist_drone_y[i] / sqrt(dist_drone_x[i] * dist_drone_x[i] + dist_drone_y[i] * dist_drone_y[i]))) 
    //+ q_3 * ((l[i] - sqrt(dist_drone_x[i] * dist_drone_x[i] + dist_drone_y[i] * dist_drone_y[i])) * (-dist_drone_x[i] / sqrt(dist_drone_x[i] * dist_drone_x[i] + dist_drone_y[i] * dist_drone_y[i])))  
    //+ q_3 * ((l[i] - sqrt(dist_drone_x[i] * dist_drone_x[i] + dist_drone_y[i] * dist_drone_y[i])) * (-dist_drone_y[i] / sqrt(dist_drone_x[i] * dist_drone_x[i] + dist_drone_y[i] * dist_drone_y[i]))) 

    //バリア関数のdBar/dxドローンの状態ベクトルの偏微分の計算
    double Bar[6][6] = {0};
    double S = (exp(h) - 1) / h;

    double Bar_d_max[4][6] = {0};
    double Bar_d_min[4][6] = {0};
    double Bar_l_max[4][6] = {0};
    double Bar_l_min[4][6] = {0};
    double Bar_v_max[6][6] = {0};

    for (int i = 0; i < 6; i++){
        if ((d1x[i] * d1x[i] + d1y[i] * d1y[i] - d_dist[0][i] * d_dist[0][i]) <= delta_d) {//ドローン1の群れまでの最小距離
            Bar_d_min[0][i] = - barrier_d * 2 * (d1x[i] * d1x[i] + d1y[i] * d1y[i] - d_dist[0][i] * d_dist[0][i]) * d1x[i] / (delta_d * delta_d) + 4 * barrier_d * d1x[i] / delta_d;//制約厳しいときの最小距離
            Bar_d_min[1][i] = - barrier_d * 2 * (d1x[i] * d1x[i] + d1y[i] * d1y[i] - d_dist[0][i] * d_dist[0][i]) * d1y[i] / (delta_d * delta_d) + 4 * barrier_d * d1y[i] / delta_d;//制約厳しいときの最小距離
        }
        else {
            Bar_d_min[0][i] = barrier_d * 2 * d1x[i] / (d1x[i] * d1x[i] + d1y[i] * d1y[i] - d_dist[0][i] * d_dist[0][i]);//最小距離
            Bar_d_min[1][i] = barrier_d * 2 * d1y[i] / (d1x[i] * d1x[i] + d1y[i] * d1y[i] - d_dist[0][i] * d_dist[0][i]);
        }


        if ((d2x[i] * d2x[i] + d2y[i] * d2y[i] - d_dist[1][i] * d_dist[1][i]) <= delta_d) {//ドローン2の群れまでの最小距離
            Bar_d_min[2][i] = - barrier_d * 2 * (d2x[i] * d2x[i] + d2y[i] * d2y[i] - d_dist[1][i] * d_dist[1][i]) * d2x[i] / (delta_d * delta_d) + 4 * barrier_d * d2x[i] / delta_d;//制約厳しいときの最小距離
            Bar_d_min[3][i] = - barrier_d * 2 * (d2x[i] * d2x[i] + d2y[i] * d2y[i] - d_dist[1][i] * d_dist[1][i]) * d2y[i] / (delta_d * delta_d) + 4 * barrier_d * d2y[i] / delta_d;//制約厳しいときの最小距離
        }
        else {
            Bar_d_min[2][i] = barrier_d * 2 * d2x[i] / (d2x[i] * d2x[i] + d2y[i] * d2y[i] - d_dist[1][i] * d_dist[1][i]);//最小距離
            Bar_d_min[3][i] = barrier_d * 2 * d2y[i] / (d2x[i] * d2x[i] + d2y[i] * d2y[i] - d_dist[1][i] * d_dist[1][i]);
        }


        if ((d_max * d_max - (d1x[i] * d1x[i] + d1y[i] * d1y[i])) <= delta_d) {//ドローン1の群れまでの最大距離
            Bar_d_max[0][i] = barrier_d * 2 * (d_max * d_max - (d1x[i] * d1x[i] + d1y[i] * d1y[i])) * d1x[i] / (delta_d * delta_d) - 4 * barrier_d * d1x[i] / delta_d;//制約厳しいときの最大距離
            Bar_d_max[1][i] = barrier_d * 2 * (d_max * d_max - (d1x[i] * d1x[i] + d1y[i] * d1y[i])) * d1y[i] / (delta_d * delta_d) - 4 * barrier_d * d1y[i] / delta_d;//制約厳しいときの最大距離
        }
        else {
            Bar_d_max[0][i] = - barrier_d * 2 * d1x[i] / (d_max * d_max - (d1x[i] * d1x[i] + d1y[i] * d1y[i]));//最大距離
            Bar_d_max[1][i] = - barrier_d * 2 * d1y[i] / (d_max * d_max - (d1x[i] * d1x[i] + d1y[i] * d1y[i]));//最大距離
        }


        if ((d_max * d_max - (d2x[i] * d2x[i] + d2y[i] * d2y[i])) <= delta_d) {//ドローン2の群れまでの最大距離
            Bar_d_max[2][i] = barrier_d * 2 * (d_max * d_max - (d2x[i] * d2x[i] + d2y[i] * d2y[i])) * d2x[i] / (delta_d * delta_d) - 4 * barrier_d * d2x[i] / delta_d;//制約厳しいときの最大距離
            Bar_d_max[3][i] = barrier_d * 2 * (d_max * d_max - (d2x[i] * d2x[i] + d2y[i] * d2y[i])) * d2y[i] / (delta_d * delta_d) - 4 * barrier_d * d2y[i] / delta_d;//制約厳しいときの最大距離
        }
        else {
            Bar_d_max[2][i] = - barrier_d * 2 * d2x[i] / (d_max * d_max - (d2x[i] * d2x[i] + d2y[i] * d2y[i]));//最大距離
            Bar_d_max[3][i] = - barrier_d * 2 * d2y[i] / (d_max * d_max - (d2x[i] * d2x[i] + d2y[i] * d2y[i]));//最大距離
        }


        if ((dist_drone_x[i] * dist_drone_x[i] + dist_drone_y[i] * dist_drone_y[i] - l_min[i] * l_min[i]) <= delta_l) {//ドローン間の最小距離
            Bar_l_min[0][i] = barrier_l * 2 * (dist_drone_x[i] * dist_drone_x[i] + dist_drone_y[i] * dist_drone_y[i] - l_min[i] * l_min[i]) * dist_drone_x[i] / (delta_l * delta_l) - 4 * barrier_l * dist_drone_x[i] / delta_l;//制約厳しいときのドローン間の最小距離
            Bar_l_min[1][i] = barrier_l * 2 * (dist_drone_x[i] * dist_drone_x[i] + dist_drone_y[i] * dist_drone_y[i] - l_min[i] * l_min[i]) * dist_drone_y[i] / (delta_l * delta_l) - 4 * barrier_l * dist_drone_y[i] / delta_l;//制約厳しいときのドローン間の最小距離
            Bar_l_min[2][i] = - barrier_l * 2 * (dist_drone_x[i] * dist_drone_x[i] + dist_drone_y[i] * dist_drone_y[i] - l_min[i] * l_min[i]) * dist_drone_x[i] / (delta_l * delta_l) + 4 * barrier_l * dist_drone_x[i] / delta_l;//制約厳しいときのドローン間の最小距離
            Bar_l_min[3][i] = - barrier_l * 2 * (dist_drone_x[i] * dist_drone_x[i] + dist_drone_y[i] * dist_drone_y[i] - l_min[i] * l_min[i]) * dist_drone_y[i] / (delta_l * delta_l) + 4 * barrier_l * dist_drone_y[i] / delta_l;//制約厳しいときのドローン間の最小距離
        }
        else {
            Bar_l_min[0][i] = - barrier_l * 2 * dist_drone_x[i] / (dist_drone_x[i] * dist_drone_x[i] + dist_drone_y[i] * dist_drone_y[i] - l_min[i] * l_min[i]);//ドローン間の最小距離
            Bar_l_min[1][i] = - barrier_l * 2 * dist_drone_y[i] / (dist_drone_x[i] * dist_drone_x[i] + dist_drone_y[i] * dist_drone_y[i] - l_min[i] * l_min[i]);//ドローン間の最小距離
            Bar_l_min[2][i] = barrier_l * 2 * dist_drone_x[i] / (dist_drone_x[i] * dist_drone_x[i] + dist_drone_y[i] * dist_drone_y[i] - l_min[i] * l_min[i]);//ドローン間の最小距離
            Bar_l_min[3][i] = barrier_l * 2 * dist_drone_y[i] / (dist_drone_x[i] * dist_drone_x[i] + dist_drone_y[i] * dist_drone_y[i] - l_min[i] * l_min[i]);//ドローン間の最小距離
        }


        if ((l_max[i] * l_max[i] - (dist_drone_x[i] * dist_drone_x[i] + dist_drone_y[i] * dist_drone_y[i])) <= delta_l) {//ドローン間の最大距離
            Bar_l_max[0][i] = - barrier_l * 2 * (l_max[i] * l_max[i] - (dist_drone_x[i] * dist_drone_x[i] + dist_drone_y[i] * dist_drone_y[i])) * dist_drone_x[i] / (delta_l * delta_l) + 4 * barrier_l * dist_drone_x[i] / delta_l;//制約厳しいときのドローン間の最大距離
            Bar_l_max[1][i] = - barrier_l * 2 * (l_max[i] * l_max[i] - (dist_drone_x[i] * dist_drone_x[i] + dist_drone_y[i] * dist_drone_y[i])) * dist_drone_y[i] / (delta_l * delta_l) + 4 * barrier_l * dist_drone_y[i] / delta_l;//制約厳しいときのドローン間の最大距離
            Bar_l_max[2][i] = barrier_l * 2 * (l_max[i] * l_max[i] - (dist_drone_x[i] * dist_drone_x[i] + dist_drone_y[i] * dist_drone_y[i])) * dist_drone_x[i] / (delta_l * delta_l) - 4 * barrier_l * dist_drone_x[i] / delta_l;//制約厳しいときのドローン間の最大距離
            Bar_l_max[3][i] = barrier_l * 2 * (l_max[i] * l_max[i] - (dist_drone_x[i] * dist_drone_x[i] + dist_drone_y[i] * dist_drone_y[i])) * dist_drone_y[i] / (delta_l * delta_l) - 4 * barrier_l * dist_drone_y[i] / delta_l;//制約厳しいときのドローン間の最大距離
        }
        else {
            Bar_l_max[0][i] = barrier_l * 2 * dist_drone_x[i] / (l_max[i] * l_max[i] - (dist_drone_x[i] * dist_drone_x[i] + dist_drone_y[i] * dist_drone_y[i]));//ドローン間の最大距離
            Bar_l_max[1][i] = barrier_l * 2 * dist_drone_y[i] / (l_max[i] * l_max[i] - (dist_drone_x[i] * dist_drone_x[i] + dist_drone_y[i] * dist_drone_y[i]));//ドローン間の最大距離
            Bar_l_max[2][i] = - barrier_l * 2 * dist_drone_x[i] / (l_max[i] * l_max[i] - (dist_drone_x[i] * dist_drone_x[i] + dist_drone_y[i] * dist_drone_y[i]));//ドローン間の最大距離
            Bar_l_max[3][i] = - barrier_l * 2 * dist_drone_y[i] / (l_max[i] * l_max[i] - (dist_drone_x[i] * dist_drone_x[i] + dist_drone_y[i] * dist_drone_y[i]));//ドローン間の最大距離
        }

//速度は0に落ち着く
        if ((DRONE_MAX_SPEED - sqrt(d1vx[i] * d1vx[i] + d1vy[i] * d1vy[i])) <= delta_v){
            Bar_v_max[0][i] = - barrier_v * S * (DRONE_MAX_SPEED - sqrt(d1vx[i] * d1vx[i] + d1vy[i] * d1vy[i])) * d1vx[i] / (sqrt(d1vx[i] * d1vx[i] + d1vy[i] * d1vy[i]) * delta_v * delta_v) + 2 * barrier_v * S * d1vx[i] / (sqrt(d1vx[i] * d1vx[i] + d1vy[i] * d1vy[i]) * delta_v);//制約厳しいときの速度
            Bar_v_max[1][i] = - barrier_v * S * (DRONE_MAX_SPEED - sqrt(d1vx[i] * d1vx[i] + d1vy[i] * d1vy[i])) * d1vy[i] / (sqrt(d1vx[i] * d1vx[i] + d1vy[i] * d1vy[i]) * delta_v * delta_v) + 2 * barrier_v * S * d1vy[i] / (sqrt(d1vx[i] * d1vx[i] + d1vy[i] * d1vy[i]) * delta_v);//制約厳しいときの速度
        }
        else {
            Bar_v_max[0][i] = barrier_v * S * d1vx[i] / (sqrt(d1vx[i] * d1vx[i] + d1vy[i] * d1vy[i]) * (DRONE_MAX_SPEED - sqrt(d1vx[i] * d1vx[i] + d1vy[i] * d1vy[i])));
            Bar_v_max[1][i] = barrier_v * S * d1vy[i] / (sqrt(d1vx[i] * d1vx[i] + d1vy[i] * d1vy[i]) * (DRONE_MAX_SPEED - sqrt(d1vx[i] * d1vx[i] + d1vy[i] * d1vy[i])));
        }


        if ((DRONE_MAX_SPEED - sqrt(d2vx[i] * d2vx[i] + d2vy[i] * d2vy[i])) <= delta_v){
            Bar_v_max[2][i] = - barrier_v * S * (DRONE_MAX_SPEED - sqrt(d2vx[i] * d2vx[i] + d2vy[i] * d2vy[i])) * d2vx[i] / (sqrt(d2vx[i] * d2vx[i] + d2vy[i] * d2vy[i]) * delta_v * delta_v) + 2 * barrier_v * S * d2vx[i] / (sqrt(d2vx[i] * d2vx[i] + d2vy[i] * d2vy[i]) * delta_v);//制約厳しいときの速度
            Bar_v_max[3][i] = - barrier_v * S * (DRONE_MAX_SPEED - sqrt(d2vx[i] * d2vx[i] + d2vy[i] * d2vy[i])) * d2vy[i] / (sqrt(d2vx[i] * d2vx[i] + d2vy[i] * d2vy[i]) * delta_v * delta_v) + 2 * barrier_v * S * d2vy[i] / (sqrt(d2vx[i] * d2vx[i] + d2vy[i] * d2vy[i]) * delta_v);//制約厳しいときの速度
        }
        else {
            Bar_v_max[2][i] = barrier_v * S * d2vx[i] / (sqrt(d2vx[i] * d2vx[i] + d2vy[i] * d2vy[i]) * (DRONE_MAX_SPEED - sqrt(d2vx[i] * d2vx[i] + d2vy[i] * d2vy[i])));
            Bar_v_max[3][i] = barrier_v * S * d2vy[i] / (sqrt(d2vx[i] * d2vx[i] + d2vy[i] * d2vy[i]) * (DRONE_MAX_SPEED - sqrt(d2vx[i] * d2vx[i] + d2vy[i] * d2vy[i])));
        }


        if ((DRONE_MAX_SPEED - sqrt(d3vx[i] * d3vx[i] + d3vy[i] * d3vy[i])) <= delta_v){
            Bar_v_max[4][i] = - barrier_v * S * (DRONE_MAX_SPEED - sqrt(d3vx[i] * d3vx[i] + d3vy[i] * d3vy[i])) * d3vx[i] / (sqrt(d3vx[i] * d3vx[i] + d3vy[i] * d3vy[i]) * delta_v * delta_v) + 2 * barrier_v * S * d3vx[i] / (sqrt(d3vx[i] * d3vx[i] + d3vy[i] * d3vy[i]) * delta_v);//制約厳しいときの速度
            Bar_v_max[5][i] = - barrier_v * S * (DRONE_MAX_SPEED - sqrt(d3vx[i] * d3vx[i] + d3vy[i] * d3vy[i])) * d3vy[i] / (sqrt(d3vx[i] * d3vx[i] + d3vy[i] * d3vy[i]) * delta_v * delta_v) + 2 * barrier_v * S * d3vy[i] / (sqrt(d3vx[i] * d3vx[i] + d3vy[i] * d3vy[i]) * delta_v);//制約厳しいときの速度
        }
        else{
            Bar_v_max[4][i] = barrier_v * S * d3vx[i] / (sqrt(d3vx[i] * d3vx[i] + d3vy[i] * d3vy[i]) * (DRONE_MAX_SPEED - sqrt(d3vx[i] * d3vx[i] + d3vy[i] * d3vy[i])));
            Bar_v_max[5][i] = barrier_v * S * d3vy[i] / (sqrt(d3vx[i] * d3vx[i] + d3vy[i] * d3vy[i]) * (DRONE_MAX_SPEED - sqrt(d3vx[i] * d3vx[i] + d3vy[i] * d3vy[i])));
        }

        Bar[0][i] = Bar_d_max[0][i] + Bar_d_min[0][i] + Bar_l_max[0][i] + Bar_l_min[0][i] + Bar_v_max[0][i];
    
        Bar[1][i] = Bar_d_max[1][i] + Bar_d_min[1][i] + Bar_l_max[1][i] + Bar_l_min[1][i] + Bar_v_max[1][i];
    
        Bar[2][i] = Bar_d_max[2][i] + Bar_d_min[2][i] + Bar_l_max[2][i] + Bar_l_min[2][i] + Bar_v_max[2][i];
    
        Bar[3][i] = Bar_d_max[3][i] + Bar_d_min[3][i] + Bar_l_max[3][i] + Bar_l_min[3][i] + Bar_v_max[3][i];
    
        Bar[4][i] = Bar_v_max[4][i];
    
        Bar[5][i] = Bar_v_max[5][i];
    }
                
    //随伴ベクトルΨ(t)の導出
    for (int i = 0; i < 6; i++){
        for (int j = 0; j < 6; j++){//現在時刻 t = j * h

            // i=0 (時刻0) のときは積分ゼロなので初期値を入れるだけ
            if (j == 5) {
                f_t[i][j] = f_T[i];
                continue; // 次のループへ
            }

            double sum = 0.5 * (L_t[i][j] + exp(T - j * h) * L_t[i][5]) + 0.5 * (Bar[i][j] + exp(T - j * h) * Bar[i][5]);  // 両端の寄与 t秒の時とt + 0.5のとき

            for (int k = j + 1; k < n; k++) {
                sum += exp((k * h) - (j * h)) * L_t[i][k] + exp((k * h) - (j * h)) * Bar[i][k];
            }

            f_t[i][j] = exp(T - j * h) * f_T[i] + sum * h;
        }
    }

    
    /*for (int i = 0; i < 6; i++){
        for (int j = 0; j < 6; j++){//現在時刻 t = j * h
            double sum = 0.5 * (L_t[i][j] + L_t[i][5]) + 0.5 * (Bar[i][j] + Bar[i][5]);  // 両端の寄与 t秒の時とt + 0.5のとき

            for (int k = j + 1; k < n; k++) {
                sum += L_t[i][k] + Bar[i][k];
            }

            f_t[i][j] = f_T[i] + sum * h;
        }
    }*/
    
}

void optimal_control_u(double u[6][6], double x_KF[NUM], double *angle_escort){
    /*
    double x[6] = { 
                   cameradrones[0].x, 
                   cameradrones[0].y, 
                   cameradrones[1].x, 
                   cameradrones[1].y, 
                   drones[0].x, 
                   drones[0].y,
                  };
    
    double A[6][6] = {
        {1.0, 0, 0, 0, 0, 0},   // d1x
        {0, 1.0, 0, 0, 0, 0},   // d1y
        {0, 0, 1.0, 0, 0, 0},   // cd1x
        {0, 0, 0, 1.0, 0, 0},   // cd1y
        {0, 0, 0, 0, 1.0, 0},   // cd2x
        {0, 0, 0, 0, 0, 1.0}    // cd2y
    };

    double B[6][6] = {
        {1.0, 0, 0, 0, 0, 0},   // d1x
        {0, 1.0, 0, 0, 0, 0},   // d1y
        {0, 0, 1.0, 0, 0, 0},   // cd1x
        {0, 0, 0, 1.0, 0, 0},   // cd1y
        {0, 0, 0, 0, 1.0, 0},   // cd2x
        {0, 0, 0, 0, 0, 1.0}    // cd2y
    };

    double F[6][6] = { //終端コストの重み行列
        {1.0, 0, 0, 0, 0, 0},   // d1x
        {0, 1.0, 0, 0, 0, 0},   // d1y
        {0, 0, 1.0, 0, 0, 0},   // cd1x
        {0, 0, 0, 1.0, 0, 0},   // cd1y
        {0, 0, 0, 0, 1.0, 0},   // cd2x
        {0, 0, 0, 0, 0, 1.0}    // cd2y
    };
    */
    

    double dt = 1.0/10.0;

    double f_t[6][6] = {0};

    optimal_control_f(f_t, u, x_KF);

    /*for (int i = 0; i < 6; i++){
        for (int j = 0; j < 6; j++){
            u[i][j] = -0.5 * f_t[i][j] / N[i][i];
        }
    }*/

    /*double learning_rate = 0.01; // 学習率（小さくする！）

    for (int i = 0; i < 6; i++){
        for (int j = 0; j < 6; j++){
            // 勾配 dH/du = R_d * u + 0.5 * f_t 
            // (係数 0.5 はハミルトニアンの定義によるので確認してください)
            double gradient = N[i][i] * u[i][j] + 1.0 * f_t[i][j];
        
            // 勾配の逆方向に更新
            u[i][j] -= learning_rate * gradient;
        }
    }*/

    double x[6] = { 
                   cameradrones[0].x, 
                   cameradrones[0].y, 
                   cameradrones[1].x, 
                   cameradrones[1].y, 
                   drones[0].x, 
                   drones[0].y,
                  };
    
    //鳥の群れの関数
    //jはMPCのサンプリング点、小さい方から現在時刻から終端時刻までのサンプル点を記憶 0,0.1,0.2,0.3,0.4,0.5秒
    double x_s[NUM][6] = {0};//swarm

    for (int i = 0; i < 5; i++){
        for (int j = 0; j < 6; j++) {
            x_s[i][j] = x_KF[i] + x_KF[i+5] * (j * dt);
        }
    }

    //ドローンの関数
    double x_d[6][6] = {0};//drone

    int n = 5;
    double T = 0.5;
    double h = T / n;  // 刻み幅

    for (int k = 0; k < 6; k++) {
    for (int i = 0; i < 6; i++) {

        // i=0 (時刻0) のときは積分ゼロなので初期値を入れるだけ
        if (i == 0) {
            x_d[k][i] = x[k];
            continue; // 次のループ (i=1) へ
        }

        // i > 0 のときだけ積分計算
        double sum = 0.5 * (exp(i * h) * u[k][0] + u[k][i]);

        for (int j = 1; j < i; j++) {
            sum += exp((i * h) - (j * h)) * u[k][j];
        }

        x_d[k][i] = exp(i * h) * x[k] + sum * h;
    }
    }

    /*for (int k = 0; k < 6; k++){
    for (int i = 0; i < 6; i++){
    double sum = 0.5 * (u[k][0] + u[k][i]);  // 両端の寄与 t秒の時とt + 0.5のとき

    for (int j = 1; j < i; j++) {
        sum += u[k][j];
    }

    x_d[k][i] = x[k] + sum * h;
    }
    }*/

    //最適制御入力の決定
    /*for (int i = 0; i < 6; i++){
        for (int j = 0; j < 6; j++){
            u[i][j] = -0.5 * f_t[i][j] / N[i][i];
        }
    }*/

    //バリア関数を入れると候補点が二つ出てくる
    //速度の候補点を計算
    double v_1[3][6] = {0};
    double v_2[3][6] = {0};
    double v[3][6] = {0};

    double d_v[3][6] = {0};
    for (int i = 0; i < NUM_CAMERADRONES+NUM_DRONES; i++){
        for (int j = 0; j < 6; j++){
            d_v[i][j] = sqrt((x_d[2*i][j] + u[2*i][j]) * (x_d[2*i][j] + u[2*i][j]) + (x_d[2*i+1][j] + u[2*i+1][j]) * (x_d[2*i+1][j] + u[2*i+1][j]));
        }
    }

    double theta[3][6] = {0};

    //群れを誘導する方向を決定する
    double swarm_vx_n = x_KF[5] / sqrt(x_KF[5] * x_KF[5] + x_KF[6] * x_KF[6]);//群れの進行方向
    double swarm_vy_n = x_KF[6] / sqrt(x_KF[5] * x_KF[5] + x_KF[6] * x_KF[6]);
    double protect_to_swarm_x[6] = {0};//群れから保護エリアへの単位ベクトル
    double protect_to_swarm_y[6] = {0};
    double v_escort[2][6] = {0};//誘導する方向

    double q_v[6] = {0};

    double q_s[6] = {0};

    for (int i = 0; i < 6; i++){
        protect_to_swarm_x[i] = (PROTECTED_AREA_x - x_s[0][i]) / sqrt((PROTECTED_AREA_x - x_s[0][i]) * (PROTECTED_AREA_x - x_s[0][i]) + (PROTECTED_AREA_x - x_s[1][i]) * (PROTECTED_AREA_x - x_s[1][i]));
        protect_to_swarm_y[i] = (PROTECTED_AREA_x - x_s[1][i]) / sqrt((PROTECTED_AREA_x - x_s[0][i]) * (PROTECTED_AREA_x - x_s[0][i]) + (PROTECTED_AREA_x - x_s[1][i]) * (PROTECTED_AREA_x - x_s[1][i]));

        if (x_s[0][i] * x_s[0][i] / (PROTECTED_AREA_a * PROTECTED_AREA_a) + x_s[1][i] * x_s[1][i] / (PROTECTED_AREA_b * PROTECTED_AREA_b) > 1){
            q_s[i] = 1 / (x_s[0][i] * x_s[0][i] / (PROTECTED_AREA_a * PROTECTED_AREA_a) + x_s[1][i] * x_s[1][i] / (PROTECTED_AREA_b * PROTECTED_AREA_b));
        }

        else {
            q_s[i] = 1;
        }

        if (protect_to_swarm_x[i] * swarm_vx_n + protect_to_swarm_y[i] * swarm_vy_n > 0 || sqrt((x_s[0][i] - PROTECTED_AREA_x) * (x_s[0][i] - PROTECTED_AREA_x) + (x_s[1][i] - PROTECTED_AREA_y) * (x_s[1][i] - PROTECTED_AREA_y)) <= DANGER_R){ //内積が９０度以内のときだけコストに入れる && sqrt((x_s[0][i] - PROTECTED_AREA_x[i]) * (x_s[0][i] - PROTECTED_AREA_x[i]) + (x_s[1][i] - PROTECTED_AREA_y[i]) * (x_s[1][i] - PROTECTED_AREA_y[i])) <= DANGER_R
            q_v[i] = Q_v * q_s[i];

            double phi = atan2((protect_to_swarm_x[i] * swarm_vy_n - protect_to_swarm_y[i] * swarm_vx_n), (protect_to_swarm_x[i] * swarm_vx_n + protect_to_swarm_y[i] * swarm_vy_n));
            double escort_angle = PI - fabs(phi);//90度

            if ((protect_to_swarm_x[i] * swarm_vy_n - protect_to_swarm_y[i] * swarm_vx_n) / (protect_to_swarm_x[i] * protect_to_swarm_x[i] + protect_to_swarm_y[i] * protect_to_swarm_y[i]) >= 0){//群れの進行方向が保護エリアの左右どっちに行くか,sinが正やったら正に回転
            v_escort[0][i] = cos(escort_angle * q_s[i]) * swarm_vx_n - sin(escort_angle * q_s[i]) * swarm_vy_n;
            v_escort[1][i] = sin(escort_angle * q_s[i]) * swarm_vx_n + cos(escort_angle * q_s[i]) * swarm_vy_n;
            }

            else {
            v_escort[0][i] = cos(-escort_angle * q_s[i]) * swarm_vx_n - sin(-escort_angle * q_s[i]) * swarm_vy_n;
            v_escort[1][i] = sin(-escort_angle * q_s[i]) * swarm_vx_n + cos(-escort_angle * q_s[i]) * swarm_vy_n;
            }
        }

        else {
            q_v[i] = 0;
        }
    }

    *angle_escort = atan2(v_escort[0][0], v_escort[1][0]);

    printf("f_t_x=%f, ", f_t[4][0]);
    printf("f_t_y=%f,\n ", f_t[5][0]);

    for (int j = 0; j < NUM_CAMERADRONES+NUM_DRONES; j++){
    for (int i = 0; i < 6; i++){
        double A = f_t[2*j][i] - 2*R_d*x_d[2*j][i];
        double B = f_t[2*j+1][i] - 2*R_d*x_d[2*j+1][i];
        double S = (exp(dt) - 1) / dt;

        printf("A=%f, ", A);
        printf("B=%f, ", B);
        
        theta[j][i] = atan2((2 * q_v[i] * v_escort[1][i] / d_v[j][i] - B) , (2 * q_v[i] * v_escort[0][i] / d_v[j][i] - A));


        double a = A * cos(theta[j][i]);
        double b = B * sin(theta[j][i]);

        v_1[j][i] = (2 * R_d * DRONE_MAX_SPEED - a * S - b * S + sqrt((2 * R_d * DRONE_MAX_SPEED - a * S - b * S) * (2 * R_d * DRONE_MAX_SPEED - a * S - b * S) + 8 * R_d * S * (S * barrier_v + a * DRONE_MAX_SPEED + b * DRONE_MAX_SPEED))) / (4 * R_d * S);
        
        v_2[j][i] = (2 * R_d * DRONE_MAX_SPEED - a * S - b * S - sqrt((2 * R_d * DRONE_MAX_SPEED - a * S - b * S) * (2 * R_d * DRONE_MAX_SPEED - a * S - b * S) + 8 * R_d * S * (S * barrier_v + a * DRONE_MAX_SPEED + b * DRONE_MAX_SPEED))) / (4 * R_d * S);

        //最大速度の範囲内のものを選択
        if (v_1[j][i] * S < DRONE_MAX_SPEED && v_2[j][i] * S >= DRONE_MAX_SPEED) v[j][i] = v_1[j][i];
        else if (v_1[j][i] * S >= DRONE_MAX_SPEED && v_2[j][i] * S < DRONE_MAX_SPEED) v[j][i] = v_2[j][i];
        
        //printf("A=%f, B=%F\n", A, B);
        //printf("角度 %f\n",theta[j][i]);
        //printf("速度候補1 %f\n",v_1[j][i] * S);
        //printf("速度候補2 %f\n",v_2[j][i] * S);
        printf("速度 %f\n\n",v[j][i] * S);

        //
        //入力の決定
        u[2*j][i] = v[j][i] * cos(theta[j][i]) - x_d[2*j][i];
        u[2*j+1][i] = v[j][i] * sin(theta[j][i]) - x_d[2*j+1][i];
    }
    }
    printf("角度 %f\n",theta[2][0]);

    /*for (int i = 0; i < 6; i++){
        double A = f_t[2][i] - 2*R_d_2*x_d[2][i];
        double B = f_t[3][i] - 2*R_d_2*x_d[3][i];
        double M = sqrt(A*A + B*B);
        if (f_t[3][i] < 2*R_d_2*x_d[3][i]){//随伴ベクトルの現在時刻での値の大小関係  fが小ならu + x_dは正
            u[3][i] = 0.5 * (B * DRONE_MAX_SPEED / M - 0.5 * B / R_d_2 
                      + sqrt((B * DRONE_MAX_SPEED / M - 0.5 * B / R_d_2) * (B * DRONE_MAX_SPEED / M - 0.5 * B / R_d_2) + 2 * B * B * (DRONE_MAX_SPEED * M + 1) / R_d_2 / M / M)) 
                      - x_d[3][i];
        }

        else {//随伴ベクトルの現在時刻での値の大小関係  fが大ならu + x_dは負
            u[3][i] = 0.5 * (B * DRONE_MAX_SPEED / M - 0.5 * B / R_d_2 
                      - sqrt((B * DRONE_MAX_SPEED / M - 0.5 * B / R_d_2) * (B * DRONE_MAX_SPEED / M - 0.5 * B / R_d_2) + 2 * B * B * (DRONE_MAX_SPEED * M + 1) / R_d_2 / M / M)) 
                      - x_d[3][i];
        }

        u[2][i] = A / B * (u[3][i] + x_d[3][i]) - x_d[2][i];
    }

    for (int i = 0; i < 6; i++){
        double A = f_t[4][i] - 2*R_d_3*x_d[4][i];
        double B = f_t[5][i] - 2*R_d_3*x_d[5][i];
        double M = sqrt(A*A + B*B);
        if (f_t[5][i] < 2*R_d_3*x_d[5][i]){//随伴ベクトルの現在時刻での値の大小関係  fが小ならu + x_dは正
            u[5][i] = 0.5 * (B * DRONE_MAX_SPEED / M - 0.5 * B / R_d_3 
                      + sqrt((B * DRONE_MAX_SPEED / M - 0.5 * B / R_d_3) * (B * DRONE_MAX_SPEED / M - 0.5 * B / R_d_3) + 2 * B * B * (DRONE_MAX_SPEED * M + 1) / R_d_3 / M / M)) 
                      - x_d[5][i];
        }

        else {//随伴ベクトルの現在時刻での値の大小関係  fが大ならu + x_dは負
            u[5][i] = 0.5 * (B * DRONE_MAX_SPEED / M - 0.5 * B / R_d_3 
                      - sqrt((B * DRONE_MAX_SPEED / M - 0.5 * B / R_d_3) * (B * DRONE_MAX_SPEED / M - 0.5 * B / R_d_3) + 2 * B * B * (DRONE_MAX_SPEED * M + 1) / R_d_3 / M / M)) 
                      - x_d[5][i];
        }

        u[4][i] = A / B * (u[5][i] + x_d[5][i]) - x_d[4][i];
    }*/

    /*double x_d_1[6][6] = {0};
    double x_d_2[6][6] = {0};

    //改めて入力決定後の位置を予測
    for (int k = 0; k < 6; k++) {
    for (int i = 0; i < 6; i++) {

        // i=0 (時刻0) のときは積分ゼロなので初期値を入れるだけ
        if (i == 0) {
            x_d_1[k][i] = x[k];
            continue; // 次のループ (i=1) へ
        }

        // i > 0 のときだけ積分計算
        double sum = 0.5 * (exp(i * h) * u_1[k][0] + u_1[k][i]);

        for (int j = 1; j < i; j++) {
            sum += exp((i * h) - (j * h)) * u_1[k][j];
        }

        x_d_1[k][i] = exp(i * h) * x[k] + sum * h;
    }
    }

    //改めて入力決定後の位置を予測
    for (int k = 0; k < 6; k++) {
    for (int i = 0; i < 6; i++) {

        // i=0 (時刻0) のときは積分ゼロなので初期値を入れるだけ
        if (i == 0) {
            x_d_2[k][i] = x[k];
            continue; // 次のループ (i=1) へ
        }

        // i > 0 のときだけ積分計算
        double sum = 0.5 * (exp(i * h) * u_2[k][0] + u_2[k][i]);

        for (int j = 1; j < i; j++) {
            sum += exp((i * h) - (j * h)) * u_2[k][j];
        }

        x_d_2[k][i] = exp(i * h) * x[k] + sum * h;
    }
    }*/    

    //入力制限内での最適制御入力の決定
    cameradrones[0].ux = u[0][0];
    cameradrones[0].uy = u[1][0];
    cameradrones[1].ux = u[2][0];
    cameradrones[1].uy = u[3][0];
    drones[0].ux = u[4][0];
    drones[0].uy = u[5][0];

    printf("入力x%f, 入力y%f, ", cameradrones[0].ux, cameradrones[0].uy);

    /*
    // 入力制限
    for (int i = 0; i < NUM_CAMERADRONES; i++){
        float speed = sqrt(cameradrones[i].ux * cameradrones[i].ux + cameradrones[i].uy * cameradrones[i].uy);  // 現在の速度
        if (speed > DRONE_MAX_SPEED) {  // 最大速度を超えていれば
            cameradrones[i].ux = (cameradrones[i].ux / speed) * DRONE_MAX_SPEED;  // 速度を最大速度に調整
            cameradrones[i].uy = (cameradrones[i].uy / speed) * DRONE_MAX_SPEED;
        }
    }

    for (int i = 0; i < NUM_DRONES; i++){
        float speed = sqrt(drones[i].ux * drones[i].ux + drones[i].uy * drones[i].uy);  // 現在の速度
        if (speed > DRONE_MAX_SPEED) {  // 最大速度を超えていれば
            drones[i].ux = (drones[i].ux / speed) * DRONE_MAX_SPEED;  // 速度を最大速度に調整
            drones[i].uy = (drones[i].uy / speed) * DRONE_MAX_SPEED;
        }
    }
    */

    cameradrones[0].vx = (cameradrones[0].x * (exp(dt) - 1) + cameradrones[0].ux * (exp(dt) - 1)) / dt;
    cameradrones[0].vy = (cameradrones[0].y * (exp(dt) - 1) + cameradrones[0].uy * (exp(dt) - 1)) / dt;
    cameradrones[1].vx = (cameradrones[1].x * (exp(dt) - 1) + cameradrones[1].ux * (exp(dt) - 1)) / dt;
    cameradrones[1].vy = (cameradrones[1].y * (exp(dt) - 1) + cameradrones[1].uy * (exp(dt) - 1)) / dt;
    drones[0].vx = (drones[0].x * (exp(dt) - 1) + drones[0].ux * (exp(dt) - 1)) / dt;
    drones[0].vy = (drones[0].y * (exp(dt) - 1) + drones[0].uy * (exp(dt) - 1)) / dt;

    //ドローンの位置更新
    cameradrones[0].x = cameradrones[0].x * exp(dt) + cameradrones[0].ux * (exp(dt) - 1);
    cameradrones[0].y = cameradrones[0].y * exp(dt) + cameradrones[0].uy * (exp(dt) - 1);
    cameradrones[1].x = cameradrones[1].x * exp(dt) + cameradrones[1].ux * (exp(dt) - 1);
    cameradrones[1].y = cameradrones[1].y * exp(dt) + cameradrones[1].uy * (exp(dt) - 1);
    drones[0].x = drones[0].x * exp(dt) + drones[0].ux * (exp(dt) - 1);
    drones[0].y = drones[0].y * exp(dt) + drones[0].uy * (exp(dt) - 1);

    /*cameradrones[0].x += cameradrones[0].ux * dt;
    cameradrones[0].y += cameradrones[0].uy * dt;
    cameradrones[1].x += cameradrones[1].ux * dt;
    cameradrones[1].y += cameradrones[1].uy * dt;
    drones[0].x += drones[0].ux * dt;
    drones[0].y += drones[0].uy * dt;*/
    
    printf("位置(x,y)=%f,%f\n", cameradrones[0].x, cameradrones[0].y);

    printf("ドローン1速度%f\n", sqrt(cameradrones[0].vx*cameradrones[0].vx + cameradrones[0].vy*cameradrones[0].vy));
    printf("ドローン2速度%f\n", sqrt(cameradrones[1].vx*cameradrones[1].vx + cameradrones[1].vy*cameradrones[1].vy));
    printf("ドローン3速度%f\n", sqrt(drones[0].vx*drones[0].vx + drones[0].vy*drones[0].vy));

    for (int i = 0; i < 6; i++){
        for (int j = 0; j < 6; j++){
            if (j < 5) u[i][j] = u[i][j+1];
            u[i][j] = u[i][j];
        }
    }

    /*
    cameradrones[0].x = cameradrones[0].vx * dt;
    cameradrones[0].y = cameradrones[0].vy * dt;
    cameradrones[1].x = cameradrones[1].vx * dt;
    cameradrones[1].y = cameradrones[1].vy * dt;
    drones[0].x = drones[0].vx * dt;
    drones[0].y = drones[0].vy * dt;*/
}