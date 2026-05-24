#ifndef TYPES_H
#define TYPES_H

#include "config.h"

// 鳥の構造体
typedef struct {
    float x, y;   // 位置
    float vx, vy; // 速度
    float ax, ay; // 加速度
} Bird;

// 鳥の群れ
typedef struct {
    float cx, cy;   // 位置
    float cvx, cvy; // 速度
    float a, b, angle;
    float va, vb, vangle;
} BirdSwarm;

// ドローンの構造体（位置）
typedef struct {
    double x, y;
    double ux, uy;
    double vx, vy;
} Drone;

// ドローンの構造体（位置、入力、速度、角度、角速度、視野角、角度グループ,検出フラグ、　カメラ情報）
typedef struct {
    double x, y;
    double ux, uy;
    double vx, vy;
    double angle;
    double w;
    double fov_angle_deg;
    int m;
    int found;

    double U; //横方向のピクセル数
    double V; //縦方向のピクセル数

    double Iw; //(m)横方向の画像長さ
    double Ih; //縦方向の画像長さアスペクト比(4:3)

    double f; //焦点距離

    //密度データ
    double q[GROUP];
} CameraDrone;

// カルマンフィルタ用の状態構造体（位置・速度と共分散）
typedef struct {
    double x[5];
    double P[5][5]; // 共分散行列（未使用）
    double k_cx;
    double k_cy;
    double k_a;
    double k_b;
    double k_angle; 
} EKFState;

// カルマンフィルタ用の状態構造体（位置・速度と共分散）
typedef struct {
    double x[5];
    double P[5][5]; // 共分散行列（未使用）
    double k_cx;
    double k_cy;
    double k_a;
    double k_b;
    double k_angle; 
} UKFState;

// 未知パラメータ(システム方程式ｘ(k+1)=Ax(k)+Bu(k)+w(k)のAの中に存在)
typedef struct {
    double x[5]; 
} Unknow;

typedef struct {
    double b[5]; 
} Bias;

#endif // TYPES_H