#ifndef GLOBALS_H
#define GLOBALS_H

#include "config.h"
#include "types.h"

// グローバル変数の宣言（実体は main.c に配置）
extern Bird birds[NUM_BIRDS];
extern BirdSwarm birdswarm;
extern Drone drones[NUM_DRONES];
extern CameraDrone cameradrones[NUM_CAMERADRONES];
extern EKFState EKF;
extern UKFState UKF;
extern Unknow unknow;
extern Bias bias;
extern float targetX;
extern float targetY;
extern double Q[GROUP][GROUP];

#endif // GLOBALS_H