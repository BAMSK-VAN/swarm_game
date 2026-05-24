#include <math.h>
#include <stdlib.h>
#include "../include/agents.h"
#include "../include/math_utils.h"

// init_birds
// 鳥の初期化（位置と速度に乱数を与える）
void init_birds() {
    double mu_x = -300.0;     // x方向の平均
    double mu_y = 300.0;    // y方向の平均
    double sigma_x = 10.0;         // x方向の標準偏差
    double sigma_y = 10.0;         // y方向の標準偏差

    for (int i = 0; i < NUM_BIRDS; i++) {
        birds[i].x = rand_normal(mu_x, sigma_x);
        birds[i].y = rand_normal(mu_y, sigma_y);
        birds[i].vx = randFloat(-MAX_SPEED, MAX_SPEED);
        birds[i].vy = randFloat(-MAX_SPEED, MAX_SPEED);
        birds[i].ax = 0.0;
        birds[i].ay = 0.0;
    }
}

// init_birdswarm
// 鳥の群れの初期化（位置と速度に乱数を与える）
void init_birdswarm() {
        birdswarm.cx = 0.0;
        birdswarm.cy = 0.0;
        birdswarm.cvx = 0.0;
        birdswarm.cvy = 0.0;

        birdswarm.a = 0.0;
        birdswarm.b = 0.0;
        birdswarm.angle = 0.0;
        birdswarm.va = 0.0;
        birdswarm.vb = 0.0;
        birdswarm.vangle = 0.0;
}

// init_drones
// ドローンの初期化（位置、角度、視野角、検出状態）
void init_drones() {
    for (int i = 0; i < NUM_DRONES; i++) {
        drones[i].x = PROTECTED_AREA_x;
        drones[i].y = PROTECTED_AREA_y;
        drones[i].ux = 0.0;
        drones[i].uy = 0.0;
        drones[i].vx = 0.0;
        drones[i].vy = 0.0;
    }
}

// init_cameradrones
// カメラドローンの初期化（位置、角度、視野角、検出状態）
void init_cameradrones() {
        /*cameradrones[0].x = PROTECTED_AREA_x + 20.0;
        cameradrones[0].y = PROTECTED_AREA_y - 20.0;
        cameradrones[1].x = PROTECTED_AREA_x - 20.0;
        cameradrones[1].y = PROTECTED_AREA_y + 20.0;*/
        cameradrones[0].x = PROTECTED_AREA_x + 0.0;
        cameradrones[0].y = PROTECTED_AREA_y + 50.0;
        cameradrones[1].x = PROTECTED_AREA_x - 0.0;
        cameradrones[1].y = PROTECTED_AREA_y - 50.0;
    for (int i = 0; i < NUM_CAMERADRONES; i++) {
        cameradrones[i].ux = 0.0;
        cameradrones[i].uy = 0.0;
        cameradrones[i].vx = 0.0;
        cameradrones[i].vy = 0.0;
        cameradrones[i].angle = rand() % 360;//ドローンカメラの初期角度
        cameradrones[i].fov_angle_deg = 60.0;//ドローンカメラの視野角
        cameradrones[i].found = 0.0;
        cameradrones[i].w = DRONE_ANGULARVELOCITY;//カメラドローンの角速度
        cameradrones[i].m = GROUP;//視野角を10分割

        //映像平面のピクセル数
        cameradrones[i].U = 1920;
        cameradrones[i].V = 1080;

        //映像平面の大きさ[m]
        cameradrones[i].Iw = 0.01;
        cameradrones[i].Ih = 0.0075;

        //ピンホールから映像平面までの距離(焦点距離[m]) レンズ角度を60度に
        cameradrones[i].f = 0.005 * sqrt(3);

        for (int j = 0; j < cameradrones[i].m; j++)
        cameradrones[i].q[j] = 0;
    }
}

// init_unknow
//拡張カルマンフィルタに用いる未知パラメータの設定
//未知パラメータの初期値設定
void init_unknow() {
        for (int i = 0; i < 5; i++){
            unknow.x[i] = 0.0;
        }
}

// init_bias
void init_bias(){
    for (int i = 0; i < 5; i++){
            bias.b[i] = 0.0;
        }    
}

// update_birds
// 鳥の位置を速度に応じて更新
void update_birds() {
    // 各鳥の行動を更新するループ
    for (int i = 0; i < NUM_BIRDS; i++) {
        // 変数の初期化（それぞれの鳥の動きに必要な要素）
        float avgVx = 0.0, avgVy = 0.0;    // 整列のための速度の平均
        float centerX = 0.0, centerY = 0.0;  // 群れの中心（結合のための位置の平均）
        float separationX = 0.0, separationY = 0.0;  // 分離のためのベクトル
        float avoidX = 0.0, avoidY = 0.0;  // ドローン回避のためのベクトル
        int count_bird = 0.0, count_drone = 0.0;  // 近くの鳥の数
        float xTodrone = 0.0, yTodrone = 0.0;  //ドローンと鳥の距離
        float AVOID_WEIGHT = 0.0; // ドローン回避の影響度

        // 近くの鳥を探索
        for (int j = 0; j < NUM_BIRDS; j++) {
            if (i==j) continue;  // 同じ鳥には影響を与えない
            float dx = birds[i].x - birds[j].x;  // 鳥iと鳥jのx軸の距離
            float dy = birds[i].y - birds[j].y;  // 鳥iと鳥jのy軸の距離
            float dist = sqrt(dx * dx + dy * dy);  // 2点間の距離(ノルム)

            if (dist < VISION_RADIUS) {  // Ri範囲内なら
                centerX += birds[j].x;  // 鳥jの位置を結合のために加算
                centerY += birds[j].y;
                avgVx += birds[j].vx;  // 鳥jの速度を整列のための平均速度に加算
                avgVy += birds[j].vy;
                separationX += dx/(dist*dist);  // 鳥iと鳥jの反発ベクトルを分離のために加算
                separationY += dy/(dist*dist);
                count_bird++;  // 視野内の鳥をカウント
            }
        }

        // 近くのドローンを探索と回避準備
        for (int j = 0; j < NUM_DRONES; j++) {
            float dx = birds[i].x - drones[j].x;  // 鳥iとドローンjのx軸の距離
            float dy = birds[i].y - drones[j].y;  // 鳥iとドローンjのy軸の距離
            float dist = sqrt(dx * dx + dy * dy);  // 2点間の距離(ノルム)

            float dxTodrone = drones[j].x - birds[i].x;  // ドローンjと鳥iのx軸の差
            float dyTodrone = drones[j].y - birds[i].y;  // ドローンjと鳥iのy軸の差

            if (dist < DRONE_AVOID_RADIUS) {  // ドローンの回避範囲内
                avoidX += dx/(dist*dist);  // 回避ベクトルを加算
                avoidY += dy/(dist*dist);

                xTodrone += dxTodrone/(dist*dist);  //すべてのドローンと鳥iの合計ベクトル
                yTodrone += dyTodrone/(dist*dist);

                //AVOID_WEIGHT += rand_normal_pdf(dist, 0.0, 10.0); // ドローン回避の影響度・・・・・正規分布に従うようにした方がいい？
                AVOID_WEIGHT += AVOID * rand_normal_pdf(dist, 0.0, DRONE_AVOID_RADIUS / 2);

                count_drone++;  //範囲内のドローンの数
            }
        }

        // 近くのカメラドローンを探索と回避準備
        for (int j = 0; j < NUM_CAMERADRONES; j++) {
            float dx = birds[i].x - cameradrones[j].x;  // 鳥iとドローンjのx軸の距離
            float dy = birds[i].y - cameradrones[j].y;  // 鳥iとドローンjのy軸の距離
            float dist = sqrt(dx * dx + dy * dy);  // 2点間の距離(ノルム)

            float dxTocameradrone = cameradrones[j].x - birds[i].x;  // ドローンjと鳥iのx軸の差
            float dyTocameradrone = cameradrones[j].y - birds[i].y;  // ドローンjと鳥iのy軸の差

            if (dist < DRONE_AVOID_RADIUS) {  // ドローンの回避範囲内
                avoidX += dx/(dist*dist);  // 回避ベクトルを加算
                avoidY += dy/(dist*dist);

                xTodrone += dxTocameradrone/(dist*dist);  //すべてのドローンと鳥iの合計ベクトル
                yTodrone += dyTocameradrone/(dist*dist);

                //AVOID_WEIGHT += rand_normal_pdf(dist, 0.0, 10.0); // ドローン回避の影響度・・・・・正規分布に従うようにした方がいい？
                AVOID_WEIGHT += AVOID * rand_normal_pdf(dist, 0.0, DRONE_AVOID_RADIUS / 2);

                count_drone++;  //範囲内のドローンの数
            }
        }

            // ドローンを横に回避
            if (count_drone > 0) {  // 近すぎるドローンがいる場合
            avoidX /= count_drone;  // 範囲内のドローンとの平均反発ベクトルを求める
            avoidY /= count_drone;
            xTodrone /= count_drone;  // 範囲内のドローンとの平均ベクトルを求める
            yTodrone /= count_drone;

            float dxToTarget = targetX - birds[i].x;  // 目標地点と鳥iのx軸の差
            float dyToTarget = targetY - birds[i].y;  // 目標地点と鳥iのy軸の差

            float dist_drone = sqrt(xTodrone * xTodrone + yTodrone * yTodrone);  // 鳥iとドローンのノルム
            float dist_Target = sqrt(dxToTarget * dxToTarget + dyToTarget * dyToTarget);  // 鳥iと目的地のノルム

            float xTodrone_n = xTodrone / dist_drone;        //それぞれ正規化
            float yTodrone_n = yTodrone / dist_drone;
            float dxToTarget_n = dxToTarget / dist_Target;
            float dyToTarget_n = dyToTarget / dist_Target;

            if ((dxToTarget_n * xTodrone_n + dyToTarget_n * yTodrone_n) >= 0){ //目的地とドローンへのベクトルのなす角が90度以下
            if (((yTodrone_n*dxToTarget_n - xTodrone_n*dyToTarget_n)/(dxToTarget_n*dxToTarget_n + dyToTarget_n*dyToTarget_n)) <= 0){ //左に避ける判断(二つのなす角のsinを用いる(正規化しないといけない))
            float theta = -90.0 * (1 + (yTodrone_n*dxToTarget_n - xTodrone_n*dyToTarget_n)/(dxToTarget_n*dxToTarget_n + dyToTarget_n*dyToTarget_n));
            float avoidX_x , avoidY_y;
            
            rotateVector(avoidX , avoidY , theta , &avoidX_x , &avoidY_y ); //ベクトルを回転

            float dist = sqrt(avoidX_x * avoidX_x + avoidY_y * avoidY_y);  // 反発ベクトルのノルム

            float avoid_vX = avoidX_x / dist * MAX_SPEED - birds[i].vx;  // avoid速度
            float avoid_vY = avoidY_y / dist * MAX_SPEED - birds[i].vy;
            float dist_v = sqrt(avoid_vX * avoid_vX + avoid_vY * avoid_vY);  // 速度のノルム

            // 加速度加算
            if (dist_v < MAX_ACCELERATION){
            birds[i].ax += avoid_vX * AVOID_WEIGHT / count_drone;
            birds[i].ay += avoid_vY * AVOID_WEIGHT / count_drone;
            }
            else{
            birds[i].ax += avoid_vX / dist_v * MAX_ACCELERATION * AVOID_WEIGHT / count_drone;
            birds[i].ay += avoid_vY / dist_v * MAX_ACCELERATION * AVOID_WEIGHT / count_drone;
            }
            }
            
            else{              //右に避ける
            float theta = 90.0 * (1 - (yTodrone_n*dxToTarget_n - xTodrone_n*dyToTarget_n)/(dxToTarget_n*dxToTarget_n + dyToTarget_n*dyToTarget_n));
            float avoidX_x , avoidY_y;
                    
            rotateVector(avoidX , avoidY , theta , &avoidX_x , &avoidY_y ); //ベクトルを回転

            float dist = sqrt(avoidX_x * avoidX_x + avoidY_y * avoidY_y);  // 反発ベクトルのノルム

            float avoid_vX = avoidX_x / dist * MAX_SPEED - birds[i].vx;  // avoid速度
            float avoid_vY = avoidY_y / dist * MAX_SPEED - birds[i].vy;
            float dist_v = sqrt(avoid_vX * avoid_vX + avoid_vY * avoid_vY);  // 速度のノルム

            // 加速度加算
            if (dist_v < MAX_ACCELERATION){
            birds[i].ax += avoid_vX * AVOID_WEIGHT / count_drone;
            birds[i].ay += avoid_vY * AVOID_WEIGHT / count_drone;
            }
            else{
            birds[i].ax += avoid_vX / dist_v * MAX_ACCELERATION * AVOID_WEIGHT / count_drone;
            birds[i].ay += avoid_vY / dist_v * MAX_ACCELERATION * AVOID_WEIGHT / count_drone;
            }
            }
            }

            else {  //なす角が90度より大きい  鳥の方が目的地に近い

                float escapeX = avoidX;  //逃げるベクトル//dxToTarget + 
                float escapeY = avoidY; //dyToTarget + 

                float dist = sqrt(escapeX * escapeX + escapeY * escapeY);  // 反発ベクトルのノルム

                float escape_vX = escapeX / dist * MAX_SPEED - birds[i].vx;  // escape速度
                float escape_vY = escapeY / dist * MAX_SPEED - birds[i].vy;
                float dist_v = sqrt(escape_vX * escape_vX + escape_vY * escape_vY);  // 速度のノルム
    
                // 加速度加算
                if (dist_v < MAX_ACCELERATION){
                birds[i].ax += escape_vX * AVOID_WEIGHT / count_drone;
                birds[i].ay += escape_vY * AVOID_WEIGHT / count_drone;
                }
                else{
                birds[i].ax += escape_vX / dist_v * MAX_ACCELERATION * AVOID_WEIGHT / count_drone;
                birds[i].ay += escape_vY / dist_v * MAX_ACCELERATION * AVOID_WEIGHT / count_drone;
                }
            }
            }
        


        // 結合
        if (count_bird > 0) {  // 視野範囲内の鳥がいる場合
            centerX /= count_bird;  // 鳥の平均位置を求める
            centerY /= count_bird;
            float cohesion_x = (centerX - birds[i].x) ;  // 位置を結合方向に調整
            float cohesion_y = (centerY - birds[i].y) ;
            float dist = sqrt(cohesion_x * cohesion_x + cohesion_y * cohesion_y);  // 2点間の距離(ノルム)

            float cohesion_vx = cohesion_x / dist * MAX_SPEED - birds[i].vx;  // cohesion速度
            float cohesion_vy = cohesion_y / dist * MAX_SPEED - birds[i].vy;
            float dist_v = sqrt(cohesion_vx * cohesion_vx + cohesion_vy * cohesion_vy);  // 速度のノルム

            // 加速度加算
            if (dist_v < MAX_ACCELERATION){
            birds[i].ax += cohesion_vx * COHESION_WEIGHT;
            birds[i].ay += cohesion_vy * COHESION_WEIGHT;
            }
            else{
            birds[i].ax += cohesion_vx / dist_v * MAX_ACCELERATION * COHESION_WEIGHT;
            birds[i].ay += cohesion_vy / dist_v * MAX_ACCELERATION * COHESION_WEIGHT;
            }
        }

        // 整列
        if (count_bird > 0) {  // 視野範囲内の鳥がいる場合
            avgVx /= count_bird;  // 範囲内の鳥の平均速度を求める
            avgVy /= count_bird;
            float alignment_x = (avgVx) ;  // 進行方向ベクトル  - birds[i].vx
            float alignment_y = (avgVy) ;
            float dist = sqrt(alignment_x * alignment_x + alignment_y * alignment_y);  // 進行方向ベクトルのノルム

            float alignment_vx = alignment_x / dist * MAX_SPEED - birds[i].vx;  // alignment速度
            float alignment_vy = alignment_y / dist * MAX_SPEED - birds[i].vy;
            float dist_v = sqrt(alignment_vx * alignment_vx + alignment_vy * alignment_vy);  // 速度のノルム

            // 加速度加算
            if (dist_v < MAX_ACCELERATION){
            birds[i].ax += alignment_vx * ALIGNMENT_WEIGHT;
            birds[i].ay += alignment_vy * ALIGNMENT_WEIGHT;
            }
            else{
            birds[i].ax += alignment_vx / dist_v * MAX_ACCELERATION * ALIGNMENT_WEIGHT;
            birds[i].ay += alignment_vy / dist_v * MAX_ACCELERATION * ALIGNMENT_WEIGHT;
            }
        }

        // 分離
        if (count_bird > 0) {  // 近すぎる鳥がいる場合
            separationX /= count_bird;  // 範囲内の鳥の平均反発ベクトルを求める
            separationY /= count_bird;
            float dist = sqrt(separationX * separationX + separationY * separationY);  // 反発ベクトルのノルム

            float separation_vx = separationX / dist * MAX_SPEED - birds[i].vx;  // separation速度
            float separation_vy = separationY / dist * MAX_SPEED - birds[i].vy;
            float dist_v = sqrt(separation_vx * separation_vx + separation_vy * separation_vy);  // 速度のノルム

            // 加速度加算
            if (dist_v < MAX_ACCELERATION){
            birds[i].ax += separation_vx * SEPARATION_WEIGHT;
            birds[i].ay += separation_vy * SEPARATION_WEIGHT;
            }
            else{
            birds[i].ax += separation_vx / dist_v * MAX_ACCELERATION * SEPARATION_WEIGHT;
            birds[i].ay += separation_vy / dist_v * MAX_ACCELERATION * SEPARATION_WEIGHT;
            }
        }

        

        // 目標地点に向かうベクトルを計算
        float dxToTarget = targetX - birds[i].x;  // 目標地点と鳥iのx軸の差
        float dyToTarget = targetY - birds[i].y;  // 目標地点と鳥iのy軸の差
        float distToTarget = sqrt(dxToTarget * dxToTarget + dyToTarget * dyToTarget);  // 目標地点までの距離

        float vxToTarget = dxToTarget / distToTarget * MAX_SPEED - birds[i].vx;  // 目標へ向かう速度ベクトル
        float vyToTarget = dyToTarget / distToTarget * MAX_SPEED - birds[i].vy;
        float distToTarget_v = sqrt(vxToTarget * vxToTarget + vyToTarget * vyToTarget);  // 速度のノルム

        // 加速度加算
        if (distToTarget_v < MAX_ACCELERATION){
        birds[i].ax += vxToTarget * TP_WEIGHT;
        birds[i].ay += vyToTarget * TP_WEIGHT;
        }
        else{
        birds[i].ax += vxToTarget / distToTarget_v * MAX_ACCELERATION * TP_WEIGHT;
        birds[i].ay += vyToTarget / distToTarget_v * MAX_ACCELERATION * TP_WEIGHT;
        }

        // 加速度制限
        float accele = sqrt(birds[i].ax * birds[i].ax + birds[i].ay * birds[i].ay);  // 現在の速度
        if (accele > MAX_ACCELERATION) {  // 最大速度を超えていれば
            birds[i].ax = (birds[i].ax / accele) * MAX_ACCELERATION;  // 速度を最大速度に調整
            birds[i].ay = (birds[i].ay / accele) * MAX_ACCELERATION;
        }


        //鳥の速度と位置の更新
        // 積分したい関数 fx(t) = birds[i].ax, fy(t) = birds[i].ay
        double dt = 1.0/10.0;

        birds[i].vx += birds[i].ax * dt;
        birds[i].vy += birds[i].ay * dt;


        // 速度制限
        float speed = sqrt(birds[i].vx * birds[i].vx + birds[i].vy * birds[i].vy);  // 現在の速度
        if (speed > MAX_SPEED) {  // 最大速度を超えていれば
            birds[i].vx = (birds[i].vx / speed) * MAX_SPEED;  // 速度を最大速度に調整
            birds[i].vy = (birds[i].vy / speed) * MAX_SPEED;
        }

        birds[i].x += birds[i].vx * dt;
        birds[i].y += birds[i].vy * dt;

    }
}