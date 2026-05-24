#include <math.h>
#include <stdio.h>
#include "../include/observation.h"
#include "../include/math_utils.h"

// カメラドローンの視野内にBirdが入っているかを判定する関数
int in_fov(CameraDrone cd, Bird b) {
    double dx = b.x - cd.x; // Boidのx座標とドローンのx座標の差（ベクトルのx成分） 焦点距離を加味
    double dy = b.y - cd.y; // Boidのy座標とドローンのy座標の差（ベクトルのy成分）

    // atan2で鳥への角度（ドローンから見た方向）を求め、度数に変換
    double angle_to_bird = atan2(dy, dx) * 180.0 / PI;

    // Boidの方向とドローンの向いている方向の角度差を計算
    double diff = angle_to_bird - cd.angle;

    // 差が180度を超える場合は反対周りに修正（例：350度と10度は20度差）
    while (diff > 180) diff -= 360.0;
    while (diff < -180) diff += 360.0;

    // 絶対値を取って、視野角の半分より小さければ視野内と判断（true:1）
    return fabs(diff) <= cd.fov_angle_deg / 2.0;
}

void cameradrones_density(double u[NUM_CAMERADRONES][NUM_BIRDS]){
    for (int i = 0; i < NUM_CAMERADRONES; i++){
        for (int j = 0; j < cameradrones[i].m; j++){
        cameradrones[i].q[j] = 0.0;
        }
    }

    //初期化
    double sum[NUM_CAMERADRONES] = {0.0};

    for (int i = 0; i < NUM_CAMERADRONES; i++) {
        for (int j = 0; j < NUM_BIRDS; j++){
        if(in_fov(cameradrones[i], birds[j])){

        //ピンホールカメラが正確とすれば，ほぼ真値
        double dx = birds[j].x - cameradrones[i].x; // Boidのx座標とドローンのx座標の差（ベクトルのx成分）
        double dy = birds[j].y - cameradrones[i].y; // Boidのy座標とドローンのy座標の差（ベクトルのy成分）

        // atan2で鳥への角度（ドローンから見た方向）を求め、度数に変換
        double angle_to_bird = atan2(dy, dx) * 180.0 / PI; //-π～π

        // Boidの方向とドローンの向いている方向の角度差を計算(Φの計算)
        double diff = angle_to_bird - cameradrones[i].angle; //(-π～π)-(0～2π)

        double diff_rad = diff * PI / 180.0;
        //画像上での位置
        u[i][j] = cameradrones[i].f * tan(diff_rad) + cameradrones[i].Iw / 2.0;// + rand_trunc_normal(0.0, 9.0, -0.005, 0.005);

        //M分割
        double group = cameradrones[i].Iw / cameradrones[i].m;

        //M分割の中のどこに入っているか
        int num = u[i][j] / group;

        //計算のずれでカメラがいなってたら，端に写ってるとみなす
        if (num < 0) num = 0;
        if (num >= cameradrones[i].m) num = cameradrones[i].m - 1;

        //M分割した入っている番号に追加
        cameradrones[i].q[num] += 1.0;

        //カメラに写った合計値
        sum[i] += 1.0;
        }
    }
    }

    for (int i = 0; i < NUM_CAMERADRONES; i++){
        for (int j = 0; j < cameradrones[i].m; j++){
            cameradrones[i].q[j] /= sum[i];
        }
    }

    for (int i = 0; i < cameradrones[0].m; i++) {
        for (int j = 0; j < cameradrones[1].m; j++){
            Q[i][j] = 0;
            Q[i][j] = cameradrones[0].q[i] * cameradrones[1].q[j];
        }
    }
}

void cameradrones_density_check(double a[NUM_CAMERADRONES][NUM_BIRDS], double q[NUM_CAMERADRONES][GROUP], double Z[GROUP][GROUP]){
    for (int i = 0; i < NUM_CAMERADRONES; i++){
        for (int j = 0; j < cameradrones[i].m; j++){
        q[i][j] = 0;
        }
    }


    for (int i = 0; i < NUM_CAMERADRONES; i++) {
        for (int j = 0; j < NUM_BIRDS; j++){
        if(in_fov(cameradrones[i], birds[j])){

        double dx = birds[j].x - cameradrones[i].x; // Boidのx座標とドローンのx座標の差（ベクトルのx成分）
        double dy = birds[j].y - cameradrones[i].y; // Boidのy座標とドローンのy座標の差（ベクトルのy成分）

        // atan2で鳥への角度（ドローンから見た方向）を求め、度数に変換
        double angle_to_bird = atan2(dy, dx) * 180.0 / PI;

        // Boidの方向とドローンの向いている方向の角度差を計算(Φの計算)
        double diff = angle_to_bird - cameradrones[i].angle;

        double diff_rad = diff * PI / 180.0;
        //画像上での位置(ノイズあり)10分割
        a[i][j] = cameradrones[i].f * tan(diff_rad) + cameradrones[i].Iw / 2.0;

        double group = cameradrones[i].Iw / cameradrones[i].m;

        int num = a[i][j] / group;

        if (num < 0) num = 0;
        if (num >= cameradrones[i].m) num = cameradrones[i].m - 1;

        q[i][num] += 1;
        }
    }
    }

    for (int i = 0; i < cameradrones[0].m; i++) {
        for (int j = 0; j < cameradrones[1].m; j++){
            Z[i][j] = 0;
            Z[i][j] = q[0][i] * q[1][j];
        }
    }
}

// 2台以上のドローンの密度スコアで視野内に収めているか判定
int all_cameradrones_found() {
    int seen_count = 0;

    for (int i = 0; i < cameradrones[0].m; i++) {
        for (int j = 0; j < cameradrones[1].m; j++){

            //ラジアンと度に気を付ける
                    double a = cameradrones[0].angle - cameradrones[0].fov_angle_deg / 2.0 + (2.0 * i + 1.0) * cameradrones[0].fov_angle_deg / (2.0 * cameradrones[0].m); //密度データの位置角度
                    double b = cameradrones[1].angle - cameradrones[1].fov_angle_deg / 2.0 + (2.0 * j + 1.0) * cameradrones[1].fov_angle_deg / (2.0 * cameradrones[1].m);

                    double a_rad = a * PI / 180.0;
                    double b_rad = b * PI / 180.0;

                    double dx = cameradrones[1].x - cameradrones[0].x;
                    double dy = cameradrones[1].y - cameradrones[0].y;

                    double d0x = cos(a_rad);
                    double d0y = sin(a_rad);
                    double d1x = cos(b_rad);
                    double d1y = sin(b_rad);

                    // 外積
                    double cross = d0x * d1y - d0y * d1x;  // = sin(b - a)
                    if (fabs(cross) < 1e-9) continue;

                    // 交点のスカラー t0
                    double t0 = (dx * d1y - dy * d1x) / cross;
                    double t1 = (dx * d0y - dy * d0x) / cross;

                    // 向きに沿って交差してるか判定
                    if (t0 <= 0 || t1 <= 0) {
                        // 観測方向とは逆側で交差している → 実質「交わらない」のでスキップ
                        continue;
                    }

        else{
            Q[i][j] = cameradrones[0].q[i] * cameradrones[1].q[j];

        if (Q[i][j] >= DENSITY){
            seen_count++;
            if (seen_count >= 1) return 1; // 3箇所見えていればOK　pcaの計算が3点ないとバグるかもやから保険で3点見つかるまで実行しない
        }
        }

        }
    }
    return 0;
}

//テスト用
// 2台以上のドローンが、同じ鳥（特定のインデックス b_index）を視野内に収めているか判定
int all_camera_found(Bird b) {
    int seen_count = 0;

    for (int i = 0; i < NUM_CAMERADRONES; i++) {
        if (in_fov(cameradrones[i], b)) {
            seen_count++;
            if (seen_count >= 2) return 1; // 2つ以上で見えていればOK
        }
    }
    return 0;
}
int camera_found() {
    int found_count = 0;
    for (int i = 0; i < NUM_BIRDS; i++) {
        if (all_camera_found(birds[i])) {
            found_count++;
            if (found_count >= 1) return 1;  // 同じ鳥3羽以上が2台以上に見つかった
        }
    }
    return 0;  // 条件未達成
}


// カメラドローンの状態更新1（鳥が視野内にいるか探す）
void update1_cameradrones() {
    double dt=1.0/10.0;
    for (int i = 0; i < NUM_CAMERADRONES; i++) {

        // ドローンがBoidを視野内に見つけたかを判定
        for (int j = 0; j < NUM_BIRDS; j++) {
            if (in_fov(cameradrones[i], birds[j])) {
                cameradrones[i].found = 1;
                break; // 見つかったので探索終了
            }
        }
        

        if (cameradrones[i].found == 0) {
            // 視野内にBoidがいなければ探し続ける（視野を回転）
            cameradrones[i].angle += cameradrones[i].w * dt;
            while (cameradrones[i].angle >= 360.0) cameradrones[i].angle -= 360.0;
            while (cameradrones[i].angle <= 360.0) cameradrones[i].angle += 360.0;
        }
    }
}

// カメラドローンの状態更新2（鳥が視野内にいたら群れの中心へ旋回）
void update2_cameradrones(double u[NUM_CAMERADRONES][NUM_BIRDS]) {
    double dt=1.0/10.0;

        cameradrones_density(u); //cameradrones[i].q[j]を初期化

        if (all_cameradrones_found()) return;

        // カメラドローン中心に鳥がくるようにする
        for (int i = 0; i < NUM_CAMERADRONES; i++) {
            double sum = 0.0;
            int count = 0;
            double com = 0.0;
            for (int j = 0; j < cameradrones[i].m; j++){
                if (cameradrones[i].q[j] > 0){
                    sum += j;
                    count += 1;
                }
            }

        if (count > 0) {
            com = sum / count;
        
            // 角度差を求める（-180〜180度に正規化）
            double diff = (2.0 * com + 1.0) * cameradrones[i].fov_angle_deg / (2.0 * cameradrones[i].m) - cameradrones[i].fov_angle_deg / 2.0;
            while (diff > 180) diff -= 360;
            while (diff < -180) diff += 360;

            // 徐々に鳥の位置平均方向に近づける（最大 ±角速度*微小時間）
            if (fabs(diff) > cameradrones[i].w * dt) {//絶対値
                cameradrones[i].angle += (diff > 0 ? cameradrones[i].w * dt : -cameradrones[i].w * dt );
                while (cameradrones[i].angle >= 360.0) cameradrones[i].angle -= 360.0;
                while (cameradrones[i].angle <= 360.0) cameradrones[i].angle += 360.0;
            } else {
                cameradrones[i].angle += diff; // ドローンの角速度*dt未満ならそのまま合わせる
                while (cameradrones[i].angle >= 360.0) cameradrones[i].angle -= 360.0;
                while (cameradrones[i].angle <= 360.0) cameradrones[i].angle += 360.0;
            }
        }
    }
}

// カメラドローンの状態更新3（楕円の中心へ旋回）
void update3_cameradrones(double z_cx, double z_cy){
    // 群れの中心（PCAで計算される）
    double dt=1.0/10.0;
    
    if (all_cameradrones_found()) {
    for (int i = 0; i < NUM_CAMERADRONES; i++) { 
            
            //2台のカメラで計算が可能(同じ鳥を発見)になれば実行

            // 群れの中心方向に向くように回転（atan2で向き計算）

            double dx = z_cx - cameradrones[i].x;
            double dy = z_cy - cameradrones[i].y;
            double target_angle = atan2(dy, dx) * 180.0 / PI;;
            
            // 角度差を求める（-180〜180度に正規化）
            double diff = target_angle - cameradrones[i].angle;
            while (diff > 180) diff -= 360;
            while (diff < -180) diff += 360;

            // 徐々に群れ中心方向に近づける（最大 ±角速度*微小時間）
            if (fabs(diff) > cameradrones[i].w * dt) {//絶対値
                cameradrones[i].angle += (diff > 0 ? cameradrones[i].w * dt : -cameradrones[i].w * dt );
                while (cameradrones[i].angle >= 360.0) cameradrones[i].angle -= 360.0;
                while (cameradrones[i].angle <= 360.0) cameradrones[i].angle += 360.0;
            } else {
                cameradrones[i].angle += diff; // ドローンの角速度*dt未満ならそのまま合わせる
                while (cameradrones[i].angle >= 360.0) cameradrones[i].angle -= 360.0;
                while (cameradrones[i].angle <= 0.0) cameradrones[i].angle += 360.0;
            }
        }
    }
}

//カメラ範囲内での鳥の位置は正確に得られているとする
// PCA：重心(cx, cy)と共分散行列covを計算
void pca(double cov[2][2]) {
    double mx = 0.0, my = 0.0 , count = 0;
    for (int i = 0; i < NUM_BIRDS; i++) {
        if (all_camera_found(birds[i])) {
            mx += birds[i].x;
            my += birds[i].y;

            count += 1;
        }
    }
    if (count == 0) return;  // 該当Boidがいなければ処理しない

    mx /= count; //実際の値
    my /= count;
    birdswarm.cx = mx;
    birdswarm.cy = my;

    double sxx = 0.0, syy = 0.0, sxy = 0.0;
    for (int i = 0; i < NUM_BIRDS; i++) {
        if(all_camera_found(birds[i])){//2台以上のカメラドローンが発見した同じ鳥の範囲で実行
        double dx = birds[i].x - mx;//実際の値
        double dy = birds[i].y - my;

        sxx += dx * dx;
        syy += dy * dy;
        sxy += dx * dy;
    }
    }
    cov[0][0] = sxx / count;
    cov[0][1] = cov[1][0] = sxy / count;
    cov[1][1] = syy / count;

}

// PCA：重心(cx, cy)と共分散行列covを計算
void pca_v(double cov_v[2][2]) {
    double mx = 0.0, my = 0.0 , count = 0.0;
    for (int i = 0; i < NUM_BIRDS; i++) {
        
            mx += birds[i].vx;
            my += birds[i].vx;

            count += 1;
        }
    
    if (count == 0) return;  // 該当Boidがいなければ処理しない

    mx /= count; //実際の値
    my /= count;
    birdswarm.cvx = mx;
    birdswarm.cvy = my;

    double sxx = 0.0, syy = 0.0, sxy = 0.0;
    for (int i = 0; i < NUM_BIRDS; i++) {
        
        double dx = birds[i].vx - mx;//実際の値
        double dy = birds[i].vy - my;

        sxx += dx * dx;
        syy += dy * dy;
        sxy += dx * dy;
    }
    cov_v[0][0] = sxx / count;
    cov_v[0][1] = cov_v[1][0] = sxy / count;
    cov_v[1][1] = syy / count;
}

/* カメラの範囲内でPCA計算，鳥の位置は正確に得られてる
void pca_test(double u[NUM_CAMERADRONES][NUM_BIRDS], double *test_cx, double *test_cy, double test_cov[2][2]) {
    double mx = 0, my = 0 , count = 0;

    for (int j = 0; j < NUM_BIRDS; j++) {
        if (all_camera_found(birds[j])) {
        mx += u[0][j];
        my += u[1][j];

        count += 1;
        }
    }
    
    if (count == 0) return;  // 該当Boidがいなければ処理しない

    mx /= count; //実際の値
    my /= count;
    

    double sxx = 0, syy = 0, sxy = 0;
    for (int i = 0; i < NUM_BIRDS; i++) {
        
        double dx = birds[i].x - mx;//実際の値
        double dy = birds[i].y - my;

        sxx += dx * dx;
        syy += dy * dy;
        sxy += dx * dy;
    }
    cov[0][0] = sxx / count;
    cov[0][1] = cov[1][0] = sxy / count;
    cov[1][1] = syy / count;
}
*/

// PCA：観測重心(z_cx, z_cy)と共分散行列z_covを計算
void z_pca(double *z_cx, double *z_cy, double z_cov[2][2]) {
    double c_x = 0.0, c_y = 0.0;
    double a = 0.0, b = 0.0, a_rad = 0.0, b_rad = 0.0; 
    double sum_x = 0.0, sum_y = 0.0, sum_w = 0.0;

    if (all_cameradrones_found()) {
        for (int i = 0; i < cameradrones[0].m; i++) {
            for (int j = 0; j < cameradrones[1].m; j++){
                if (Q[i][j] >= DENSITY){

                    //三角測量
                    //ラジアンと度に気を付ける
                    a = cameradrones[0].angle - cameradrones[0].fov_angle_deg / 2.0 + (2.0 * i + 1.0) * cameradrones[0].fov_angle_deg / (2.0 * cameradrones[0].m); //密度データの位置角度
                    b = cameradrones[1].angle - cameradrones[1].fov_angle_deg / 2.0 + (2.0 * j + 1.0) * cameradrones[1].fov_angle_deg / (2.0 * cameradrones[1].m);

                    a_rad = a * PI / 180.0;
                    b_rad = b * PI / 180.0;

                    double dx = cameradrones[1].x - cameradrones[0].x;
                    double dy = cameradrones[1].y - cameradrones[0].y;

                    double d0x = cos(a_rad);
                    double d0y = sin(a_rad);
                    double d1x = cos(b_rad);
                    double d1y = sin(b_rad);

                    // 外積
                    double cross = d0x * d1y - d0y * d1x;  // = sin(b - a)
                    if (fabs(cross) < 1e-9) continue;

                    // 交点のスカラー t0,t1
                    double t0 = (dx * d1y - dy * d1x) / cross;
                    double t1 = (dx * d0y - dy * d0x) / cross;

                    // 向きに沿って交差してるか判定
                    if (t0 <= 0 || t1 <= 0) {
                        // 観測方向とは逆側で交差している → 実質「交わらない」のでスキップ
                        continue;
                    }

                    c_x = cameradrones[0].x + t0 * d0x;
                    c_y = cameradrones[0].y + t0 * d0y;

                    double w = Q[i][j]; 
                    
                    sum_x += c_x * w;
                    sum_y += c_y * w;
                    sum_w += w;

                    /*m_cx += c_x;
                    m_cy += c_y;

                    count += 1;*/
                }
            }
        }
    }
    if (sum_w == 0) return;  // 該当Boidがいなければ処理しない

    *z_cx = sum_x / sum_w;
    *z_cy = sum_y / sum_w;


    double z_sxx = 0.0, z_syy = 0.0, z_sxy = 0.0, a0 = 0.0, b0 = 0.0, a0_rad = 0.0, b0_rad = 0.0, x = 0.0, y = 0.0;

    if(all_cameradrones_found() && sum_w > 0){//2台以上のカメラドローンが発見した同じ鳥の範囲で実行
        for (int i = 0; i < cameradrones[0].m; i++) {
            for (int j = 0; j < cameradrones[1].m; j++){
                if (Q[i][j] >= DENSITY){
                    a0 = cameradrones[0].angle - cameradrones[0].fov_angle_deg / 2.0 + (2.0 * i + 1.0) * cameradrones[0].fov_angle_deg / (2.0 * cameradrones[0].m); //密度データの位置角度
                    b0 = cameradrones[1].angle - cameradrones[1].fov_angle_deg / 2.0 + (2.0 * j + 1.0) * cameradrones[1].fov_angle_deg / (2.0 * cameradrones[1].m);

                    a0_rad = a0 * PI / 180.0;
                    b0_rad = b0 * PI / 180.0;

                    double dx = cameradrones[1].x - cameradrones[0].x;
                    double dy = cameradrones[1].y - cameradrones[0].y;

                    double d0x = cos(a0_rad);
                    double d0y = sin(a0_rad);
                    double d1x = cos(b0_rad);
                    double d1y = sin(b0_rad);

                    // 外積
                    double cross = d0x * d1y - d0y * d1x;  // = sin(b - a)
                    if (fabs(cross) < 1e-9) continue;

                    // 交点のスカラー t0
                    double t0 = (dx * d1y - dy * d1x) / cross;
                    double t1 = (dx * d0y - dy * d0x) / cross;

                    // 向きに沿って交差してるか判定
                    if (t0 <= 0 || t1 <= 0) {
                        // 観測方向とは逆側で交差している → 実質「交わらない」のでスキップ
                        continue;
                    }

                    x = cameradrones[0].x + t0 * d0x;
                    y = cameradrones[0].y + t0 * d0y;
                    
                    double w = Q[i][j];
                    double z_dx = x - *z_cx;//観測の値
                    double z_dy = y - *z_cy;
                    z_sxx += w * z_dx * z_dx;
                    z_syy += w * z_dy * z_dy;
                    z_sxy += w * z_dx * z_dy;

                    /*z_sxx += z_dx * z_dx;
                    z_syy += z_dy * z_dy;
                    z_sxy += z_dx * z_dy;*/
                }

            }
        }
    }
    z_cov[0][0] = z_sxx / sum_w;
    z_cov[0][1] = z_cov[1][0] = z_sxy / sum_w;
    z_cov[1][1] = z_syy / sum_w;

    printf("共分散行列\n%f, %f\n%f, %f\n",z_cov[0][0], z_cov[0][1], z_cov[1][0], z_cov[1][1]);
}


// PCA：重心(cx, cy)と共分散行列covの時間関数を計算
void pca_t(double t, double *cx_t, double *cy_t, double cov_t[2][2]) {
    double mx = 0.0, my = 0.0 , count = 0;
    for (int i = 0; i < NUM_BIRDS; i++) {
        if (all_camera_found(birds[i])) {
            mx += birds[i].x + birds[i].vx * t + birds[i].ax * t * t / 2.0; //x(t) = x(t0)+v(t0)*t+a(t0)*t*t/2
            my += birds[i].y + birds[i].vy * t + birds[i].ay * t * t / 2.0;

            count += 1;
        }
    }
    if (count == 0) return;  // 該当Boidがいなければ処理しない

    mx /= count; //実際の値
    my /= count;
    *cx_t = mx;
    *cy_t = my;

    double sxx = 0.0, syy = 0.0, sxy = 0.0;
    for (int i = 0; i < NUM_BIRDS; i++) {
        if(all_camera_found(birds[i])){//2台以上のカメラドローンが発見した同じ鳥の範囲で実行
        double dx = birds[i].x + birds[i].vx * t + birds[i].ax * t * t / 2.0 - mx;
        double dy = birds[i].y + birds[i].vy * t + birds[i].ay * t * t / 2.0 - my;

        sxx += dx * dx;
        syy += dy * dy;
        sxy += dx * dy;
    }
    }
    cov_t[0][0] = sxx / count;
    cov_t[0][1] = cov_t[1][0] = sxy / count;
    cov_t[1][1] = syy / count;

}


void all_pca(double *all_cx, double *all_cy, double all_cov[2][2]) {
    double mx = 0.0, my = 0.0 , count = 0;
    for (int i = 0; i < NUM_BIRDS; i++) {
            mx += birds[i].x;
            my += birds[i].y;

            count += 1;
    }

    mx /= count; //実際の値
    my /= count;
    *all_cx = mx;
    *all_cy = my;

    double sxx = 0.0, syy = 0.0, sxy = 0.0;
    for (int i = 0; i < NUM_BIRDS; i++) {
        double dx = birds[i].x - mx;//実際の値
        double dy = birds[i].y - my;

        sxx += dx * dx;
        syy += dy * dy;
        sxy += dx * dy;
    }
    all_cov[0][0] = sxx / count;
    all_cov[0][1] = all_cov[1][0] = sxy / count;
    all_cov[1][1] = syy / count;

}

// 固有ベクトルと回転角を求める（主成分方向の単位ベクトルを返す）
void compute_rotation_and_axes(double cov[2][2]) {
    double eig1, eig2 , eigv1[2][1] , eigv2[2][1];
    eigen2x2(cov[0][0], cov[0][1], cov[1][1], &eig1, &eig2 , eigv1 , eigv2);
    
    // 主成分に対応する固有ベクトル（大きい固有値に対応）
    double vx = eigv1[0][0];
    double vy = eigv1[1][0];

    // ---------------------------------------------------------
    // 【重要】PCA特有の「180度反転問題」の対策
    // 固有ベクトルは向き（±）が定まらないため、前回の向きと逆なら反転させる
    // ---------------------------------------------------------
    double prev_vx = cos(birdswarm.angle);
    double prev_vy = sin(birdswarm.angle);
    
    // 内積が負なら、逆方向を向いているのでベクトルをひっくり返す
    if (vx * prev_vx + vy * prev_vy < 0) {
        vx = -vx;
        vy = -vy;
    }

    // ---------------------------------------------------------
    // 角度の連続化（アンラッピング）処理
    // ---------------------------------------------------------
    double current_raw_angle = atan2(vy, vx); // -π ～ π の値
    double diff = current_raw_angle - birdswarm.angle; // 前回からの変化量

    // 変化量 diff を -π ～ π の範囲に補正する（最短経路を選ぶ）
    while (diff > M_PI)  diff -= 2.0 * M_PI;
    while (diff <= -M_PI) diff += 2.0 * M_PI;

    // 前回の角度に「補正した変化量」を足すことで、連続的な値にする
    birdswarm.angle += diff;
    
    //birdswarm.angle = atan2(vy, vx);

    birdswarm.a = sqrt(eig1 * 5.991); // 長軸（スケーリング）
    birdswarm.b = sqrt(eig2 * 5.991); // 短軸
}

//使わん
void compute_rotation_and_axes_v(double cov_v[2][2]) {
    double eig1, eig2 , eigv1[2][1] , eigv2[2][1];
    eigen2x2(cov_v[0][0], cov_v[0][1], cov_v[1][1], &eig1, &eig2 , eigv1 , eigv2);
    
    // 主成分に対応する固有ベクトル（大きい固有値に対応）
    double vx = eigv1[0][0];
    double vy = eigv1[1][0];
    
    birdswarm.vangle = atan2(vy, vx);

    birdswarm.va = sqrt(eig1 * 5.991); // 長軸（スケーリング）
    birdswarm.vb = sqrt(eig2 * 5.991); // 短軸
}

void z_compute_rotation_and_axes(double z_cov[2][2], double *z_angle_rad, double *z_a, double *z_b) {
    double z_eig1, z_eig2 , z_eigv1[2][1] , z_eigv2[2][1];
    eigen2x2(z_cov[0][0], z_cov[0][1], z_cov[1][1], &z_eig1, &z_eig2 , z_eigv1 , z_eigv2);

    // 主成分に対応する固有ベクトル（大きい固有値に対応）観測データに基づく
    double z_vx = z_eigv1[0][0];
    double z_vy = z_eigv1[1][0];
    
    // ---------------------------------------------------------
    // 【重要】PCA特有の「180度反転問題」の対策
    // 固有ベクトルは向き（±）が定まらないため、前回の向きと逆なら反転させる
    // ---------------------------------------------------------
    double prev_vx = cos(*z_angle_rad);
    double prev_vy = sin(*z_angle_rad);
    
    // 内積が負なら、逆方向を向いているのでベクトルをひっくり返す
    if (z_vx * prev_vx + z_vy * prev_vy < 0) {
        z_vx = -z_vx;
        z_vy = -z_vy;
    }

    // ---------------------------------------------------------
    // 角度の連続化（アンラッピング）処理
    // ---------------------------------------------------------
    double current_raw_angle = atan2(z_vy, z_vx); // -π ～ π の値
    double diff = current_raw_angle - *z_angle_rad; // 前回からの変化量

    // 変化量 diff を -π ～ π の範囲に補正する（最短経路を選ぶ）
    while (diff > M_PI)  diff -= 2.0 * M_PI;
    while (diff <= -M_PI) diff += 2.0 * M_PI;

    // 前回の角度に「補正した変化量」を足すことで、連続的な値にする
    *z_angle_rad += diff;

    //*z_angle_rad = atan2(z_vy, z_vx);
        //printf("角度%f\n",*z_angle_rad);

    *z_a = sqrt(z_eig1 * 5.991); // 長軸（スケーリング）
    *z_b = sqrt(z_eig2 * 5.991); // 短軸
    //printf("固有値　大%f\n",z_eig1);
    //printf("固有値　小%f\n",z_eig2);
    //printf("長軸%f\n",*z_a);
    //printf("短軸%f\n",*z_b);
}

//使わん
// 固有ベクトルと回転角の時間関数を求める（主成分方向の単位ベクトルを返す）
void compute_rotation_and_axes_t(double cov_t[2][2], double *angle_rad_t, double *a_t, double *b_t) {
    double eig1_t, eig2_t, eigv1_t[2][1], eigv2_t[2][1];
    eigen2x2(cov_t[0][0], cov_t[0][1], cov_t[1][1], &eig1_t, &eig2_t, eigv1_t, eigv2_t);
    
    // 主成分に対応する固有ベクトル（大きい固有値に対応）
    double vx_t = eigv1_t[0][0];
    double vy_t = eigv1_t[1][0];
    
        *angle_rad_t = atan2(vy_t, vx_t);

    *a_t = sqrt(eig1_t * 5.991); // 長軸（スケーリング）
    *b_t = sqrt(eig2_t * 5.991); // 短軸
}


void all_compute_rotation_and_axes(double all_cov[2][2], double *all_angle_rad, double *all_a, double *all_b) {
    double all_eig1, all_eig2 , all_eigv1[2][1] , all_eigv2[2][1];
    eigen2x2(all_cov[0][0], all_cov[0][1], all_cov[1][1], &all_eig1, &all_eig2 , all_eigv1 , all_eigv2);

    // 主成分に対応する固有ベクトル（大きい固有値に対応）観測データに基づく
    double all_vx = all_eigv1[0][0];
    double all_vy = all_eigv1[1][0];
    
    // ---------------------------------------------------------
    // 【重要】PCA特有の「180度反転問題」の対策
    // 固有ベクトルは向き（±）が定まらないため、前回の向きと逆なら反転させる
    // ---------------------------------------------------------
    double prev_vx = cos(*all_angle_rad);
    double prev_vy = sin(*all_angle_rad);
    
    // 内積が負なら、逆方向を向いているのでベクトルをひっくり返す
    if (all_vx * prev_vx + all_vy * prev_vy < 0) {
        all_vx = -all_vx;
        all_vy = -all_vy;
    }

    // ---------------------------------------------------------
    // 角度の連続化（アンラッピング）処理
    // ---------------------------------------------------------
    double current_raw_angle = atan2(all_vy, all_vx); // -π ～ π の値
    double diff = current_raw_angle - *all_angle_rad; // 前回からの変化量

    // 変化量 diff を -π ～ π の範囲に補正する（最短経路を選ぶ）
    while (diff > M_PI)  diff -= 2.0 * M_PI;
    while (diff <= -M_PI) diff += 2.0 * M_PI;

    // 前回の角度に「補正した変化量」を足すことで、連続的な値にする
    *all_angle_rad += diff;

    //*all_angle_rad = atan2(all_vy, all_vx);

    *all_a = sqrt(all_eig1 * 5.991); // 長軸（スケーリング）
    *all_b = sqrt(all_eig2 * 5.991); // 短軸
}