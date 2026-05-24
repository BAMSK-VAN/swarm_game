#include <SDL2/SDL.h>         // SDLによるウィンドウと描画の制御用
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

// 分割したヘッダファイルの読み込み
#include "../include/config.h"
#include "../include/types.h"
#include "../include/globals.h"
#include "../include/math_utils.h"
#include "../include/agents.h"
#include "../include/observation.h"
#include "../include/filter.h"
#include "../include/control.h"
#include "../include/render.h"
#include "../include/logger.h"

// ---------------------------------------------------------
// グローバル変数の「実体」をここで定義（メモリを確保）
// ---------------------------------------------------------
Bird birds[NUM_BIRDS];
BirdSwarm birdswarm;
Drone drones[NUM_DRONES];
CameraDrone cameradrones[NUM_CAMERADRONES];
EKFState EKF;
UKFState UKF;
Unknow unknow;
Bias bias;
float targetX = TARGETX, targetY = TARGETY; // 目標地点（TP）
double Q[GROUP][GROUP];                     // 密度データ

// ---------------------------------------------------------
// メイン関数
// ---------------------------------------------------------
int main() {
    // OBSを録画開始モードで起動（パスは自分の環境に合わせて書き換え）
    //system("cmd.exe /c start \"\" /d \"C:\\Program Files\\obs-studio\\bin\\64bit\" obs64.exe --startrecording --minimize-to-tray");

    SDL_Init(SDL_INIT_VIDEO); // SDLの初期化
    SDL_Window *window = SDL_CreateWindow("Birds+Drone+CameraDrone+PCA+KF",
        SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, WINDOW_WIDTH, WINDOW_HEIGHT, 0); // ウィンドウ作成
    SDL_Renderer *renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED); // レンダラ作成

    const int TARGET_FPS = 10;
    const int FRAME_DELAY = 1000 / TARGET_FPS; // 100ms

    Uint32 frameStart;
    int frameTime;

    srand(time(NULL)); // 乱数の初期化
    init_birds();      // 鳥初期化
    init_birdswarm();  // 鳥の群れ初期化
    init_drones();     // ドローン初期化
    init_cameradrones();     // カメラドローン初期化

    init_unknow();  //未知パラメータの初期値設定

    for (int i = 0; i < 4; i++) EKF.x[i] = 0.0; // カルマンフィルタ状態の初期化
    for (int i = 0; i < 4; i++) UKF.x[i] = 0.0; // カルマンフィルタ状態の初期化

    int done = 0;
    int pca_done = 0;
    double t = 0.0;
    double dt = 1.0 / 10.0;
    int first_call = 1;

    double x[NUM] = {0.0, 0.0, 0.0, 0.0, 0.0, 10.0, 10.0, 0.0, 0.0, 0.0};//, bias.b[0], bias.b[1], bias.b[2], bias.b[3], bias.b[4]}; // 状態ベクトル
    double x_UKF[NUM] = {0.0, 0.0, 0.0, 0.0, 0.0, 10.0, 10.0, 0.0, 0.0, 0.0};//, bias.b[0], bias.b[1], bias.b[2], bias.b[3], bias.b[4]}; // 状態ベクトル
    double x_10[10] = {0.0};//, bias.b[0], bias.b[1], bias.b[2], bias.b[3], bias.b[4]}; // 初期値を設定する
    double x_UKF_10[10] = {0.0};//, bias.b[0], bias.b[1], bias.b[2], bias.b[3], bias.b[4]}; // 初期値を設定する

    double p_[5] = {0.0};//パラメータ結局0

        double all_cx, all_cy, all_cov[2][2];
        double all_angle, all_a, all_b;
        double all_cvx, all_cvy, all_va, all_vb, all_vangle;

        double cov[2][2], cov_v[2][2], z_cx, z_cy, z_cov[2][2];
        double z_angle, z_a, z_b;
        double z[5];

        double angle_escort = 0;

        double u[6][6] = {0};//ドローンの制御入力　最初はドローンが静止しているようにする
        for (int i = 0; i < 6; i++){
            u[0][i] = -cameradrones[0].x - 1;
            u[1][i] = -cameradrones[0].y - 1;
        }
        for (int i = 0; i < 6; i++){
            u[2][i] = -cameradrones[1].x - 1;
            u[3][i] = -cameradrones[1].y - 1;
        }
        for (int i = 0; i < 6; i++){
            u[4][i] = -drones[0].x - 1;
            u[5][i] = -drones[0].y - 1;
        }

        //カメラ性能試験用宣言
        double u_camera[NUM_CAMERADRONES][NUM_BIRDS];
        double a[NUM_CAMERADRONES][NUM_BIRDS];
        double q[NUM_CAMERADRONES][GROUP];
        double Z[GROUP][GROUP];
        //

        double k = 0.0;//UKFのスケーリングパラメータ

        double P[NUM][NUM] = {
            {100.0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
            {0, 100.0, 0, 0, 0, 0, 0, 0, 0, 0},
            {0, 0, 100.0, 0, 0, 0, 0, 0, 0, 0},
            {0, 0, 0, 100.0, 0, 0, 0, 0, 0, 0},
            {0, 0, 0, 0, 1.0, 0, 0, 0, 0, 0},
            {0, 0, 0, 0, 0, 100.0, 0, 0, 0, 0},
            {0, 0, 0, 0, 0, 0, 100.0, 0, 0, 0},
            {0, 0, 0, 0, 0, 0, 0, 100.0, 0, 0},
            {0, 0, 0, 0, 0, 0, 0, 0, 100.0, 0},
            {0, 0, 0, 0, 0, 0, 0, 0, 0, 1.0}
        };

        /*double P_10[10][10] = {
            {100, 0, 0, 0, 0, 0, 0, 0, 0, 0},
            {0, 100, 0, 0, 0, 0, 0, 0, 0, 0},
            {0, 0, 100, 0, 0, 0, 0, 0, 0, 0},
            {0, 0, 0, 100, 0, 0, 0, 0, 0, 0},
            {0, 0, 0, 0, 1, 0, 0, 0, 0, 0},
            {0, 0, 0, 0, 0, 1e-2, 0, 0, 0, 0},
            {0, 0, 0, 0, 0, 0, 1e-2, 0, 0, 0},
            {0, 0, 0, 0, 0, 0, 0, 1e-2, 0, 0},
            {0, 0, 0, 0, 0, 0, 0, 0, 1e-2, 0},
            {0, 0, 0, 0, 0, 0, 0, 0, 0, 1e-4},
        };*/

        double P_UKF[NUM][NUM] = {
            {100.0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
            {0, 100.0, 0, 0, 0, 0, 0, 0, 0, 0},
            {0, 0, 100.0, 0, 0, 0, 0, 0, 0, 0},
            {0, 0, 0, 100.0, 0, 0, 0, 0, 0, 0},
            {0, 0, 0, 0, 1.0, 0, 0, 0, 0, 0},
            {0, 0, 0, 0, 0, 100.0, 0, 0, 0, 0},
            {0, 0, 0, 0, 0, 0, 100.0, 0, 0, 0},
            {0, 0, 0, 0, 0, 0, 0, 100.0, 0, 0},
            {0, 0, 0, 0, 0, 0, 0, 0, 100.0, 0},
            {0, 0, 0, 0, 0, 0, 0, 0, 0, 1.0}
        };

        /*double P_UKF_10[10][10] = {
            {100, 0, 0, 0, 0, 0, 0, 0, 0, 0},
            {0, 100, 0, 0, 0, 0, 0, 0, 0, 0},
            {0, 0, 100, 0, 0, 0, 0, 0, 0, 0},
            {0, 0, 0, 100, 0, 0, 0, 0, 0, 0},
            {0, 0, 0, 0, 1, 0, 0, 0, 0, 0},
            {0, 0, 0, 0, 0, 1e-2, 0, 0, 0, 0},
            {0, 0, 0, 0, 0, 0, 1e-2, 0, 0, 0},
            {0, 0, 0, 0, 0, 0, 0, 1e-2, 0, 0},
            {0, 0, 0, 0, 0, 0, 0, 0, 1e-2, 0},
            {0, 0, 0, 0, 0, 0, 0, 0, 0, 1e-4},
        };*/

        //Mの初期値設定、初期値が真値に近い自信がないとき大きくとる、もしくは、観測値が信用できない場合も大きくとる
        double M[NUM][NUM] = {0.0};
        double M_UKF[NUM][NUM] = {0.0};
        /*double M_10[10][10] = {0.0};
        double M_UKF_10[10][10] = {0.0};*/

        double W[NUM][NUM] = {
            {1e-1, 0, 0, 0, 0, 0, 0, 0, 0, 0},
            {0, 1e-1, 0, 0, 0, 0, 0, 0, 0, 0},
            {0, 0, 1e-1, 0, 0, 0, 0, 0, 0, 0},
            {0, 0, 0, 1e-1, 0, 0, 0, 0, 0, 0},
            {0, 0, 0, 0, 1e-3, 0, 0, 0, 0, 0},
            {0, 0, 0, 0, 0, 2e-1, 0, 0, 0, 0},
            {0, 0, 0, 0, 0, 0, 2e-1, 0, 0, 0},
            {0, 0, 0, 0, 0, 0, 0, 2e-1, 0, 0},
            {0, 0, 0, 0, 0, 0, 0, 0, 2e-1, 0},
            {0, 0, 0, 0, 0, 0, 0, 0, 0, 2e-3},
        };

        /*double W_10[10][10] = {
            {1, 0, 0, 0, 0, 0, 0, 0, 0, 0},
            {0, 1, 0, 0, 0, 0, 0, 0, 0, 0},
            {0, 0, 1, 0, 0, 0, 0, 0, 0, 0},
            {0, 0, 0, 1, 0, 0, 0, 0, 0, 0},
            {0, 0, 0, 0, 1e-2, 0, 0, 0, 0, 0},
            {0, 0, 0, 0, 0, 1e-4, 0, 0, 0, 0},
            {0, 0, 0, 0, 0, 0, 1e-4, 0, 0, 0},
            {0, 0, 0, 0, 0, 0, 0, 1e-4, 0, 0},
            {0, 0, 0, 0, 0, 0, 0, 0, 1e-4, 0},
            {0, 0, 0, 0, 0, 0, 0, 0, 0, 1e-6},
        };*/

        /*double W[NUM][NUM] = {
            {3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
            {0, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
            {0, 0, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
            {0, 0, 0, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
            {0, 0, 0, 0, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
            {0, 0, 0, 0, 0, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
            {0, 0, 0, 0, 0, 0, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
            {0, 0, 0, 0, 0, 0, 0, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
            {0, 0, 0, 0, 0, 0, 0, 0, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
            {0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
            {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0},
            {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 0, 0, 0, 0, 0, 0, 0, 0},
            {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 0, 0, 0, 0, 0, 0, 0},
            {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 0, 0, 0, 0, 0, 0},
            {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 0, 0, 0, 0, 0},
            {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 0, 0, 0, 0},
            {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 0, 0, 0},
            {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 0, 0},
            {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 0},
            {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3}
        };*/

        double V[5][5] = {
            {25.0, 0, 0, 0, 0},
            {0, 25.0, 0, 0, 0},
            {0, 0, 25.0, 0, 0},
            {0, 0, 0, 25.0, 0},
            {0, 0, 0, 0, 0.25}
        };

    int quit = 0;
    int manual_mode = 0; // 手動操作モード（0:自動制御, 1:手動操作）
    SDL_Event event;
    while (!quit) {
        frameStart = SDL_GetTicks(); // ループ開始時刻を取得
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT) quit = 1; // 終了イベント処理
        }

        update_birds(); // 鳥の移動更新
        
        update1_cameradrones(); // カメラドローンの視野・旋回更新
        update2_cameradrones(u_camera); // カメラドローンの視野・旋回更新

        if (done == 0){
            all_pca(&all_cx, &all_cy, all_cov);
            all_compute_rotation_and_axes(all_cov, &all_angle, &all_a, &all_b);

            done = 1;
        }
        
        else {
            x_10[0] = all_cx;//位置のの真値　kステップ目
            x_10[1] = all_cy;
            x_10[2] = all_a;
            x_10[3] = all_b;
            x_10[4] = all_angle;

            all_pca(&all_cx, &all_cy, all_cov);
            all_compute_rotation_and_axes(all_cov, &all_angle, &all_a, &all_b);

            //速度計算
            all_cvx = (all_cx - x_10[0]) / dt;//0.1秒
            all_cvy = (all_cy - x_10[1]) / dt;
            all_va = (all_a - x_10[2]) / dt;
            all_vb = (all_b - x_10[3]) / dt;
            all_vangle = (all_angle - x_10[4]) / dt;

            //未知パラメータの計算
            p_[0] = (((all_cx - all_cvx * dt)/x_10[0]) - 1) / dt;
            p_[1] = (((all_cy - all_cvy * dt)/x_10[1]) - 1) / dt;
            p_[2] = (((all_a - all_va * dt)/x_10[2]) - 1) / dt;
            p_[3] = (((all_b - all_vb * dt)/x_10[3]) - 1) / dt;
            p_[4] = (((all_angle - all_vangle * dt)/x_10[4]) - 1) / dt;
        }

        cameradrones_density(u_camera);
        cameradrones_density_check(a, q, Z);
        
    if (camera_found()) {//2台以上のカメラドローンが同じ鳥を発見すれば実行
        pca(cov);
        compute_rotation_and_axes(cov);//実際

        z_pca(&z_cx, &z_cy, z_cov);//観測での重心と共分散
        z_compute_rotation_and_axes(z_cov, &z_angle, &z_a, &z_b);//観測

        update3_cameradrones(all_cx, all_cy); // カメラドローンの視野・旋回更新

        //観測ベクトル(一旦カメラモデルは通していない，真値にノイズがのっているものとする)
        z[0] = all_cx + rand_normal(0.0,5.0);// 観測値
        z[1] = all_cy + rand_normal(0.0,5.0);
        z[2] = all_a + rand_normal(0.0,5.0);
        z[3] = all_b + rand_normal(0.0,5.0);
        z[4] = all_angle + rand_normal(0.0,0.5);

        /*z[0] = birdswarm.cx + rand_normal(0.0,5.0);// 観測値
        z[1] = birdswarm.cy + rand_normal(0.0,5.0);
        z[2] = birdswarm.a + rand_normal(0.0,5.0);
        z[3] = birdswarm.b + rand_normal(0.0,5.0);
        z[4] = birdswarm.angle + rand_normal(0.0,0.5);*/
        
        /*z[0] = z_cx + rand_normal(0.0,5.0);// 観測値
        z[1] = z_cy + rand_normal(0.0,5.0);
        z[2] = z_a + rand_normal(0.0,5.0);
        z[3] = z_b + rand_normal(0.0,5.0);
        z[4] = z_angle + rand_normal(0.0,0.5);*/

        if (pca_done == 0){

            x[0] = z[0];// 初期値に観測値を与える観測値
            x[1] = z[1];
            x[2] = z[2];
            x[3] = z[3];
            x[4] = z[4];

            x_UKF[0] = z[0];// 初期値に観測値を与える観測値
            x_UKF[1] = z[1];
            x_UKF[2] = z[2];
            x_UKF[3] = z[3];
            x_UKF[4] = z[4];

            pca_done = 1;
        }
        
        else {
            kalman_estimate_EKF(x, z, P, M, W, V, dt); // カルマン予測   
            kalman_estimate_UKF(x_UKF, z, P_UKF, M_UKF, W, V, dt,k); // カルマン予測
        }
       
        optimal_control_u(u, x_UKF, &angle_escort);
    }

    else printf("birdswarm not found\n");

    // キーボードの現在の状態を配列として取得
    const Uint8 *state = SDL_GetKeyboardState(NULL);
            
    // 速度の設定
    double normal_speed = DRONE_MAX_SPEED * 0.5; // ゆっくり（通常の半分の速度）
    double dash_speed   = DRONE_MAX_SPEED * 1.0; // ダッシュ（最大速度オーバー）
    double current_speed = normal_speed;

    // 左Shift または 右Shift が押されているか判定
    if (state[SDL_SCANCODE_LSHIFT] || state[SDL_SCANCODE_RSHIFT]) {
        current_speed = dash_speed;
    }
            
    drones[1].vx = 0.0;
    drones[1].vy = 0.0;

    // 矢印キーで速度を決定（斜め移動も可能）
    if (state[SDL_SCANCODE_UP])    drones[1].vy =  current_speed;
    if (state[SDL_SCANCODE_DOWN])  drones[1].vy = -current_speed;
    if (state[SDL_SCANCODE_LEFT])  drones[1].vx = -current_speed;
    if (state[SDL_SCANCODE_RIGHT]) drones[1].vx =  current_speed;

    // MPCで計算された位置を破棄し、手動入力で上書き更新
    drones[1].x = drones[1].x + drones[1].vx * dt;
    drones[1].y = drones[1].y + drones[1].vy * dt;
            
    // ログや次ステップの計算がおかしくならないように入力 u も上書きしておく
    drones[1].ux = drones[1].vx;
    drones[1].uy = drones[1].vy;
        

        SDL_SetRenderDrawColor(renderer, 225, 225, 225, 255); // 背景色（白）
        SDL_RenderClear(renderer); // 描画クリア

        // 鳥を描画
        drawBirds(renderer);

        //保護区域の描画
        draw_protect(renderer);

        //目的地の描画
        drawTP(renderer);

        //ドローンを描画
        for (int i = 0; i < 1; i++) {
            SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255); // ドローンの色（赤）
            SDL_Rect r = {WINDOW_WIDTH / 2 + (int)drones[i].x - DRONE_SIZE / 2, WINDOW_WIDTH / 2 - (int)drones[i].y - DRONE_SIZE / 2, DRONE_SIZE, DRONE_SIZE};
            SDL_RenderFillRect(renderer, &r); // ドローンを四角形で描画
        }
        for (int i = 1; i < NUM_DRONES; i++) {
            SDL_SetRenderDrawColor(renderer, 0, 255, 0, 255); // ドローンの色（赤）
            SDL_Rect r = {WINDOW_WIDTH / 2 + (int)drones[i].x - DRONE_SIZE / 2, WINDOW_WIDTH / 2 - (int)drones[i].y - DRONE_SIZE / 2, DRONE_SIZE, DRONE_SIZE};
            SDL_RenderFillRect(renderer, &r); // ドローンを四角形で描画
        }
        //カメラドローンを描画
        for (int i = 0; i < NUM_CAMERADRONES; i++) {
            SDL_SetRenderDrawColor(renderer, 0, 0, 255, 255); // ドローンの色（青）
            SDL_Rect r = {WINDOW_WIDTH / 2 + (int)cameradrones[i].x - DRONE_SIZE / 2, WINDOW_WIDTH / 2 - (int)cameradrones[i].y - DRONE_SIZE / 2, DRONE_SIZE, DRONE_SIZE};
            SDL_RenderFillRect(renderer, &r); // ドローンを四角形で描画
        }

        //ドローンカメラの視野角を描写
        for (int i = 0; i < NUM_CAMERADRONES; i++) {
        SDL_Rect r = {WINDOW_WIDTH / 2 + (int)cameradrones[i].x - DRONE_SIZE / 2, WINDOW_WIDTH / 2 - (int)cameradrones[i].y - DRONE_SIZE / 2, DRONE_SIZE, DRONE_SIZE};
        SDL_RenderFillRect(renderer, &r);

        // 🔽 視野角（FOV）を描画
        draw_fov(renderer, cameradrones[i], 100);  // 100ピクセルの長さ
        }

        // PCAに基づく楕円を描画
        k_draw_ellipse(renderer, x_UKF);//カルマンフィルター適用PCA

        //draw_ellipse(renderer);
        all_draw_ellipse(renderer, all_cx, all_cy, all_a, all_b, all_angle);

        //z_draw_ellipse(renderer, z_cx, z_cy, z_a, z_b, z_angle); //EKFとの比較用観測データそのものから楕円推定

        SDL_RenderPresent(renderer); // 描画を反映

        simulate_unknow(t, x, "unknown.csv", first_call);
        estimate_swarm(t, "estimate_swarm.csv", first_call, all_cx, all_cy, all_a, all_b, all_angle, z[0], z[1], z[2], z[3], z[4], x);

        simulate_unknow(t, x_UKF, "unknown_UKF.csv", first_call);
        estimate_swarm(t, "estimate_swarm_UKF.csv", first_call, all_cx, all_cy, all_a, all_b, all_angle, z[0], z[1], z[2], z[3], z[4], x_UKF);

        double t_ = t - dt;
        simulate_vec(t_, "velocity.csv", first_call, all_cvx, all_cvy, all_va, all_vb, all_vangle);
        simulate_p(t_, p_, "parameter.csv", first_call);

        drone_route(t, "drone_route.csv", first_call, angle_escort);
        
        /*check_camera(t, u_camera, a, "camera_check.csv", first_call);
        check_camera_q(t, q, "camera_check_q.csv", first_call);
        check_camera_Q(t, Z, "camera_check_Q_score.csv", first_call);
        */

        first_call = 0;
        t += dt;

        /*
        printf("bird[1].ax=%f\n",birds[1].ax);
        printf("bird[1].vx=%f\n",birds[1].vx);
        */

        //printf("%f\n",P[0][0]);

        // かかった時間を計算
        frameTime = SDL_GetTicks() - frameStart;

        // もし16msより早く終わったら、残りの時間だけ待つ
        if (FRAME_DELAY > frameTime) {
            SDL_Delay(FRAME_DELAY - frameTime);
        }
    }

    //system("cmd.exe /c taskkill /IM obs64.exe");

    SDL_DestroyRenderer(renderer); // 後始末
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0; // 正常終了
}