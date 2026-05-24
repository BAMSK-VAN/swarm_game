#include <math.h>
#include "../include/render.h"
#include "../include/observation.h"

void draw_point_centered(SDL_Renderer *renderer, int x, int y) {
    // (0, 0) をウィンドウ中心とみなして描画
    SDL_RenderDrawPoint(renderer, WINDOW_WIDTH / 2 + x, WINDOW_HEIGHT / 2 - y);
}

// PCAに基づく楕円描画（太線バージョン）
void k_draw_ellipse(SDL_Renderer *renderer, double x[NUM]) {
    if(camera_found()){//2台以上のカメラドローンが発見した同じ鳥2羽の範囲で実行
    SDL_SetRenderDrawColor(renderer, 0, 255, 0, 255); // 緑で描画
    int thickness = 2;  // 太さ（1以上の整数）

    for (int i = 0; i < 360; i++) {
        double theta = i * PI / 180.0;
        double X = x[2] * cos(theta);
        double y = x[3] * sin(theta);

        // 回転行列で回す
        double xr = X * cos(x[4]) - y * sin(x[4]);
        double yr = X * sin(x[4]) + y * cos(x[4]);

        int px = (int)(x[0] + xr);
        int py = (int)(x[1] + yr);

        // 太さ分だけ周囲にも点を描画
        for (int dx = -thickness; dx <= thickness; dx++) {
            for (int dy = -thickness; dy <= thickness; dy++) {
                draw_point_centered(renderer, px + dx, py + dy);
            }
        }
    }
    }
}

// PCAに基づく楕円描画（太線バージョン）
void all_draw_ellipse(SDL_Renderer *renderer, double all_cx, double all_cy, double all_a, double all_b, double all_angle) {
    SDL_SetRenderDrawColor(renderer, 0, 255, 255, 255); // 緑で描画
    int thickness = 2;  // 太さ（1以上の整数）

    for (int i = 0; i < 360; i++) {
        double theta = i * PI / 180.0;
        double x = all_a * cos(theta);
        double y = all_b * sin(theta);

        // 回転行列で回す
        double xr = x * cos(all_angle) - y * sin(all_angle);
        double yr = x * sin(all_angle) + y * cos(all_angle);

        int px = (int)(all_cx + xr);
        int py = (int)(all_cy + yr);

        // 太さ分だけ周囲にも点を描画
        for (int dx = -thickness; dx <= thickness; dx++) {
            for (int dy = -thickness; dy <= thickness; dy++) {
                draw_point_centered(renderer, px + dx, py + dy);
            }
        }
    }
}

// PCAに基づく楕円描画（太線バージョン）
void draw_ellipse(SDL_Renderer *renderer) {
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255); // 緑で描画
    int thickness = 2;  // 太さ（1以上の整数）

    for (int i = 0; i < 360; i++) {
        double theta = i * PI / 180.0;
        double x = birdswarm.a * cos(theta);
        double y = birdswarm.b * sin(theta);

        // 回転行列で回す
        double xr = x * cos(birdswarm.angle) - y * sin(birdswarm.angle);
        double yr = x * sin(birdswarm.angle) + y * cos(birdswarm.angle);

        int px = (int)(birdswarm.cx + xr);
        int py = (int)(birdswarm.cy + yr);

        // 太さ分だけ周囲にも点を描画
        for (int dx = -thickness; dx <= thickness; dx++) {
            for (int dy = -thickness; dy <= thickness; dy++) {
                draw_point_centered(renderer, px + dx, py + dy);
            }
        }
    }
}

// PCAに基づく楕円描画（太線バージョン）
void z_draw_ellipse(SDL_Renderer *renderer, double z_cx, double z_cy, double z_a, double z_b, double z_angle) {
    SDL_SetRenderDrawColor(renderer, 0, 255, 200, 255); // 緑で描画
    int thickness = 2;  // 太さ（1以上の整数）

    for (int i = 0; i < 360; i++) {
        double theta = i * PI / 180.0;
        double x = z_a * cos(theta);
        double y = z_b * sin(theta);

        // 回転行列で回す
        double xr = x * cos(z_angle) - y * sin(z_angle);
        double yr = x * sin(z_angle) + y * cos(z_angle);

        int px = (int)(z_cx + xr);
        int py = (int)(z_cy + yr);

        // 太さ分だけ周囲にも点を描画
        for (int dx = -thickness; dx <= thickness; dx++) {
            for (int dy = -thickness; dy <= thickness; dy++) {
                draw_point_centered(renderer, px + dx, py + dy);
            }
        }
    }
}


// 保護区域の楕円描画（太線バージョン）
void draw_protect(SDL_Renderer *renderer) {
    SDL_SetRenderDrawColor(renderer, 0, 0, 255, 255); // 赤で描画
    int thickness = 1;  // 太さ（1以上の整数）

    for (int i = 0; i < 360; i++) {
        double theta = i * PI / 180.0;
        double x = PROTECTED_AREA_a * cos(theta);
        double y = PROTECTED_AREA_b * sin(theta);


        int px = (int)(PROTECTED_AREA_x + x);
        int py = (int)(PROTECTED_AREA_y + y);

        // 太さ分だけ周囲にも点を描画
        for (int dx = -thickness; dx <= thickness; dx++) {
            for (int dy = -thickness; dy <= thickness; dy++) {
                draw_point_centered(renderer, px + dx, py + dy);
            }
        }
    }
    }



// 鳥を描画する
void drawBirds(SDL_Renderer *renderer) {
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255); // 鳥の色を黒に設定
    for (int i = 0; i < NUM_BIRDS; i++) {
        // 鳥を黒い円で描画
        for (int dx = -BIRD_SIZE; dx <= BIRD_SIZE; dx++) {
            for (int dy = -BIRD_SIZE; dy <= BIRD_SIZE; dy++) {
                if (dx * dx + dy * dy <= BIRD_SIZE * BIRD_SIZE) {
                    draw_point_centered(renderer, birds[i].x + dx, birds[i].y + dy);
                }
            }
        }
    }
}

// カメラドローンの視野（FOV）を扇形で描画
/*void draw_fov(SDL_Renderer *renderer, CameraDrone cd, double length) {
    SDL_SetRenderDrawColor(renderer, 0, 0, 255, 255);  // 青い線

    double center_angle = cd.angle * PI / 180.0;          //ドローンカメラの初期中心角度
    double half_fov = cd.fov_angle_deg / 2.0 * PI / 180.0;//ドローンカメラの視野角の半分

    // 中心から視野の左右端の角度を計算
    double left_angle = center_angle - half_fov;
    double right_angle = center_angle + half_fov;

    // 視野の2本の端線を描く
    int x1 = (int)(cd.x + length * cos(left_angle));
    int y1 = (int)(cd.y + length * sin(left_angle));
    int x2 = (int)(cd.x + length * cos(right_angle));
    int y2 = (int)(cd.y + length * sin(right_angle));

    SDL_RenderDrawLine(renderer, (int)cd.x, (int)cd.y, x1, y1);
    SDL_RenderDrawLine(renderer, (int)cd.x, (int)cd.y, x2, y2);
}*/

void draw_fov(SDL_Renderer *renderer, CameraDrone cd, double length) {
    SDL_SetRenderDrawColor(renderer, 0, 0, 255, 255);  // 青い線

    // 画面中心の座標
    double offset_x = WINDOW_WIDTH / 2;
    double offset_y = WINDOW_HEIGHT / 2;

    double center_angle = cd.angle * PI / 180.0;
    double half_fov = cd.fov_angle_deg / 2.0 * PI / 180.0;

    // 中心から視野の左右端の角度を計算
    double left_angle = center_angle - half_fov;
    double right_angle = center_angle + half_fov;

    // ドローン本体の画面上での座標に変換
    double drone_scr_x = (int)(offset_x + cd.x);
    double drone_scr_y = (int)(offset_y - cd.y); // Y軸反転

    // 左端の点の画面座標を計算
    int x1 = (int)(offset_x + (cd.x + length * cos(left_angle)));
    int y1 = (int)(offset_y - (cd.y + length * sin(left_angle)));

    // 右端の点の画面座標を計算
    int x2 = (int)(offset_x + (cd.x + length * cos(right_angle)));
    int y2 = (int)(offset_y - (cd.y + length * sin(right_angle)));

    // 変換後の座標で線を描画
    SDL_RenderDrawLine(renderer, drone_scr_x, drone_scr_y, x1, y1);
    SDL_RenderDrawLine(renderer, drone_scr_x, drone_scr_y, x2, y2);
}

//目的地の描画
void drawTP(SDL_Renderer *renderer) {
    SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255); // TPの色（赤）

    for (int dx = -5; dx <= 5; dx++) {
            for (int dy = -5; dy <= 5; dy++) {
                if (dx * dx + dy * dy <= 5.0 * 5.0) {
                    draw_point_centered(renderer, targetX + dx, targetY + dy);
                }
            }
        }
    }