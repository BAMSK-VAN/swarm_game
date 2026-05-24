#ifndef CONFIG_H
#define CONFIG_H

#define TARGETX 400             // 目標位置
#define TARGETY -300
#define NUM_BIRDS 50            // 鳥の個体数
#define NUM_DRONES 2            // ドローンの数
#define NUM_CAMERADRONES 2      // カメラドローンの数
#define PI 3.14159265358979323846
#define BIRD_SIZE 3
#define DRONE_SIZE 7
#define GROUP 10         //カメラのグループ分割数

//鳥のパラメータ
#define MAX_SPEED 20.0       // 鳥の最大速度
#define MAX_ACCELERATION 20.0       // 鳥の最大加速度

//鳥モデルのパラメータ
#define VISION_RADIUS 100.0  // 視野範囲
#define DRONE_AVOID_RADIUS 100.0 // ドローン回避半径
/*#define ALIGNMENT_WEIGHT 0.0  // 整列の影響度
#define COHESION_WEIGHT 0.0    // 結合の影響度
#define SEPARATION_WEIGHT 0.0   // 分離の影響度
#define TP_WEIGHT 5          // 目標地点への影響度
#define AVOID 5              //ドローンからの回避の影響度*/

#define ALIGNMENT_WEIGHT 1.0  // 整列の影響度
#define COHESION_WEIGHT 3.0    // 結合の影響度
#define SEPARATION_WEIGHT 2.5   // 分離の影響度
#define TP_WEIGHT 12          // 目標地点への影響度
#define AVOID 25              //ドローンからの回避の影響度

//ドローンのパラメータ
#define DRONE_ANGULARVELOCITY 180.0       // ドローンの角速度
#define DRONE_ANGULARACCELERATION 20.0       // ドローンの角加速度
#define DRONE_MAX_SPEED 20.0       // ドローンの最大速度

//Protected Areaのパラメータ
#define PROTECTED_AREA_x 0 //保護エリア(楕円)のx座標中心
#define PROTECTED_AREA_y 0 //保護エリア(楕円)のy座標中心
#define PROTECTED_AREA_a 150 //保護エリア(楕円)の長軸
#define PROTECTED_AREA_b 100 //保護エリア(楕円)の短軸

#define DANGER_R 300 //保護エリア中心からの危険領域

//密度データ
#define DENSITY 0.05 //1%

//状態ベクトルの行数
#define NUM 10

//最適制御の各重み
#define q_1 2.0e7 //検証用
#define q_2 1.0e4
#define q_3 0.0
#define q_d 4000.0
#define Q_d_1 4000.0
#define Q_d_2 4000.0
#define Q_d_3 4000.0
#define Q_v 3.0e4
#define s_1 0.0
#define s_2 0.0
#define s_3 0.0
#define s_4 0.0
#define s_d 0.1
#define s_c 0.0
#define R_d 1.0 //0だとドローンの入力0

#define barrier_v 3.0 //速度制約
#define barrier_d 0.02 //観測距離制約
#define barrier_l 0.01 //ドローン間の距離制約

/*#define q_1 3.0e7 //検証用
#define q_2 600.0
#define q_3 0.0
#define q_d 1200.0
#define Q_d_1 1200.0
#define Q_d_2 1200.0
#define Q_d_3 1200.0
#define Q_v 3000.0
#define s_1 0.0
#define s_2 0.0
#define s_3 0.0
#define s_4 0.0
#define s_d 0.1
#define s_c 0.0
#define R_d 10.0 //0だとドローンの入力0

#define barrier_v 1.0 //速度制約
#define barrier_d 0.01 //観測距離制約
#define barrier_l 1.0 //ドローン間の距離制約*/

////////////////////////////////////////////////////////////////
/*#define q_1 2.0e6 //本番用，確定してないときはいじらない！！！！
#define q_2 8.0
#define q_3 1.0
#define q_d 1.0
#define q_d_1 3.0
#define q_d_2 3.0
#define q_d_3 10.0
#define s_1 0.0
#define s_2 0.0
#define s_3 0.0
#define s_4 0.0
#define s_d 0.01
#define s_c 0.0
#define R_d_1 20.0 //0だとドローンの入力0
#define R_d_2 20.0
#define R_d_3 20.0*/
////////////////////////////////////////////////////////////////

#define WINDOW_WIDTH 1000
#define WINDOW_HEIGHT 1000

#endif // CONFIG_H