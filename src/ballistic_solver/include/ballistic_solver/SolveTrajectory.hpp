#ifndef __SOLVETRAJECTORY_H__
#define __SOLVETRAJECTORY_H__

#ifndef PI
#define PI 3.1415926535f
#endif

#define GRAVITY 9.78f

typedef unsigned char uint8_t;

enum ARMOR_ID {
    ARMOR_OUTPOST = 0,
    ARMOR_HERO = 1,
    ARMOR_ENGINEER = 2,
    ARMOR_INFANTRY3 = 3,
    ARMOR_INFANTRY4 = 4,
    ARMOR_INFANTRY5 = 5,
    ARMOR_GUARD = 6,
    ARMOR_BASE = 7
};

enum ARMOR_NUM {
    ARMOR_NUM_BALANCE = 2,
    ARMOR_NUM_OUTPOST = 3,
    ARMOR_NUM_NORMAL = 4
};

enum BULLET_TYPE {
    BULLET_17 = 0,
    BULLET_42 = 1
};

// 参数结构体
struct SolveTrajectoryParams {
    float k;
    enum BULLET_TYPE bullet_type;
    float current_v;
    float current_pitch;
    float current_yaw;

    float xw, yw, zw;
    float vxw, vyw, vzw;
    float tar_yaw, v_yaw;
    float r1, r2, dz;

    int bias_time;
    float s_bias, z_bias;

    enum ARMOR_ID armor_id;
    enum ARMOR_NUM armor_num;
};

// 模型函数接口
void setSolveTrajectoryParams(const SolveTrajectoryParams &params);
float monoDirectionalAirResistanceModel(float s, float v, float angle);
float completeAirResistanceModel(float s, float v, float angle);
float pitchTrajectoryCompensation(float s, float z, float v);
void autoSolveTrajectory(float *pitch, float *yaw,
                         float *aim_x, float *aim_y, float *aim_z);

#endif // __SOLVETRAJECTORY_H__