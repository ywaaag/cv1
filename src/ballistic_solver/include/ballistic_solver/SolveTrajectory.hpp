#ifndef __SOLVETRAJECTORY_H__
#define __SOLVETRAJECTORY_H__

#ifndef PI
#define PI 3.1415926535f
#endif
#define GRAVITY 9.78f

// 
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

enum BULLET_TYPE { BULLET_17 = 0, BULLET_42 = 1 };

// 参数结构体
struct SolveTrajectoryParams {
  // 物理参数
  float k;         // 空气阻力系数
  float current_v; // 当前弹速

  // 目标信息
  float xw, yw, zw;    // 目标在机器人坐标系下的位置
  float vxw, vyw, vzw; // 目标的速度
  // 补偿/偏移
  int bias_time;        // 预测时间偏移 (ms)
  float s_bias, z_bias; // 距离和高度的固定偏移
};

// 无状态的弹道解算函数
void solveForStaticTarget(const SolveTrajectoryParams &params, // 输入：所有参数
                          float *pitch, // 输出：计算出的Pitch (弧度)
                          float *yaw,   // 输出：计算出的Yaw (弧度)
                          float *time_of_flight // 输出：子弹飞行时间 (秒)
);

/**
 * 计算理论最大射程
 * @param v0 初始发射速度 (m/s)
 * @param k1 空气阻力系数
 * @return 最大射程 (m)
 */
float calculateMaxRange(float v0, float k1);

/**
 * 计算标准条件下的空气阻力系数
 * 基于小弹丸的物理参数自动计算
 * @return 空气阻力系数 k1
 */
float calculateDragCoefficient();

/**
 * 验证弹道解算结果的有效性
 * @param pitch 俯仰角 (弧度)
 * @param yaw 偏航角 (弧度)
 * @param time_of_flight 飞行时间 (秒)
 * @return true if valid, false otherwise
 */
bool isBallisticSolutionValid(float pitch, float yaw, float time_of_flight);

// --- 旧的、不推荐使用的函数 (保留以供参考) ---
// void setSolveTrajectoryParams(const SolveTrajectoryParams &params);
// void autoSolveTrajectory(float *pitch, float *yaw,
//                          float *aim_x, float *aim_y, float *aim_z);

#endif // __SOLVETRAJECTORY_H__