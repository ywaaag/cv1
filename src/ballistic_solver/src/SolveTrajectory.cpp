#include "ballistic_solver/SolveTrajectory.hpp"
#include <cmath>
#include <cstdio>

// 小弹丸物理参数 (基于你提供的数据)
static const float BULLET_MASS = 0.0032f;     // 质量 3.2g
static const float BULLET_DIAMETER = 0.0168f; // 直径 16.8mm
static const float AIR_DENSITY = 1.169f; // 25°C标准大气压下空气密度 kg/m³
static const float DRAG_COEFFICIENT = 0.47f; // 球体摩擦系数
static const float BULLET_AREA =
    M_PI * (BULLET_DIAMETER / 2) * (BULLET_DIAMETER / 2); // 截面积

// 计算空气阻力系数 k1 = k0/m, 其中 k0 = C*ρ*S/2
float calculateDragCoefficient() {
  // 注意：理论计算的值通常偏大，实际竞赛中需要使用经验修正值
  // 理论计算：
  // float k0 = (DRAG_COEFFICIENT * AIR_DENSITY * BULLET_AREA) / 2.0f;
  // float k1_theoretical = k0 / BULLET_MASS;  // ≈ 19.19 (过大)

  return 0.038f; //  0.092f
}

/**
 * 改进的单方向空气阻力模型
 * 基于微分方程解：s = (1/k1) * ln(k1*vx0*t + 1)
 * 反解得：t = (e^(k1*s) - 1) / (k1*vx0)
 *
 * @param s 水平距离
 * @param v0 初始发射速度
 * @param angle 发射角度（弧度）
 * @param k1 空气阻力系数
 * @param time_of_flight 输出飞行时间
 * @return 垂直落点高度
 */
static float improvedAirResistanceModel(float s, float v0, float angle,
                                        float k1, float *time_of_flight) {
  // 水平初速度分量
  float vx0 = v0 * std::cos(angle);
  float vy0 = v0 * std::sin(angle);

  if (vx0 <= 0.0f || s <= 0.0f) {
    *time_of_flight = 0.0f;
    return 0.0f;
  }

  // 根据空气阻力模型反解时间：t = (e^(k1*s) - 1) / (k1*vx0)
  float exp_term = std::exp(k1 * s);
  float t = (exp_term - 1.0f) / (k1 * vx0);

  if (t <= 0.0f) {
    *time_of_flight = 0.0f;
    return 0.0f;
  }

  *time_of_flight = t;

  // 垂直方向运动：z = v0*sin(θ)*t - 0.5*g*t²
  float z = vy0 * t - 0.5f * GRAVITY * t * t;
  return z;
}

/**
 * 改进的俯仰角补偿迭代算法
 * 使用更精确的物理模型和收敛判断
 *
 * @param s 目标水平距离
 * @param z 目标垂直高度
 * @param v0 发射速度
 * @param k1 空气阻力系数
 * @param time_of_flight 输出最终飞行时间
 * @return 所需发射俯仰角（弧度）
 */
static float improvedPitchCompensation(float s, float z, float v0, float k1,
                                       float *time_of_flight) {
  float z_temp = z; // 临时目标高度
  float angle_pitch = 0.0f;
  float t_temp = 0.0f;
  float prev_error = 1000.0f; // 用于检测收敛

  const int MAX_ITERATIONS = 25;
  const float CONVERGENCE_THRESHOLD = 0.0005f; // 收敛阈值
  const float DAMPING_FACTOR = 0.8f;           // 阻尼因子防止振荡

  for (int i = 0; i < MAX_ITERATIONS; ++i) {
    // 计算当前瞄准角度
    angle_pitch = std::atan2(z_temp, s);

    // 使用改进的空气阻力模型计算实际落点
    float z_actual =
        improvedAirResistanceModel(s, v0, angle_pitch, k1, &t_temp);

    if (z_actual == 0.0f) { // 超出最大射程
      *time_of_flight = 0.0f;
      return 0.0f;
    }

    // 计算高度误差
    float error = z - z_actual;
    *time_of_flight = t_temp;

    // 检查收敛
    if (std::fabs(error) < CONVERGENCE_THRESHOLD) {
      break;
    }

    // 使用阻尼因子更新，防止过度补偿导致振荡
    float correction = error * DAMPING_FACTOR;

    // 如果误差在增大，减小修正量
    if (std::fabs(error) > std::fabs(prev_error)) {
      correction *= 0.5f;
    }

    z_temp += correction;
    prev_error = error;

    // 防止异常值
    if (std::fabs(z_temp) > 100.0f) { // 假设目标不会超过100米高
      break;
    }
  }

  return angle_pitch;
}

/**
 * 改进的弹道解算主函数
 * 集成了更精确的物理模型和预测算法
 */
void solveForStaticTarget(const SolveTrajectoryParams &params, float *pitch,
                          float *yaw, float *time_of_flight) {
  // 使用实际物理参数计算空气阻力系数
  float k1 = (params.k > 0) ? params.k : calculateDragCoefficient();

  // 1. 改进的目标位置预测
  float initial_dist = std::sqrt(params.xw * params.xw + params.yw * params.yw +
                                 params.zw * params.zw);

  // 考虑空气阻力的初始时间估算（比纯线性更准确）
  float time_est =
      initial_dist / (params.current_v * 0.9f); // 0.9因子考虑阻力减速

  // 多次迭代预测，提高精度
  for (int pred_iter = 0; pred_iter < 3; ++pred_iter) {
    float time_delay = time_est + params.bias_time * 0.001f;

    // 预测目标位置
    float aim_x = params.xw + params.vxw * time_delay;
    float aim_y = params.yw + params.vyw * time_delay;
    float aim_z = params.zw + params.vzw * time_delay;

    // 重新计算距离和时间估算
    float pred_dist = std::sqrt(aim_x * aim_x + aim_y * aim_y + aim_z * aim_z);
    time_est = pred_dist / (params.current_v * 0.9f);
  }

  // 最终预测位置
  float final_time_delay = time_est + params.bias_time * 0.001f;
  float aim_x = params.xw + params.vxw * final_time_delay;
  float aim_y = params.yw + params.vyw * final_time_delay;
  float aim_z = params.zw + params.vzw * final_time_delay;

  // 2. 计算偏航角（Yaw）
  *yaw = std::atan2(aim_y, aim_x);

  // 3. 计算俯仰角（Pitch）
  float horiz_dist = std::sqrt(aim_x * aim_x + aim_y * aim_y);
  float s_compensated = horiz_dist - params.s_bias;
  float z_compensated = aim_z + params.z_bias;

  // 边界检查
  if (s_compensated <= 0.0f) {
    *pitch = 0.0f;
    *time_of_flight = 0.0f;
    return;
  }

  // 使用改进的俯仰角补偿算法
  *pitch = improvedPitchCompensation(s_compensated, z_compensated,
                                     params.current_v, k1, time_of_flight);

  // 最终验证：检查解的合理性
  if (*time_of_flight <= 0.0f || std::isnan(*pitch) || std::isnan(*yaw)) {
    *pitch = 0.0f;
    *yaw = 0.0f;
    *time_of_flight = 0.0f;
  }
}

/**
 * 辅助函数：计算弹道的最大射程
 * 可用于预先判断目标是否在射程内
 */
float calculateMaxRange(float v0, float k1) {
  // 最优发射角约为45度减去一个由空气阻力决定的修正值
  float optimal_angle = M_PI / 4.0f - k1 * v0 * 0.1f; // 简化估算

  float max_s = 0.0f;
  float temp_time = 0.0f;

  // 二分搜索找到最大射程
  float s_low = 0.0f, s_high = v0 * v0 / GRAVITY; // 无阻力情况下的理论最大射程

  for (int i = 0; i < 20; ++i) {
    float s_mid = (s_low + s_high) / 2.0f;
    float z_result =
        improvedAirResistanceModel(s_mid, v0, optimal_angle, k1, &temp_time);

    if (z_result > 0.0f) {
      max_s = s_mid;
      s_low = s_mid;
    } else {
      s_high = s_mid;
    }
  }

  return max_s;
}

/**
 * 验证弹道解算结果的有效性
 */
bool isBallisticSolutionValid(float pitch, float yaw, float time_of_flight) {
  // 检查NaN值
  if (std::isnan(pitch) || std::isnan(yaw) || std::isnan(time_of_flight)) {
    return false;
  }

  // 检查时间有效性
  if (time_of_flight <= 0.0f ||
      time_of_flight > 10.0f) { // 假设最大飞行时间10秒
    return false;
  }

  // 检查角度合理性
  if (std::abs(pitch) > M_PI / 2 ||
      std::abs(yaw) > M_PI) { // 俯仰角不超过90度，偏航角不超过180度
    return false;
  }

  return true;
}