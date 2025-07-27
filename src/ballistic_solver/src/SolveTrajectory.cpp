#include "ballistic_solver/SolveTrajectory.hpp"
#include <cmath>
#include <cstdio>

// 全局参数
static SolveTrajectoryParams g_params;
static float g_t = 0.0f;

void setSolveTrajectoryParams(const SolveTrajectoryParams &params) {
    g_params = params;
}

float monoDirectionalAirResistanceModel(float s, float v, float angle) {
    float exp_term = std::exp(g_params.k * s);
    g_t = (exp_term - 1.0f) / (g_params.k * v * std::cos(angle));
    if (g_t <= 0.0f) {
        std::printf("[WARN] Exceeding the maximum range!\n");
        g_t = 0.0f;
        return 0.0f;
    }
    float z = v * std::sin(angle) * g_t - 0.5f * GRAVITY * g_t * g_t;
    return z;
}

float completeAirResistanceModel(float s, float v, float angle) {
    // 暂时复用简化模型，后续可扩展
    return monoDirectionalAirResistanceModel(s, v, angle);
}

float pitchTrajectoryCompensation(float s, float z, float v) {
    float z_temp = z;
    float angle_pitch = 0.0f;
    for (int i = 0; i < 20; ++i) {
        angle_pitch = std::atan2(z_temp, s);
        float z_actual = monoDirectionalAirResistanceModel(s, v, angle_pitch);
        if (z_actual == 0.0f) break;
        float dz = 0.3f * (z - z_actual);
        z_temp += dz;
        if (std::fabs(dz) < 1e-5f) break;
    }
    return angle_pitch;
}

void autoSolveTrajectory(float *pitch, float *yaw,
                         float *aim_x, float *aim_y, float *aim_z) {
    // 预测时间
    float time_delay = g_params.bias_time * 0.001f + g_t;
    g_params.tar_yaw += g_params.v_yaw * time_delay;

    int count = static_cast<int>(g_params.armor_num);
    bool use_r1 = true;
    struct { float x,y,z,yaw; } tar[4];

    for (int i = 0; i < count; ++i) {
        float tmp_yaw = g_params.tar_yaw + i * (2.0f * M_PI / count);
        float r = use_r1 ? g_params.r1 : g_params.r2;
        tar[i].x = g_params.xw - r * std::cos(tmp_yaw);
        tar[i].y = g_params.yw - r * std::sin(tmp_yaw);
        tar[i].z = use_r1 ? g_params.zw : (g_params.zw + g_params.dz);
        tar[i].yaw = tmp_yaw;
        use_r1 = !use_r1;
    }
    // 选最小偏航差
    int idx = 0;
    float min_diff = std::fabs(g_params.current_yaw - tar[0].yaw);
    for (int i = 1; i < count; ++i) {
        float diff = std::fabs(g_params.current_yaw - tar[i].yaw);
        if (diff < min_diff) { min_diff = diff; idx = i; }
    }
    // 预测落点
    *aim_x = tar[idx].x + g_params.vxw * time_delay;
    *aim_y = tar[idx].y + g_params.vyw * time_delay;
    *aim_z = tar[idx].z + g_params.vzw * time_delay;
    // 计算仰角与偏航
    float horiz = std::sqrt((*aim_x)*(*aim_x) + (*aim_y)*(*aim_y));
    *pitch = -pitchTrajectoryCompensation(horiz - g_params.s_bias,
                                           *aim_z + g_params.z_bias,
                                           g_params.current_v);
    *yaw   = std::atan2(*aim_y, *aim_x);
}