#pragma once

#include "rls2.hpp"

#include <cstdint>

/// 功率模型参数配置
struct PowerModelConfig {
    float k1_init = 0.22f;         ///< 速度相关损耗系数 (RLS 初始值)
    float k2_init = 1.2f;          ///< 电流²相关损耗系数 (RLS 初始值)
    float k3 = 2.78f;              ///< 恒定功耗 (W)
    float output_to_torque = 0;    ///< PID输出 → 扭矩(Nm) 转换系数
    // M3508 默认: (20.0f / 16384.0f) * 0.3f * (3591.0f / 187.0f)
};

/// 能量环配置
struct EnergyLoopConfig {
    float kp = 50.0f;                ///< 能量环 P 增益
    float kd = 0.2f;                 ///< 能量环 D 增益
    float max_extra_power = 300.0f;  ///< PD 输出上限 (W)
    float base_buff_target = 50.0f;  ///< 缓冲能量下限目标 (J)
    float full_buff_target = 60.0f;  ///< 缓冲能量上限目标 (J)
};

/// 功率限制器总配置
struct PowerLimiterConfig {
    PowerModelConfig model{};
    EnergyLoopConfig energy{};
    float rls_lambda  = 0.9999f;   ///< RLS 遗忘因子
    float rls_delta   = 1e-5f;     ///< RLS 初始协方差
    float rls_deadzone = 5.0f;     ///< |P_measured| < 此值不更新 RLS (W)
    uint8_t motor_count = 4;       ///< 电机数量
};

/// 传入功率限制器的每电机状态（由 Chassis 层填充）
struct PowerMotorState {
    float pid_output;     ///< ComputeOutput() 结果（in/out, 可被修改）
    float speed_rad;      ///< 角速度 (rad/s)
    float set_speed_rad;  ///< 目标角速度 (rad/s), 用于 error-confidence
    float max_output;     ///< PID 最大输出（用于 clamp）
};

/// 功率限制器
///
/// 三层功率控制算法:
///   1. 能量环 — PD on √(energy), 计算动态 maxPower
///   2. RLS 在线辨识 — 估计功率模型参数 k1, k2
///   3. 二次功率分配 — error/prop 混合权重 + 二次方程求解
///
/// 典型用法（Chassis 控制循环内）:
///   for (i : N) states[i].pid_output = motors[i].ComputeOutput(dt);
///   limiter.UpdateEnergyLoop(ref_limit, buf_energy, measured_power, dt);
///   limiter.Limit(states, N);
///   for (i : N) motors[i].ApplyOutput(states[i].pid_output);
///   DjiDriver::FlushAll();
class PowerLimiter {
public:
    void Init(const PowerLimiterConfig& cfg);

    /// 能量环更新 — 计算动态 maxPower
    /// @param referee_power_limit  裁判系统功率上限 (W)
    /// @param buffer_energy        缓冲能量反馈 (J)
    /// @param measured_power       实测底盘功率 (W)
    /// @param dt                   时间间隔 (s)
    void UpdateEnergyLoop(float referee_power_limit, float buffer_energy,
                          float measured_power, float dt);

    /// 功率限制 — 修改 states 中的 pid_output
    /// @param states   电机状态数组（in/out: pid_output 会被修改）
    /// @param count    电机数量
    void Limit(PowerMotorState* states, uint8_t count);

    float GetMaxPower()       const { return max_power_; }
    float GetK1()             const { return k1_; }
    float GetK2()             const { return k2_; }
    float GetEstimatedPower() const { return estimated_power_; }

private:
    /// RLS 更新（辨识 k1, k2）
    void UpdateRls(const PowerMotorState* states, uint8_t count,
                   float measured_power);

    /// 单电机功率预测: P = τ*ω + k1*|ω| + k2*τ² + k3/N
    float PredictPower(float torque, float speed_rad) const;

    /// 二次方程求解: 给定功率预算, 求最大允许扭矩
    float SolveMaxTorque(float power_budget, float speed_rad) const;

    PowerLimiterConfig cfg_{};
    Rls2 rls_{};
    float k1_ = 0, k2_ = 0, k3_ = 0;
    float max_power_ = 0;
    float base_max_power_ = 0;
    float full_max_power_ = 0;
    float estimated_power_ = 0;
    float last_measured_power_ = 0;

    // 能量环 PD 状态
    float last_base_err_ = 0;
    float last_full_err_ = 0;
};
