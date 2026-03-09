#include "power_limiter.hpp"
#include "general_def.hpp"

#include <cmath>

void PowerLimiter::Init(const PowerLimiterConfig& cfg) {
    cfg_ = cfg;
    k1_ = cfg.model.k1_init;
    k2_ = cfg.model.k2_init;
    k3_ = cfg.model.k3;
    rls_.Init(cfg.rls_lambda, cfg.rls_delta,
              cfg.model.k1_init, cfg.model.k2_init);
    last_base_err_ = 0;
    last_full_err_ = 0;
    last_measured_power_ = 0;
    estimated_power_ = 0;
    max_power_ = 0;
}

// ---- 能量环: PD on √(energy) ----
void PowerLimiter::UpdateEnergyLoop(float referee_power_limit,
                                    float buffer_energy,
                                    float measured_power,
                                    float dt) {
    last_measured_power_ = measured_power;

    // 最低功率限制: 始终保留 5W 余量
    constexpr float kMinPower = 5.0f;
    const float sqrt_buf = std::sqrt(std::fmax(buffer_energy, 0.0f));
    const float inv_dt = (dt > 1e-6f) ? (1.0f / dt) : 0.0f;

    // base PD: 保证缓冲能量不被耗尽
    {
        const float target = std::sqrt(cfg_.energy.base_buff_target);
        const float err = target - sqrt_buf;
        const float d_err = (err - last_base_err_) * inv_dt;
        base_max_power_ = referee_power_limit
                          - cfg_.energy.kp * err
                          - cfg_.energy.kd * d_err;
        base_max_power_ = std::fmax(base_max_power_, kMinPower);
        base_max_power_ = std::fmin(base_max_power_,
                                    referee_power_limit + cfg_.energy.max_extra_power);
        last_base_err_ = err;
    }

    // full PD: 防止过充（超过目标时减功率）
    {
        const float target = std::sqrt(cfg_.energy.full_buff_target);
        const float err = target - sqrt_buf;
        const float d_err = (err - last_full_err_) * inv_dt;
        full_max_power_ = referee_power_limit
                          - cfg_.energy.kp * err
                          - cfg_.energy.kd * d_err;
        full_max_power_ = std::fmax(full_max_power_, kMinPower);
        full_max_power_ = std::fmin(full_max_power_,
                                    referee_power_limit + cfg_.energy.max_extra_power);
        last_full_err_ = err;
    }

    // maxPower = clamp(referee_limit, full_max_power, base_max_power)
    max_power_ = Clamp(referee_power_limit, full_max_power_, base_max_power_);
}

// ---- RLS 更新 ----
void PowerLimiter::UpdateRls(const PowerMotorState* states, uint8_t count,
                             float measured_power) {
    // 死区: 低功率时不更新
    if (std::fabs(measured_power) < cfg_.rls_deadzone)
        return;

    const float otq = cfg_.model.output_to_torque;
    float sum_abs_w = 0;
    float sum_tau_sq = 0;
    float sum_tau_w  = 0;

    for (uint8_t i = 0; i < count; ++i) {
        const float tau = states[i].pid_output * otq;
        const float w   = states[i].speed_rad;
        sum_abs_w  += std::fabs(w);
        sum_tau_sq += tau * tau;
        sum_tau_w  += tau * w;
    }

    // y = P_measured - Σ(τ·ω) - k3
    const float y = measured_power - sum_tau_w - k3_;
    const float x[2] = { sum_abs_w, sum_tau_sq };

    rls_.Update(x, y);
    k1_ = rls_.K1();
    k2_ = rls_.K2();
}

// ---- 单电机功率预测 ----
float PowerLimiter::PredictPower(float torque, float speed_rad) const {
    const float n = static_cast<float>(cfg_.motor_count);
    return torque * speed_rad
           + k1_ * std::fabs(speed_rad)
           + k2_ * torque * torque
           + k3_ / n;
}

// ---- 二次方程求解: k2*τ² + ω*τ + (k1*|ω| + k3/N - P_budget) = 0 ----
float PowerLimiter::SolveMaxTorque(float power_budget, float speed_rad) const {
    const float n = static_cast<float>(cfg_.motor_count);
    const float a = k2_;
    const float b = speed_rad;
    const float c = k1_ * std::fabs(speed_rad) + k3_ / n - power_budget;

    if (std::fabs(a) < 1e-10f) {
        // 退化为线性: b*τ + c = 0
        if (std::fabs(b) < 1e-10f)
            return 0;
        return -c / b;
    }

    const float disc = b * b - 4.0f * a * c;
    if (disc < 0)
        return 0;  // 无实数解 → 无法在此功率下运行

    const float sqrt_disc = std::sqrt(disc);
    const float tau1 = (-b + sqrt_disc) / (2.0f * a);
    const float tau2 = (-b - sqrt_disc) / (2.0f * a);

    // 取绝对值较大的解（给电机最大执行空间）
    return (std::fabs(tau1) > std::fabs(tau2)) ? tau1 : tau2;
}

// ---- 功率分配 ----
void PowerLimiter::Limit(PowerMotorState* states, uint8_t count) {
    if (count == 0)
        return;

    const float otq = cfg_.model.output_to_torque;

    // 1. RLS 在线辨识
    UpdateRls(states, count, last_measured_power_);

    // 2. 预测每电机命令功率
    float cmd_power[8];   // 最多支持 8 电机
    float total_cmd = 0;
    const uint8_t n = (count <= 8) ? count : 8;

    for (uint8_t i = 0; i < n; ++i) {
        const float tau = states[i].pid_output * otq;
        cmd_power[i] = PredictPower(tau, states[i].speed_rad);
        total_cmd += cmd_power[i];
    }
    estimated_power_ = total_cmd;

    // 3. 检查是否超功率
    if (total_cmd <= max_power_)
        return;  // 未超限, 不修改

    // 4. 计算 error-confidence (误差越大 → 越倾向误差均匀分配)
    float sum_error = 0;
    for (uint8_t i = 0; i < n; ++i) {
        sum_error += std::fabs(states[i].set_speed_rad - states[i].speed_rad);
    }
    constexpr float kErrLow  = 15.0f;
    constexpr float kErrHigh = 20.0f;
    const float err_conf = Clamp((sum_error - kErrLow) / (kErrHigh - kErrLow),
                                 0.0f, 1.0f);

    // 5. 混合权重: blend(equal, cmd_proportion, errorConfidence)
    float weight[8];
    float weight_sum = 0;
    const float equal_share = 1.0f / static_cast<float>(n);

    for (uint8_t i = 0; i < n; ++i) {
        const float cmd_prop = (total_cmd > 1e-6f)
                               ? (cmd_power[i] / total_cmd)
                               : equal_share;
        weight[i] = err_conf * equal_share + (1.0f - err_conf) * cmd_prop;
        weight_sum += weight[i];
    }

    // 归一化权重
    if (weight_sum > 1e-6f) {
        const float inv_sum = 1.0f / weight_sum;
        for (uint8_t i = 0; i < n; ++i)
            weight[i] *= inv_sum;
    }

    // 6. 按权重分配功率, 二次求解每电机最大扭矩
    for (uint8_t i = 0; i < n; ++i) {
        const float budget = max_power_ * weight[i];
        const float max_tau = SolveMaxTorque(budget, states[i].speed_rad);

        // 转回 pid_output 单位
        float new_output = (std::fabs(otq) > 1e-10f) ? (max_tau / otq) : 0.0f;
        // 保持原方向
        if ((states[i].pid_output > 0) != (new_output > 0))
            new_output = -new_output;
        // 取较小绝对值（只缩减不放大）
        if (std::fabs(new_output) < std::fabs(states[i].pid_output))
            states[i].pid_output = new_output;
        // clamp
        states[i].pid_output = ClampAbs(states[i].pid_output, states[i].max_output);
    }
}
