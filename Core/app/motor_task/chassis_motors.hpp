#pragma once

#include "motor.hpp"
#include "dji_driver.hpp"
#include "cascade_pid.hpp"
#include "power_limiter.hpp"

class ChassisMotors {
public:
    void Init();

    // --- robot_task 调用 (语义化, 跨任务安全) ---
    void SetWheelSpeeds(float lf, float rf, float lb, float rb);
    void Enable();
    void Disable();
    void SetPowerFeedback(float limit_w, float buffer_j, float measured_w);

    // --- motor_task 调用 ---
    void Tick(float dt);

private:
    using M = Motor<DjiDriver, CascadePid>;
    M* lf_ = nullptr;
    M* rf_ = nullptr;
    M* lb_ = nullptr;
    M* rb_ = nullptr;
    PowerLimiter limiter_;

    // 缓冲字段 (robot_task 写, motor_task 读)
    float ref_[4] = {};
    bool enabled_ = false;
    float power_limit_ = 80.0f;
    float buffer_energy_ = 60.0f;
    float measured_power_ = 0.0f;
};
