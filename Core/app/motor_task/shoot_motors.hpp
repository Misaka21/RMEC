#pragma once

#include "motor.hpp"
#include "dji_driver.hpp"
#include "cascade_pid.hpp"

#include <cstdint>

class ShootMotors {
public:
    void Init();

    // --- 摩擦轮 (速度环) ---
    void SetFrictionSpeed(float left, float right);
    void EnableFriction();
    void DisableFriction();

    // --- 拨弹盘 (调用不同方法自动切换环路) ---
    void SetLoaderAngle(float angle);
    void SetLoaderSpeed(float speed);
    void EnableLoader();
    void DisableLoader();

    // --- motor_task 调用 ---
    void Tick(float dt);

private:
    using M = Motor<DjiDriver, CascadePid>;
    M* friction_l_ = nullptr;
    M* friction_r_ = nullptr;
    M* loader_     = nullptr;

    // 缓冲字段 (robot_task 写, motor_task 读)
    float friction_ref_[2]    = {};
    bool  friction_enabled_   = false;
    float loader_ref_         = 0.0f;
    uint8_t loader_loop_mode_ = 0;  // loop_mode::SPEED
    bool  loader_enabled_     = false;
};
