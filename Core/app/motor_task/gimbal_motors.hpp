#pragma once

#include "motor.hpp"
#include "dji_driver.hpp"
#include "cascade_pid.hpp"
#include "topic.hpp"
#include "ins_data.hpp"

class GimbalMotors {
public:
    void Init();

    // --- robot_task 调用 (语义化, 跨任务安全) ---
    void SetYawAngle(float total_angle);
    void SetPitchAngle(float angle);
    void Enable();
    void Disable();
    float YawSingleRound() const;

    // --- motor_task 调用 ---
    void Tick(float dt);

private:
    using M = Motor<DjiDriver, CascadePid>;
    M* yaw_   = nullptr;
    M* pitch_ = nullptr;
    TopicReader<InsData>* ins_reader_ = nullptr;
    InsData ins_cache_{};

    // 缓冲字段 (robot_task 写, motor_task 读)
    float yaw_ref_   = 0.0f;
    float pitch_ref_ = 0.0f;
    bool enabled_    = false;
};
