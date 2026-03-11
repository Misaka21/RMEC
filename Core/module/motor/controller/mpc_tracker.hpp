#pragma once

#include "pid_controller.hpp"
#include "motor_measure.hpp"
#include "feedback_override.hpp"

/// MPC 轨迹跟踪设定值
struct MpcRef {
    float position = 0;
    float velocity = 0;
    float acceleration = 0;
    float feedforward_torque = 0;
};

/// MPC 轨迹跟踪控制器配置
struct MpcTrackerConfig {
    PidConfig position_pid{};
    PidConfig velocity_pid{};
    float inertia = 0;             // 转动惯量 (kg·m²)
    FeedbackOverride feedback_override{};
};

/// MPC 轨迹跟踪控制器（Header-only）
/// 位置环 + 速度环级联, 带加速度前馈与力矩前馈
///   vel_cmd = pos_pid(fb, ref.position) + ref.velocity
///   output  = vel_pid(fb, vel_cmd) + ref.acceleration * inertia + ref.feedforward_torque
class MpcTracker {
public:
    using Ref = MpcRef;

    MpcTracker() = default;

    explicit MpcTracker(const MpcTrackerConfig& cfg)
        : pos_pid_(cfg.position_pid)
        , vel_pid_(cfg.velocity_pid)
        , inertia_(cfg.inertia)
        , fb_override_(cfg.feedback_override) {}

    void Init(const MpcTrackerConfig& cfg) {
        pos_pid_.Init(cfg.position_pid);
        vel_pid_.Init(cfg.velocity_pid);
        inertia_ = cfg.inertia;
        fb_override_ = cfg.feedback_override;
    }

    float Compute(const MpcRef& ref, const MotorMeasure& measure, float dt) {
        // 位置环
        float pos_fb = (fb_override_.angle_fb != nullptr)
            ? *fb_override_.angle_fb
            : measure.total_angle;
        float vel_cmd = pos_pid_.Calculate(pos_fb, ref.position, dt) + ref.velocity;

        // 速度环
        float speed_fb = (fb_override_.speed_fb != nullptr)
            ? *fb_override_.speed_fb
            : measure.speed_aps;
        float output = vel_pid_.Calculate(speed_fb, vel_cmd, dt);

        // 前馈: 加速度 × 惯量 + 力矩前馈
        output += ref.acceleration * inertia_ + ref.feedforward_torque;
        return output;
    }

    PidController& PosPid() { return pos_pid_; }
    PidController& VelPid() { return vel_pid_; }

    void SetFeedbackOverride(const FeedbackOverride& fb) { fb_override_ = fb; }

private:
    PidController pos_pid_{};
    PidController vel_pid_{};
    float inertia_ = 0;
    FeedbackOverride fb_override_{};
};
