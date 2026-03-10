#pragma once

#include "pid_controller.hpp"
#include "motor_measure.hpp"

#include <cstdint>

/// 级联 PID 闭环模式（位掩码）
namespace loop_mode {
inline constexpr uint8_t OPEN        = 0b000;
inline constexpr uint8_t SPEED       = 0b010;
inline constexpr uint8_t ANGLE       = 0b100;
inline constexpr uint8_t ANGLE_SPEED = 0b110;  // angle → speed 级联
} // namespace loop_mode

/// 外部反馈覆盖（IMU 角度/角速度等）
struct FeedbackOverride {
    const float* angle_fb  = nullptr;   // 若非空，角度环使用此反馈
    const float* speed_fb  = nullptr;   // 若非空，速度环使用此反馈
};

/// 级联 PID 控制器配置
struct CascadePidConfig {
    PidConfig speed_pid{};
    PidConfig angle_pid{};
    uint8_t loop_mode = loop_mode::SPEED;
    bool reverse = false;
    FeedbackOverride feedback_override{};
};

/// 级联 PID 控制器（Header-only）
/// 根据 loop_mode 位掩码决定控制环路：
///   kSpeed      → 仅速度环
///   kAngle      → 仅角度环（输出直接作为控制量）
///   kAngleSpeed → angle_pid → speed_pid 级联
class CascadePid {
public:
    CascadePid() = default;

    explicit CascadePid(const CascadePidConfig& cfg)
        : speed_pid_(cfg.speed_pid)
        , angle_pid_(cfg.angle_pid)
        , loop_mode_(cfg.loop_mode)
        , reverse_(cfg.reverse)
        , fb_override_(cfg.feedback_override) {}

    void Init(const CascadePidConfig& cfg) {
        speed_pid_.Init(cfg.speed_pid);
        angle_pid_.Init(cfg.angle_pid);
        loop_mode_ = cfg.loop_mode;
        reverse_ = cfg.reverse;
        fb_override_ = cfg.feedback_override;
    }

    /// 计算控制输出
    /// @param ref     外部设定值
    /// @param measure 电机反馈数据
    /// @param dt      时间间隔(s)
    /// @return 控制量（电流/电压指令）
    float Compute(float ref, const MotorMeasure& measure, float dt) {
        if (reverse_)
            ref = -ref;

        float output = ref;

        // 角度环
        if (loop_mode_ & loop_mode::ANGLE) {
            float angle_fb = (fb_override_.angle_fb != nullptr)
                ? *fb_override_.angle_fb
                : measure.total_angle;
            output = angle_pid_.Calculate(angle_fb, ref, dt);
        }

        // 速度环
        if (loop_mode_ & loop_mode::SPEED) {
            float speed_fb = (fb_override_.speed_fb != nullptr)
                ? *fb_override_.speed_fb
                : measure.speed_aps;
            output = speed_pid_.Calculate(speed_fb, output, dt);
        }

        return output;
    }

    void SetLoopMode(uint8_t mode) { loop_mode_ = mode; }
    uint8_t GetLoopMode() const { return loop_mode_; }

    PidController& SpeedPid() { return speed_pid_; }
    PidController& AnglePid() { return angle_pid_; }

    void SetFeedbackOverride(const FeedbackOverride& fb) { fb_override_ = fb; }

private:
    PidController speed_pid_{};
    PidController angle_pid_{};
    uint8_t loop_mode_ = loop_mode::SPEED;
    bool reverse_ = false;
    FeedbackOverride fb_override_{};
};
