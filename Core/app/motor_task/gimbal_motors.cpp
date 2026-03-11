#include "gimbal_motors.hpp"
#include "robot_def.hpp"
#include "robot_topics.hpp"

#include "can.h"

void GimbalMotors::Init() {
    // 订阅 ins_topic (在调度器启动前)
    ins_reader_ = ins_topic.Subscribe();

    // Yaw GM6020 — 角度→速度级联, 使用 IMU 反馈
    {
        CascadePidConfig cfg{};
        cfg.loop_mode = loop_mode::ANGLE_SPEED;
        cfg.angle_pid = {
            .kp      = YAW_ANGLE_KP,
            .ki      = YAW_ANGLE_KI,
            .kd      = YAW_ANGLE_KD,
            .max_out = YAW_ANGLE_MAX_OUT,
        };
        cfg.speed_pid = {
            .kp            = YAW_SPEED_KP,
            .ki            = YAW_SPEED_KI,
            .kd            = YAW_SPEED_KD,
            .max_out       = YAW_SPEED_MAX_OUT,
            .improve_flags = pid::INTEGRAL_LIMIT,
            .integral_limit = YAW_SPEED_INTEGRAL_LIMIT,
        };
        // IMU 反馈覆盖: 角度用 yaw_total, 角速度用 gyro[2] (z 轴)
        cfg.feedback_override.angle_fb = &ins_cache_.yaw_total;
        cfg.feedback_override.speed_fb = &ins_cache_.gyro[2];

        DjiDriverConfig drv_cfg{};
        drv_cfg.motor_type = DjiMotorType::GM6020;
        drv_cfg.can_handle = &YAW_CAN_HANDLE;
        drv_cfg.motor_id   = YAW_MOTOR_ID;

        yaw_ = new M(drv_cfg, cfg);
    }

    // Pitch GM6020 — 角度→速度级联, 使用 IMU 反馈
    {
        CascadePidConfig cfg{};
        cfg.loop_mode = loop_mode::ANGLE_SPEED;
        cfg.angle_pid = {
            .kp      = PITCH_ANGLE_KP,
            .ki      = PITCH_ANGLE_KI,
            .kd      = PITCH_ANGLE_KD,
            .max_out = PITCH_ANGLE_MAX_OUT,
        };
        cfg.speed_pid = {
            .kp            = PITCH_SPEED_KP,
            .ki            = PITCH_SPEED_KI,
            .kd            = PITCH_SPEED_KD,
            .max_out       = PITCH_SPEED_MAX_OUT,
            .improve_flags = pid::INTEGRAL_LIMIT,
            .integral_limit = PITCH_SPEED_INTEGRAL_LIMIT,
        };
        // IMU 反馈覆盖: 角度用 euler[1] (pitch), 角速度用 gyro[1] (y 轴)
        cfg.feedback_override.angle_fb = &ins_cache_.euler[1];
        cfg.feedback_override.speed_fb = &ins_cache_.gyro[1];

        DjiDriverConfig drv_cfg{};
        drv_cfg.motor_type = DjiMotorType::GM6020;
        drv_cfg.can_handle = &PITCH_CAN_HANDLE;
        drv_cfg.motor_id   = PITCH_MOTOR_ID;

        pitch_ = new M(drv_cfg, cfg);
    }
}

void GimbalMotors::SetYawAngle(float total_angle)  { yaw_ref_ = total_angle; }
void GimbalMotors::SetPitchAngle(float angle)       { pitch_ref_ = angle; }
void GimbalMotors::Enable()                          { enabled_ = true; }
void GimbalMotors::Disable()                         { enabled_ = false; }

float GimbalMotors::YawSingleRound() const {
    if (!yaw_) return 0.0f;
    return yaw_->Measure().angle_single_round;
}

void GimbalMotors::Tick(float dt) {
    // 1. 读最新 IMU 数据到本地缓存 (ins_cache_ 同线程, FeedbackOverride 指向此处)
    ins_reader_->Read(ins_cache_);

    // 2. 从缓冲读取 ref, 设置使能状态
    yaw_->SetRef(yaw_ref_);
    pitch_->SetRef(pitch_ref_);

    if (!enabled_) {
        yaw_->Disable();
        pitch_->Disable();
    } else {
        yaw_->Enable();
        pitch_->Enable();
    }

    // 3. 一步更新 — disabled 时 Update 内部会 SetOutput(0) 归零 CAN 缓冲
    yaw_->Update(dt);
    pitch_->Update(dt);
}
