#include "gimbal_motors.hpp"
#include "robot_def.hpp"
#include "robot_topics.hpp"

#include "can.h"

void GimbalMotors::Init() {
    // 订阅 Topic
    cmd_reader_ = gimbal_cmd_topic.Subscribe();
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

void GimbalMotors::Tick(float dt) {
    // 1. 读最新 IMU + 命令
    ins_reader_->Read(ins_cache_);
    cmd_reader_->Read(cmd_cache_);

    // 2. 模式处理
    if (cmd_cache_.mode == GimbalMode::ZERO_FORCE) {
        yaw_->Disable();
        pitch_->Disable();
    } else {
        yaw_->Enable();
        pitch_->Enable();
        yaw_->SetRef(cmd_cache_.yaw);
        pitch_->SetRef(cmd_cache_.pitch);
    }

    // 3. 更新 PID (disabled 时 Update 内部写零)
    yaw_->Update(dt);
    pitch_->Update(dt);

    // 4. 发布反馈 (gimbal_feed_topic 的唯一写者)
    GimbalFeedData feed{};
    feed.yaw_total        = ins_cache_.yaw_total;
    feed.yaw_single_round = yaw_->Measure().angle_single_round;
    feed.pitch            = ins_cache_.euler[1];
    gimbal_feed_topic.Publish(feed);
}
