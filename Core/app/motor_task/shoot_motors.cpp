#include "shoot_motors.hpp"
#include "robot_def.hpp"

#include "can.h"

void ShootMotors::Init() {
    // 订阅命令 Topic
    cmd_reader_ = shoot_cmd_topic.Subscribe();

    // 摩擦轮 M3508 — 速度环
    CascadePidConfig friction_cfg{};
    friction_cfg.loop_mode = loop_mode::SPEED;
    friction_cfg.speed_pid = {
        .kp            = FRICTION_SPEED_KP,
        .ki            = FRICTION_SPEED_KI,
        .kd            = FRICTION_SPEED_KD,
        .max_out       = FRICTION_SPEED_MAX_OUT,
        .improve_flags = pid::INTEGRAL_LIMIT,
        .integral_limit = FRICTION_SPEED_INTEGRAL_LIMIT,
    };

    {
        DjiDriverConfig drv_cfg{};
        drv_cfg.motor_type = DjiMotorType::M3508;
        drv_cfg.can_handle = &FRICTION_CAN_HANDLE;
        drv_cfg.motor_id   = FRICTION_L_MOTOR_ID;
        friction_l_ = new M(drv_cfg, friction_cfg);
    }
    {
        DjiDriverConfig drv_cfg{};
        drv_cfg.motor_type = DjiMotorType::M3508;
        drv_cfg.can_handle = &FRICTION_CAN_HANDLE;
        drv_cfg.motor_id   = FRICTION_R_MOTOR_ID;
        friction_r_ = new M(drv_cfg, friction_cfg);
    }

    // 拨弹盘 M2006 — 速度+角度环
    {
        CascadePidConfig cfg{};
        cfg.loop_mode = loop_mode::SPEED;
        cfg.speed_pid = {
            .kp            = LOADER_SPEED_KP,
            .ki            = LOADER_SPEED_KI,
            .kd            = LOADER_SPEED_KD,
            .max_out       = LOADER_SPEED_MAX_OUT,
            .improve_flags = pid::INTEGRAL_LIMIT,
            .integral_limit = LOADER_SPEED_INTEGRAL_LIMIT,
        };
        cfg.angle_pid = {
            .kp      = LOADER_ANGLE_KP,
            .ki      = LOADER_ANGLE_KI,
            .kd      = LOADER_ANGLE_KD,
            .max_out = LOADER_ANGLE_MAX_OUT,
        };

        DjiDriverConfig drv_cfg{};
        drv_cfg.motor_type = DjiMotorType::M2006;
        drv_cfg.can_handle = &LOADER_CAN_HANDLE;
        drv_cfg.motor_id   = LOADER_MOTOR_ID;
        loader_ = new M(drv_cfg, cfg);
    }
}

void ShootMotors::Tick(float dt) {
    cmd_reader_->Read(cmd_cache_);

    // ---- 摩擦轮 ----
    if (cmd_cache_.friction_mode == FrictionMode::OFF) {
        friction_l_->Disable();
        friction_r_->Disable();
    } else {
        friction_l_->Enable();
        friction_r_->Enable();
        friction_l_->SetRef( FRICTION_DEFAULT_SPEED);
        friction_r_->SetRef(-FRICTION_DEFAULT_SPEED);  // 对转
    }
    friction_l_->Update(dt);
    friction_r_->Update(dt);

    // ---- 拨弹盘 ----
    switch (cmd_cache_.load_mode) {
    case LoaderMode::STOP:
        loader_->Disable();
        loader_angle_target_ = loader_->Measure().total_angle;  // 记录当前位置
        break;

    case LoaderMode::REVERSE:
        loader_->Enable();
        loader_->GetController().SetLoopMode(loop_mode::SPEED);
        loader_->SetRef(-LOADER_BURST_SPEED);
        break;

    case LoaderMode::SINGLE:
    case LoaderMode::TRIPLE: {
        loader_->Enable();
        loader_->GetController().SetLoopMode(loop_mode::ANGLE_SPEED);
        float bullets = (cmd_cache_.load_mode == LoaderMode::SINGLE) ? 1.0f : 3.0f;
        // 首次进入: 设定角度目标 (当前位置 + N 发弹丸角度)
        float target = loader_->Measure().total_angle
                     + bullets * ONE_BULLET_DELTA_ANGLE * REDUCTION_RATIO_LOADER;
        if (loader_angle_target_ < target)
            loader_angle_target_ = target;
        loader_->SetRef(loader_angle_target_);
        break;
    }

    case LoaderMode::BURST:
        loader_->Enable();
        loader_->GetController().SetLoopMode(loop_mode::SPEED);
        loader_->SetRef(LOADER_BURST_SPEED);
        break;
    }

    // 热量保护: 剩余热量为 0 则停转
    if (cmd_cache_.rest_heat == 0 && cmd_cache_.load_mode != LoaderMode::REVERSE) {
        loader_->Disable();
    }

    loader_->Update(dt);
}
