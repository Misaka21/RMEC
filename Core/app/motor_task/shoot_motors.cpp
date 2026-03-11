#include "shoot_motors.hpp"
#include "robot_def.hpp"

#include "can.h"

void ShootMotors::Init() {
    // 初始化默认环路模式
    loader_loop_mode_ = loop_mode::SPEED;

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

void ShootMotors::SetFrictionSpeed(float left, float right) {
    friction_ref_[0] = left;
    friction_ref_[1] = right;
}

void ShootMotors::EnableFriction()  { friction_enabled_ = true; }
void ShootMotors::DisableFriction() { friction_enabled_ = false; }

void ShootMotors::SetLoaderAngle(float angle) {
    loader_ref_ = angle;
    loader_loop_mode_ = loop_mode::ANGLE_SPEED;
}

void ShootMotors::SetLoaderSpeed(float speed) {
    loader_ref_ = speed;
    loader_loop_mode_ = loop_mode::SPEED;
}

void ShootMotors::EnableLoader()  { loader_enabled_ = true; }
void ShootMotors::DisableLoader() { loader_enabled_ = false; }

void ShootMotors::Tick(float dt) {
    // --- 摩擦轮 ---
    friction_l_->SetRef(friction_ref_[0]);
    friction_r_->SetRef(friction_ref_[1]);

    if (!friction_enabled_) {
        friction_l_->Disable();
        friction_r_->Disable();
    } else {
        friction_l_->Enable();
        friction_r_->Enable();
    }
    // disabled 时 Update 内部会 SetOutput(0) 归零 CAN 缓冲
    friction_l_->Update(dt);
    friction_r_->Update(dt);

    // --- 拨弹盘 ---
    loader_->GetController().SetLoopMode(loader_loop_mode_);
    loader_->SetRef(loader_ref_);

    if (!loader_enabled_) {
        loader_->Disable();
    } else {
        loader_->Enable();
    }
    loader_->Update(dt);
}
