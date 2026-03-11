#include "chassis_motors.hpp"
#include "robot_def.hpp"
#include "general_def.hpp"

#include "can.h"

void ChassisMotors::Init() {
    // M3508 速度环 PID 配置
    CascadePidConfig cascade_cfg{};
    cascade_cfg.loop_mode = loop_mode::SPEED;
    cascade_cfg.speed_pid = {
        .kp            = CHASSIS_SPEED_KP,
        .ki            = CHASSIS_SPEED_KI,
        .kd            = CHASSIS_SPEED_KD,
        .max_out       = CHASSIS_SPEED_MAX_OUT,
        .improve_flags = pid::INTEGRAL_LIMIT,
        .integral_limit = CHASSIS_SPEED_INTEGRAL_LIMIT,
    };

    // 4 个 M3508, motor_id 1-4, CAN2
    auto make = [&](uint8_t id) -> M* {
        DjiDriverConfig drv_cfg{};
        drv_cfg.motor_type = DjiMotorType::M3508;
        drv_cfg.can_handle = &CHASSIS_CAN_HANDLE;
        drv_cfg.motor_id   = id;
        return new M(drv_cfg, cascade_cfg);
    };

    lf_ = make(1);
    rf_ = make(2);
    lb_ = make(3);
    rb_ = make(4);

    // 功率限制器初始化
    PowerLimiterConfig pl_cfg{};
    pl_cfg.model.output_to_torque = CHASSIS_OUTPUT_TO_TORQUE;
    pl_cfg.motor_count = 4;
    limiter_.Init(pl_cfg);
}

void ChassisMotors::SetWheelSpeeds(float lf, float rf, float lb, float rb) {
    ref_[0] = lf;
    ref_[1] = rf;
    ref_[2] = lb;
    ref_[3] = rb;
}

void ChassisMotors::Enable()  { enabled_ = true; }
void ChassisMotors::Disable() { enabled_ = false; }

void ChassisMotors::SetPowerFeedback(float limit_w, float buffer_j, float measured_w) {
    power_limit_    = limit_w;
    buffer_energy_  = buffer_j;
    measured_power_ = measured_w;
}

void ChassisMotors::Tick(float dt) {
    // 1. 从缓冲读取 ref, 设置使能状态
    lf_->SetRef(ref_[0]);
    rf_->SetRef(ref_[1]);
    lb_->SetRef(ref_[2]);
    rb_->SetRef(ref_[3]);

    if (!enabled_) {
        lf_->Disable(); rf_->Disable();
        lb_->Disable(); rb_->Disable();
    } else {
        lf_->Enable(); rf_->Enable();
        lb_->Enable(); rb_->Enable();
    }

    // 2. 两阶段 PID — disabled 时 ComputeOutput 返回 0, ApplyOutput 写零
    M* motors[4] = {lf_, rf_, lb_, rb_};
    PowerMotorState states[4] = {};
    for (int i = 0; i < 4; ++i) {
        states[i].pid_output    = motors[i]->ComputeOutput(dt);
        states[i].speed_rad     = motors[i]->Measure().speed_aps * DEGREE_2_RAD;
        states[i].set_speed_rad = ref_[i] * DEGREE_2_RAD;
        states[i].max_output    = CHASSIS_SPEED_MAX_OUT;
    }

    // 3. 功率限制
    limiter_.UpdateEnergyLoop(power_limit_, buffer_energy_, measured_power_, dt);
    limiter_.Limit(states, 4);

    // 4. 应用输出 (disabled 时归零 CAN 缓冲)
    for (int i = 0; i < 4; ++i)
        motors[i]->ApplyOutput(states[i].pid_output);
}
