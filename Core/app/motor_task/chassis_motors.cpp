#include "chassis_motors.hpp"
#include "robot_def.hpp"
#include "general_def.hpp"

#include "can.h"

void ChassisMotors::Init() {
    // 订阅命令 Topic
    cmd_reader_ = chassis_cmd_topic.Subscribe();

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

void ChassisMotors::Tick(float dt) {
    cmd_reader_->Read(cmd_cache_);

    M* motors[4] = {lf_, rf_, lb_, rb_};

    // ZERO_FORCE: 失能并写零到 CAN 缓冲
    if (cmd_cache_.mode == ChassisMode::ZERO_FORCE) {
        for (auto* m : motors) {
            m->Disable();
            m->Update(dt);
        }
        return;
    }

    // 麦轮分解: vx/vy/wz → 四轮转速
    float vx = cmd_cache_.vx, vy = cmd_cache_.vy, wz = cmd_cache_.wz;
    lf_->SetRef( vx - vy - wz);
    rf_->SetRef( vx + vy + wz);
    lb_->SetRef( vx + vy - wz);
    rb_->SetRef( vx - vy + wz);

    for (auto* m : motors)
        m->Enable();

    // 两阶段 PID + 功率限制
    PowerMotorState states[4] = {};
    for (int i = 0; i < 4; ++i) {
        states[i].pid_output    = motors[i]->ComputeOutput(dt);
        states[i].speed_rad     = motors[i]->Measure().speed_aps * DEGREE_2_RAD;
        states[i].set_speed_rad = motors[i]->GetRef() * DEGREE_2_RAD;
        states[i].max_output    = CHASSIS_SPEED_MAX_OUT;
    }

    limiter_.UpdateEnergyLoop(cmd_cache_.power_limit, cmd_cache_.buffer_energy,
                              cmd_cache_.measured_power, dt);
    limiter_.Limit(states, 4);

    for (int i = 0; i < 4; ++i)
        motors[i]->ApplyOutput(states[i].pid_output);
}
