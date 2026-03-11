#pragma once

#include "motor.hpp"
#include "dji_driver.hpp"
#include "cascade_pid.hpp"
#include "power_limiter.hpp"
#include "topic.hpp"
#include "robot_topics.hpp"

/// 底盘电机执行层（motor_task 独占，外部通过 chassis_cmd_topic 下发命令）
///
/// 数据流: chassis_cmd_topic → Tick() → 麦轮分解 → PID → 功率限制 → CAN 缓冲
class ChassisMotors {
public:
    void Init();
    void Tick(float dt);

private:
    using M = Motor<DjiDriver, CascadePid>;
    M* lf_ = nullptr;
    M* rf_ = nullptr;
    M* lb_ = nullptr;
    M* rb_ = nullptr;
    PowerLimiter limiter_;

    TopicReader<ChassisCmdData>* cmd_reader_ = nullptr;
    ChassisCmdData cmd_cache_{};
};
