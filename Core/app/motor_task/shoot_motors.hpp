#pragma once

#include "motor.hpp"
#include "dji_driver.hpp"
#include "cascade_pid.hpp"
#include "topic.hpp"
#include "robot_topics.hpp"

#include <cstdint>

/// 发射电机执行层（motor_task 独占，外部通过 shoot_cmd_topic 下发命令）
///
/// 数据流: shoot_cmd_topic → Tick() → 模式处理 → PID → CAN 缓冲
class ShootMotors {
public:
    void Init();
    void Tick(float dt);

private:
    using M = Motor<DjiDriver, CascadePid>;
    M* friction_l_ = nullptr;
    M* friction_r_ = nullptr;
    M* loader_     = nullptr;

    TopicReader<ShootCmdData>* cmd_reader_ = nullptr;
    ShootCmdData cmd_cache_{};

    float loader_angle_target_ = 0;  // 单发/三连角度目标
};
