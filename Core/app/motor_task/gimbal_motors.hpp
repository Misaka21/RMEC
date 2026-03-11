#pragma once

#include "motor.hpp"
#include "dji_driver.hpp"
#include "cascade_pid.hpp"
#include "topic.hpp"
#include "robot_topics.hpp"
#include "ins_data.hpp"

/// 云台电机执行层（motor_task 独占，外部通过 gimbal_cmd_topic 下发命令）
///
/// 数据流: gimbal_cmd_topic + ins_topic → Tick() → PID → CAN 缓冲
///         Tick() → gimbal_feed_topic (发布反馈供 cmd_task 使用)
class GimbalMotors {
public:
    void Init();
    void Tick(float dt);

private:
    using M = Motor<DjiDriver, CascadePid>;
    M* yaw_   = nullptr;
    M* pitch_ = nullptr;

    TopicReader<GimbalCmdData>* cmd_reader_ = nullptr;
    TopicReader<InsData>* ins_reader_       = nullptr;
    GimbalCmdData cmd_cache_{};
    InsData ins_cache_{};
};
