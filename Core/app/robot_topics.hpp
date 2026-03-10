#pragma once

#include "topic.hpp"
#include "ins_data.hpp"
#include "robot_types.hpp"

// IMU 姿态 (1 kHz, ins_task 发布)
inline Topic<InsData> ins_topic;

// CMD → 子系统 命令 (200 Hz, robot_cmd 发布)
inline Topic<ChassisCmdData>  chassis_cmd_topic;
inline Topic<GimbalCmdData>   gimbal_cmd_topic;
inline Topic<ShootCmdData>    shoot_cmd_topic;

// 子系统 → CMD 反馈 (200 Hz, 各子系统发布)
inline Topic<ChassisFeedData> chassis_feed_topic;
inline Topic<GimbalFeedData>  gimbal_feed_topic;
inline Topic<ShootFeedData>   shoot_feed_topic;
