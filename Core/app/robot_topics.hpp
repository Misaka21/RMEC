#pragma once

#include "topic.hpp"
#include "ins_data.hpp"
#include "dt7_data.hpp"

#include <cstdint>
#include <type_traits>

// ======================== 机器人全局状态 ========================

enum class RobotStatus : uint8_t { STOP, READY };
enum class AppStatus   : uint8_t { OFFLINE, ONLINE, ERROR };

// ======================== 子系统模式 ========================

enum class ChassisMode : uint8_t {
    ZERO_FORCE,
    FOLLOW_GIMBAL_YAW,
    NO_FOLLOW,
    ROTATE,
};

enum class GimbalMode : uint8_t {
    ZERO_FORCE,
    FREE_MODE,
    GYRO_MODE,
};

enum class ShootMode    : uint8_t { OFF, ON };
enum class FrictionMode : uint8_t { OFF, ON };
enum class LidMode      : uint8_t { OPEN, CLOSE };
enum class LoaderMode   : uint8_t {
    STOP,
    REVERSE,
    SINGLE,
    TRIPLE,
    BURST,
};
enum class BulletSpeed  : uint8_t { SMALL_15, SMALL_18, SMALL_30 };

// ======================== CMD → 子系统 命令 ========================

struct ChassisCmdData {
    float vx           = 0;
    float vy           = 0;
    float wz           = 0;
    float offset_angle = 0;
    ChassisMode mode   = ChassisMode::ZERO_FORCE;
    int speed_buff     = 100;
};

struct GimbalCmdData {
    float yaw          = 0;
    float pitch        = 0;
    GimbalMode mode    = GimbalMode::ZERO_FORCE;
};

struct ShootCmdData {
    ShootMode    shoot_mode    = ShootMode::OFF;
    LoaderMode   load_mode     = LoaderMode::STOP;
    FrictionMode friction_mode = FrictionMode::OFF;
    LidMode      lid_mode      = LidMode::CLOSE;
    BulletSpeed  bullet_speed  = BulletSpeed::SMALL_15;
    float        shoot_rate    = 0;
    uint8_t      rest_heat     = 0;
};

// ======================== 子系统 → CMD 反馈 ========================

struct ChassisFeedData {
    uint8_t     rest_heat    = 0;
    BulletSpeed bullet_speed = BulletSpeed::SMALL_15;
};

struct GimbalFeedData {
    float yaw_total        = 0;
    float yaw_single_round = 0;
    float pitch            = 0;
};

struct ShootFeedData {
    // 预留
};

static_assert(std::is_trivially_copyable_v<ChassisCmdData>);
static_assert(std::is_trivially_copyable_v<GimbalCmdData>);
static_assert(std::is_trivially_copyable_v<ShootCmdData>);
static_assert(std::is_trivially_copyable_v<ChassisFeedData>);
static_assert(std::is_trivially_copyable_v<GimbalFeedData>);
static_assert(std::is_trivially_copyable_v<ShootFeedData>);

// ======================== Topic 实例 ========================

// IMU 姿态 (1 kHz, ins_task 发布)
inline Topic<InsData> ins_topic;

// 遥控器 (~70 Hz, ISR 回调发布)
inline Topic<remote::Dt7Data> remote_topic;

// CMD → 子系统 命令 (200 Hz, robot_cmd 发布)
inline Topic<ChassisCmdData>  chassis_cmd_topic;
inline Topic<GimbalCmdData>   gimbal_cmd_topic;
inline Topic<ShootCmdData>    shoot_cmd_topic;

// 子系统 → CMD 反馈 (200 Hz, 各子系统发布)
inline Topic<ChassisFeedData> chassis_feed_topic;
inline Topic<GimbalFeedData>  gimbal_feed_topic;
inline Topic<ShootFeedData>   shoot_feed_topic;
