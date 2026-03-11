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
    float vx             = 0;
    float vy             = 0;
    float wz             = 0;
    float offset_angle   = 0;
    ChassisMode mode     = ChassisMode::ZERO_FORCE;
    float power_limit    = 80.0f;    // 裁判系统功率上限 (W)
    float buffer_energy  = 60.0f;    // 缓冲能量 (J)
    float measured_power = 0.0f;     // 实测底盘功率 (W)
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

// ======================== 双板通信数据 ========================

// 按物理方向定义, 两板共用同一份代码
#pragma pack(1)
struct Gimbal2ChassisData {
    uint8_t placeholder[8];  // TODO: 填入实际字段 (如底盘命令)
};
struct Chassis2GimbalData {
    uint8_t placeholder[8];  // TODO: 填入实际字段 (如功率反馈)
};
#pragma pack()

static_assert(std::is_trivially_copyable_v<Gimbal2ChassisData>);
static_assert(std::is_trivially_copyable_v<Chassis2GimbalData>);

// 板级自动切换 Tx/Rx 方向, 改 define 即全切
#if defined(GIMBAL_BOARD)
using BoardCommTxData = Gimbal2ChassisData;
using BoardCommRxData = Chassis2GimbalData;
#elif defined(CHASSIS_BOARD)
using BoardCommTxData = Chassis2GimbalData;
using BoardCommRxData = Gimbal2ChassisData;
#else // ONE_BOARD — 不使用双板通信, 保留类型定义供编译通过
using BoardCommTxData = Gimbal2ChassisData;
using BoardCommRxData = Chassis2GimbalData;
#endif

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

// 双板通信接收数据 (100 Hz, comm_task 发布)
inline Topic<BoardCommRxData> board_comm_topic;
