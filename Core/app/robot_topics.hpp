#pragma once

#include "topic.hpp"
#include "ins_data.hpp"
#include "dt7_data.hpp"
#include "vision_data.hpp"
#include "referee_def.hpp"

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

// 云台板 → 底盘板: 控制命令 + 云台姿态
struct Gimbal2ChassisData {
    ChassisMode chassis_mode = ChassisMode::ZERO_FORCE;
    float vx           = 0;     // 前进速度
    float vy           = 0;     // 横移速度
    float wz           = 0;     // 旋转速度
    float offset_angle = 0;     // 云台-底盘偏转角 (deg)
    float yaw_ins      = 0;     // 云台 yaw 陀螺仪角度 (deg), 底盘跟随用
};

// 底盘板 → 云台板: 裁判系统数据转发
struct Chassis2GimbalData {
    // 功率热量 (0x0202, 10Hz)
    uint16_t buffer_energy          = 0;  // 缓冲能量 (J)
    uint16_t shooter_17mm_heat      = 0;  // 17mm 枪口热量
    uint16_t shooter_42mm_heat      = 0;  // 42mm 枪口热量

    // 机器人状态 (0x0201, 10Hz)
    uint16_t shooter_heat_limit     = 0;  // 热量上限
    uint16_t shooter_cooling_value  = 0;  // 每秒冷却值
    uint16_t chassis_power_limit    = 0;  // 功率上限
    uint8_t  robot_level            = 0;  // 机器人等级
    uint8_t  robot_id               = 0;  // 机器人 ID (< 7 红方, > 7 蓝方)
    uint8_t  power_output           = 0;  // bit0:gimbal, bit1:chassis, bit2:shooter

    // 允许发弹量 (0x0208, 10Hz)
    uint16_t projectile_allowance_17mm = 0;
    uint16_t projectile_allowance_42mm = 0;
    uint16_t remaining_gold_coin       = 0;

    // 射击信息 (0x0207, 弹丸发射后)
    float    bullet_speed           = 0;  // 实测弹速 (m/s)

    // 比赛状态 (0x0001, 1Hz)
    uint8_t  game_progress          = 0;  // 比赛阶段

    // 增益 (0x0204, 3Hz)
    uint8_t  recovery_buff          = 0;
    uint16_t cooling_buff           = 0;  // 冷却增益 (具体值, uint16_t)
    uint16_t attack_buff            = 0;

    // 哨兵信息 (0x020D, 1Hz)
    uint32_t sentry_info            = 0;
    uint16_t sentry_info_2          = 0;
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

// 视觉通信接收数据 (ISR 回调发布)
inline Topic<vision::VisionRxData> vision_topic;

// 裁判系统数据 (10 Hz, referee_task 发布)
inline Topic<referee::RefereeData> referee_topic;

// 双板通信接收数据 (100 Hz, comm_task 发布)
inline Topic<BoardCommRxData> board_comm_topic;
