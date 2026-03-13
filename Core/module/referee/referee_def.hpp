#pragma once

#include <cstdint>
#include <type_traits>

namespace referee {

// ======================== 比赛状态 (0x0001, 1Hz) ========================

struct GameStatus {
    uint8_t  game_type          = 0;  // bit 0-3
    uint8_t  game_progress      = 0;  // bit 4-7
    uint16_t stage_remain_time  = 0;  // 当前阶段剩余时间 (s)
    uint64_t sync_time_stamp    = 0;  // UNIX 时间, 当机器人正确连接裁判系统后生效
};

// ======================== 比赛结果 (0x0002, 比赛结束后) ========================

struct GameResult {
    uint8_t winner = 0;  // 0=平局, 1=红方, 2=蓝方
};

// ======================== 机器人血量 (0x0003, 3Hz) ========================

struct GameRobotHp {
    // 红方
    uint16_t red_1_hp  = 0;  // 英雄
    uint16_t red_2_hp  = 0;  // 工程
    uint16_t red_3_hp  = 0;  // 步兵3
    uint16_t red_4_hp  = 0;  // 步兵4
    uint16_t red_7_hp  = 0;  // 哨兵
    uint16_t red_outpost_hp = 0;  // 前哨站
    uint16_t red_base_hp    = 0;  // 基地
    // 蓝方
    uint16_t blue_1_hp = 0;
    uint16_t blue_2_hp = 0;
    uint16_t blue_3_hp = 0;
    uint16_t blue_4_hp = 0;
    uint16_t blue_7_hp = 0;
    uint16_t blue_outpost_hp = 0;
    uint16_t blue_base_hp    = 0;
};

// ======================== 场地事件 (0x0101, 3Hz) ========================

struct EventData {
    uint32_t event_type = 0;  // bit 编码的场地事件
};

// ======================== 裁判警告 (0x0104, 警告后) ========================

struct RefereeWarning {
    uint8_t  level           = 0;  // 警告等级
    uint8_t  offending_robot = 0;  // 犯规机器人 ID
    uint8_t  count           = 0;  // 犯规次数
};

// ======================== 飞镖信息 (0x0105, 1Hz) ========================

struct DartInfo {
    uint8_t  dart_remaining_time = 0;
    uint16_t dart_info           = 0;  // bit 编码
};

// ======================== 机器人状态 (0x0201, 10Hz) ========================

struct RobotStatus {
    uint8_t  robot_id                = 0;
    uint8_t  robot_level             = 0;
    uint16_t current_hp              = 0;
    uint16_t maximum_hp              = 0;
    uint16_t shooter_barrel_cooling_value = 0;  // 每秒冷却值
    uint16_t shooter_barrel_heat_limit    = 0;  // 热量上限
    uint16_t chassis_power_limit     = 0;
    uint8_t  power_management_output = 0;  // bit0:gimbal, bit1:chassis, bit2:shooter
};

// ======================== 功率热量 (0x0202, 10Hz) ========================
// V1.2.0: 前6字节保留, 无 chassis_power

struct PowerHeat {
    uint16_t reserved1          = 0;
    uint16_t reserved2          = 0;
    uint16_t reserved3          = 0;
    uint16_t buffer_energy      = 0;  // 缓冲能量 (J)
    uint16_t shooter_17mm_1_barrel_heat = 0;
    uint16_t shooter_17mm_2_barrel_heat = 0;
    uint16_t shooter_42mm_barrel_heat   = 0;
};

// ======================== 机器人位置 (0x0203, 10Hz) ========================

struct RobotPos {
    float x     = 0;
    float y     = 0;
    float angle = 0;  // 枪口朝向角度
};

// ======================== 增益 (0x0204, 3Hz) ========================

struct Buff {
    uint8_t  recovery_buff     = 0;
    uint8_t  cooling_buff      = 0;
    uint8_t  defence_buff      = 0;
    uint8_t  vulnerability_buff = 0;
    uint16_t attack_buff       = 0;
    bool     power_rune_active = false;
    bool     power_rune_available = false;
};

// ======================== 伤害状态 (0x0206, 受伤后) ========================

struct HurtData {
    uint8_t armor_id  = 0;  // bit 0-3: 装甲板 ID
    uint8_t hurt_type = 0;  // bit 4-7: 伤害类型
};

// ======================== 射击信息 (0x0207, 弹丸发射后) ========================

struct ShootData {
    uint8_t bullet_type  = 0;  // 1=17mm, 2=42mm
    uint8_t shooter_num  = 0;  // 发射机构 ID
    uint8_t launching_frequency = 0;  // Hz
    float   initial_speed = 0;  // m/s
};

// ======================== 允许发弹量 (0x0208, 10Hz) ========================

struct ProjectileAllowance {
    uint16_t projectile_allowance_17mm = 0;
    uint16_t projectile_allowance_42mm = 0;
    uint16_t remaining_gold_coin       = 0;
};

// ======================== RFID 状态 (0x0209, 3Hz) ========================

struct RfidStatus {
    uint32_t rfid_status = 0;  // bit 编码
    uint8_t  rfid_buff   = 0;
};

// ======================== 全场机器人位置 (0x020B, 1Hz, 哨兵专用) ========================

struct GroundRobotPosition {
    float hero_x       = 0;
    float hero_y       = 0;
    float engineer_x   = 0;
    float engineer_y   = 0;
    float standard_3_x = 0;
    float standard_3_y = 0;
    float standard_4_x = 0;
    float standard_4_y = 0;
    float reserved_x   = 0;
    float reserved_y   = 0;
};

// ======================== 哨兵信息同步 (0x020D, 1Hz) ========================

struct SentryInfo {
    uint32_t sentry_info   = 0;
    uint16_t sentry_info_2 = 0;
};

// ======================== 自定义控制器 (0x0302, 30Hz, 图传链路) ========================

struct CustomController {
    uint8_t data[30] = {};
    bool    updated  = false;
};

// ======================== 聚合数据: 大结构体嵌套小结构体 ========================

struct RefereeData {
    GameStatus           game_status;
    GameResult           game_result;
    GameRobotHp          game_robot_hp;
    EventData            event_data;
    RefereeWarning       referee_warning;
    DartInfo             dart_info;
    RobotStatus          robot_status;
    PowerHeat            power_heat;
    RobotPos             robot_pos;
    Buff                 buff;
    HurtData             hurt_data;
    ShootData            shoot_data;
    ProjectileAllowance  projectile_allowance;
    RfidStatus           rfid_status;
    GroundRobotPosition  ground_robot_position;
    SentryInfo           sentry_info;
    CustomController     custom_controller;
};

static_assert(std::is_trivially_copyable_v<GameStatus>);
static_assert(std::is_trivially_copyable_v<GameResult>);
static_assert(std::is_trivially_copyable_v<GameRobotHp>);
static_assert(std::is_trivially_copyable_v<RobotStatus>);
static_assert(std::is_trivially_copyable_v<PowerHeat>);
static_assert(std::is_trivially_copyable_v<ShootData>);
static_assert(std::is_trivially_copyable_v<ProjectileAllowance>);
static_assert(std::is_trivially_copyable_v<RefereeData>);

} // namespace referee
