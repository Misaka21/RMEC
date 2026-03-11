#pragma once

#include <cstdint>
#include <type_traits>

namespace vision {

// ======================== 自瞄模式 ========================

enum class AimMode : uint8_t {
    OFF        = 0,
    AUTO_AIM   = 1,
    SMALL_BUFF = 2,
    BIG_BUFF   = 3,
};

// ======================== 敌方颜色 ========================

enum class EnemyColor : uint8_t {
    UNKNOWN = 0,
    RED     = 1,
    BLUE    = 2,
};

// ======================== 视觉 → 电控 (上位机发) ========================
// 32B 协议布局:
//   [0] head=0xFF, [1] control, [2] shoot,
//   [3-6] yaw, [7-10] pitch, [11-28] reserved,
//   [29-30] CRC16, [31] tail=0x0D

struct VisionRxData {
    uint8_t control;    // 控制标志: 1=控制
    uint8_t shoot;      // 射击标志: 1=射击
    float   yaw;        // 目标偏航角 (rad)
    float   pitch;      // 目标俯仰角 (rad)
};

// ======================== 电控 → 视觉 (电控发) ========================
// 32B 协议布局:
//   [0] head=0xFF, [1] mode, [2] aiming_lock,
//   [3-6] bullet_speed, [7-10] yaw, [11-14] pitch, [15-18] roll,
//   [19] enemy_color, [20-28] reserved,
//   [29-30] CRC16, [31] tail=0x0D

struct VisionTxData {
    AimMode    aim_mode      = AimMode::OFF;
    uint8_t    aiming_lock   = 0;
    float      bullet_speed  = 15.0f;    // m/s
    float      yaw           = 0;        // rad
    float      pitch         = 0;        // rad
    float      roll          = 0;        // rad
    EnemyColor enemy_color   = EnemyColor::UNKNOWN;
};

static_assert(std::is_trivially_copyable_v<VisionRxData>);
static_assert(std::is_trivially_copyable_v<VisionTxData>);

} // namespace vision
