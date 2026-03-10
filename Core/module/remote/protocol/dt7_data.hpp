#pragma once

#include <cstdint>
#include <type_traits>

namespace remote {

// ======================== DT7/DR16 开关位置 ========================

enum class SwitchPos : uint8_t {
    UP   = 1,
    DOWN = 2,
    MID  = 3,
};

// ======================== DT7/DR16 键盘按键索引 ========================
// 与 DBUS 协议 16-bit key word 的 bit 位置一一对应
// 使用 unscoped enum 以便直接作为数组下标

enum Dt7Key : uint8_t {
    KEY_W = 0, KEY_S, KEY_D, KEY_A,
    KEY_SHIFT, KEY_CTRL,
    KEY_Q, KEY_E, KEY_R, KEY_F, KEY_G,
    KEY_Z, KEY_X, KEY_C, KEY_V, KEY_B,
    KEY_COUNT = 16,
};

// ======================== 按键修饰模式 ========================

enum Dt7KeyMode : uint8_t {
    KEY_PRESS            = 0,   // 无修饰键
    KEY_PRESS_WITH_CTRL  = 1,   // Ctrl 组合
    KEY_PRESS_WITH_SHIFT = 2,   // Shift 组合
    KEY_MODE_COUNT       = 3,
};

// ======================== 通道常量 ========================

inline constexpr int16_t CH_OFFSET = 1024;
inline constexpr int16_t CH_MAX    = 660;

// ======================== Dt7Data ========================
// 轻量级数据结构, 可通过 Topic 传输
// 与 ins_data.hpp 同级: 仅包含数据 + 极简 helper

struct Dt7Data {
    // 摇杆通道 [-660, 660]
    int16_t ch_r_x;     // 右摇杆水平
    int16_t ch_r_y;     // 右摇杆垂直
    int16_t ch_l_x;     // 左摇杆水平
    int16_t ch_l_y;     // 左摇杆垂直
    int16_t dial;        // 侧面拨轮

    // 拨杆
    SwitchPos sw_l;
    SwitchPos sw_r;

    // 鼠标
    int16_t mouse_x;
    int16_t mouse_y;
    uint8_t mouse_l;
    uint8_t mouse_r;

    // 键盘
    uint16_t keys;               // 原始 16-bit 按键位域
    uint16_t keys_with_ctrl;     // Ctrl 按下时的按键
    uint16_t keys_with_shift;    // Shift 按下时的按键

    // 上升沿计数器 [修饰模式][按键索引]
    uint8_t key_count[KEY_MODE_COUNT][KEY_COUNT];

    // ---- Helper ----
    bool KeyPressed(Dt7Key key) const { return (keys >> key) & 1; }
    bool KeyToggled(Dt7KeyMode mode, Dt7Key key) const {
        return key_count[mode][key] & 1;  // 奇数次 = toggle on
    }
};

static_assert(std::is_trivially_copyable_v<Dt7Data>);

} // namespace remote
