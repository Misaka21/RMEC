#pragma once

#include "sal_can.h"
#include "motor_measure.hpp"
#include "general_def.hpp"

#include <cstdint>

/// LK 电机常量
namespace lk_motor {
inline constexpr float ECD_ANGLE_COEF    = 360.0f / 65536.0f;
inline constexpr float SPEED_SMOOTH_COEF = 0.85f;
inline constexpr float CURRENT_SMOOTH_COEF = 0.9f;
inline constexpr int16_t I_MIN           = -2000;
inline constexpr int16_t I_MAX           =  2000;
} // namespace lk_motor

/// LK 电机驱动配置
struct LkDriverConfig {
    CAN_HandleTypeDef* can_handle = nullptr;
    uint8_t motor_id = 1;  // 电机 ID, 1-4
};

/// LK (瓴控) 电机驱动（多电机批量发送，类似 DJI）
///
/// 协议:
///   - 多电机命令帧 tx_id = 0x280，每电机 2 字节 int16 电流
///   - 反馈帧 rx_id = 0x140 + motor_id
///   - 编码器 65536 counts，反馈含速度(dps)/电流(int16)/温度
///
/// 分组规则 (2 个 TxGroup):
///   Group 0: CAN1, 0x280, motor_id 1-4
///   Group 1: CAN2, 0x280, motor_id 1-4
class LkDriver {
public:
    static constexpr uint8_t MAX_MOTORS        = 4;
    static constexpr uint8_t GROUP_COUNT       = 2;
    static constexpr uint16_t OFFLINE_THRESHOLD = 100;

    explicit LkDriver(const LkDriverConfig& cfg);

    /// 设置控制输出（电流/力矩值），写入 TxGroup 缓冲区
    void SetOutput(float output);

    /// 获取反馈数据
    const MotorMeasure& Measure() const { return measure_; }

    /// 离线检测
    void TickOffline() { online_cnt_++; }

    /// 是否在线
    bool IsOnline() const { return online_cnt_ < OFFLINE_THRESHOLD; }

    /// 批量发送所有激活的 TxGroup
    static void FlushAll();

private:
    /// CAN 反馈解码 (ISR context)
    void DecodeFeedback(const uint8_t* data);

    // ---- 发送分组 ----
    struct TxGroup {
        uint8_t data[8] = {};
        sal::CanInstance* sender = nullptr;
        bool enabled = false;
    };

    static TxGroup tx_groups_[GROUP_COUNT];

    // ---- 实例成员 ----
    MotorMeasure measure_{};
    sal::CanInstance* can_ = nullptr;
    uint8_t group_idx_ = 0;
    uint8_t msg_offset_ = 0;    // 帧内字节偏移 (0/2/4/6)
    uint16_t online_cnt_ = OFFLINE_THRESHOLD;
};
