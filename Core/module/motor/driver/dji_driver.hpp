#pragma once

#include "sal_can.h"
#include "motor_measure.hpp"
#include "general_def.hpp"

#include <cstdint>

/// 速度和电流低通滤波系数
inline constexpr float SPEED_SMOOTH_COEF   = 0.85f;
inline constexpr float CURRENT_SMOOTH_COEF = 0.9f;

/// DJI 电机类型
enum class DjiMotorType : uint8_t {
    M3508 = 0,
    M2006,
    GM6020,
};

/// DJI 电机物理参数（SetOutput 输入单位: M3508/M2006 为安培, GM6020 为伏特）
namespace dji_motor {
inline constexpr float M3508_MAX_CURRENT  = 20.0f;             // A
inline constexpr float M3508_RAW_PER_AMP  = 16384.0f / 20.0f;  // ≈819.2
inline constexpr float M2006_MAX_CURRENT  = 10.0f;             // A
inline constexpr float M2006_RAW_PER_AMP  = 10000.0f / 10.0f;  // =1000.0
inline constexpr float GM6020_MAX_VOLTAGE = 24.0f;             // V
inline constexpr float GM6020_RAW_PER_VOLT = 30000.0f / 24.0f; // =1250.0
} // namespace dji_motor

/// DJI 电机驱动配置
struct DjiDriverConfig {
    DjiMotorType motor_type = DjiMotorType::M3508;
    CAN_HandleTypeDef* can_handle = nullptr;
    uint8_t motor_id = 1;  // 电调/拨码 ID, 1-8(M3508/M2006), 1-7(GM6020)
};

/// DJI 电机驱动（CAN 协议编解码 + 批量发送）
///
/// 负责:
///   - 接收 CAN 反馈数据并解码到 MotorMeasure
///   - 将控制量写入分组缓冲区
///   - 静态 FlushAll() 批量发送所有 CAN 帧
///
/// 分组规则 (6 个 TxGroup):
///   Group 0: CAN1, 0x200, M3508/M2006 ID 1-4
///   Group 1: CAN1, 0x1FF, M3508/M2006 ID 5-8, GM6020 ID 1-4
///   Group 2: CAN1, 0x2FF, GM6020 ID 5-7
///   Group 3: CAN2, 0x200, M3508/M2006 ID 1-4
///   Group 4: CAN2, 0x1FF, M3508/M2006 ID 5-8, GM6020 ID 1-4
///   Group 5: CAN2, 0x2FF, GM6020 ID 5-7
class DjiDriver {
public:
    static constexpr uint8_t MAX_MOTORS   = 12;
    static constexpr uint8_t GROUP_COUNT  = 6;
    static constexpr uint16_t OFFLINE_THRESHOLD = 100;  // 100 次 Tick 未收到反馈视为离线

    explicit DjiDriver(const DjiDriverConfig& cfg);

    /// 设置控制输出（M3508/M2006: 安培, GM6020: 伏特），Driver 内部换算为 CAN 原始值
    void SetOutput(float output);

    /// 获取反馈数据
    const MotorMeasure& Measure() const { return measure_; }
    MotorMeasure& MeasureMut() { return measure_; }

    /// 离线检测：每次控制循环调用，递增离线计数器
    void TickOffline() { online_cnt_++; }

    /// 是否在线
    bool IsOnline() const { return online_cnt_ < OFFLINE_THRESHOLD; }

    /// 获取电机类型
    DjiMotorType MotorType() const { return motor_type_; }

    /// 批量发送所有激活的 TxGroup（全局调用，每个控制周期调用一次）
    static void FlushAll();

private:
    /// CAN 反馈解码 (ISR context)
    void DecodeFeedback(const uint8_t* data);

    /// 计算发送分组索引和帧内偏移
    void SetupGrouping(const DjiDriverConfig& cfg);

    // ---- 发送分组 ----
    struct TxGroup {
        uint8_t data[8] = {};               // 发送缓冲区
        sal::CanInstance* sender = nullptr;  // 借用该组某个电机的 CAN 实例发送
        bool enabled = false;               // 是否有电机注册到此组
    };

    static TxGroup tx_groups_[GROUP_COUNT];
    static constexpr uint16_t GROUP_STD_IDS[GROUP_COUNT] = {
        0x200, 0x1FF, 0x2FF,  // CAN1
        0x200, 0x1FF, 0x2FF,  // CAN2
    };

    // ---- 实例成员 ----
    DjiMotorType motor_type_{};
    MotorMeasure measure_{};
    sal::CanInstance* can_ = nullptr;       // 接收反馈用（生命周期由 SAL 管理）
    uint8_t group_idx_ = 0;                // 所属 TxGroup 索引
    uint8_t msg_offset_ = 0;               // 帧内字节偏移 (0/2/4/6)
    uint16_t online_cnt_ = OFFLINE_THRESHOLD;  // 初始视为离线
    float physical_to_raw_ = 1.0f;         // 物理量→CAN 原始值换算因子
    float max_physical_ = 0.0f;            // 物理量上限 (用于 Clamp)
};
