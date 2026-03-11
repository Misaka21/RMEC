#pragma once

#include "sal_can.h"
#include "motor_measure.hpp"
#include "general_def.hpp"
#include "mit_codec.hpp"

#include <cstdint>

/// HT 电机参数范围常量
namespace ht_motor {
inline constexpr float P_MIN  = -95.5f;
inline constexpr float P_MAX  =  95.5f;
inline constexpr float V_MIN  = -45.0f;
inline constexpr float V_MAX  =  45.0f;
inline constexpr float T_MIN  = -18.0f;
inline constexpr float T_MAX  =  18.0f;
inline constexpr float KP_MIN =   0.0f;
inline constexpr float KP_MAX = 500.0f;
inline constexpr float KD_MIN =   0.0f;
inline constexpr float KD_MAX =   5.0f;

inline constexpr float SPEED_BIAS        = -0.0109901428f;  // 速度偏差补偿 (rad/s)
inline constexpr uint8_t SPEED_BUF_SIZE  = 5;               // 滑动均值滤波窗口
} // namespace ht_motor

/// HT 电机模式命令
enum class HtModeCmd : uint8_t {
    MOTOR_MODE    = 0xFC,   // 使能，响应控制指令
    RESET_MODE    = 0xFD,   // 停止
    ZERO_POSITION = 0xFE,   // 将当前位置设为编码器零位
};

/// HT 电机反馈（原始 float 值）
struct HtFeedback {
    float position = 0;     // rad (多圈，±95.5)
    float velocity = 0;     // rad/s (经滑动均值滤波)
    float torque   = 0;     // N·m
};

/// HT 电机驱动配置
struct HtDriverConfig {
    CAN_HandleTypeDef* can_handle = nullptr;
    uint32_t tx_id = 0;
    uint32_t rx_id = 0;
};

/// HT (海泰) 电机驱动（MIT 协议，单电机独立发送）
///
/// 与 DM 电机的关键差异:
///   - 位置范围 ±95.5 rad (vs DM ±12.5)
///   - 速度使用滑动均值滤波 + 偏差补偿
///   - 编码器校准需发送全零 MIT 帧后再发 ZeroPosition 命令
///   - 无 T_Mos/T_Rotor 温度反馈
///   - 掉线时建议在 daemon 回调中调用 SendModeCmd(MOTOR_MODE) 尝试恢复
///
/// 反馈到 MotorMeasure 的映射:
///   total_angle ← position × RAD_2_DEGREE (deg)
///   speed_aps   ← velocity × RAD_2_DEGREE (deg/s)
class HtDriver {
public:
    static constexpr uint8_t MAX_MOTORS        = 4;
    static constexpr uint16_t OFFLINE_THRESHOLD = 100;

    explicit HtDriver(const HtDriverConfig& cfg);

    /// 设置控制输出（纯力矩模式）
    void SetOutput(float torque);

    /// 全 MIT 五参数控制
    void SetMitOutput(float pos, float vel, float kp, float kd, float torque);

    /// 获取电机反馈（MotorMeasure 格式，角度/速度已转换为 deg）
    const MotorMeasure& Measure() const { return measure_; }

    /// 获取 MIT 原始反馈（float，rad/rad·s/N·m）
    const HtFeedback& HtMeasure() const { return ht_fb_; }

    /// 离线检测
    void TickOffline() { online_cnt_++; }

    /// 是否在线
    bool IsOnline() const { return online_cnt_ < OFFLINE_THRESHOLD; }

    /// 发送模式命令
    void SendModeCmd(HtModeCmd cmd);

    /// 编码器校准（发送全零 MIT 帧 + ZeroPosition 命令）
    void CalibrateEncoder();

    /// 批量发送所有已注册实例的控制帧
    static void FlushAll();

private:
    /// CAN 反馈解码 (ISR context)
    void DecodeFeedback(const uint8_t* data);

    /// 滑动均值滤波
    float AverageFilter(float new_val);

    MotorMeasure measure_{};
    HtFeedback ht_fb_{};
    sal::CanInstance* can_ = nullptr;
    uint8_t tx_buf_[8] = {};
    bool tx_pending_ = false;
    uint16_t online_cnt_ = OFFLINE_THRESHOLD;

    // 速度滑动均值滤波缓冲
    float speed_buf_[ht_motor::SPEED_BUF_SIZE] = {};
    uint8_t speed_buf_idx_ = 0;

    // ---- 静态实例注册 ----
    static HtDriver* instances_[MAX_MOTORS];
    static uint8_t instance_count_;
};
