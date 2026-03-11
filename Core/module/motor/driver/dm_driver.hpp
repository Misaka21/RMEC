#pragma once

#include "sal_can.h"
#include "motor_measure.hpp"
#include "general_def.hpp"
#include "mit_codec.hpp"

#include <cstdint>

/// DM 电机参数范围常量
namespace dm_motor {
inline constexpr float P_MIN  = -12.5f;
inline constexpr float P_MAX  =  12.5f;
inline constexpr float V_MIN  = -45.0f;
inline constexpr float V_MAX  =  45.0f;
inline constexpr float T_MIN  = -18.0f;
inline constexpr float T_MAX  =  18.0f;
inline constexpr float KP_MIN =   0.0f;
inline constexpr float KP_MAX = 500.0f;
inline constexpr float KD_MIN =   0.0f;
inline constexpr float KD_MAX =   5.0f;
} // namespace dm_motor

/// DM 电机模式命令
enum class DmModeCmd : uint8_t {
    MOTOR_MODE    = 0xFC,   // 使能，响应控制指令
    RESET_MODE    = 0xFD,   // 停止
    ZERO_POSITION = 0xFE,   // 将当前位置设为编码器零位
    CLEAR_ERROR   = 0xFB,   // 清除电机过热错误
};

/// DM 电机 MIT 协议反馈（原始 float 值）
struct DmFeedback {
    float position = 0;   // rad
    float velocity = 0;   // rad/s
    float torque   = 0;   // N·m
    float t_mos    = 0;   // MOS 温度 (°C)
    float t_rotor  = 0;   // 转子温度 (°C)
};

/// DM 电机驱动配置
struct DmDriverConfig {
    CAN_HandleTypeDef* can_handle = nullptr;
    uint32_t tx_id = 0;   // 主控 → 电机
    uint32_t rx_id = 0;   // 电机 → 主控
};

/// DM (达妙) 电机驱动（MIT 协议，单电机独立发送）
///
/// 负责:
///   - MIT 协议帧编解码
///   - 接收 CAN 反馈并解析 position/velocity/torque/温度
///   - 支持纯力矩模式 (SetOutput) 和全 MIT 五参数控制 (SetMitOutput)
///   - 静态 FlushAll() 逐一发送所有已注册实例的控制帧
///
/// 反馈到 MotorMeasure 的映射:
///   total_angle ← position × RAD_2_DEGREE (deg)
///   speed_aps   ← velocity × RAD_2_DEGREE (deg/s)
///   ecd 相关字段不使用（MIT 电机无原始编码器值）
///   原始 float 数据通过 DmMeasure() 获取
class DmDriver {
public:
    static constexpr uint8_t MAX_MOTORS        = 4;
    static constexpr uint16_t OFFLINE_THRESHOLD = 100;

    explicit DmDriver(const DmDriverConfig& cfg);

    /// 设置控制输出（纯力矩模式：Kp=0, Kd=0, pos=0, vel=0）
    void SetOutput(float torque);

    /// 全 MIT 五参数控制
    void SetMitOutput(float pos, float vel, float kp, float kd, float torque);

    /// 获取电机反馈（MotorMeasure 格式，角度/速度已转换为 deg）
    const MotorMeasure& Measure() const { return measure_; }

    /// 获取 MIT 原始反馈（float，rad/rad·s/N·m）
    const DmFeedback& DmMeasure() const { return dm_fb_; }

    /// 离线检测：每次控制循环调用
    void TickOffline() { online_cnt_++; }

    /// 是否在线
    bool IsOnline() const { return online_cnt_ < OFFLINE_THRESHOLD; }

    /// 发送模式命令（使能/停止/零位/清错）
    void SendModeCmd(DmModeCmd cmd);

    /// 批量发送所有已注册实例的控制帧
    static void FlushAll();

private:
    /// CAN 反馈解码 (ISR context)
    void DecodeFeedback(const uint8_t* data);

    MotorMeasure measure_{};
    DmFeedback dm_fb_{};
    sal::CanInstance* can_ = nullptr;
    uint8_t tx_buf_[8] = {};
    bool tx_pending_ = false;
    uint16_t online_cnt_ = OFFLINE_THRESHOLD;

    // ---- 静态实例注册 ----
    static DmDriver* instances_[MAX_MOTORS];
    static uint8_t instance_count_;
};
