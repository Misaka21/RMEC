#pragma once

#include <cmath>
#include <cstdint>

/// PID 改进项标志位
namespace pid {
inline constexpr uint32_t NONE                       = 0x00;
inline constexpr uint32_t INTEGRAL_LIMIT             = 0x01;
inline constexpr uint32_t DERIVATIVE_ON_MEASUREMENT  = 0x02;
inline constexpr uint32_t TRAPEZOIDAL_INTEGRAL       = 0x04;
inline constexpr uint32_t PROPORTIONAL_ON_MEASUREMENT = 0x08;
inline constexpr uint32_t OUTPUT_FILTER              = 0x10;
inline constexpr uint32_t CHANGING_INTEGRATION_RATE  = 0x20;
inline constexpr uint32_t DERIVATIVE_FILTER          = 0x40;
inline constexpr uint32_t ERROR_HANDLE               = 0x80;
} // namespace pid

/// PID 控制器配置
struct PidConfig {
    float kp = 0;
    float ki = 0;
    float kd = 0;
    float max_out = 0;
    float dead_band = 0;
    uint32_t improve_flags = pid::NONE;
    float integral_limit = 0;
    float coef_a = 0;               // 变速积分上限宽度
    float coef_b = 0;               // 变速积分下限（全积分区域）
    float output_lpf_rc = 0;
    float derivative_lpf_rc = 0;
};

/// PID 堵转检测错误类型
enum class PidError : uint8_t {
    NONE = 0,
    MOTOR_BLOCKED = 1,
};

/// PID 控制器（位置式，dt 通过参数注入，不依赖 DWT）
class PidController final {
public:
    PidController() noexcept = default;
    explicit PidController(const PidConfig& cfg) noexcept;

    void Init(const PidConfig& cfg) noexcept;

    /// 计算 PID 输出
    /// @param measure 反馈值
    /// @param ref     设定值
    /// @param dt      距上次调用的时间间隔（秒）
    [[nodiscard]] float Calculate(float measure, float ref, float dt) noexcept;

    void Reset() noexcept;

    [[nodiscard]] float Output() const noexcept { return output_; }
    [[nodiscard]] PidError LastError() const noexcept { return error_type_; }

private:
    void ErrorHandle() noexcept;

    // config
    float kp_{}, ki_{}, kd_{};
    float max_out_{};
    float dead_band_{};
    uint32_t improve_flags_{};
    float integral_limit_{};
    float coef_a_{}, coef_b_{};
    float output_lpf_rc_{}, derivative_lpf_rc_{};

    // runtime state
    float measure_{}, last_measure_{};
    float err_{}, last_err_{};
    float ref_{};
    float pout_{}, iout_{}, dout_{};
    float iterm_{}, last_iterm_{};
    float output_{}, last_output_{};
    float last_dout_{};

    // error handle
    uint32_t error_count_{};
    PidError error_type_{PidError::NONE};
};
