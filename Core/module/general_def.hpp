#pragma once

#include <cstdint>

// ---- 数学常量 ----
inline constexpr float PI                  = 3.1415926535f;
inline constexpr float PI2                 = PI * 2.0f;
inline constexpr float RAD_2_DEGREE        = 57.2957795f;       // 180/PI
inline constexpr float DEGREE_2_RAD        = 0.01745329252f;    // PI/180
inline constexpr float RPM_2_ANGLE_PER_SEC = 6.0f;              // 360/60
inline constexpr float RPM_2_RAD_PER_SEC   = 0.104719755f;      // 2*PI/60

// ---- 编码器常量 ----
inline constexpr float ECD_ANGLE_COEF      = 0.043945f;         // 360/8192

// ---- 钳位 ----
inline constexpr float Clamp(float val, float min_val, float max_val) {
    return val < min_val ? min_val : (val > max_val ? max_val : val);
}
inline constexpr float ClampAbs(float val, float limit) {
    return val > limit ? limit : (val < -limit ? -limit : val);
}

// ---- 低通滤波器 ----
// 一阶 IIR: y = (1-alpha)*y_prev + alpha*x
// alpha = coef (0~1), coef 越大新值权重越大
inline constexpr float LowPassFilter(float prev, float new_val, float coef) {
    return (1.0f - coef) * prev + coef * new_val;
}
