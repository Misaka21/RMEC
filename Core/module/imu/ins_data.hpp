#pragma once

#include <cstdint>
#include <type_traits>

struct InsData {
    float gyro[3]           = {};   // rad/s
    float accel[3]          = {};   // m/s²
    float q[4]              = {};   // 四元数 [w,x,y,z]
    float euler[3]          = {};   // [yaw, pitch, roll] degrees
    float yaw_total         = 0;    // 多圈 yaw
    float motion_accel_b[3] = {};   // 机体系运动加速度
    float motion_accel_n[3] = {};   // 导航系运动加速度
    float temperature       = 0;    // °C
};
static_assert(std::is_trivially_copyable_v<InsData>);

struct InsConfig {
    float accel_lpf_tau = 0.0085f;  // 运动加速度 LPF 时间常数 (s)
};
