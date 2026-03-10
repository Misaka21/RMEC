#pragma once

#include "general_def.hpp"

#include <cmath>
#include <cstdint>
#include <cstring>

namespace ahrs {

inline float InvSqrt(float x) {
    float halfx = 0.5f * x;
    float y = x;
    uint32_t i;
    std::memcpy(&i, &y, sizeof(i));
    i = 0x5f375a86 - (i >> 1);
    std::memcpy(&y, &i, sizeof(y));
    y = y * (1.5f - (halfx * y * y));
    return y;
}

inline void Norm3d(float v[3]) {
    float inv_norm = InvSqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    v[0] *= inv_norm;
    v[1] *= inv_norm;
    v[2] *= inv_norm;
}

inline float Dot3d(const float a[3], const float b[3]) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

inline void Cross3d(const float a[3], const float b[3], float out[3]) {
    out[0] = a[1] * b[2] - a[2] * b[1];
    out[1] = a[2] * b[0] - a[0] * b[2];
    out[2] = a[0] * b[1] - a[1] * b[0];
}

inline void BodyFrameToEarthFrame(const float *bf, float *ef, const float *q) {
    ef[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * bf[0] +
                     (q[1] * q[2] - q[0] * q[3]) * bf[1] +
                     (q[1] * q[3] + q[0] * q[2]) * bf[2]);

    ef[1] = 2.0f * ((q[1] * q[2] + q[0] * q[3]) * bf[0] +
                     (0.5f - q[1] * q[1] - q[3] * q[3]) * bf[1] +
                     (q[2] * q[3] - q[0] * q[1]) * bf[2]);

    ef[2] = 2.0f * ((q[1] * q[3] - q[0] * q[2]) * bf[0] +
                     (q[2] * q[3] + q[0] * q[1]) * bf[1] +
                     (0.5f - q[1] * q[1] - q[2] * q[2]) * bf[2]);
}

inline void EarthFrameToBodyFrame(const float *ef, float *bf, const float *q) {
    bf[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * ef[0] +
                     (q[1] * q[2] + q[0] * q[3]) * ef[1] +
                     (q[1] * q[3] - q[0] * q[2]) * ef[2]);

    bf[1] = 2.0f * ((q[1] * q[2] - q[0] * q[3]) * ef[0] +
                     (0.5f - q[1] * q[1] - q[3] * q[3]) * ef[1] +
                     (q[2] * q[3] + q[0] * q[1]) * ef[2]);

    bf[2] = 2.0f * ((q[1] * q[3] + q[0] * q[2]) * ef[0] +
                     (q[2] * q[3] - q[0] * q[1]) * ef[1] +
                     (0.5f - q[1] * q[1] - q[2] * q[2]) * ef[2]);
}

/// Compute initial quaternion from averaged accelerometer reading (gravity direction).
/// Yaw is set to 0. Input should be raw averaged accel, not normalized.
inline void GravityToQuaternion(const float acc_avg[3], float q_out[4]) {
    float acc[3] = {acc_avg[0], acc_avg[1], acc_avg[2]};
    Norm3d(acc);

    const float gravity_n[3] = {0.0f, 0.0f, 1.0f};
    float angle = acosf(Dot3d(acc, gravity_n));

    float axis[3];
    Cross3d(acc, gravity_n, axis);
    Norm3d(axis);

    float sin_half = sinf(angle / 2.0f);
    q_out[0] = cosf(angle / 2.0f);
    q_out[1] = axis[0] * sin_half;
    q_out[2] = axis[1] * sin_half;
    q_out[3] = axis[2] * sin_half;
}

} // namespace ahrs
