#pragma once

#include "ins_data.hpp"
#include "quaternion_ekf.hpp"
#include "ahrs_math.hpp"
#include "general_def.hpp"

#include <cstring>

class Ins {
public:
    void Init(const float init_q[4], const InsConfig& cfg = {}) {
        accel_lpf_tau_ = cfg.accel_lpf_tau;
        ekf_.Init(init_q, {});
        data_ = {};
        std::memset(motion_accel_b_, 0, sizeof(motion_accel_b_));
    }

    void Update(const float gyro[3], const float accel[3], float dt, float temperature) {
        ekf_.Update(gyro[0], gyro[1], gyro[2],
                    accel[0], accel[1], accel[2], dt);

        // 复制原始传感器数据
        for (int i = 0; i < 3; i++) {
            data_.gyro[i] = gyro[i];
            data_.accel[i] = accel[i];
        }
        data_.temperature = temperature;

        // 复制 EKF 输出
        const auto& out = ekf_.Output();
        for (int i = 0; i < 4; i++) data_.q[i] = out.q[i];
        for (int i = 0; i < 3; i++) data_.euler[i] = out.euler[i];
        data_.yaw_total = out.yaw_total;

        // 运动加速度: 减去重力分量后 LPF
        const float gravity_n[3] = {0.0f, 0.0f, 9.81f};
        float gravity_b[3];
        ahrs::EarthFrameToBodyFrame(gravity_n, gravity_b, data_.q);

        float tau = accel_lpf_tau_;
        for (int i = 0; i < 3; i++) {
            motion_accel_b_[i] = (accel[i] - gravity_b[i]) * dt / (tau + dt)
                               + motion_accel_b_[i] * tau / (tau + dt);
        }

        ahrs::BodyFrameToEarthFrame(motion_accel_b_, data_.motion_accel_n, data_.q);
        std::memcpy(data_.motion_accel_b, motion_accel_b_, sizeof(motion_accel_b_));
    }

    const InsData& Data() const { return data_; }

private:
    QuaternionEkf ekf_;
    InsData data_{};
    float motion_accel_b_[3] = {};
    float accel_lpf_tau_ = 0.0085f;
};
