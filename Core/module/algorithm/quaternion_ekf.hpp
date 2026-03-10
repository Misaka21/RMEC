#pragma once

#include "matrix.h"
#ifdef PI
#undef PI // arm_math.h 定义了 PI 宏, 与 general_def.hpp 的 constexpr 冲突
#endif

#include <cstdint>

struct QuaternionEkfConfig {
    float q1            = 10.0f;      // 四元数过程噪声
    float q2            = 0.001f;     // 陀螺仪零飘过程噪声
    float r             = 1000000.0f; // 加速度计量测噪声
    float lambda        = 1.0f;       // 渐消因子 (≤1)
    float acc_lpf_coef  = 0.0f;       // 加速度 LPF 系数 (0=不启用)
};

struct QuaternionEkfOutput {
    float q[4]         = {};   // [w,x,y,z]
    float gyro_bias[3] = {};
    float euler[3]     = {};   // [yaw, pitch, roll] degrees
    float yaw_total    = 0;
};

class QuaternionEkf {
public:
    void Init(const float init_q[4], const QuaternionEkfConfig& cfg);
    void Update(float gx, float gy, float gz,
                float ax, float ay, float az, float dt);
    const QuaternionEkfOutput& Output() const { return output_; }

private:
    template <int N> using Vec = Matrixf<N, 1>;
    template <int R, int C> using Mat = Matrixf<R, C>;

    // EKF 矩阵 (跨子函数共享)
    Vec<6> xhat_{}, xhatminus_{};
    Vec<3> z_{};
    Mat<6,6> P_{}, Pminus_{}, F_{}, Q_{};
    Mat<3,6> H_{};
    Mat<3,3> R_{};

    // 算法状态
    QuaternionEkfOutput output_{};
    QuaternionEkfConfig cfg_{};
    float accel_filtered_[3]{}, gyro_corrected_[3]{}, orientation_cosine_[3]{};
    bool converge_flag_ = false, stable_flag_ = false;
    uint64_t error_count_ = 0, update_count_ = 0;
    int16_t yaw_round_count_ = 0;
    float yaw_last_ = 0, chi_sq_threshold_ = 1e-8f, adaptive_gain_scale_ = 1.0f, dt_ = 0;

    // EKF 子步骤
    void Predict();
    void LinearizeAndFade();
    void PredictCovariance();
    void ComputeH();
    void ComputeGainAndUpdate();
    void ExtractEuler();
};
