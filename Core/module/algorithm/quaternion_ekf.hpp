#pragma once

#include "arm_math.h"
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
    // NX=6, NZ=3 固定大小矩阵数据
    float xhat_data_[6]{}, xhatminus_data_[6]{}, z_data_[3]{};
    float P_data_[36]{}, Pminus_data_[36]{};
    float F_data_[36]{}, FT_data_[36]{};
    float H_data_[18]{}, HT_data_[18]{};
    float Q_data_[36]{}, R_data_[9]{}, K_data_[18]{};
    float S_data_[36]{};
    float tmp_mat_data_[36]{}, tmp_mat1_data_[36]{};
    float tmp_vec_data_[6]{}, tmp_vec1_data_[6]{};
    float chi_sq_data_[1]{};

    // ARM DSP 矩阵描述符
    arm_matrix_instance_f32 mat_xhat_{}, mat_xhatminus_{}, mat_z_{};
    arm_matrix_instance_f32 mat_P_{}, mat_Pminus_{};
    arm_matrix_instance_f32 mat_F_{}, mat_FT_{};
    arm_matrix_instance_f32 mat_H_{}, mat_HT_{};
    arm_matrix_instance_f32 mat_Q_{}, mat_R_{}, mat_K_{};
    arm_matrix_instance_f32 mat_S_{};
    arm_matrix_instance_f32 mat_tmp_{}, mat_tmp1_{};
    arm_matrix_instance_f32 mat_vec_{}, mat_vec1_{};
    arm_matrix_instance_f32 mat_chi_sq_{};

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
