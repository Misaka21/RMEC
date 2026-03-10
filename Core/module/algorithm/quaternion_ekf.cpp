#include "quaternion_ekf.hpp"
#include "ahrs_math.hpp"
#include "general_def.hpp"

#include <cmath>
#include <cstring>

// ============================================================
// Init
// ============================================================

void QuaternionEkf::Init(const float init_q[4], const QuaternionEkfConfig& cfg) {
    cfg_ = cfg;
    if (cfg_.lambda > 1.0f) cfg_.lambda = 1.0f;

    chi_sq_threshold_ = 1e-8f;
    converge_flag_ = false;
    error_count_ = 0;
    update_count_ = 0;
    adaptive_gain_scale_ = 1.0f;
    yaw_round_count_ = 0;
    yaw_last_ = 0;
    dt_ = 0;
    stable_flag_ = false;

    std::memset(accel_filtered_, 0, sizeof(accel_filtered_));
    std::memset(gyro_corrected_, 0, sizeof(gyro_corrected_));
    std::memset(orientation_cosine_, 0, sizeof(orientation_cosine_));

    // 清零所有矩阵数据
    std::memset(xhat_data_, 0, sizeof(xhat_data_));
    std::memset(xhatminus_data_, 0, sizeof(xhatminus_data_));
    std::memset(z_data_, 0, sizeof(z_data_));
    std::memset(Pminus_data_, 0, sizeof(Pminus_data_));
    std::memset(FT_data_, 0, sizeof(FT_data_));
    std::memset(H_data_, 0, sizeof(H_data_));
    std::memset(HT_data_, 0, sizeof(HT_data_));
    std::memset(Q_data_, 0, sizeof(Q_data_));
    std::memset(R_data_, 0, sizeof(R_data_));
    std::memset(K_data_, 0, sizeof(K_data_));
    std::memset(S_data_, 0, sizeof(S_data_));
    std::memset(tmp_mat_data_, 0, sizeof(tmp_mat_data_));
    std::memset(tmp_mat1_data_, 0, sizeof(tmp_mat1_data_));
    std::memset(tmp_vec_data_, 0, sizeof(tmp_vec_data_));
    std::memset(tmp_vec1_data_, 0, sizeof(tmp_vec1_data_));
    std::memset(chi_sq_data_, 0, sizeof(chi_sq_data_));

    // 设初始状态
    for (int i = 0; i < 4; i++) xhat_data_[i] = init_q[i];

    // 设初始 P: 对角 [100000,100000,100000,100000,100,100], 非对角 0.1
    for (int i = 0; i < 36; i++) P_data_[i] = 0.1f;
    P_data_[0]  = 100000.0f;
    P_data_[7]  = 100000.0f;
    P_data_[14] = 100000.0f;
    P_data_[21] = 100000.0f;
    P_data_[28] = 100.0f;
    P_data_[35] = 100.0f;

    // F 初始化为单位阵
    std::memset(F_data_, 0, sizeof(F_data_));
    F_data_[0] = F_data_[7] = F_data_[14] = F_data_[21] = F_data_[28] = F_data_[35] = 1.0f;

    // 初始化 ARM DSP 矩阵描述符
    arm_mat_init_f32(&mat_xhat_, 6, 1, xhat_data_);
    arm_mat_init_f32(&mat_xhatminus_, 6, 1, xhatminus_data_);
    arm_mat_init_f32(&mat_z_, 3, 1, z_data_);
    arm_mat_init_f32(&mat_P_, 6, 6, P_data_);
    arm_mat_init_f32(&mat_Pminus_, 6, 6, Pminus_data_);
    arm_mat_init_f32(&mat_F_, 6, 6, F_data_);
    arm_mat_init_f32(&mat_FT_, 6, 6, FT_data_);
    arm_mat_init_f32(&mat_H_, 3, 6, H_data_);
    arm_mat_init_f32(&mat_HT_, 6, 3, HT_data_);
    arm_mat_init_f32(&mat_Q_, 6, 6, Q_data_);
    arm_mat_init_f32(&mat_R_, 3, 3, R_data_);
    arm_mat_init_f32(&mat_K_, 6, 3, K_data_);
    arm_mat_init_f32(&mat_S_, 3, 3, S_data_);
    arm_mat_init_f32(&mat_tmp_, 6, 6, tmp_mat_data_);
    arm_mat_init_f32(&mat_tmp1_, 6, 6, tmp_mat1_data_);
    arm_mat_init_f32(&mat_vec_, 6, 1, tmp_vec_data_);
    arm_mat_init_f32(&mat_vec1_, 6, 1, tmp_vec1_data_);
    arm_mat_init_f32(&mat_chi_sq_, 1, 1, chi_sq_data_);

    // 初始化输出
    output_ = {};
    for (int i = 0; i < 4; i++) output_.q[i] = init_q[i];
}

// ============================================================
// Update  (主入口, 移植自 IMU_QuaternionEKF_Update)
// ============================================================

void QuaternionEkf::Update(float gx, float gy, float gz,
                           float ax, float ay, float az, float dt) {
    dt_ = dt;

    // 1. 陀螺仪减零飘
    gyro_corrected_[0] = gx - output_.gyro_bias[0];
    gyro_corrected_[1] = gy - output_.gyro_bias[1];
    gyro_corrected_[2] = gz - output_.gyro_bias[2];

    // 2. 重置 F 为单位阵, 设左上 4x4 四元数运动学
    float halfgxdt = 0.5f * gyro_corrected_[0] * dt;
    float halfgydt = 0.5f * gyro_corrected_[1] * dt;
    float halfgzdt = 0.5f * gyro_corrected_[2] * dt;

    std::memset(F_data_, 0, sizeof(F_data_));
    F_data_[0] = F_data_[7] = F_data_[14] = F_data_[21] = F_data_[28] = F_data_[35] = 1.0f;

    F_data_[1]  = -halfgxdt;
    F_data_[2]  = -halfgydt;
    F_data_[3]  = -halfgzdt;

    F_data_[6]  =  halfgxdt;
    F_data_[8]  =  halfgzdt;
    F_data_[9]  = -halfgydt;

    F_data_[12] =  halfgydt;
    F_data_[13] = -halfgzdt;
    F_data_[15] =  halfgxdt;

    F_data_[18] =  halfgzdt;
    F_data_[19] =  halfgydt;
    F_data_[20] = -halfgxdt;

    // 3. 加速度 LPF
    if (update_count_ == 0) {
        accel_filtered_[0] = ax;
        accel_filtered_[1] = ay;
        accel_filtered_[2] = az;
    }
    float lpf = cfg_.acc_lpf_coef;
    float lpf_denom = dt + lpf;
    accel_filtered_[0] = accel_filtered_[0] * lpf / lpf_denom + ax * dt / lpf_denom;
    accel_filtered_[1] = accel_filtered_[1] * lpf / lpf_denom + ay * dt / lpf_denom;
    accel_filtered_[2] = accel_filtered_[2] * lpf / lpf_denom + az * dt / lpf_denom;

    // 4. 归一化加速度 → 量测向量 z
    float accel_inv_norm = ahrs::InvSqrt(
        accel_filtered_[0] * accel_filtered_[0] +
        accel_filtered_[1] * accel_filtered_[1] +
        accel_filtered_[2] * accel_filtered_[2]);
    z_data_[0] = accel_filtered_[0] * accel_inv_norm;
    z_data_[1] = accel_filtered_[1] * accel_inv_norm;
    z_data_[2] = accel_filtered_[2] * accel_inv_norm;

    // 5. StableFlag: 角速度小且加速度接近重力
    float gyro_norm = 1.0f / ahrs::InvSqrt(
        gyro_corrected_[0] * gyro_corrected_[0] +
        gyro_corrected_[1] * gyro_corrected_[1] +
        gyro_corrected_[2] * gyro_corrected_[2]);
    float accel_norm = 1.0f / accel_inv_norm;
    stable_flag_ = (gyro_norm < 0.3f && accel_norm > 9.3f && accel_norm < 10.3f);

    // 6. 设 Q/R 对角元素
    std::memset(Q_data_, 0, sizeof(Q_data_));
    Q_data_[0]  = cfg_.q1 * dt;
    Q_data_[7]  = cfg_.q1 * dt;
    Q_data_[14] = cfg_.q1 * dt;
    Q_data_[21] = cfg_.q1 * dt;
    Q_data_[28] = cfg_.q2 * dt;
    Q_data_[35] = cfg_.q2 * dt;

    std::memset(R_data_, 0, sizeof(R_data_));
    R_data_[0] = cfg_.r;
    R_data_[4] = cfg_.r;
    R_data_[8] = cfg_.r;

    // 7–11. EKF 五步
    Predict();
    LinearizeAndFade();
    PredictCovariance();
    ComputeH();
    ComputeGainAndUpdate();

    // 12. 提取状态
    for (int i = 0; i < 4; i++) output_.q[i] = xhat_data_[i];
    output_.gyro_bias[0] = xhat_data_[4];
    output_.gyro_bias[1] = xhat_data_[5];
    output_.gyro_bias[2] = 0; // z 轴零飘不可观测

    // 13. 欧拉角 + 多圈 yaw
    ExtractEuler();

    update_count_++;
}

// ============================================================
// EKF 子步骤
// ============================================================

void QuaternionEkf::Predict() {
    // xhatminus = F * xhat (F 右 4x2 列为零, bias 不参与预测)
    (void)arm_mat_mult_f32(&mat_F_, &mat_xhat_, &mat_xhatminus_);
}

void QuaternionEkf::LinearizeAndFade() {
    // 归一化预测四元数
    float q0 = xhatminus_data_[0], q1 = xhatminus_data_[1];
    float q2 = xhatminus_data_[2], q3 = xhatminus_data_[3];
    float inv_norm = ahrs::InvSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    for (int i = 0; i < 4; i++) xhatminus_data_[i] *= inv_norm;

    q0 = xhatminus_data_[0];
    q1 = xhatminus_data_[1];
    q2 = xhatminus_data_[2];
    q3 = xhatminus_data_[3];

    // 设 F 右上 4x2 分块 (零飘对四元数的耦合 Jacobian)
    float half_dt = dt_ / 2.0f;
    F_data_[4]  =  q1 * half_dt;
    F_data_[5]  =  q2 * half_dt;
    F_data_[10] = -q0 * half_dt;
    F_data_[11] =  q3 * half_dt;
    F_data_[16] = -q3 * half_dt;
    F_data_[17] = -q0 * half_dt;
    F_data_[22] =  q2 * half_dt;
    F_data_[23] = -q1 * half_dt;

    // 渐消滤波: 防止零飘方差过度收敛
    P_data_[28] /= cfg_.lambda;
    P_data_[35] /= cfg_.lambda;
    if (P_data_[28] > 10000.0f) P_data_[28] = 10000.0f;
    if (P_data_[35] > 10000.0f) P_data_[35] = 10000.0f;
}

void QuaternionEkf::PredictCovariance() {
    // FT = transpose(F)
    (void)arm_mat_trans_f32(&mat_F_, &mat_FT_);

    // tmp(6x6) = F * P
    mat_tmp_.numRows = 6;
    mat_tmp_.numCols = 6;
    (void)arm_mat_mult_f32(&mat_F_, &mat_P_, &mat_tmp_);

    // tmp1(6x6) = tmp * FT
    mat_tmp1_.numRows = 6;
    mat_tmp1_.numCols = 6;
    (void)arm_mat_mult_f32(&mat_tmp_, &mat_FT_, &mat_tmp1_);

    // Pminus = tmp1 + Q
    (void)arm_mat_add_f32(&mat_tmp1_, &mat_Q_, &mat_Pminus_);
}

void QuaternionEkf::ComputeH() {
    float q0 = xhatminus_data_[0], q1 = xhatminus_data_[1];
    float q2 = xhatminus_data_[2], q3 = xhatminus_data_[3];
    float dq0 = 2 * q0, dq1 = 2 * q1, dq2 = 2 * q2, dq3 = 2 * q3;

    // H (3x6), 后两列恒为零
    std::memset(H_data_, 0, sizeof(H_data_));

    H_data_[0]  = -dq2;
    H_data_[1]  =  dq3;
    H_data_[2]  = -dq0;
    H_data_[3]  =  dq1;

    H_data_[6]  =  dq1;
    H_data_[7]  =  dq0;
    H_data_[8]  =  dq3;
    H_data_[9]  =  dq2;

    H_data_[12] =  dq0;
    H_data_[13] = -dq1;
    H_data_[14] = -dq2;
    H_data_[15] =  dq3;
}

void QuaternionEkf::ComputeGainAndUpdate() {
    // ---- S = H·Pminus·HT + R, 然后求逆 ----

    // HT(6x3) = transpose(H)
    (void)arm_mat_trans_f32(&mat_H_, &mat_HT_);

    // tmp(3x6) = H * Pminus
    mat_tmp_.numRows = 3;
    mat_tmp_.numCols = 6;
    (void)arm_mat_mult_f32(&mat_H_, &mat_Pminus_, &mat_tmp_);

    // tmp1(3x3) = tmp * HT
    mat_tmp1_.numRows = 3;
    mat_tmp1_.numCols = 3;
    (void)arm_mat_mult_f32(&mat_tmp_, &mat_HT_, &mat_tmp1_);

    // S(3x3) = tmp1 + R
    mat_S_.numRows = 3;
    mat_S_.numCols = 3;
    (void)arm_mat_add_f32(&mat_tmp1_, &mat_R_, &mat_S_);

    // inv(S) → tmp1(3x3)
    mat_tmp1_.numRows = 3;
    mat_tmp1_.numCols = 3;
    (void)arm_mat_inverse_f32(&mat_S_, &mat_tmp1_);

    // ---- h(x): 从预测四元数推算重力方向 ----

    float q0 = xhatminus_data_[0], q1 = xhatminus_data_[1];
    float q2 = xhatminus_data_[2], q3 = xhatminus_data_[3];

    mat_vec_.numRows = 3;
    mat_vec_.numCols = 1;
    tmp_vec_data_[0] = 2 * (q1 * q3 - q0 * q2);
    tmp_vec_data_[1] = 2 * (q0 * q1 + q2 * q3);
    tmp_vec_data_[2] = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    // OrientationCosine (可观测性加权用)
    for (int i = 0; i < 3; i++)
        orientation_cosine_[i] = acosf(fabsf(tmp_vec_data_[i]));

    // ---- innovation = z - h(x) ----

    mat_vec1_.numRows = 3;
    mat_vec1_.numCols = 1;
    (void)arm_mat_sub_f32(&mat_z_, &mat_vec_, &mat_vec1_);

    // ---- 卡方检验: χ² = innovation' * inv(S) * innovation ----

    // tmp(3x1) = inv(S) * innovation
    mat_tmp_.numRows = 3;
    mat_tmp_.numCols = 1;
    (void)arm_mat_mult_f32(&mat_tmp1_, &mat_vec1_, &mat_tmp_);

    // vec(1x3) = innovation'
    mat_vec_.numRows = 1;
    mat_vec_.numCols = 3;
    (void)arm_mat_trans_f32(&mat_vec1_, &mat_vec_);

    // chi²(1x1) = vec * tmp
    (void)arm_mat_mult_f32(&mat_vec_, &mat_tmp_, &mat_chi_sq_);

    // ---- 卡方决策 ----

    if (chi_sq_data_[0] < 0.5f * chi_sq_threshold_)
        converge_flag_ = true;

    if (chi_sq_data_[0] > chi_sq_threshold_ && converge_flag_) {
        if (stable_flag_)
            error_count_++;
        else
            error_count_ = 0;

        if (error_count_ > 50) {
            // 滤波器发散, 重置收敛标志, 继续更新
            converge_flag_ = false;
        } else {
            // 残差过大 → 仅预测, 不融合
            std::memcpy(xhat_data_, xhatminus_data_, sizeof(float) * 6);
            std::memcpy(P_data_, Pminus_data_, sizeof(float) * 36);
            return;
        }
    } else {
        // 自适应增益缩放
        if (chi_sq_data_[0] > 0.1f * chi_sq_threshold_ && converge_flag_)
            adaptive_gain_scale_ = (chi_sq_threshold_ - chi_sq_data_[0]) / (0.9f * chi_sq_threshold_);
        else
            adaptive_gain_scale_ = 1.0f;
        error_count_ = 0;
    }

    // ---- K = Pminus * HT * inv(S) ----

    // tmp(6x3) = Pminus * HT
    mat_tmp_.numRows = 6;
    mat_tmp_.numCols = 3;
    (void)arm_mat_mult_f32(&mat_Pminus_, &mat_HT_, &mat_tmp_);

    // K(6x3) = tmp * inv(S)   (inv(S) 仍在 mat_tmp1_)
    (void)arm_mat_mult_f32(&mat_tmp_, &mat_tmp1_, &mat_K_);

    // 自适应增益
    for (int i = 0; i < 18; i++)
        K_data_[i] *= adaptive_gain_scale_;

    // 零飘行 (4,5) 乘 OrientationCosine 加权
    for (int i = 4; i < 6; i++)
        for (int j = 0; j < 3; j++)
            K_data_[i * 3 + j] *= orientation_cosine_[i - 4] / (PI / 2.0f);

    // ---- correction = K * innovation ----

    mat_vec_.numRows = 6;
    mat_vec_.numCols = 1;
    (void)arm_mat_mult_f32(&mat_K_, &mat_vec1_, &mat_vec_);

    // 零飘修正限幅
    if (converge_flag_) {
        float limit = 1e-2f * dt_;
        for (int i = 4; i < 6; i++) {
            if (tmp_vec_data_[i] > limit) tmp_vec_data_[i] = limit;
            if (tmp_vec_data_[i] < -limit) tmp_vec_data_[i] = -limit;
        }
    }

    // 不修正 yaw
    tmp_vec_data_[3] = 0;

    // ---- xhat = xhatminus + correction ----

    mat_vec_.numRows = 6;
    mat_vec_.numCols = 1;
    (void)arm_mat_add_f32(&mat_xhatminus_, &mat_vec_, &mat_xhat_);

    // ---- P = Pminus - K·H·Pminus ----

    // tmp(6x6) = K * H
    mat_tmp_.numRows = 6;
    mat_tmp_.numCols = 6;
    (void)arm_mat_mult_f32(&mat_K_, &mat_H_, &mat_tmp_);

    // tmp1(6x6) = tmp * Pminus
    mat_tmp1_.numRows = 6;
    mat_tmp1_.numCols = 6;
    (void)arm_mat_mult_f32(&mat_tmp_, &mat_Pminus_, &mat_tmp1_);

    // P = Pminus - tmp1
    (void)arm_mat_sub_f32(&mat_Pminus_, &mat_tmp1_, &mat_P_);
}

void QuaternionEkf::ExtractEuler() {
    float q0 = output_.q[0], q1 = output_.q[1];
    float q2 = output_.q[2], q3 = output_.q[3];

    output_.euler[0] = atan2f(2.0f * (q0 * q3 + q1 * q2),
                              2.0f * (q0 * q0 + q1 * q1) - 1.0f) * RAD_2_DEGREE;
    output_.euler[1] = atan2f(2.0f * (q0 * q1 + q2 * q3),
                              2.0f * (q0 * q0 + q3 * q3) - 1.0f) * RAD_2_DEGREE;
    output_.euler[2] = asinf(-2.0f * (q1 * q3 - q0 * q2)) * RAD_2_DEGREE;

    // 多圈 yaw 追踪
    if (output_.euler[0] - yaw_last_ > 180.0f)
        yaw_round_count_--;
    else if (output_.euler[0] - yaw_last_ < -180.0f)
        yaw_round_count_++;

    output_.yaw_total = 360.0f * yaw_round_count_ + output_.euler[0];
    yaw_last_ = output_.euler[0];
}
