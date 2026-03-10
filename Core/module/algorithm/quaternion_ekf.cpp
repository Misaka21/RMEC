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

    // 初始化 EKF 矩阵
    xhat_ = matrixf::zeros<6,1>();
    for (int i = 0; i < 4; i++) xhat_[i][0] = init_q[i];

    xhatminus_ = matrixf::zeros<6,1>();
    z_ = matrixf::zeros<3,1>();

    // P: 非对角 0.1, 对角 [100000,100000,100000,100000,100,100]
    P_ = matrixf::ones<6,6>() * 0.1f;
    P_[0][0] = 100000.0f;
    P_[1][1] = 100000.0f;
    P_[2][2] = 100000.0f;
    P_[3][3] = 100000.0f;
    P_[4][4] = 100.0f;
    P_[5][5] = 100.0f;

    Pminus_ = matrixf::zeros<6,6>();
    F_ = matrixf::eye<6,6>();
    Q_ = matrixf::zeros<6,6>();
    H_ = matrixf::zeros<3,6>();
    R_ = matrixf::zeros<3,3>();

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

    F_ = matrixf::eye<6,6>();

    F_[0][1] = -halfgxdt;
    F_[0][2] = -halfgydt;
    F_[0][3] = -halfgzdt;

    F_[1][0] =  halfgxdt;
    F_[1][2] =  halfgzdt;
    F_[1][3] = -halfgydt;

    F_[2][0] =  halfgydt;
    F_[2][1] = -halfgzdt;
    F_[2][3] =  halfgxdt;

    F_[3][0] =  halfgzdt;
    F_[3][1] =  halfgydt;
    F_[3][2] = -halfgxdt;

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
    z_[0][0] = accel_filtered_[0] * accel_inv_norm;
    z_[1][0] = accel_filtered_[1] * accel_inv_norm;
    z_[2][0] = accel_filtered_[2] * accel_inv_norm;

    // 5. StableFlag: 角速度小且加速度接近重力
    float gyro_norm = 1.0f / ahrs::InvSqrt(
        gyro_corrected_[0] * gyro_corrected_[0] +
        gyro_corrected_[1] * gyro_corrected_[1] +
        gyro_corrected_[2] * gyro_corrected_[2]);
    float accel_norm = 1.0f / accel_inv_norm;
    stable_flag_ = (gyro_norm < 0.3f && accel_norm > 9.3f && accel_norm < 10.3f);

    // 6. 设 Q/R 对角元素
    Q_ = matrixf::zeros<6,6>();
    Q_[0][0] = cfg_.q1 * dt;
    Q_[1][1] = cfg_.q1 * dt;
    Q_[2][2] = cfg_.q1 * dt;
    Q_[3][3] = cfg_.q1 * dt;
    Q_[4][4] = cfg_.q2 * dt;
    Q_[5][5] = cfg_.q2 * dt;

    R_ = matrixf::zeros<3,3>();
    R_[0][0] = cfg_.r;
    R_[1][1] = cfg_.r;
    R_[2][2] = cfg_.r;

    // 7–11. EKF 五步
    Predict();
    LinearizeAndFade();
    PredictCovariance();
    ComputeH();
    ComputeGainAndUpdate();

    // 12. 提取状态
    for (int i = 0; i < 4; i++) output_.q[i] = xhat_[i][0];
    output_.gyro_bias[0] = xhat_[4][0];
    output_.gyro_bias[1] = xhat_[5][0];
    output_.gyro_bias[2] = 0; // z 轴零飘不可观测

    // 13. 欧拉角 + 多圈 yaw
    ExtractEuler();

    update_count_++;
}

// ============================================================
// EKF 子步骤
// ============================================================

void QuaternionEkf::Predict() {
    xhatminus_ = F_ * xhat_;
}

void QuaternionEkf::LinearizeAndFade() {
    // 归一化预测四元数
    float q0 = xhatminus_[0][0], q1 = xhatminus_[1][0];
    float q2 = xhatminus_[2][0], q3 = xhatminus_[3][0];
    float inv_norm = ahrs::InvSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    for (int i = 0; i < 4; i++) xhatminus_[i][0] *= inv_norm;

    q0 = xhatminus_[0][0];
    q1 = xhatminus_[1][0];
    q2 = xhatminus_[2][0];
    q3 = xhatminus_[3][0];

    // 设 F 右上 4x2 分块 (零飘对四元数的耦合 Jacobian)
    float half_dt = dt_ / 2.0f;
    F_[0][4] =  q1 * half_dt;
    F_[0][5] =  q2 * half_dt;
    F_[1][4] = -q0 * half_dt;
    F_[1][5] =  q3 * half_dt;
    F_[2][4] = -q3 * half_dt;
    F_[2][5] = -q0 * half_dt;
    F_[3][4] =  q2 * half_dt;
    F_[3][5] = -q1 * half_dt;

    // 渐消滤波: 防止零飘方差过度收敛
    P_[4][4] /= cfg_.lambda;
    P_[5][5] /= cfg_.lambda;
    if (P_[4][4] > 10000.0f) P_[4][4] = 10000.0f;
    if (P_[5][5] > 10000.0f) P_[5][5] = 10000.0f;
}

void QuaternionEkf::PredictCovariance() {
    Pminus_ = F_ * P_ * F_.trans() + Q_;
}

void QuaternionEkf::ComputeH() {
    float q0 = xhatminus_[0][0], q1 = xhatminus_[1][0];
    float q2 = xhatminus_[2][0], q3 = xhatminus_[3][0];
    float dq0 = 2 * q0, dq1 = 2 * q1, dq2 = 2 * q2, dq3 = 2 * q3;

    // H (3x6), 后两列恒为零
    H_ = matrixf::zeros<3,6>();

    H_[0][0] = -dq2;
    H_[0][1] =  dq3;
    H_[0][2] = -dq0;
    H_[0][3] =  dq1;

    H_[1][0] =  dq1;
    H_[1][1] =  dq0;
    H_[1][2] =  dq3;
    H_[1][3] =  dq2;

    H_[2][0] =  dq0;
    H_[2][1] = -dq1;
    H_[2][2] = -dq2;
    H_[2][3] =  dq3;
}

void QuaternionEkf::ComputeGainAndUpdate() {
    // ---- S = H·Pminus·HT + R, 然后求逆 ----

    Mat<3,3> S = H_ * Pminus_ * H_.trans() + R_;
    Mat<3,3> S_inv = matrixf::inv(S);

    // ---- h(x): 从预测四元数推算重力方向 ----

    float q0 = xhatminus_[0][0], q1 = xhatminus_[1][0];
    float q2 = xhatminus_[2][0], q3 = xhatminus_[3][0];

    Vec<3> hx;
    hx[0][0] = 2 * (q1 * q3 - q0 * q2);
    hx[1][0] = 2 * (q0 * q1 + q2 * q3);
    hx[2][0] = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    // OrientationCosine (可观测性加权用)
    for (int i = 0; i < 3; i++)
        orientation_cosine_[i] = acosf(fabsf(hx[i][0]));

    // ---- innovation = z - h(x) ----

    Vec<3> innov = z_ - hx;

    // ---- 卡方检验: χ² = innovation' * inv(S) * innovation ----

    Mat<1,1> chi_sq_mat = innov.trans() * S_inv * innov;
    float chi_sq = chi_sq_mat[0][0];

    // ---- 卡方决策 ----

    if (chi_sq < 0.5f * chi_sq_threshold_)
        converge_flag_ = true;

    if (chi_sq > chi_sq_threshold_ && converge_flag_) {
        if (stable_flag_)
            error_count_++;
        else
            error_count_ = 0;

        if (error_count_ > 50) {
            // 滤波器发散, 重置收敛标志, 继续更新
            converge_flag_ = false;
        } else {
            // 残差过大 → 仅预测, 不融合
            xhat_ = xhatminus_;
            P_ = Pminus_;
            return;
        }
    } else {
        // 自适应增益缩放
        if (chi_sq > 0.1f * chi_sq_threshold_ && converge_flag_)
            adaptive_gain_scale_ = (chi_sq_threshold_ - chi_sq) / (0.9f * chi_sq_threshold_);
        else
            adaptive_gain_scale_ = 1.0f;
        error_count_ = 0;
    }

    // ---- K = Pminus * HT * inv(S) ----

    Mat<6,3> K = Pminus_ * H_.trans() * S_inv;

    // 自适应增益
    K *= adaptive_gain_scale_;

    // 零飘行 (4,5) 乘 OrientationCosine 加权
    for (int i = 4; i < 6; i++)
        for (int j = 0; j < 3; j++)
            K[i][j] *= orientation_cosine_[i - 4] / (PI / 2.0f);

    // ---- correction = K * innovation ----

    Vec<6> correction = K * innov;

    // 零飘修正限幅
    if (converge_flag_) {
        float limit = 1e-2f * dt_;
        for (int i = 4; i < 6; i++) {
            if (correction[i][0] > limit) correction[i][0] = limit;
            if (correction[i][0] < -limit) correction[i][0] = -limit;
        }
    }

    // 不修正 yaw
    correction[3][0] = 0;

    // ---- xhat = xhatminus + correction ----

    xhat_ = xhatminus_ + correction;

    // ---- P = Pminus - K·H·Pminus ----

    P_ = Pminus_ - K * H_ * Pminus_;
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
