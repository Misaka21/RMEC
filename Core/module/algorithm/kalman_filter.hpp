#pragma once

#include "matrix.h"

/// @file kalman_filter.hpp
/// @brief 线性卡尔曼滤波器 (KF) 和扩展卡尔曼滤波器 (EKF)
///
/// 模板参数在编译期约束矩阵维度，使用 Matrixf<R,C>（CMSIS-DSP 后端，float）。
/// EKF 通过函数指针回调提供非线性模型及 Jacobian，零动态分配。
/// 协方差更新采用 Joseph form 保证单精度下对称正定。

namespace kf {

// ════════════════════════════════════════════════════════════════════════
//  类型别名
// ════════════════════════════════════════════════════════════════════════

template <int N>
using Vec = Matrixf<N, 1>;

template <int R, int C>
using Mat = Matrixf<R, C>;

// ════════════════════════════════════════════════════════════════════════
//  EKF 回调结果结构体
// ════════════════════════════════════════════════════════════════════════

/// 预测回调返回值：预测状态 + 状态转移 Jacobian F
template <int N_X>
struct PredictResult {
    Vec<N_X>      x_pred;  ///< f(x) 或 f(x,dt) 预测后的状态
    Mat<N_X, N_X> F;       ///< 状态转移 Jacobian ∂f/∂x
};

/// 观测回调返回值：预测观测 + 观测 Jacobian H
template <int N_X, int N_Z>
struct MeasureResult {
    Vec<N_Z>      z_pred;  ///< h(x) 预测观测
    Mat<N_Z, N_X> H;       ///< 观测 Jacobian ∂h/∂x
};

// ════════════════════════════════════════════════════════════════════════
//  线性卡尔曼滤波器
// ════════════════════════════════════════════════════════════════════════

/// @tparam N_X 状态维度
/// @tparam N_Z 观测维度
template <int N_X, int N_Z>
class KalmanFilter final {
public:
    KalmanFilter() noexcept = default;

    /// 初始化滤波器全部参数
    void Init(const Vec<N_X>&      x0,
              const Mat<N_X, N_X>& P0,
              const Mat<N_X, N_X>& F,
              const Mat<N_Z, N_X>& H,
              const Mat<N_X, N_X>& Q,
              const Mat<N_Z, N_Z>& R) noexcept
    {
        x_ = x0;
        P_ = P0;
        F_ = F;
        H_ = H;
        Q_ = Q;
        R_ = R;
    }

    /// 预测步：x = F*x, P = F*P*F' + Q
    void Predict() noexcept
    {
        x_ = F_ * x_;
        P_ = F_ * P_ * F_.trans() + Q_;
    }

    /// 更新步（Joseph form）
    void Update(const Vec<N_Z>& z) noexcept
    {
        // 新息
        Vec<N_Z> y = z - H_ * x_;

        // 新息协方差
        Mat<N_Z, N_Z> S = H_ * P_ * H_.trans() + R_;

        // 卡尔曼增益 K = P*H'*inv(S)
        Mat<N_X, N_Z> K = P_ * H_.trans() * matrixf::inv(S);

        // 状态更新
        x_ = x_ + K * y;

        // 协方差更新 (Joseph form): P = (I-KH)*P*(I-KH)' + K*R*K'
        Mat<N_X, N_X> I_KH = matrixf::eye<N_X, N_X>() - K * H_;
        P_ = I_KH * P_ * I_KH.trans() + K * R_ * K.trans();
    }

    /// 预测 + 更新一步完成
    void Step(const Vec<N_Z>& z) noexcept
    {
        Predict();
        Update(z);
    }

    // ── 参数设置 ──

    void SetF(const Mat<N_X, N_X>& F) noexcept { F_ = F; }
    void SetH(const Mat<N_Z, N_X>& H) noexcept { H_ = H; }
    void SetQ(const Mat<N_X, N_X>& Q) noexcept { Q_ = Q; }
    void SetR(const Mat<N_Z, N_Z>& R) noexcept { R_ = R; }

    /// 直接设置状态（用于外部重置或注入）
    void SetState(const Vec<N_X>& x) noexcept { x_ = x; }

    /// 直接设置协方差（用于外部重置）
    void SetCovariance(const Mat<N_X, N_X>& P) noexcept { P_ = P; }

    // ── 状态访问 ──

    [[nodiscard]] const Vec<N_X>&      x() const noexcept { return x_; }
    [[nodiscard]] const Mat<N_X, N_X>& P() const noexcept { return P_; }

private:
    Vec<N_X>      x_{};    ///< 状态向量
    Mat<N_X, N_X> P_{};    ///< 状态协方差
    Mat<N_X, N_X> F_{};    ///< 状态转移矩阵
    Mat<N_Z, N_X> H_{};    ///< 观测矩阵
    Mat<N_X, N_X> Q_{};    ///< 过程噪声协方差
    Mat<N_Z, N_Z> R_{};    ///< 观测噪声协方差
};

// ════════════════════════════════════════════════════════════════════════
//  扩展卡尔曼滤波器
// ════════════════════════════════════════════════════════════════════════

/// @tparam N_X 状态维度
/// @tparam N_Z 观测维度
template <int N_X, int N_Z>
class ExtendedKalmanFilter final {
public:
    // ── 回调函数类型 ──

    using StateVec = Vec<N_X>;

    /// 无时间步预测回调：f(x) → {x_pred, F}
    using PredictFunc   = PredictResult<N_X> (*)(const StateVec& x);

    /// 含时间步预测回调：f(x, dt) → {x_pred, F}
    using PredictFuncDt = PredictResult<N_X> (*)(const StateVec& x, float dt);

    /// 观测回调：h(x) → {z_pred, H}
    using MeasureFunc   = MeasureResult<N_X, N_Z> (*)(const StateVec& x);

    ExtendedKalmanFilter() noexcept = default;

    /// 初始化 EKF（F/H 由回调动态提供，此处只设初值和噪声）
    void Init(const Vec<N_X>&      x0,
              const Mat<N_X, N_X>& P0,
              const Mat<N_X, N_X>& Q,
              const Mat<N_Z, N_Z>& R) noexcept
    {
        x_ = x0;
        P_ = P0;
        Q_ = Q;
        R_ = R;
    }

    /// 预测步（无时间参数）
    void Predict(PredictFunc f) noexcept
    {
        auto [x_pred, F] = f(x_);
        x_ = x_pred;
        P_ = F * P_ * F.trans() + Q_;
    }

    /// 预测步（含时间步 dt）
    void Predict(PredictFuncDt f, float dt) noexcept
    {
        auto [x_pred, F] = f(x_, dt);
        x_ = x_pred;
        P_ = F * P_ * F.trans() + Q_;
    }

    /// 更新步（Joseph form）
    void Update(MeasureFunc h, const Vec<N_Z>& z) noexcept
    {
        auto [z_pred, H] = h(x_);

        // 新息
        Vec<N_Z> y = z - z_pred;

        // 新息协方差
        Mat<N_Z, N_Z> S = H * P_ * H.trans() + R_;

        // 卡尔曼增益
        Mat<N_X, N_Z> K = P_ * H.trans() * matrixf::inv(S);

        // 状态更新
        x_ = x_ + K * y;

        // 协方差更新 (Joseph form)
        Mat<N_X, N_X> I_KH = matrixf::eye<N_X, N_X>() - K * H;
        P_ = I_KH * P_ * I_KH.trans() + K * R_ * K.trans();
    }

    /// 预测 + 更新（无时间参数）
    void Step(PredictFunc f, MeasureFunc h, const Vec<N_Z>& z) noexcept
    {
        Predict(f);
        Update(h, z);
    }

    /// 预测 + 更新（含时间步 dt）
    void Step(PredictFuncDt f, float dt,
              MeasureFunc h, const Vec<N_Z>& z) noexcept
    {
        Predict(f, dt);
        Update(h, z);
    }

    // ── 参数设置 ──

    void SetQ(const Mat<N_X, N_X>& Q) noexcept { Q_ = Q; }
    void SetR(const Mat<N_Z, N_Z>& R) noexcept { R_ = R; }

    /// 直接设置状态（用于外部重置或注入）
    void SetState(const Vec<N_X>& x) noexcept { x_ = x; }

    /// 直接设置协方差（用于外部重置）
    void SetCovariance(const Mat<N_X, N_X>& P) noexcept { P_ = P; }

    // ── 状态访问 ──

    [[nodiscard]] const Vec<N_X>&      x() const noexcept { return x_; }
    [[nodiscard]] const Mat<N_X, N_X>& P() const noexcept { return P_; }

private:
    Vec<N_X>      x_{};    ///< 状态向量
    Mat<N_X, N_X> P_{};    ///< 状态协方差
    Mat<N_X, N_X> Q_{};    ///< 过程噪声协方差
    Mat<N_Z, N_Z> R_{};    ///< 观测噪声协方差
};

// ════════════════════════════════════════════════════════════════════════
//  便利工厂函数
// ════════════════════════════════════════════════════════════════════════

/// 生成匀速模型的状态转移矩阵 F（N_DIM 维位置 + N_DIM 维速度）
/// 状态排列 [p0, p1, ..., v0, v1, ...]
///
/// F = | I   dt*I |
///     | 0     I  |
///
/// @tparam N_DIM 空间维度（如 2D 或 3D）
/// @return 2*N_DIM × 2*N_DIM 状态转移矩阵
template <int N_DIM>
[[nodiscard]] inline Mat<2 * N_DIM, 2 * N_DIM>
MakeConstantVelocityF(float dt) noexcept
{
    auto F = matrixf::eye<2 * N_DIM, 2 * N_DIM>();
    for (int i = 0; i < N_DIM; ++i) {
        F[i][i + N_DIM] = dt;
    }
    return F;
}

/// 生成仅观测位置的观测矩阵 H（N_DIM 维位置 + N_DIM 维速度状态）
///
/// H = | I   0 |   (N_DIM × 2*N_DIM)
///
/// @tparam N_DIM 空间维度
/// @return N_DIM × 2*N_DIM 观测矩阵
template <int N_DIM>
[[nodiscard]] inline Mat<N_DIM, 2 * N_DIM>
MakePositionObservationH() noexcept
{
    auto H = matrixf::zeros<N_DIM, 2 * N_DIM>();
    for (int i = 0; i < N_DIM; ++i) {
        H[i][i] = 1.0f;
    }
    return H;
}

// ════════════════════════════════════════════════════════════════════════
//  单轴多项式动力学滤波器
// ════════════════════════════════════════════════════════════════════════

/// @brief 1D 标量滤波器，内部状态为 [x, dx/dt, d²x/dt², ...] 共 ORDER 阶
///
/// 状态转移矩阵为 Taylor 展开：
///   F(dt) = | 1  dt  dt²/2  dt³/6 ... |
///           | 0   1   dt    dt²/2 ... |
///           | 0   0    1     dt   ... |
///           | ...                     |
/// 观测仅取第 0 维（位置）。
///
/// 典型用法：
///   - ORDER=2 : 匀速模型，状态 [位置, 速度]
///   - ORDER=3 : 匀加速模型，状态 [位置, 速度, 加速度]
///
/// @tparam ORDER 多项式阶数（≥1）
template <int ORDER>
class SingleAxisKF final {
    static_assert(ORDER >= 1, "ORDER must be >= 1");

public:
    SingleAxisKF() noexcept = default;

    /// 用对角 Q 和标量 R 初始化（初始协方差设大值，第一次观测主导）
    /// @param q  过程噪声对角元素 [q_pos, q_vel, q_acc, ...]
    /// @param r  观测噪声方差（标量）
    void Init(const float (&q)[ORDER], float r) noexcept
    {
        auto x0 = matrixf::zeros<ORDER, 1>();
        auto P0 = matrixf::eye<ORDER, ORDER>() * 1e6f;

        auto Q = matrixf::zeros<ORDER, ORDER>();
        for (int i = 0; i < ORDER; ++i) Q[i][i] = q[i];

        auto R = matrixf::zeros<1, 1>();
        R[0][0] = r;

        auto H = matrixf::zeros<1, ORDER>();
        H[0][0] = 1.0f;

        auto F = matrixf::eye<ORDER, ORDER>();

        kf_.Init(x0, P0, F, H, Q, R);
    }

    /// 带初始状态的初始化（协方差为单位阵，表示对初值有信心）
    /// @param x0  初始状态 [pos, vel, ...]
    /// @param q   过程噪声对角元素
    /// @param r   观测噪声方差
    void Init(const float (&x0)[ORDER],
              const float (&q)[ORDER], float r) noexcept
    {
        Init(q, r);

        Vec<ORDER> xv;
        for (int i = 0; i < ORDER; ++i) xv[i][0] = x0[i];
        kf_.SetState(xv);
        kf_.SetCovariance(matrixf::eye<ORDER, ORDER>());
    }

    /// 送入一次观测，自动用 dt 构建 F 矩阵并完成 predict + update
    /// @param measurement 标量观测值
    /// @param dt          距上次调用的时间间隔（秒）
    void Update(float measurement, float dt) noexcept
    {
        kf_.SetF(MakePolyF(dt));

        Vec<1> z;
        z[0][0] = measurement;
        kf_.Step(z);
    }

    /// 仅预测（不更新），返回预测后的完整状态
    [[nodiscard]] Vec<ORDER> Predict(float dt) const noexcept
    {
        return MakePolyF(dt) * kf_.x();
    }

    /// 预测 dt 秒后的位置（第 0 维）
    [[nodiscard]] float PredictPosition(float dt) const noexcept
    {
        return Predict(dt)[0][0];
    }

    /// 直接设置状态向量
    void SetState(const float (&x)[ORDER]) noexcept
    {
        Vec<ORDER> xv;
        for (int i = 0; i < ORDER; ++i) xv[i][0] = x[i];
        kf_.SetState(xv);
    }

    /// 重置滤波器（清零状态，协方差恢复大值）
    void Reset() noexcept
    {
        kf_.SetState(matrixf::zeros<ORDER, 1>());
        kf_.SetCovariance(matrixf::eye<ORDER, ORDER>() * 1e6f);
    }

    // ── 状态访问 ──

    /// 当前位置估计（第 0 维）
    [[nodiscard]] float Position() const noexcept { return kf_.x()[0][0]; }

    /// 当前速度估计（第 1 维），ORDER >= 2 时有效
    [[nodiscard]] float Velocity() const noexcept
    {
        static_assert(ORDER >= 2, "Velocity requires ORDER >= 2");
        return kf_.x()[1][0];
    }

    /// 当前加速度估计（第 2 维），ORDER >= 3 时有效
    [[nodiscard]] float Acceleration() const noexcept
    {
        static_assert(ORDER >= 3, "Acceleration requires ORDER >= 3");
        return kf_.x()[2][0];
    }

    /// 完整状态向量
    [[nodiscard]] const Vec<ORDER>& x() const noexcept { return kf_.x(); }

    /// 协方差矩阵
    [[nodiscard]] const Mat<ORDER, ORDER>& P() const noexcept { return kf_.P(); }

private:
    /// 构造多项式状态转移矩阵 F(dt)
    /// F[i][j] = dt^(j-i) / (j-i)!  对于 j >= i
    [[nodiscard]] static Mat<ORDER, ORDER> MakePolyF(float dt) noexcept
    {
        auto F = matrixf::eye<ORDER, ORDER>();
        for (int i = 0; i < ORDER; ++i) {
            float coeff = 1.0f;
            for (int j = i + 1; j < ORDER; ++j) {
                coeff *= dt / static_cast<float>(j - i);
                F[i][j] = coeff;
            }
        }
        return F;
    }

    KalmanFilter<ORDER, 1> kf_{};
};

}  // namespace kf
